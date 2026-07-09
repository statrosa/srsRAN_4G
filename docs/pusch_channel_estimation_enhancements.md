# PUSCH Channel Estimation Enhancements — Implementation Plan

This document is the detailed implementation plan for six enhancements to the LTE uplink
(PUSCH) channel estimator in srsRAN 4G. It explains the problems being solved, the design
decisions and the mathematics behind them, and the concrete code changes, file by file.
The work lands as six commits — two test commits bracketing four estimator commits — so
that every estimator change is justified by a measured baseline and locked in by a
tightened regression threshold.

**Scope:** `lib/src/phy/ch_estimation/chest_ul.c`, `chest_common.c`, their headers, and
the tests under `lib/src/phy/ch_estimation/test/` and `lib/src/phy/phch/test/`.
PUCCH and SRS estimation paths are explicitly out of scope and must remain bit-identical.

---

## 1. Background and motivation

### 1.1 How the UL estimator works today

`srsran_chest_ul_estimate_pusch()` (in `chest_ul.c`) estimates the channel for a PUSCH
grant from the two DMRS symbols (one per slot, symbol 3 of each 7-symbol slot for normal
CP):

1. **DMRS extraction** — `srsran_refsignal_dmrs_pusch_get()` pulls the received pilots
   from the resource grid at the grant's PRBs.
2. **Least-squares (LS) estimate** — the received pilots are multiplied by the conjugate
   of the known DMRS sequence (`srsran_vec_prod_conj_ccc`), giving one raw channel sample
   per pilot subcarrier: `p[k] = H[k] + n[k]`.
3. **Frequency smoothing** — `average_pilots()` convolves each slot's LS estimates with a
   fixed 3-tap flat filter (`smooth_filter`, coefficients ≈ [⅓ ⅓ ⅓]) via
   `srsran_chest_average_pilots()` → `srsran_conv_same_cf()`.
4. **Noise estimation** — `estimate_noise_pilots()` measures the power of
   (raw LS − smoothed) and divides it by an empirical polynomial to undo the fact that
   the smoother absorbs part of the noise into the "signal".
5. **Time interpolation** — `interpolate_pilots()` holds each slot's smoothed estimate
   across all 7 symbols of that slot (linear interpolation exists behind a disabled
   `DO_LINEAR_INTERPOLATION` flag).
6. **Measurements** — cross-slot pilot phase → CFO estimate; pilot autocorrelation across
   frequency → timing advance; EPRE/SNR reported in `srsran_chest_ul_res_t`.

### 1.2 What is wrong with it

Three quality problems and one correctness bug, all concentrated on **narrow, low-SNR
grants** — exactly what a cell-edge UE gets (1–4 PRB QPSK allocations):

* **(Bug) Frequency hopping corrupts estimation.** The CE grid is indexed by the
  *pre-hopping* PRB start `grant.n_prb[]`, but both DMRS extraction (`refsignal_ul.c`)
  and the PUSCH demapper (`pusch_cp`) use the *post-hopping* `grant.n_prb_tilde[]`.
  Whenever they differ (type-2 hopping always; type-1 with inter-subframe hopping on odd
  transmissions), estimates are written to the wrong resource elements and the decoder
  equalizes with stale or zero estimates. The estimator detects the intra-subframe case
  and just logs `ERROR: intra-subframe frequency hopping not supported` — then continues
  with corrupted output.
* **The fixed 3-tap smoother is far from optimal at low SNR.** A 3-tap average reduces
  noise by only ~3 dB. At 0 dB SNR on a 1-PRB grant the resulting CE error is ≈0.58·N0,
  i.e. barely 2.4 dB better than raw LS, and it is the dominant impairment for QPSK
  demodulation at the SNRs where those grants operate.
* **Band-edge handling amplifies noise.** `srsran_conv_same_cf()` handles the band edges
  by *linearly extrapolating* the input beyond the allocation. The extrapolated edge
  outputs carry up to 1.89× the input noise power — worse than no smoothing at all. On a
  1-PRB grant, 2 of the 12 subcarriers are edge outputs.
* **The noise estimate is biased on narrow grants.** The residual-to-noise correction is
  an empirical polynomial (`a = 7.419w² + 0.1117w − 0.005387`, then `power/(a·0.8)`)
  calibrated for the 3-tap filter on *wide* allocations. Because the edge outputs behave
  differently from the interior, the polynomial overestimates noise by **+0.97 dB at
  1 PRB** and **+0.45 dB at 2 PRB**. This bias directly pollutes the MMSE equalizer
  regularizer and the UL SNR the MAC uses for link adaptation.
* **No cross-slot combining.** Each slot's smoothed estimate is simply held across its 7
  symbols. When the channel is time-flat within the 1 ms subframe (the common case at
  low mobility), the two DMRS estimates are redundant and averaging them would halve the
  noise — a free 3 dB that is currently left on the table.

### 1.3 Why the tests must come first

Neither existing test can see any of this:

* `chest_test_ul` fills the grid with **random data instead of DMRS** and checks a loose
  mean-absolute-error bound — it cannot measure estimator quality or hopping correctness.
* `pusch_test` decodes with an **identity channel estimate**
  (`srsran_chest_ul_res_set_identity`) — the estimator is never in the decode loop.

So the plan starts by building measurement infrastructure, asserting today's numbers as
baselines, then improving the estimator and tightening the thresholds commit by commit.

---

## 2. Commit-by-commit implementation plan

### Commit 1 — `test,chest_ul`: estimation quality mode for `chest_test_ul`

**Files:** `lib/src/phy/ch_estimation/test/chest_test_ul.c`,
`lib/src/phy/ch_estimation/test/CMakeLists.txt`

Add a *quality mode* to the existing test, selected with `-L <L_prb>`, that measures the
estimator against ground truth:

1. Generate and place **real PUSCH DMRS** with `srsran_refsignal_dmrs_pusch_gen()` +
   `srsran_refsignal_dmrs_pusch_put()` for an `L_prb`-wide grant, exactly as `ue_ul.c`
   does.
2. Apply a known channel `H[k]`: flat by default, or a smooth frequency-selective profile
   with `-S`. Optionally rotate symbols for a CFO stimulus and add AWGN at `-s <snr_db>`.
3. Run `srsran_chest_ul_estimate_pusch()` and compute, averaged over `-N` subframes:
   * **CE NMSE relative to N0** — `E{|ĥ−H|²}/N0` on the grant REs. Raw LS scores 1.0 by
     construction, so this directly reads "how much better than no smoothing".
   * **Noise-estimate accuracy** — `10·log10(N̂0/N0)` in dB.
4. `-M <max_nmse>` and `-E <max_db>` turn the measurements into pass/fail bounds.
5. `-H` stimulates **intra-subframe frequency hopping**: the two slots are placed at
   different PRB offsets via `grant.n_prb_tilde[]`, matching what a hopping UE transmits.
   (These hopping assertions become the unit-level guard for Commit 2.)

Register ctest cases asserting the **current** estimator's measured baselines (flat
channel): 1 PRB @ 0 dB → NMSE 0.58, noise bias +0.97 dB; 2 PRB → 0.46 / +0.45 dB;
4 PRB → 0.40 / +0.24 dB. Thresholds start just above these (e.g. `-M 0.70 -E 1.5` for
1 PRB) and are tightened by later commits. Also add:

* a noiseless sanity case (`-M 0.0001`) — the estimator must be exact without noise;
* a **frequency-selective 30 dB** case (`-S -M 95`, threshold in raw MSE terms) as a
  permanent regression guard against over-smoothing a selective channel.

### Commit 2 — `chest_ul`: fix PUSCH estimation under frequency hopping

**File:** `lib/src/phy/ch_estimation/chest_ul.c`

The minimal, surgical fix:

1. In `srsran_chest_ul_estimate_pusch()`, pass `cfg->grant.n_prb_tilde` (not
   `cfg->grant.n_prb`) into `chest_ul_estimate()`. Every consumer of the CE grid indexes
   by `n_prb_tilde`; without hopping the two arrays are identical, so **non-hopping
   operation is bit-exact** after this change.
2. In `chest_ul_estimate()`, replace the log-and-continue error with correct behavior:
   when the two slots sit at different PRBs (`nslots == 2 && n_prb[0] != n_prb[1]`), set
   `res->cfo_hz = NAN`. Rationale: the CFO is measured as the phase of
   `Σ p₀[k]·conj(p₁[k])`; under hopping that phase contains the *channel difference
   between two frequency blocks*, not a frequency offset, so any value reported would be
   garbage. Downstream code already handles NAN CFO (single-slot SRS does the same).
3. Document why the dormant `DO_LINEAR_INTERPOLATION` block must stay disabled: its
   `cesymb()` macro indexes the grid by `n_prb[0]` only, so enabling it under hopping
   would read/write the wrong REs. The per-slot copy fallback is hopping-correct.

Enable the `-H` hopping ctest cases from Commit 1 (they fail before this fix).

### Commit 3 — `chest_ul`: exact noise-bias correction for the smoothing residual

**Files:** `lib/src/phy/ch_estimation/chest_common.{h,c}`,
`lib/include/srsran/phy/ch_estimation/chest_ul.h`, `chest_ul.c`

**The math.** Let the smoother's output *i* be a linear combination of inputs,
`ŷᵢ = Σⱼ wᵢⱼ·pⱼ`. For a channel that is locally smooth (removed by the filter) and white
noise of power N0 on the pilots, the expected residual power of output *i* is
`E{|pᵢ − ŷᵢ|²} = N0 · Σⱼ (wᵢⱼ − δᵢⱼ)²`. Averaging over all `nrefs` outputs gives an exact
**bias factor** `B = (1/nrefs)·Σᵢ Σⱼ (wᵢⱼ − δᵢⱼ)²` such that `N̂0 = P_residual / B` is
unbiased — for **any** filter, **any** allocation width, and **either** band-edge
convention. The old polynomial is a curve fit of exactly this quantity for one filter on
wide allocations; we replace the fit with the closed form.

**Implementation** — new `srsran_chest_estimate_noise_bias(filter, filter_len, nrefs,
extrapolate_edges)` in `chest_common.c`:

* Interior outputs (all but `h = filter_len/2` at each edge) share the weight vector
  `filter − unit impulse`; their contribution is `(nrefs − 2h)·‖filter − δ‖²`.
* The `h` outputs at **each** band edge get their effective weights built explicitly.
  For `extrapolate_edges=true` this models `srsran_conv_same_cf()`'s linear
  extrapolation: a virtual sample `t` positions before the band maps to
  `(2+h−t)·input[1] − (1+h−t)·input[0]` (and mirrored at the tail — note the two edges
  of `conv_same_cf` are *not* mirror images of each other, so both are coded
  separately). For `extrapolate_edges=false` it models truncate-and-renormalize
  (needed by Commit 4).
* Accumulate in `double`, return `float`. Degenerate cases (`nrefs ≤ 2h`, zero-length
  filter) fall back to the interior value / 1.0.

**Wiring into `chest_ul.c`** — in `estimate_noise_pilots()`, replace the polynomial with
`power / q->noise_bias`, where `q->noise_bias` is cached in `srsran_chest_ul_t`
(new fields `noise_bias`, `noise_bias_filter_len`, `noise_bias_nrefs`) and recomputed
only when the filter length or allocation width changes — the O(M²·h) computation runs
once per reconfiguration, not per subframe.

**Result / tests:** noise estimate within 0.1 dB for 1/2/4/6-PRB grants (was up to
~1 dB). Tighten the `-E` thresholds of the Commit-1 ctest cases to 0.5 dB. PUCCH noise
estimation (`estimate_noise_pilots_pucch`) is intentionally untouched.

### Commit 4 — `chest_ul`: adaptive PUSCH smoothing with truncated band edges

**Files:** `chest_common.{h,c}`, `chest_ul.h`, `chest_ul.c`

Two cooperating pieces:

**(a) Truncated-edge smoother** — new `srsran_chest_smooth_pilots_trunc()` in
`chest_common.c`. Same interior behavior as `srsran_conv_same_cf`, but at the band edges
it **drops the taps that fall outside the allocation and renormalizes the remainder**
(`Σ taps = 1`). Properties: edge outputs stay unbiased for a flat channel, and their
noise gain is always ≤ the interior's (a renormalized average of fewer samples), versus
1.89× amplification from linear extrapolation. This matters most exactly where this plan
aims: on a 1-PRB grant the edges are 2 of 12 outputs.

**(b) Per-grant filter selection** — new `pusch_select_filter()` in `chest_ul.c`, driven
by a cheap SNR pre-estimate that must not depend on the smoothing filter itself:

* **SNR prior:** second differences of the LS pilots, `d[k] = p[k−1] − 2p[k] + p[k+1]`,
  cancel any locally-linear channel exactly; for white noise
  `E{|d|²} = 6·N0`. So `N̂0_raw = avg_power(d)/6` (averaged over both slots) and
  `snr_prior = EPRE / N̂0_raw`. The residual channel-curvature term is negligible at
  15 kHz subcarrier spacing. Computed with existing vector primitives
  (`srsran_vec_sum_ccc`/`sub_ccc`/`avg_power_cf`) using `q->tmp_noise` as scratch.
* **Tiers** (linear power thresholds `PUSCH_SMOOTH_SNR_HIGH = 20` ≈ 13 dB,
  `PUSCH_SMOOTH_SNR_LOW = 3.16` ≈ 5 dB):
  * ≥ 13 dB → keep the legacy 3-tap taps (16QAM+ region; preserve selectivity; only the
    edge handling changes).
  * 5–13 dB → 5-tap Gaussian (`srsran_chest_set_smooth_filter_gauss`).
  * < 5 dB → Gaussian scaled with allocation width, `len = (nrefs/3)|1` clamped to
    [5, 9]. The 9-tap cap spans 135 kHz — still below the coherence bandwidth of even
    long-delay-spread channels, so the low-SNR tier cannot destroy a selective channel.
* The result lives in **separate state** `q->pusch_filter[] / pusch_filter_len /
  pusch_snr_prior` so the shared `q->smooth_filter` used by PUCCH and SRS is never
  touched. Gate everything behind a new `q->pusch_adaptive_smoothing` flag (default
  **on**; public setter arrives with Commit 5).

**Plumbing:** thread `(filter, filter_len, trunc_edges)` parameters through
`chest_ul_estimate()`, `average_pilots()` and `estimate_noise_pilots()`. The noise-bias
cache from Commit 3 gains `noise_bias_trunc` and `noise_bias_tap0` keys and calls
`srsran_chest_estimate_noise_bias(..., extrapolate_edges = !trunc_edges)`, so the noise
estimate stays unbiased (within 0.06 dB measured) for **every** tier — this is why
Commit 3 had to model both edge conventions. SRS (`srsran_chest_ul_estimate_srs`)
explicitly passes the legacy `smooth_filter` with `trunc_edges=false`.

**Result / tests:** flat-channel CE NMSE at 0 dB: 1 PRB 0.58→0.28, 2 PRB 0.46→0.15,
4 PRB 0.40→0.15; 1 PRB @ 20 dB 0.58→0.37. The frequency-selective 30 dB guard case is
unchanged (0.1440 vs 0.1440 MSE). Tighten the `-M` thresholds accordingly.

### Commit 5 — `chest_ul`: phase-aligned cross-slot DMRS averaging

**Files:** `chest_ul.h`, `chest_ul.c`

**Idea.** When the channel is time-flat across the subframe, the two per-slot smoothed
estimates `h₀, h₁` are two independent noisy views of the same channel; averaging halves
the noise on **every** symbol (~3 dB more CE quality, stacking with Commit 4).

**Phase alignment.** Small residual CFO rotates slot 1 relative to slot 0 by the
cross-slot phase φ (already measured for the CFO estimate as
`carg(Σ p₀·conj(p₁))`). Combine as:

```
c0 = (h0 + h1·e^{+jφ}) / 2        # slot-1 view aligned onto slot 0
c1 = c0 · e^{−jφ}                  # aligned back so slot 1 keeps its own mean phase
```

then hold `c0` across slot 0's symbols and `c1` across slot 1's. Because each slot keeps
its own mean phase, the combining is exact under small CFO rather than merely tolerant
of it.

**Gating — combine only when it is a win.** All three must hold:

1. **No hopping:** `nslots == 2 && n_prb[0] == n_prb[1]` (under hopping the slots see
   different channels; also reuses Commit 2's `hopping` flag).
2. **Low SNR:** `pusch_snr_prior < PUSCH_SMOOTH_SNR_HIGH` (~13 dB). Above it, per-slot
   estimates are already accurate and per-slot time tracking is worth more than further
   noise reduction.
3. **Time-flat channel:** `|φ| < gate`. The subtlety is that φ itself is noisy: its
   standard deviation is ≈ `1/sqrt(nrefs·snr)`. A fixed gate tight enough to reject
   Doppler would also reject *time-flat* subframes at low SNR purely through the gate's
   own measurement noise — the exact regime this feature targets. So the gate is
   `clamp(2.5·(1/sqrt(nrefs·snr_prior)), 0.2 rad, 1.0 rad)`
   (constants `PUSCH_CROSS_SLOT_MAX_PHASE_RAD`, `PUSCH_CROSS_SLOT_PHASE_NSTD`,
   `PUSCH_CROSS_SLOT_PHASE_CAP_RAD`): floor 0.2 rad ≈ tolerates ≤ ~64 Hz residual CFO,
   cap 1.0 rad keeps rejecting genuinely time-varying channels. Since the combiner
   *aligns* with the measured phase, the gate only needs to reject genuine channel
   change, not the measurement noise itself.

If any condition fails, fall through to the unchanged `interpolate_pilots()` per-slot
hold — the feature is strictly additive. Restructure `chest_ul_estimate()` so
`estimate_noise_pilots()` runs **before** any cross-slot processing (the residual must
match the modeled per-slot filter bias), and the combine/hold choice happens in the
single `write_estimates` block. Uses `q->tmp_noise` as scratch (free at that point) —
no new allocation.

**Public API:** `srsran_chest_ul_set_pusch_opts(q, adaptive_smoothing, cross_slot_avg)`
toggles both PUSCH improvements; both default **on** in `srsran_chest_ul_init`.
`pusch_select_filter()` now also runs when only cross-slot averaging is enabled (the
gate needs `pusch_snr_prior`). SRS passes `cross_slot_max_phase = 0` (disabled; it is
single-slot anyway).

**Result / tests:** flat channel at 0 dB: 1 PRB 0.28→0.19, 2 PRB 0.15→0.10,
4 PRB 0.15→0.09. With 50 Hz CFO: 0.20 (combining still engaged and exact). With 300 Hz
CFO the gate correctly falls back to per-slot estimates. The frequency-selective 30 dB
case is **bit-identical** (its ~1 rad cross-slot phase is rejected by the cap). Tighten
`-M` thresholds again.

### Commit 6 — `test,pusch`: end-to-end decoding through the real UL estimator

**Files:** `lib/src/phy/phch/test/pusch_test.c`,
`lib/src/phy/phch/test/CMakeLists.txt`

Close the loop: prove the estimator improvements survive the full transmit → channel →
estimate → equalize → decode chain, and pin the hopping fix at system level.

1. New extensive parameter `-p use_chest`: the test inserts PUSCH DMRS exactly as
   `ue_ul.c` does (`srsran_refsignal_dmrs_pusch_gen` + `_put`), then decodes with the
   output of a real `srsran_chest_ul_t` (`srsran_chest_ul_estimate_pusch`) instead of
   `srsran_chest_ul_res_set_identity`.
2. Apply a **smooth frequency-selective channel** to the whole subframe — unit average
   power, amplitude ramp 0.8→1.2, phase spanning a full turn across the band:
   `h(k) = (0.8 + 0.4·k/K)·e^{j(0.7 + 2πk/K)}`. Because the phase rotates a full turn,
   decoding *fails* unless the estimates are measured at the **right subcarriers** —
   this is what makes the test sensitive to the Commit-2 indexing fix.
3. `-p snr_db <val>` adds AWGN via `srsran_ch_awgn_c` (PUSCH/DMRS REs have unit power,
   so `n0 = 10^{−snr/10}` directly).
4. `-F 3` now runs the **real** `srsran_ra_ul_pusch_hopping()` computation with type-2
   intra-subframe hopping — the case where `n_prb_tilde ≠ n_prb`. (Type-1 hopping is
   covered through the existing `-F 0` path.) Validate the hopped allocation fits the
   cell before using it. Also fix the missing `R` in the `getopt` string so the
   documented `-R <riv>` option actually parses.
5. New ctest cases: 1/2/4-PRB QPSK grants noiseless and at 3–5 dB SNR, plus type-1 and
   type-2 hopping cases. `pusch_test_chest_hop_type2` **fails against the pre-Commit-2
   estimator** — it is the end-to-end proof of the hopping fix.

---

## 3. Design invariants (apply to every commit)

* **PUCCH and SRS bit-exactness.** All new behavior is keyed off PUSCH-only state
  (`pusch_filter`, `pusch_adaptive_smoothing`, `pusch_cross_slot_avg`) or PUSCH-only
  call sites. `estimate_noise_pilots_pucch` and `srsran_chest_ul_estimate_srs` keep the
  legacy filter, legacy edges, and no cross-slot combining.
* **Non-hopping bit-exactness of the hopping fix.** `n_prb_tilde == n_prb` when hopping
  is off, so Commit 2 changes nothing for the common case.
* **Every heuristic has an off-switch.** `srsran_chest_ul_set_pusch_opts()` restores the
  legacy estimator (modulo the hopping and noise-bias corrections, which are pure
  fixes).
* **Every improvement is measured before and after**, and the measurement becomes a
  ctest threshold so regressions are caught. Over-smoothing is guarded by the dedicated
  frequency-selective high-SNR case.
* **No per-subframe cost blowups.** The noise-bias factor is cached; filter selection is
  O(nrefs) vector ops; cross-slot combining is four vector ops on scratch memory.

## 4. Cumulative expected results

| Metric (flat channel) | Before | After | Commits |
|---|---|---|---|
| CE NMSE, 1 PRB @ 0 dB | 0.58·N0 | 0.19·N0 | 4, 5 |
| CE NMSE, 2 PRB @ 0 dB | 0.46·N0 | 0.10·N0 | 4, 5 |
| CE NMSE, 4 PRB @ 0 dB | 0.40·N0 | 0.09·N0 | 4, 5 |
| Noise-estimate bias, 1 PRB | +0.97 dB | < 0.1 dB | 3 |
| Noise-estimate bias, 2 PRB | +0.45 dB | < 0.1 dB | 3 |
| Freq-selective 30 dB MSE | 0.1440 | 0.1440 (bit-identical) | guard |
| Hopped-grant decoding | corrupted | correct | 2, 6 |

A CE NMSE drop from 0.58·N0 to 0.19·N0 is ≈4.8 dB less estimation noise; at the SNRs
where 1-PRB QPSK grants live, post-equalizer effective SNR is estimation-limited, so
this translates directly into decoding margin (verified end-to-end by the 3–5 dB SNR
`pusch_test` cases).

## 5. Verification plan

```sh
cmake -B build && make -C build -j chest_test_ul pusch_test
# Unit-level estimator quality + hopping:
ctest --test-dir build -R "chest_test_ul_quality"
# End-to-end decode through the real estimator:
ctest --test-dir build -R "pusch_test_chest"
# Full regression (PUCCH/SRS/DL estimation untouched):
ctest --test-dir build -R "chest"
```

Negative controls: reverting the Commit-2 indexing fix must fail
`pusch_test_chest_hop_type2` and the `-H` chest_test_ul cases; disabling both options
via `srsran_chest_ul_set_pusch_opts(q, false, false)` must reproduce legacy CE output on
non-hopped grants.

---

# Follow-up enhancements

A second series of commits adds three more PUSCH-only improvements. The options moved to
a struct — `srsran_chest_ul_set_pusch_opts(q, &(srsran_chest_ul_pusch_opts_t){...})` —
with five flags, all default-on: `adaptive_smoothing`, `cross_slot_avg` (the original
two), plus `ta_derotation`, `dft_denoise` and `time_interp` below. Internally the
per-call knobs of `chest_ul_estimate()` were collapsed into a `chest_ul_proc_t`
descriptor. PUCCH and SRS remain bit-exact.

## 7. Timing-offset (TA) de-rotation before smoothing — `ta_derotation`

A residual timing offset τ rotates the LS pilots by `e^(−j2π·15kHz·τ·k)` across
frequency. The smoothers average across that rotation (biasing the estimate low — at
τ = 4 µs the 1-PRB 20 dB CE degrades beyond raw LS, NMSE 1.13, and the noise estimate
reads +3.4 dB high) and the ramp inflates the second-difference SNR prior. The slope is
measured with the same estimator the TA measurement uses — it *is* the TA — the pilots
are flattened before any frequency-domain processing, and the ramp is re-applied to the
smoothed rows after the noise residual is taken. Unitary, so noise statistics and the
modeled filter bias are unchanged; the TA measurement is taken over from the flattened
path for free. This removes the msg3-style regression risk (long low-SNR filters on
coarse PRACH-only timing). Measured at 4 µs / 0 dB / 4 PRB: CE 0.159 → 0.092·N0, noise
bias +0.43 → +0.05 dB; the `-t` stimulus cases also assert `ta_us` within 0.2 µs.

## 8. Delay-domain denoising for wide grants — `dft_denoise`

For grants ≥ 8 PRB (96 pilots), the LS estimates are projected onto the delay bins a
physical channel can occupy: unitary DFT → keep bins `|d| ≤ ceil(CP/2·bin rate) + 3` →
DFT back. The TA de-rotation centers the mean delay at bin 0, which is what makes the
symmetric half-CP window sufficient. Any within-CP channel passes undistorted at every
SNR, so unlike the FIR tiers there is no selectivity penalty to protect against; the
noise gain is the kept-bin fraction (~0.07–0.13). The projection engages only when that
fraction beats the FIR's squared tap norm, so narrow/flat cases keep the FIR. The noise
estimate divides the residual by the exact projection fraction `(nrefs−K)/nrefs` — an
orthogonal projection has no edge effects. Measured: 25 PRB @ 10 dB CE 0.124 → 0.072·N0;
noiseless case exact; stacks with de-rotation (0.073 with a 3 µs offset).

## 9. Phase-corrected time interpolation at high SNR — `time_interp`

The time processing gains a middle branch between cross-slot averaging (low SNR,
time-flat) and the per-slot hold: at `snr_prior ≥ ~13 dB` and cross-slot phase
`|φ| ≤ 1 rad`, every symbol is written as
`h(t) = [(1−t)·h0 + t·h1·e^(+jφ)]·e^(−jφt)` with `t = (l−L1)/(L2−L1)` — linear
amplitude interpolation with the phase applied as a linear ramp in time, exact for a
pure-CFO channel (plain complex interpolation shrinks the magnitude between pilots).
The ≤ 2.23× noise amplification on the extrapolated edge symbols is cheap at high SNR.
Measured at 4 PRB / 25 dB with 200 Hz CFO: per-slot hold NMSE 10.5 (phase-error
dominated) → 0.396, identical to the zero-CFO case; the frequency-selective guards
also halve (the stimulus varies linearly in time), thresholds tightened accordingly.

## Decision tree after all nine changes

```
srsran_chest_ul_estimate_pusch()
  ├─ measure slope → flatten pilots (7)           [ta_derotation]
  ├─ pusch_select_filter(): SNR prior on clean pilots → FIR tier
  ├─ wide grant & kept-fraction < Σw²? → delay-domain projection (8)   [dft_denoise]
  ├─ average_pilots → estimate_noise_pilots (exact bias, FIR or projection)
  ├─ re-apply slope to the smoothed rows (7)
  └─ time processing:
       snr < 13 dB & |φ| in gate  → cross-slot average (5)
       snr ≥ 13 dB & |φ| ≤ 1 rad  → phase-corrected interpolation (9)  [time_interp]
       otherwise / hopping / SRS  → per-slot hold
```
