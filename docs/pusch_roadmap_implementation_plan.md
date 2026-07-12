# PUSCH Receiver Roadmap: Implementation Plan for the Remaining Enhancements

## 1. Context — where the receiver stands

This branch has taken the LTE uplink receiver through three layers of improvement, each
measured, gated, and regression-locked (196 ctests green):

**Channel estimation** (`lib/src/phy/ch_estimation/chest_ul.c`):
frequency-hopping indexing fix → exact noise-bias correction → SNR-tiered adaptive
smoothing with truncated band edges → TA de-rotation → delay-domain projection →
phase-aligned cross-slot averaging → phase-corrected time interpolation. CE error on
cell-edge grants went from 0.58·N0 (legacy) to 0.04–0.09·N0 — estimation stopped being
the bottleneck (≤ ~0.2 dB effective-SNR loss).

**Timing/hypothesis architecture** (`srsran_chest_ul_rank_pusch`,
`..._estimate_pusch_win`): a coarse/fine sweep API for window-sweeping decoders — rank
candidates from the pilot delay-power profile, centered-window estimate for the winner,
energy-capture guard converting the projection's one catastrophic failure mode into a
graceful FIR fallback. Offsets 5× beyond the blind range decode at full quality.

**Cross-transmission tracking** (`srsran_chest_ul_track_t`,
`..._estimate_pusch_prior`): per-UE gated EMAs of N0, CFO, delay centroid and delay
spread fed back as priors — DTX-immune, slew-limited, aging, warm-up. Recovered the
cross-slot combining gain for stable-CFO UEs (0.37→0.28 at 300 Hz), −1.9 dB CE at 0 dB
via window narrowing, made the moving-UE interpolation gate deterministic (4.24→0.32).

**FEC layer** (`sch.c`): CRC-aided flip list decoding of short code blocks — pinned
re-decodes of the least-reliable bits, CRC-selected. ~0.23 dB shift of the 10% BLER
point, BLER halved on the waterfall, zero undetected errors observed, off by default.

**Established engineering rules** (every item below follows them):
1. Measure before and after; thresholds become ctests with headroom; negative controls
   prove each test fails without its feature.
2. Every heuristic has an off-switch and a safe fallback; default-off for anything that
   costs compute on the success path.
3. Trust gates on inputs (DTX/CRC/SNR), profit gates on outputs, slew limits between.
4. PUCCH/SRS and knob-off paths stay bit-exact.
5. Unlimited-compute features convert the 24-bit CRC into a hypothesis oracle
   (false-accept ≤ attempts × 2⁻²⁴) — budget-bounded, deterministic schedules.

The remaining items, ordered by value-per-effort with dependencies:

| # | Item | Expected gain | Effort | Depends on |
|---|---|---|---|---|
| A | Wiener PDP delay-domain weighting | +0.5–1 dB CE at low SNR; retires the narrowing gate | S | tracker |
| B | Per-symbol CFO pre-compensation | 300 Hz case 0.28→~0.19; all moderate-CFO UEs | S | tracker |
| C | eNB integration (tracker + flip knob) | unlocks A/B/D/F in the real eNB | M | — |
| D | Iterative CE↔decode (virtual pilots) | +0.3–0.7 dB at the waterfall | M | — |
| E | Deeper flip schedules — **DONE** (pool 48, scored pairs, triples, cap 2048; OSD-2 remains open) | measured: BLER halved at 8–10 PRB, 2.7× at 16QAM-tier CB sizes | S | flip engine |
| F | Cross-TTI coefficient combining | +2–3 dB for SPS / non-adaptive HARQ retx | M | B, C |
| G | Successive interference cancellation | 3–10 dB where interference-limited | L | C |

---

## 2. Item A — Wiener/MMSE delay-domain weighting from a tracked PDP

**Motivation.** The projection window is binary (keep/discard) and its narrowing needed
an SNR gate because a hard edge clips the channel's own leakage (bias ∝ S) while saving
noise (∝ N0). The optimal linear estimator given the power-delay profile is per-bin
Wiener weighting `w_d = P_d / (P_d + N0)`: it self-adjusts with SNR (no gate needed),
passes strong bins untouched, attenuates weak ones proportionally, and strictly
dominates every hard window.

**Design.**
- **Tracked PDP**: extend `srsran_chest_ul_track_t` with `float pdp[SRSRAN_CHEST_PDP_NBINS]`
  (64 bins spanning ±CP/2 on a *fixed µs grid*, since `bins_per_µs = 0.015·nrefs`
  varies per grant width). Update: per accepted subframe (same DTX/CRC gate as spread),
  resample the floor-subtracted measured profile onto the µs grid, EMA with the spread
  filter's asymmetry (grow fast, shrink slow). The scalar `spread_us_ema` stays as the
  compatibility fallback.
- **Estimator**: in `average_pilots()`'s DFT branch, when a PDP prior is valid,
  resample `pdp[]` to the grant's bin grid and apply `delay[d] *= P_d/(P_d + n0_prior)`
  instead of zeroing outside the window. Fall back to the hard window when the PDP or
  n0 prior is invalid (bit-exact with today).
- **Noise estimation**: a diagonal weighting has residual noise fraction
  `Σ(1−w_d)²/nrefs` — but weak channel bins leak `(1−w_d)²·P_d` into the residual too.
  Robust choice: compute the noise estimate only over bins with `w_d < 0.1` (channel
  contribution negligible by construction), divide by their exact count fraction — the
  same closed-form philosophy as `srsran_chest_estimate_noise_bias`.
- **Guard**: the energy-capture guard remains, testing captured fraction under the
  effective window `w_d > 0.5` — same threshold semantics.

**Files**: `chest_ul.{h,c}` (tracker fields, prior struct `pdp_valid/pdp[]`, weighting
branch, noise-bin selection); `chest_test_ul.c` (add a two-path channel stimulus
`-P <delay_us>,<rel_dB>` so the PDP has real structure — the current flat/-S channels
cannot distinguish Wiener from a window).

**Tests**: two-path channel (e.g. 0/−3 dB at 1.5 µs) at 0/5/10 dB, tracked: assert
Wiener ≤ hard-window NMSE at every point (it should win at all three, removing the
5 dB gate's regime split); flat-channel parity vs the narrowing path at 0 dB; guard
trip still graceful with a wrong PDP (feed `-t` offset against a learned profile).

**Expected**: +0.5–1 dB CE at ≤5 dB on structured channels, parity or better
everywhere else; `PUSCH_SPREAD_NARROW_SNR_MAX` becomes dead code to delete.

---

## 3. Item B — per-symbol CFO pre-compensation

**Motivation.** The 300 Hz tracked case sits at 0.282·N0, not ~0.19, because the
channel rotates ±0.47 rad *within* each slot and every cross-slot method only fixes
inter-slot phase. The data symbols rotate identically — so if the CE carries the same
intra-slot ramp, the equalizer cancels it.

**Design.** With a valid CFO prior, after time processing (hold and combine paths
only — the interpolation path already models a linear ramp), rotate each CE symbol row
by `e^{j2π·f̂·(t_l − t_DMRS_slot)}`: 14 scalar-vector products using the *tracked* (not
per-subframe) CFO, clamped to the tracker's slew-bounded value. On the combine path the
rotation references each slot's own DMRS instant, preserving the existing
mean-phase-per-slot property.

**Files**: `chest_ul.c` (`chest_ul_estimate` write path; needs symbol-time table
`t_l` — derive from `SRSRAN_CP_NSYMB` and CP lengths); no API change (consumes the
existing `cfo_valid` prior).

**Tests**: tighten `chest_test_ul_track_cfo300` from `-M 0.35` toward ~0.22 (measure
first); add a 100 Hz mid-SNR case; verify the zero-CFO tracked cases stay bit-exact
(prior ≈ 0 → rotation ≈ identity — add an explicit epsilon shortcut so it *is* exact).

**Expected**: 0.282 → ~0.19–0.21 at 300 Hz/1 PRB/0 dB; proportional gains for all
moderate-CFO low-SNR UEs; prerequisite quality bar for item F.

---

## 4. Item C — eNB integration (tracker + flip budget)

**Motivation.** Everything above lives in `lib/` with a faithful test-harness
integration; the production eNB still calls the plain estimator and never sets the flip
budget. This item wires both.

**Design.**
- **Tracker home**: one `srsran_chest_ul_track_t` per RNTI per carrier in
  `srsenb/hdr/phy/phy_ue_db.h` (already RNTI-keyed, mutexed). API:
  `phy_ue_db::get_chest_prior(rnti, cc, tti, prior*)` and
  `::update_chest_track(rnti, cc, tti, res*, crc_ok)`.
- **Call sites**: `srsenb/src/phy/lte/cc_worker.cc::decode_pusch()` — fetch prior,
  call `srsran_chest_ul_estimate_pusch_prior`, and after the decode result is known,
  update with `crc_ok = pusch_res.crc`. Updates are commutative scalar filters, so
  pipelined workers with slightly stale priors are equivalent to one extra TTI of aging
  (no ordering requirement beyond the existing mutex).
- **TA hook**: where the MAC issues TA commands (`srsenb/src/stack/mac/*`), call
  through to `srsran_chest_ul_track_notify_delay_shift(-cmd_us)` — plumb via the
  existing `ta_info` path in reverse.
- **Config**: expert-section options `pusch_tracking_enable` (default true),
  `pusch_flip_attempts` (default 0), mapped into `ul_cfg.pusch.max_flip_attempts` and a
  tracker bypass. Mirror the `pusch_max_its` plumbing.
- **Reset triggers**: tracker reset on UE removal/re-establishment (phy_ue_db already
  observes both).

**Tests**: `srsenb/test/phy/enb_phy_test` smoke (compiles, runs, no behavior assert);
the real validation is the lib-level `-T` harness plus a manual ZMQ eNB↔UE run
(documented procedure, not CI). Risk note: this is the first item touching the eNB
runtime — land it behind the config flags and validate on hardware/ZMQ before
defaulting anything on.

---

## 5. Item D — iterative CE ↔ decode (virtual pilots)

**Motivation.** Two DMRS symbols out of 14 carry pilot energy. After even a *failed*
decode, the soft data symbols carry usable channel information; feeding them back as
virtual pilots multiplies pilot energy ~7× and lets the channel be estimated per
symbol, superseding the hold/combine/interp trilemma within the iteration.

**Design** (LTE SC-FDMA makes this delay-domain friendly):
1. First pass: standard estimate + equalize + decode (with flip budget if enabled).
2. On CRC failure: from the decoder's APP LLRs (already exported by
   `srsran_tdec_get_app`), re-encode the *systematic* soft estimates through the rate
   matcher's mapping to symbol positions, form soft symbols `E[s_k]` (tanh combination
   per QPSK/16QAM bit LLRs) with reliabilities `|E[s_k]|²`.
3. Virtual-pilot LS: `p̂_l[k] = Y_l[k]·E[s_{l,k}]*`, weighted by reliability; project
   each symbol's LS vector through the existing delay window/Wiener weighting; combine
   with the DMRS estimates (weights ∝ pilot vs virtual-pilot effective SNR).
4. Re-equalize, re-decode. 1–2 outer iterations, budget-gated
   (`nof_ce_iterations` in `srsran_pusch_cfg_t`, default 0).
**Placement**: orchestrated in `srsran_pusch_decode()` (`pusch.c`) — it owns the
symbol buffer, the chest result, and the decoder; the chest API needs one addition:
an entry point accepting external per-symbol pilot estimates
(`srsran_chest_ul_refine_pusch(q, virtual_pilots, weights, res)`).

**Robustness**: iterate only on CRC failure (success path untouched); reliability
weighting automatically discounts garbage feedback at very low SNR; hard cap on
iterations; all-zero reliability degenerates to pass-1 estimates (no regression
possible by construction — verify with a negative control).

**Tests**: waterfall shift measurement on the flip-decoding operating point (the two
compose: better LLRs in → better flip candidates); target ≥0.3 dB additional; assert
BLER at 0.5 dB ≤ measured bound; bit-exactness at `nof_ce_iterations=0`.

---

## 6. Item E — deeper flip schedules

**Motivation.** The measured 0.23 dB is bounded by the singles+pairs schedule; failures
with ≥3 bit errors survive. The engine's schedule generator is the extension point.

**Design** (incremental, all inside `decode_cb_flip`):
- Triples among the top-8 positions (56 candidates) appended after pairs.
- Adaptive ordering: score pairs by `|APP_i|+|APP_j|` ascending instead of index order.
- Optional research tier: order-2 OSD on the systematic bits for CBs ≤ 256 bits
  (Gaussian elimination over the most-reliable basis) — separate knob, documented as
  experimental.

**Tests**: re-measure the waterfall at budget 128/256; extend
`pusch_test_flip_stress` only if the gain is ≥0.05 BLER at the pinned point (avoid
enshrining noise); undetected-error guard already in place and becomes *more*
important — keep the content-match assert.

---

## 7. Item F — cross-TTI coefficient combining (HARQ/SPS)

**Motivation.** LTE UL non-adaptive HARQ retransmissions and SPS/VoLTE reuse the same
PRBs. At low mobility the channel coefficients are correlated across the 8 ms RTT —
the last unexploited energy source, worth up to 3 dB per combined transmission for
exactly the population that retransmits most.

**Design.**
- **Per-UE CE cache** (item C's `phy_ue_db` state): last smoothed per-slot CE (≤1200
  cf_t ≈ 9.6 KB/UE/carrier), its PRB range, TTI, and the tracked CFO at capture.
- **Gates** (all must hold): identical PRB allocation; age ≤ coherence bound from a
  tracked Doppler proxy (innovation variance of the CFO track); phase-align via
  `e^{j2π·f̂·Δt}` (needs item B's CFO quality) plus a measured residual-phase gate on
  the overlap — the cross-slot combiner's gate logic generalized to Δt = n·1 ms.
- **Combining**: prior CE as one extra observation with weight
  `w = ρ(age)/(1+ρ(age))` (ρ from the tracked coherence), applied before time
  processing; noise estimate unchanged (per-subframe residual, as with cross-slot).
- **Fallback**: any gate fails → current-only, bit-exact.

**Tests**: extend `-T` with a retransmission emulation (`-R <n>`: same channel
realization re-noised n times, 8 sf apart); assert combined CE ≈ single/√n; CFO-drift
and PRB-change negative controls must show zero combining.

**Expected**: 2–3 dB CE on retx #2+ at low mobility — compounding with HARQ IR at
exactly the SNRs where retransmissions happen.

---

## 8. Item G — successive interference cancellation

**Motivation.** Where the cell is interference-limited, everything above is fighting
the wrong noise. Decode-and-subtract converts decoded UEs' signals into cleaned
spectrum for the weak ones: 3–10 dB where it applies.

**Design sketch** (largest item, needs its own detailed plan when scheduled):
- Subframe-level orchestration in `srsran_enb_ul`/`cc_worker`: first pass decodes all
  scheduled UEs in descending post-equalizer SNR; for each CRC pass, reconstruct the
  transmit signal (re-encode → modulate → DFT-precode → map → apply that UE's
  smoothed CE) and subtract from the received grid; second pass re-estimates and
  re-decodes the failures against the cleaned grid; iterate to a fixed point
  (≤2 rounds).
- Reconstruction fidelity is the crux: it inherits every estimator improvement on this
  branch (CE error directly becomes residual interference). Rule of thumb: 0.05·N0 CE
  error → ≥13 dB cancellation depth.
- Robustness: subtract only CRC-passed UEs (reconstruction is then near-exact);
  bound rounds; per-UE opt-out for signals with failed reconstruction sanity checks
  (residual power at the subtracted UE's REs must *decrease*).
- Extends naturally to cross-cell UEs (uplink CoMP) where neighbor demod is possible.

**Tests**: multi-UE stimulus is the gap — `pusch_test` is single-UE. Needs a two-UE
overlap harness (MU-MIMO cyclic-shift pair or adjacent-PRB overlap with power
imbalance) before the feature; that harness is itself the first deliverable.

---

## 9. Cross-cutting engineering plan

- **Order**: A → B → C → D → E → F → G. A and B are small, self-contained, and
  immediately measurable in the existing harness; C unblocks production value for
  everything already built; D/E deepen the FEC layer; F needs B+C; G needs C plus a
  new harness.
- **Each item is one or two commits** in the branch convention: engine + measured
  tests/docs, negative controls stated in the commit message.
- **Regression floor**: the full 196-test suite must stay green after every commit;
  bit-exactness asserts for every knob-off path.
- **Honest reporting**: expected-gain numbers above are targets from analysis and
  literature; measured numbers replace them at commit time, including the misses (as
  with the flip decoder's 0.23 dB vs the literature's 0.4–0.9 dB, and the narrowing
  gate's measured 5 dB crossover vs the theoretical 10 dB).

## 10. Risk register

| Risk | Affected | Mitigation |
|---|---|---|
| PDP resampling bugs across grant widths | A | fixed-µs grid + noiseless exactness tests per width |
| CFO rotation sign/reference errors | B, F | epsilon-identity shortcut + bit-exact zero-CFO asserts |
| eNB threading/staleness | C, F | commutative updates, mutex-only contract, config-gated rollout |
| Soft-feedback divergence at very low SNR | D | reliability weighting, iteration cap, CRC-failure-only engagement |
| Undetected-error creep with larger lists | E | content-match assert in harness, budget documentation, 2⁻²⁴ accounting |
| Stale CE combining across channel changes | F | triple gate (PRB/age/phase), tracked-coherence bound |
| Reconstruction error re-injection | G | CRC-passed-only subtraction, residual-power sanity check per UE |
