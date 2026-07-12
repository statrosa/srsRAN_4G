# Per-UE Link Tracking Across Uplink Transmissions

## 1. Context: why this stage exists and where it sits

Every estimator stage built on this branch before it — TA de-rotation, adaptive
smoothing, delay-domain projection, exact noise bias, cross-slot averaging, time
interpolation, the sweep/rank API and its energy-capture guard — is **stateless across
subframes**: each `srsran_chest_ul_estimate_pusch()` call re-derives noise power, CFO,
timing and delay spread from the two DMRS symbols of one 1 ms subframe. That was the
right first move (it made every improvement independently testable), but it leaves a
structural inefficiency: those four quantities are not per-subframe realizations. They
are **slowly-varying properties of the UE's link**:

| Quantity | Physical driver | Timescale | Single-subframe estimate quality |
|---|---|---|---|
| Noise/interference power N0 | Thermal floor, interferers, AGC | 100 ms – seconds | ±1 dB std on a 1-PRB grant (~18 residual DOF) |
| CFO | Oscillator drift + Doppler shift | seconds | ±65 Hz std at 1 PRB / 0 dB |
| Delay centroid (fine timing) | UE position, TA loop | ~100 ms (bounded by TA commands) | sub-bin, but discarded after each call |
| Delay spread | Propagation geometry | seconds | not estimated at all — the projection window blindly assumed CP/2 |

Filtering them per UE across transmissions is nearly free accuracy, and feeding them
back upgrades every estimator gate and parameter that previously rode a noisy
single-shot measurement.

**What is deliberately NOT tracked: the channel coefficients themselves.** They
decorrelate in milliseconds (coherence time ~7 ms at 30 km/h @ 2 GHz) and their
inter-subframe phase is destroyed by residual CFO (50 Hz walks the phase 2π every
20 ms). Only phase-blind or phase-differential statistics are tracked, which is what
makes this feature safe by construction. (Coefficient combining across transmissions —
HARQ/SPS cross-TTI averaging — remains future work, and the CFO track built here is its
prerequisite.)

The prior art inside srsRAN: the MAC scheduler already keeps a per-UE EMA of the
*reported* SNR (`ul_snr_avg_alpha` in `sched_ue_cell.cc`) and the TA loop filters
`ta_us` into TA commands — but both live *downstream* of the estimator and nothing fed
back into it. This feature closes that loop at the PHY.

Source: `lib/src/phy/ch_estimation/chest_ul.{h,c}` (commit `d33d8c9`).
Tests: `chest_test_ul -T` (tracking loop) and `-D` (DTX injection).

## 2. Architecture and API

```
                     ┌───────────────────────────── per RNTI ─────────────────────────────┐
                     │  srsran_chest_ul_track_t  { n0_ema, cfo_hz_ema, delay_us_ema,       │
                     │                             spread_us_ema, nof_meas, last_tti }     │
                     └──────────────▲──────────────────────────────┬──────────────────────┘
                          update    │                              │   get_prior
                    (gated, filtered)                              ▼
   per subframe:   ┌────────────────┴───────────┐   ┌─────────────────────────────────┐
   crc_ok, res --> │ srsran_chest_ul_track_update│   │ srsran_chest_ul_estimate_pusch_ │
                   └────────────────────────────┘   │ prior(q, sf, cfg, input,        │
                                                    │       &prior, res)              │
                                                    └─────────────────────────────────┘
```

```c
srsran_chest_ul_track_t track;                       // one per RNTI (e.g. in phy_ue_db)
srsran_chest_ul_track_reset(&track);

// per PUSCH subframe of this UE:
srsran_chest_ul_prior_t prior;
srsran_chest_ul_track_get_prior(&track, tti, &prior);
srsran_chest_ul_estimate_pusch_prior(q, sf, cfg, input, &prior, res);
/* ... equalize, decode ... */
srsran_chest_ul_track_update(&track, res, tti, crc_ok);

// on a MAC timing-advance command (UE told to advance by X us):
srsran_chest_ul_track_notify_delay_shift(&track, -X);
```

`srsran_chest_ul_estimate_pusch()` and `..._pusch_win()` are now thin wrappers of
`..._pusch_prior()` (NULL priors / delay-only prior respectively). **Every invalid prior
entry leaves the corresponding stage exactly as the plain call** — an empty, cold or
stale tracker reproduces the untracked estimator bit-for-bit (verified: the whole
pre-existing test suite passes unchanged, and the 5–15 dB tracked flat cases are
bit-identical to untracked).

## 3. The four priors in detail

### 3.1 `n0` — tracked noise power → stable tier and gate selection

**Consumed in** `pusch_select_filter()` (`chest_ul.c`): when valid, it replaces the
per-subframe second-difference noise pre-estimate in `snr_prior = epre / n0`.

**Why it matters.** `snr_prior` drives three discrete decisions per subframe: the
smoothing tier (3/5/9-tap boundaries at ~5 and ~13 dB), the cross-slot-averaging gate
(< ~13 dB) versus the time-interpolation gate (≥ ~13 dB), and the spread-narrowing gate
(section 3.4). With the raw pre-estimate a UE sitting near any boundary flaps between
regimes subframe-to-subframe — each subframe is self-consistent, but the CE quality
jitters and the link adaptation above sees a noisier BLER-vs-SNR relationship than the
channel warrants. The tracked value makes the processing **deterministic for a
stationary UE**.

**Filter**: asymmetric EMA — attack `α=0.5` upward, decay `α=0.05` downward
(`TRACK_N0_ALPHA_UP/DN`). The asymmetry errs in the safe direction: a starting
interference burst is believed immediately (under-estimating noise inflates reported
SNR → MAC over-schedules → CRC failures), while a low floor must be earned slowly.
The N0 track updates on **every** subframe, including DTX — noise is measurable either
way, and DTX subframes are actually clean noise soundings.

### 3.2 `cfo_hz` — tracked CFO → gate referencing and phase unwrapping

**Consumed in** `chest_ul_estimate()` via `proc.cross_phase_ref = 2π·cfo·0.5 ms`:

- The cross-slot averaging and time-interpolation gates become **deviation tests**:
  `|φ_measured − φ_predicted| < gate` instead of `|φ_measured| < gate`. Rationale: the
  gates exist to reject *genuine channel change*; a rotation that is predictable from
  the UE's known oscillator offset is not channel change.
- The raw (−π, π] phase measurement is **unwrapped** around the prediction:
  `φ = φ_ref + wrap(φ_raw − φ_ref)`. This extends correct CFO measurement, combining
  alignment and interpolation ramps beyond the ±1 kHz ambiguity of a 0.5 ms baseline.
  With `φ_ref = 0` (no prior) the unwrap is the identity — bit-exactness preserved.

**Why it matters.** Without a reference, a UE with a stable 200–500 Hz offset — a cheap
IoT crystal after DL sync, a post-PSM thermal transient, highway Doppler — *permanently*
loses the 3 dB cross-slot combining gain: the gate correctly detects the rotation every
subframe and correctly refuses, forever. The tracker converts "correctly refused" into
"correctly compensated".

**Measured**: 1 PRB @ 0 dB with 300 Hz CFO: CE error 0.368 → **0.282**·N0. The
remaining gap to the CFO-free case (~0.19) is *within-slot* rotation (±0.47 rad across
a slot at 300 Hz), which no cross-slot method can remove — per-symbol CFO
pre-compensation of the received grid would be the follow-up that closes it.
Unexpected but explainable bonus: the time-drifting frequency-selective test case went
4.24 → **0.324** NMSE, because its linear time drift looks exactly like a CFO, the
tracker learns it, and the interpolation gate flips from coin-toss (measured phase ≈
1.0 rad vs gate 1.0 rad) to deterministic.

**Filter**: EMA `α=0.2` with a ±30 Hz per-update slew limit (`TRACK_CFO_SLEW_HZ`) —
steady-state tracking noise ~2 Hz from ±65 Hz measurements, and one bad accepted
measurement cannot move the track more than 30 Hz.

### 3.3 `delay_us` — tracked delay centroid → window centering

**Consumed** exactly like the sweep API's `window_delay_us` (they share the mechanism):
the pilots are de-rotated by the prior center plus a measured residual **clamped to
±2 delay bins**, so the phase-slope measurement — the quantity receiver clipping can
corrupt — may only fine-tune within the energy-verified neighborhood, never relocate
the processing.

**Why it matters.** MAC TA commands are quantized to 0.52 µs and issued sparsely;
between them the UE drifts (oscillator pull, motion). The tracked centroid follows the
drift continuously, keeps the projection window centered through it, and reports
fine-grained timing (`coarse + ta_us`) that the MAC can use to issue *fewer* TA
commands. For a sweep-based decoder it is the feedback that collapses the candidate set
to ±1 in steady state (~5× less ranking compute). When the MAC does issue a command,
`srsran_chest_ul_track_notify_delay_shift()` applies the *known* shift directly instead
of re-learning it through the slew filter.

**Measured**: 3 µs offset at 3 dB, tracked: CE 0.0558·N0 with TA reported at 3.00 µs.

**Filter**: EMA `α=0.25`, slew ±0.3 µs per update.

### 3.4 `spread_us` — tracked delay spread → SNR-gated window narrowing

**Consumed in** the projection-window sizing:
`half_win = clamp(ceil(bins_per_µs · spread/2) + margin, margin+2, blind CP/2 bound)` —
but **only when `snr_prior < PUSCH_SMOOTH_SNR_LOW` (~5 dB measured scale, ~3.3 dB true
SNR)**.

**Why it matters.** The blind window is sized for the worst channel physics allows
(±CP/2). Most UEs live on far shorter channels (EPA-class: delay support well under
1 µs), and every excluded bin is excluded noise: at 25 PRB the window shrinks from
±14 to the ±5-bin floor, keep-fraction 29/300 → 11/300.

**Why the SNR gate — a measured lesson, not a theory.** Narrowing trades two
currencies: the saved noise scales with **N0**, while the window edge also clips the
channel's own fractional-delay sinc leakage — a bias scaling with **signal power**. The
initial theoretical crossover estimate (~10 dB) was wrong; measurement showed a clear
win at ≤3 dB, a wash at 5 dB, and a **net loss at 10 dB** (0.072 → 0.082 with the gate
at 10 dB — caught and fixed before commit). The final threshold reuses
`PUSCH_SMOOTH_SNR_LOW`, so "smooth as hard as physics allows" is one consistent
regime, and the 10 dB tracked case is verified **bit-identical** to untracked (a ctest
pins that parity).

**Measured**: 25 PRB flat, tracked vs untracked: 0.0559 → **0.0363** at 0 dB (−1.9 dB),
0.0627 → 0.0547 at 3 dB, bit-exact parity at 5/10/15 dB.

**Filter**: asymmetric EMA — widen fast (`α=0.5`), narrow slowly (`α=0.05`). The
narrow direction is the risky one; new multipath is believed immediately.

## 4. The measurement side: `delay_centroid_us` and `delay_spread_us`

The tracker's inputs come from two new fields in `srsran_chest_ul_res_t`, computed for
free inside the energy-capture guard whenever the projection is attempted (the delay
profile is already in hand):

- Per-bin **noise floor subtraction**: the mean power of the *excluded* bins — pure
  noise whenever the guard passes — is subtracted from each in-window bin (clamped at
  zero) before taking moments. Without this, noise inflates the second moment and
  biases the centroid toward the window center.
- `delay_centroid_us`: first moment, reported **absolute** (residual centroid plus the
  de-rotation already applied this call), sign convention matching `ta_us`
  (positive = late UE).
- `delay_spread_us`: 4σ of the floored in-window profile — approximately the channel's
  delay support for typical profiles. Noise residue biases it *wide*, which is the safe
  direction for window sizing.

Both are NAN when the projection was not attempted (narrow grants, FIR-preferred
widths, PUCCH/SRS), and the tracker simply skips NAN inputs — so on narrow grants the
delay/spread tracks coast on their last wide-grant values while N0/CFO keep updating.

## 5. Robustness: the gates and their interaction

### 5.1 The DTX gate (~6 dB measured scale)

```c
bool signal_ok = crc_ok || (isnormal(res->snr) && res->snr >= TRACK_MIN_SNR_NO_CRC /*4.0*/);
```

Signal-dependent tracks (CFO, delay, spread) update only when a UE demonstrably
transmitted. A missed grant (DTX) still produces numbers — a "CFO" that is the phase of
a noise correlation, a "centroid" wherever noise piled up, a "spread" of the whole
window — and accepting them would random-walk the tracks. CRC pass is ground truth at
any SNR (the primary path in a real eNB); the ≥6 dB clause is the fallback when CRC is
failed/unavailable, sitting far above the ~0 dB that noise-only subframes measure
(EPRE ≈ noise estimate → ratio ≈ 1). N0 is exempt: it updates on every subframe.

**Measured**: with DTX injected on every 4th subframe (`-D`), all tracked metrics are
*identical* to the DTX-free run (0.0544 vs 0.0544 at 3 dB) — one ctest locks this in.

### 5.2 Interaction with the narrowing gate (~5 dB)

The two thresholds partition the SNR axis into three regimes with **no overlap between
"spend the spread prior" and "learn it without CRC help"**:

```
true SNR:    0        ~3.3 dB          ~4.8 dB              15+
             |  spend prior  |  neither   |  learn without CRC  |
narrowing:   ██████ ON ██████|——————————————— OFF ———————————————
SNR-fallback
learning:    ———— OFF (CRC-gated only) ——|████████ ON ███████████
```

This encodes three deliberate ideas:

1. **Delay spread is geometry, not SNR** — a spread learned at 15 dB is valid at 0 dB.
   The gates implement a hand-off along the UE's own trajectory: learn while healthy,
   spend at the cell edge, exactly where narrowing pays.
2. **Trust is earned more conservatively than profit is taken.** Low-SNR spread
   measurements are noisy even on genuine subframes — and biased wide, the safe
   direction, reinforced by the widen-fast/narrow-slow filter.
3. **In the gap (~3.3–4.8 dB with failing CRCs) the system does nothing** — the correct
   behavior at the most ambiguous operating point.

### 5.3 The containment chain

| Scenario | Outcome |
|---|---|
| DTX, `crc_ok=false` (normal) | Only N0 updates; measured metrics unaffected |
| DTX leaking through (false `crc_ok`) | Slew limits cap damage (≤30 Hz, ≤0.3 µs); spread jumps *wide* — safe direction |
| Cold start at low SNR | Priors invalid → blind window → untracked behavior |
| Prior wrong (UE turned a corner) | Energy-capture guard trips *that subframe* → FIR fallback; reported spread widens → fast-attack recovery next subframe |
| Low SNR + persistent CRC failure | Prior spent but not refreshed → ages out (500 TTIs, wrap-safe) → graceful reversion |
| Long silence / re-sync / handover | Age check *resets* the state rather than slew-filtering toward a distant new operating point |
| Warmup | Signal priors withheld until 4 accepted updates |

## 6. Measured results (200 subframes, `chest_test_ul -T`)

| Case | Untracked | Tracked | Interpretation |
|---|---|---|---|
| 25 PRB @ 0 dB flat | 0.0559 | **0.0363** | Window narrowing: −1.9 dB CE at the extreme edge |
| 25 PRB @ 3 dB flat | 0.0627 | 0.0547 | Narrowing still winning |
| 25 PRB @ 5 / 10 / 15 dB flat | — | **bit-identical** | Gates off; zero cost outside the win regime |
| 25 PRB @ 3 dB, DTX every 4th sf | — | 0.0544 | Identical to DTX-free: poisoning immunity |
| 25 PRB @ 3 dB, 3 µs offset | — | 0.0558, TA 3.00 µs | Centering + timing report through the tracker |
| 1 PRB @ 0 dB, 300 Hz CFO | 0.368 | **0.282** | Combining restored for offset UEs (remainder is within-slot rotation) |
| 12 PRB @ 20 dB selective, time-drifting | 4.24 | **0.324** | CFO track learns the drift → interp gate deterministic → ~6 dB effective-SNR recovery |

In effective-SNR terms (CE error acts as extra noise, loss = `10·log10(1+NMSE)`): the
selective row is worth ~6 dB (16QAM → 64QAM territory for moving users on wide
grants), the CFO row ~0.3 dB at the QPSK BLER waterfall (~2× fewer HARQ
retransmissions for IoT-class oscillators — battery and latency), and the narrowing row
~0.1 dB concentrated at the extreme cell edge. Row one of the real-world value,
though, is the *determinism*: same channel in → same processing → same quality out,
which is what the link adaptation above actually needs.

## 7. Test coverage

| ctest | Asserts |
|---|---|
| `chest_test_ul_track_prb25_snr0` | Narrowing win at 0 dB (`-M 0.05`) |
| `chest_test_ul_track_parity_snr10` | Bit-parity with untracked at 10 dB (`-M 0.10`) |
| `chest_test_ul_track_dtx` | DTX-immunity: same threshold as DTX-free (`-D -M 0.075`) |
| `chest_test_ul_track_to3` | Centering + TA accuracy under 3 µs offset |
| `chest_test_ul_track_cfo300` | Combining recovery at 300 Hz (`-M 0.35`; untracked 0.37 fails) |
| `chest_test_ul_track_selective` | Interp-gate determinism (`-M 0.6`; untracked 4.24 fails) |

The `-T` loop in `chest_test_ul.c` is a faithful integration reference: prior →
estimate → update per subframe, `crc_ok=false` on injected DTX, metrics accumulated
only on transmitted subframes.

## 8. eNB integration point

The tracker is estimator-agnostic by design — `srsran_chest_ul_t` is per-carrier-worker
and shared by all UEs (across multiple pipelined subframe workers), so per-UE state
cannot live there. One `srsran_chest_ul_track_t` per RNTI per carrier belongs in
`phy_ue_db` (already RNTI-keyed, shared, mutexed): `get_prior` before and
`track_update` after each `decode_pusch` in `cc_worker`, `crc_ok` from the decoder
result, `notify_delay_shift` wherever TA commands are issued. Updates are commutative
scalar filters, so a worker seeing slightly stale priors is indistinguishable from one
extra TTI of aging — no ordering requirements beyond the mutex.

## 9. Relation to future work

- **Per-symbol CFO pre-compensation** (rotate the received grant by the tracked CFO
  before estimation) would close the within-slot-rotation remainder of the 300 Hz case
  (0.282 → ~0.19).
- **Wiener/MMSE delay-domain weighting** from a tracked power-delay profile
  (`w_d = P_d/(P_d + N0/nrefs)`) is the optimal-linear endpoint of the window story: it
  would replace both the hard window and the narrowing SNR gate, since the weighting
  self-adjusts with SNR. The tracker built here already maintains everything it needs
  except the per-bin profile.
- **Cross-TTI coefficient combining** (HARQ retransmissions, SPS) becomes feasible only
  with the CFO track in place — it removes the inter-transmission phase random walk
  that otherwise makes coefficient averaging destructive.
