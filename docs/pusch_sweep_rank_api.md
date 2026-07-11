# Coarse/Fine Timing Sweep: the Rank + Centered-Window API

This document describes the API added for decoders that sweep the receive FFT window
over coarse timing candidates (e.g. `[-512, +64]` samples in steps of 64 ≈ one CP at
10 MHz) and pick the alignment that decodes best — and how every existing estimator
stage slots into the resulting architecture.

Source: `lib/src/phy/ch_estimation/chest_ul.{h,c}`. Test coverage:
`chest_test_ul -W` (full sweep emulation) and `-w` (explicit center), see
`lib/src/phy/ch_estimation/test/CMakeLists.txt`.

## 1. The architecture

```
                 COARSE (caller)                          FINE (estimator)
 ┌─────────────────────────────────────────┐   ┌────────────────────────────────────┐
 │ for each window shift k (~CP steps):    │   │ winner only:                       │
 │   FFT the shifted window (DMRS symbols) │   │  srsran_chest_ul_estimate_pusch_win│
 │   srsran_chest_ul_rank_pusch()          │──▶│   (input, center = rank.delay_us)  │
 │     -> energy_frac, delay_us            │   │  -> CE, noise, SNR, ta_us, decode  │
 │ keep argmax(energy_frac)                │   │  MAC timing = k*Ts + ta_us         │
 └─────────────────────────────────────────┘   └────────────────────────────────────┘
```

- **Rank** answers, per candidate: *"how much pilot energy sits where a physical
  channel could sit, and exactly where?"* One DFT of the LS pilots per slot, a
  constrained sliding-window search over the delay-power profile, no smoothing, no
  time processing, no de-rotation, no decode. Cost per candidate ≈ two small FFTs.
- **The full estimate runs once**, on the winner, with the delay processing centered
  on the delay the ranking measured. Only that pass spends the smoothing, cross-slot,
  interpolation and turbo-decoding effort.

## 2. API reference

### `srsran_chest_ul_rank_pusch(q, sf, cfg, input, max_delay_us, rank)`

Outputs (`srsran_chest_ul_rank_t`):

| Field | Meaning |
|---|---|
| `energy_frac` | Pilot energy fraction captured by the best window placement (0..1). **The ranking metric** — compare across candidates. |
| `delay_us` | Power centroid of the winning window (µs, positive = late UE, same sign as `ta_us`). The fine timing: pass to `..._pusch_win`, add to the candidate's coarse shift for the MAC. |
| `epre` | Pilot energy per RE, for absolute cross-candidate checks (e.g. dead-air rejection). |
| `reliable` | False for grants < 2 PRB (the clamped window covers too much of a 12-bin profile to discriminate). |

Details that matter:

- **`max_delay_us` bounds the window-center search** around delay 0. Pass
  `step/2 + CP/2` for a sweep with the given step: enough to cover the residual
  quantization of the winning candidate plus the centered channel spread, tight
  enough that the search cannot lock onto a cyclic-shift-multiplexed MU-MIMO
  partner's delay peak or an alias, and tight enough that adjacent candidates stay
  distinguishable. `<= 0` = full unambiguous span (±33 µs).
- **The reported delay is the power centroid of the winning window, not the argmax.**
  The captured-energy function is flat wherever the whole channel fits inside the
  window (a plateau as wide as `2·half_win − channel spread`), so the argmax is
  degenerate; the centroid recovers the channel's center with sub-bin resolution.
- **No de-rotation happens during ranking** — by design. The estimator's automatic
  de-rotation makes it invariant to exactly the variable the sweep varies; ranking
  through it would make every hypothesis score alike. This is also why ranking is
  robust under clipping: broadband clipping spurs raise the whole profile floor but
  rarely beat the true in-window maximum.
- Expect a **plateau-and-cliff metric shape** across candidates: two adjacent
  candidates whose windows both contain the channel score nearly identically. Break
  ties however suits the decoder (e.g. toward the later window, keeping ISI margin);
  the centroid makes the final timing exact either way.

### `srsran_chest_ul_estimate_pusch_win(q, sf, cfg, input, window_delay_us, res)`

The full estimator with the delay processing centered at `window_delay_us`:

- The pilots are de-rotated by the supplied center **plus a measured residual clamped
  to ±2 delay bins**. The energy-verified center is authoritative; the phase-slope
  measurement (the part clipping can corrupt) may only fine-tune inside it.
- `res->ta_us` reports the total applied slope, so `coarse_shift + ta_us` is the UE's
  true timing for MAC TA commands.
- `window_delay_us = NAN` gives the automatic behavior of
  `srsran_chest_ul_estimate_pusch()` (which is now a wrapper), with the automatic
  slope clamped to ±9.4 µs (~2 CP) — beyond that, use the sweep.
- `res->delay_energy_frac` reports the fraction of pilot energy the projection window
  captured (NAN when the projection was not attempted) — reuse it as a per-subframe
  confidence/health signal.

### The energy-capture guard (automatic, all modes)

Whenever the delay-domain projection is about to run, the estimator measures the
pilot-energy fraction inside the (centered) window. Below
`max(0.25, 2·keep/nrefs)` — an SNR-independent threshold, so a clipping-skewed SNR
prior cannot weaken it — the projection **yields to the legacy 3-tap FIR** for that
subframe. Measured effect with a center wrong by 5 µs on a 25-PRB grant at 10 dB:

| | CE error vs N0 | Noise-estimate error |
|---|---|---|
| Guard active (fallback) | **0.209** | +0.26 dB |
| Guard disabled (negative control) | **9.91** (10 dB worse than raw LS) | **+10.75 dB** |

The guard converts the pipeline's one catastrophic failure mode (projection deleting
an out-of-window channel) into a graceful few-dB degradation.

## 3. How every previous stage fits

| Stage | Role in the sweep architecture |
|---|---|
| **LS estimation** | Shared verbatim by rank and estimate (same scratch buffers). |
| **TA de-rotation** (stage 7) | *Excluded from ranking* (would flatten the metric). In the winner's estimate it applies the external center + clamped residual; its automatic mode gets the ±9.4 µs physical clamp. It remains what centers the delay window — now fed by measured energy instead of a trusted phase slope. |
| **SNR prior / FIR tiers** (stage 4) | Unchanged, and run on centered pilots, so the tier choice is clean. The 3-tap tier doubles as the guard's fallback filter. |
| **Delay-domain projection** (stage 8) | The heart of both sides: its forward DFT *is* the ranking profile, its window *is* what the center aligns, and the guard wraps it. Rank and projection share the same window-width formula. |
| **Exact noise bias** (stage 3) | Unchanged. Guard and centering decide FIR-vs-projection *before* the noise residual is taken, so the bias model always matches the smoother that actually ran — the noise estimate stays within ~0.1 dB even mid-fallback. |
| **Cross-slot averaging** (stage 5) | Unchanged, downstream. The centering ramp is common to both slots, so the cross-slot phase, its gate and the combining are bit-identical with or without a supplied center. |
| **Phase-corrected time interpolation** (stage 9) | Unchanged, downstream, same reasoning. |
| **Frequency-hopping fix** (stage 2) | Unchanged; rank averages the two slots' profiles (the delay support is common even when the PRBs hop), and the estimate keeps its per-slot indexing. |

In short: ranking borrows the projection's delay-domain view *before* de-rotation;
the winner's estimate is the standard pipeline with the de-rotation's input swapped
from "trusted phase measurement" to "energy-verified center + clamped phase residual";
everything downstream is untouched.

## 4. Measured end-to-end (sweep emulation, `chest_test_ul -W`)

10 candidates at 4.17 µs (64-sample) spacing, rank each, centered-estimate the winner:

| Case | CE error vs N0 | Recovered timing |
|---|---|---|
| 25 PRB @ 5 dB, 20 µs offset | 0.071 (≈ well-timed quality) | 20.01 µs |
| 8 PRB @ 0 dB, 25 µs offset | 0.085 | 24.84 µs |
| 12 PRB noiseless, 12 µs offset | exact (−103 dB) | 11.97 µs |
| 25 PRB @ 10 dB, correct external center | 0.073 | — |
| same, center wrong by 5 µs (guard) | 0.209 (graceful) | — |

Offsets of 20–25 µs are 2–5× beyond the blind de-rotation's clamp — unreachable by
the automatic pipeline — yet decode at well-timed quality through the sweep, while the
recovered coarse+fine timing lands within 0.2 µs of truth.

## 5. Edge cases and conditions handled

- **Narrow grants**: rank clamps the window to ≤ nrefs/6 bins so the metric keeps
  discriminating; below 2 PRB `reliable=false` (metric computed but weak).
- **Degenerate window ≥ profile**: `energy_frac=1, delay=0, reliable=false`.
- **Dead air / zero input**: `energy_frac=0, reliable=false`; the estimate path falls
  back to FIR with `delay_energy_frac=0`.
- **Frequency hopping**: profiles averaged across slots; all downstream hopping
  behavior unchanged (no combining, per-slot CE indexing, CFO=NAN).
- **Clipping**: ranking metric is energy-based (spur-tolerant); the applied slope is
  either clamped-to-physical (auto) or clamped-to-center (±2 bins); the guard catches
  whatever still gets through. Cedron never sits in the critical path of this flow.
- **MU-MIMO / aliases**: the `max_delay_us` constraint keeps the search inside the
  candidate's own plausibility region.
- **Absurd centers**: the center is clamped to ±0.45 cycles/sample (just inside
  Nyquist of the pilot lattice); the guard handles the rest.
- **DFT replan failures / invalid grants**: explicit error returns; no partial state.
