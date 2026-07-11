# Best Stage Combination for Wide PUSCH Grants at Low SNR

**Question:** which combination of estimator stages gives the best channel-estimation
quality for a very wide grant (100 PRB, the LTE maximum = 1200 pilots/slot) at
SNR < 5 dB (cell-edge QPSK)?

**Answer: TA de-rotation + delay-domain projection + cross-slot averaging — the
default configuration.** The FIR taps (including the 9-tap Gaussian that the `<5 dB`
tier would nominally select) are outperformed by the projection at this width and are
correctly bypassed by the noise-gain comparison rule.

## Measured sweep

Every combination was measured, not derived: the option defaults in
`srsran_chest_ul_init()` were toggled per configuration, `chest_test_ul` rebuilt, and
the quality mode run at 100 PRB / 3 dB over 100 subframes
(`chest_test_ul -r 100 -L 100 -s 3 -N 100`, plus a frequency-selective variant `-S`
and a 3 µs timing-offset variant `-t 3.0`). Values are CE error as a fraction of N0
(raw LS = 1.0; lower is better). `time_interp` is inactive at this SNR by design (its
gate requires ≥ ~13 dB) and is omitted.

| # | Stage combination | Flat | Selective | Flat + 3 µs timing offset |
|---|---|---|---|---|
| A | legacy fixed 3-tap + per-slot hold (all off) | 0.335 | 0.513 | 0.337 |
| B | 9-tap Gaussian only (`adaptive_smoothing`) | 0.137 | 0.321 | 0.181 |
| C | 9-tap Gaussian + `cross_slot_avg` | 0.069 | 0.321 | 0.113 |
| D | projection only (`dft_denoise`) | 0.077 | 0.254 | **2.073** |
| E | projection + `cross_slot_avg` | **0.039** | **0.254** | **2.034** |
| F | **all on: + `ta_derotation` (default)** | 0.043 | 0.261 | **0.043** |

## Reading the table

**F (the default) is the best robust combination.** It is within 0.4 dB of the
flat-channel optimum, matches it on selective channels, and is the *only*
projection-based configuration that survives a timing offset.

- **Projection vs 9-tap taps (D vs B):** at 1200 pilots the projection keeps
  93 delay bins → noise gain 0.078, versus the 9-tap's Σw² = 0.137 — ~2.5 dB better,
  and it wins again on the selective channel (0.254 vs 0.321) because a within-CP
  channel passes the delay window undistorted while the 9-tap FIR attenuates genuine
  selectivity. This is why the estimator's comparison rule
  (`kept-bin fraction < Σw²`) hands wide grants to the projection.
- **Cross-slot averaging stacks (C vs B, E vs D):** halves the flat-channel error
  (0.137→0.069, 0.077→0.039). At 3 dB with 1200 pilots the cross-slot phase
  measurement is extremely clean, so the gate sits at its 0.2 rad floor and engages
  whenever the channel is genuinely time-flat. On the selective stimulus the gate
  correctly rejects combining (its cross-slot phase is too large), so those columns
  show no change — a feature, not a miss.
- **TA de-rotation is mandatory for the projection, not an optional companion
  (E vs F, offset column):** a 3 µs timing offset is a linear phase ramp that shifts
  the channel to delay bin ~54 (3 µs × 15 kHz × 1200), *outside* the ±46-bin keep
  window — so without de-rotation the projection deletes the channel itself and the
  estimate degrades beyond raw LS (2.03·N0). With de-rotation the mean delay is
  re-centered at bin 0 and the offset case (0.043) becomes indistinguishable from the
  perfectly-timed one. The FIR combos degrade far more gracefully under offset
  (C: 0.069→0.113) — wide, shallow filters vs a hard window — which is exactly why
  the hard window must never run without the re-centering stage.
- **Why E edges out F by 0.4 dB on the idealized flat case:** the test's flat channel
  sits at exactly integer delay bin 0, so with de-rotation *disabled* it suffers zero
  spectral leakage. De-rotation by the (noisy, ±fractional-bin) measured slope moves
  it to a fractional delay whose sinc tails leak slightly past the window edge. Real
  channels never sit on an integer bin, so this margin is a test artifact; against
  it stands the offset column, where E loses 17 dB. Robustness wins.

## Bottom line

At <5 dB on a very wide grant the winning chain is:

```
LS estimates
  → measure & remove pilot phase slope      (ta_derotation   - re-centers delay at bin 0)
  → project onto ±(CP/2 + margin) delay bins (dft_denoise     - keeps 93/1200 bins, −11 dB noise)
  → exact-residual noise estimate            (projection fraction, +0.02 dB accuracy)
  → phase-aligned cross-slot average         (cross_slot_avg  - another −3 dB when time-flat)
  → per-slot hold fallback if the gate rejects
```

Net: ~**13.8 dB less estimation noise than raw LS** (0.043·N0), ~9 dB better than the
legacy estimator (0.335·N0), with the FIR taps reserved for the narrow grants where
their truncated edges and small footprint genuinely win. No configuration change is
needed — this is what the defaults already select at this operating point.
