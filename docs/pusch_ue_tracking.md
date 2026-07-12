# Per-UE Link Tracking Across Uplink Transmissions

The estimator stages so far are stateless across subframes: every call re-derives noise
power, CFO, timing and delay spread from two DMRS symbols. Those four quantities are not
per-subframe realizations, though — they are slowly-varying properties of the UE's link.
This feature tracks them per UE across transmissions and feeds them back as priors,
upgrading every gate and parameter that previously rode a noisy single-shot measurement.
The channel *coefficients* themselves are deliberately not averaged across subframes
(they decorrelate in milliseconds and are phase-fragile); only phase-blind or
phase-differential statistics are tracked, which is what makes this safe.

Source: `lib/src/phy/ch_estimation/chest_ul.{h,c}`. Tests: `chest_test_ul -T` (tracking
loop) and `-D` (DTX injection).

## API

```
srsran_chest_ul_track_t track;                       // one per RNTI (e.g. in phy_ue_db)
srsran_chest_ul_track_reset(&track);

per PUSCH subframe:
  srsran_chest_ul_prior_t prior;
  srsran_chest_ul_track_get_prior(&track, tti, &prior);
  srsran_chest_ul_estimate_pusch_prior(q, sf, cfg, input, &prior, res);
  ... decode ...
  srsran_chest_ul_track_update(&track, res, tti, crc_ok);

on MAC TA command (UE advances by X us):
  srsran_chest_ul_track_notify_delay_shift(&track, -X);
```

`srsran_chest_ul_estimate_pusch()` and `..._pusch_win()` are wrappers of
`..._pusch_prior()` (NULL priors / delay-only prior respectively). Any invalid prior
entry leaves the corresponding stage exactly as the plain call — an empty or stale
tracker reproduces the untracked estimator bit-for-bit.

## What each prior does

| Prior | Consumption | Effect |
|---|---|---|
| `n0` | Replaces the per-subframe second-difference SNR pre-estimate | Tier selection and time-processing gates stop flapping with the estimate's own noise (±1 dB per-TTI → ~±0.2 dB effective) |
| `cfo_hz` | The cross-slot gates become deviation tests around the predicted phase `2π·cfo·0.5ms`, and the measured phase is unwrapped around it | A UE with a stable frequency offset no longer loses cross-slot averaging/time interpolation to a perfectly predictable rotation; usable CFO range extends beyond ±1 kHz |
| `delay_us` | Centers the delay window (same mechanism as the sweep API's `window_delay_us`) | Fine timing follows the UE between MAC TA commands; the measured slope only fine-tunes within ±2 bins of it |
| `spread_us` | Narrows the projection window below the blind ±(CP/2+3) bound, floor `margin+2` bins, **only when `snr_prior < PUSCH_SMOOTH_SNR_LOW` (~5 dB)** | Every excluded bin is excluded noise: −1.9 dB more CE quality at 0 dB on wide grants. The SNR gate exists because the narrower window also cuts the channel's own fractional-delay sinc leakage — a bias ∝ channel power that overtakes the saved noise (∝ N0) above ~5 dB (measured: clear win at ≤3 dB, wash at 5 dB, loss at 10 dB) |

The energy-capture guard remains the backstop for the narrowed window: if it ever misses
the energy, the estimator falls back to the FIR *that same subframe*, the reported
`delay_spread_us` grows, and the tracker widens fast.

## Tracker robustness rules

- **DTX gate** (the critical one): signal-dependent tracks (CFO, delay, spread) update
  only on a CRC pass or a comfortably detected signal (≥ ~6 dB). A missed grant leaves
  noise-only "measurements" that would otherwise steer the tracks; with DTX injected on
  every 4th subframe the tracked metrics are measured *identical* to the DTX-free run.
- **Noise updates always** — the floor is measurable even on DTX — with **fast attack /
  slow decay** (α = 0.5 up, 0.05 down): an interference burst is believed immediately,
  a low floor must be earned.
- **Slew limits**: CFO ±30 Hz and delay ±0.3 µs per accepted update — one bad accepted
  measurement cannot move a track far.
- **Spread widens fast, narrows slowly** (0.5/0.05) — the window-narrowing direction is
  the risky one.
- **Warmup**: signal-dependent priors are withheld until 4 accepted updates.
- **Aging**: priors expire after 500 TTIs without an update (10240-TTI wrap handled);
  a long silence resets the state entirely rather than slew-filtering toward a possibly
  distant new operating point.
- **TA commands**: the known delay shift is applied directly instead of re-learned.

## New measurements in `srsran_chest_ul_res_t`

`delay_centroid_us` (absolute, noise-floor-subtracted power centroid of the pilot delay
profile) and `delay_spread_us` (4σ of the in-window profile) are computed for free in
the guard block whenever the projection is attempted, and are what the tracker consumes.
The floor subtraction uses the excluded bins — pure noise whenever the guard passes.

## Measured results (200 subframes, `chest_test_ul -T`)

| Case | Untracked | Tracked |
|---|---|---|
| 25 PRB @ 0 dB flat | 0.0559 | **0.0363** (−1.9 dB) |
| 25 PRB @ 3 dB flat | 0.0627 | 0.0547 |
| 25 PRB @ 5–15 dB flat | — | bit-identical parity (gates off) |
| 25 PRB @ 3 dB with DTX every 4th subframe | — | 0.0544 (identical to DTX-free) |
| 25 PRB @ 3 dB, 3 µs offset | — | 0.0558, TA 3.00 µs |
| 1 PRB @ 0 dB, 300 Hz CFO | 0.368 | **0.282** (combining restored; the remainder is within-slot rotation no cross-slot method can remove) |
| 12 PRB @ 20 dB selective (time-drifting) | 4.24 | **0.324** (the CFO track learns the drift, making the interpolation gate deterministic) |

## eNB integration point

The tracker is deliberately estimator-agnostic: one `srsran_chest_ul_track_t` per RNTI
per carrier belongs in `phy_ue_db` (shared, mutexed — updates are commutative scalar
filters), with `get_prior` before and `track_update` after each `decode_pusch`, `crc_ok`
from the decoder, and `notify_delay_shift` wherever TA commands are issued. Multiple
pipelined subframe workers can share it safely; a worker seeing slightly stale priors is
indistinguishable from one more TTI of aging.
