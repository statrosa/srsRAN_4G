# Soft-Threshold Wiener Weighting of the Delay Window

Roadmap item A, implemented **tracker-free** after field feedback showed the
cross-transmission tracker rarely warms up under real bursty traffic. The delay-domain
projection's hard window keeps every in-window bin at weight 1 — on a 25 PRB grant,
~29 bins for a channel that typically occupies 2–4 — and the tracked spread-narrowing
that addressed this needed a ~5 dB SNR gate (window-edge bias ∝ signal power) plus a
warm, per-UE tracker. This feature replaces the window's interior with **per-bin
weights computed from the current subframe alone**, and retires the narrowing gate.

## The estimator

All inputs come from the guard block's existing computation (delay-power profile of
both slots, per-bin noise floor `nfloor` from the excluded bins) — zero extra
transforms:

```
pass_d    = profile_d > α·nfloor                     (α = 2.5)
mask      = pass dilated by ±2 bins                  (leakage protection)
w_d       = mask_d ? max(0, 1 − nfloor/profile_d) : 0    (≡ P̂/(P̂+N), spectral subtraction)
```

applied inside the ±(CP/2+3) window only; outside stays hard-zeroed (the physics bound
and energy-capture guard are unchanged). Weights are recomputed every subframe; a UE's
very first transmission gets them.

**Two protections, both earned by measurement rather than assumed:**

1. **±2-bin dilation.** The first implementation without it measured a wash at 5–10 dB:
   zeroing "empty" bins also clipped the sinc-leakage tails of fractional-delay taps —
   the same bias mechanism (∝ S) that forced the narrowing gate, recreated per-bin.
   Leakage concentrates next to occupied bins, so letting a pass-bin's neighbors
   through removes most of the bias at the cost of ~4 extra noise bins.
2. **Measured self-gate.** Both sides of the remaining trade are directly observable in
   the profile: `noise_saved = Σ_zeroed nfloor` and `sig_clipped = Σ_zeroed
   max(0, profile − nfloor)`. The weights are applied only when
   `sig_clipped < 0.5·noise_saved` — a per-subframe, data-driven decision with no SNR
   heuristic. (A tighter 0.25 margin was measured and rejected: it costs more at the
   flat-channel wins than it saves at the structured-channel washes.) The gate also
   makes the negative control free: α→∞ zeroes everything → gate rejects → exact
   hard-window behavior.

Noise estimation uses the exact diagonal-operator residual fraction
`Σ(1−w_d)²/nrefs` (reducing to the projection's discarded fraction when no weights
apply); measured accuracy stays within +0.3 dB worst case (adaptive-weight Stein bias,
inside the ±0.5 dB asserts).

## Measured (25 PRB, 200 subframes/point, untracked — no priors involved)

| Case | Hard window | Wiener | Note |
|---|---|---|---|
| Flat 0 dB | 0.0559 | **0.0369** | matches the tracked narrowing's 0.0363 with zero tracker state |
| Flat 5 dB | 0.0708 | **0.0649** | the old gate forced hard-window here |
| Flat 10 dB | 0.0722 | **0.0633** | ditto |
| Flat 20 dB | 0.1173 | **0.0418** | biggest win — the regime the narrowing could never touch |
| Two-path (1.5 µs, −3 dB) @ 0 dB | 0.0549 | **0.0399** | structured channel |
| Two-path @ 10 dB | 0.1022 | 0.1089 | honest wash (−0.03 dB effective): self-gate margin at work |
| Noiseless | exact | exact | `nfloor→0 ⇒ w→1`: exactness by construction |

Three ctests pin the wins with thresholds the hard window fails
(`chest_test_ul_wiener_flat0/_flat20/_2path`); the two-path stimulus is the new
`-P <delay_us>,<rel_dB>` option. Full suite: 202/202.

## What it supersedes and what remains

- The **tracked spread-narrowing** and its `PUSCH_SPREAD_NARROW_SNR_MAX` gate are
  removed; `spread_us` remains in the tracker and results as a diagnostic. The tracked
  ctest points now pass via the stateless weights.
- The tracker keeps its other, still-unique payloads (CFO gate referencing, N0
  stability, delay centering) — but the CE-quality feature that most depended on it is
  now unconditional.
- A tracked **PDP prior** could still reduce weight variance on very low-SNR subframes
  (the per-bin power estimate has 2 observations); that refinement stays on the
  roadmap as optional, to be adopted only if it measures.
