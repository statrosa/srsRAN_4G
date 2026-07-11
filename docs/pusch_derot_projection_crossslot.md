# TA De-rotation + Delay-Domain Projection + Cross-Slot Averaging

This document contains the full source of the three estimator stages that form the
best-measured combination for wide, low-SNR PUSCH grants (see
`pusch_wide_grant_low_snr.md` for the measured sweep), and — the part that matters most
— the ordering contract that makes them compose correctly. All code is verbatim from
`lib/src/phy/ch_estimation/chest_ul.c` unless noted. The stages are gated by
`pusch_opts.ta_derotation`, `pusch_opts.dft_denoise` and `pusch_opts.cross_slot_avg`
(all default on, PUSCH only; PUCCH and SRS are untouched).

## 0. Where each stage sits in the pipeline

```
srsran_chest_ul_estimate_pusch()
  1. LS estimates                p[k] = recv[k] * conj(dmrs[k])
  2. TA DE-ROTATION              measure slope fe -> flatten pilots      [stage 1a]
  3. pusch_select_filter()       SNR prior + FIR tier (on clean pilots)
  4. PROJECTION SELECTION        delay window vs FIR noise gain          [stage 2a]
  5. CROSS-SLOT GATE             SNR-adaptive phase gate                 [stage 3a]
  chest_ul_estimate()
  6. cross-slot phase            phi = carg( sum p0 * conj(p1) )  -> CFO
  7. smoothing                   PROJECTION (or FIR) per slot            [stage 2b]
  8. noise estimate              residual in the FLATTENED domain
  9. RE-ROTATION                 re-apply fe to the two smoothed rows    [stage 1b]
 10. time processing             CROSS-SLOT AVERAGE / interp / hold      [stage 3b]
 11. TA override                 res->ta_us from fe                      [stage 1c]
```

Steps 2→8→9→10 form a strict ordering contract; section 4 explains what breaks if any
two are swapped.

---

## 1. Stage 1 — TA de-rotation

A residual timing offset `ta` rotates the LS pilots by `e^{-j*2pi*15kHz*ta*k}` across
frequency. The slope is measured (it *is* the TA), removed before all frequency-domain
processing, and re-applied to the smoothed estimates.

### 1a. Measurement helpers

```c
/**
 * Mean phase slope of the LS pilot estimates across frequency, in normalized frequency units
 * (cycles/sample), averaged over the slots. This is the same measurement the TA estimator performs: a
 * timing offset ta rotates the pilots by e^{-j*2pi*15kHz*ta*k}, for which this returns +15kHz*ta.
 */
static float
measure_pilot_slope(srsran_chest_ul_t* q, uint32_t nslots, uint32_t nrefs_sym, bool use_cedron_alg)
{
  float fe = 0.0f;
  for (uint32_t i = 0; i < nslots; i++) {
    if (use_cedron_alg) {
      fe += srsran_cedron_freq_estimate(&q->srsran_cedron_freq_est, &q->pilot_estimates[i * nrefs_sym], nrefs_sym) /
            nslots;
    } else {
      fe += srsran_vec_estimate_frequency(&q->pilot_estimates[i * nrefs_sym], nrefs_sym) / nslots;
    }
  }
  return fe;
}

/// Converts a pilot phase slope (normalized frequency, cycles/sample) to a time alignment error in
/// micro-seconds, rounded to one tenth of micro-second (same conversion the legacy TA path applies)
static float pilot_slope_to_ta_us(float fe, uint32_t stride)
{
  if (!isnormal(fe) || stride == 0) {
    return 0.0f;
  }
  fe /= (float)stride; // Divide by the pilot spacing
  fe /= 15e3f;         // Convert from normalized frequency to seconds
  fe *= 1e6f;          // Convert to micro-seconds
  return roundf(fe * 10.0f) / 10.0f;
}
```

### 1b. Flattening (in `srsran_chest_ul_estimate_pusch`, before filter selection)

```c
  // Measure and remove the pilot phase slope (i.e. the timing offset) before any frequency-domain
  // processing: a residual timing error rotates the pilots by e^{-j*2pi*15kHz*ta*k}, which smoothing
  // filters average across (biasing the estimates low, worst for the long low-SNR filters on
  // pre-TA-convergence grants such as msg3) and which inflates the second-difference SNR pre-estimate.
  // De-rotation is unitary, so the noise statistics and the modeled filter bias are unchanged; the slope
  // is re-applied to the smoothed estimates inside chest_ul_estimate.
  bool  meas_ta_en = cfg->meas_ta_en;
  float derot_ta_us = 0.0f;
  if (q->pusch_opts.ta_derotation) {
    float fe = measure_pilot_slope(q, SRSRAN_NOF_SLOTS_PER_SF, nrefs_sym, cfg->use_cedron_alg);
    if (isnormal(fe)) {
      // Pilots carry e^{-j*2pi*fe*k}: multiply by the conjugate ramp (srsran_vec_apply_cfo applies
      // e^{+j*2pi*cfo*n}). The slope is common to both slots - even under hopping only the constant
      // phase offset differs, and that folds into the per-slot channel estimate.
      for (uint32_t i = 0; i < SRSRAN_NOF_SLOTS_PER_SF; i++) {
        srsran_vec_apply_cfo(&q->pilot_estimates[i * nrefs_sym], fe, &q->pilot_estimates[i * nrefs_sym], nrefs_sym);
      }
      proc.derot_cfo = fe;
      // The measured slope IS the time alignment error; the flattened pilots would measure ~0, so take
      // over the TA measurement from chest_ul_estimate
      derot_ta_us = pilot_slope_to_ta_us(fe, 1);
      meas_ta_en  = false;
    }
  }
```

### 1c. Re-rotation (in `chest_ul_estimate`, after the noise residual, before time processing)

```c
    // Re-apply the timing-offset slope that was removed from the pilots, so the estimates carry the true
    // channel phase. Must run after the noise residual (which is computed in the flattened domain, where
    // it matches the modeled filter bias) and before any time-domain processing.
    if (proc->derot_cfo != 0.0f) {
      for (uint32_t i = 0; i < nslots; i++) {
        cf_t* row =
            &res->ce[SRSRAN_REFSIGNAL_UL_L(i, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE + n_prb[i] * SRSRAN_NRE];
        srsran_vec_apply_cfo(row, -proc->derot_cfo, row, nrefs_sym);
      }
    }
```

### 1d. TA hand-over (back in `srsran_chest_ul_estimate_pusch`, after estimation)

```c
  // TA measured by the de-rotation stage (chest_ul_estimate saw already-flattened pilots)
  if (cfg->meas_ta_en && proc.derot_cfo != 0.0f) {
    res->ta_us = derot_ta_us;
  }
```

---

## 2. Stage 2 — delay-domain projection

For grants of ≥ 8 PRB, the flattened LS estimates are projected onto the delay bins a
within-CP channel can occupy: unitary DFT to the delay domain, keep
`|d| <= half_win`, unitary DFT back. Noise gain = kept-bin fraction (0.078 at 100 PRB).

### 2a. Plans (in `srsran_chest_ul_init` / `srsran_chest_ul_free`)

```c
    // Delay-domain denoising transforms, unitary in each direction so fwd+bwd is an identity. Planned at
    // the maximum width so per-grant replans (bounded by init_size) never allocate.
    if (srsran_dft_plan_c(&q->dft_fwd, MAX_REFS_SYM, SRSRAN_DFT_FORWARD) ||
        srsran_dft_plan_c(&q->dft_bwd, MAX_REFS_SYM, SRSRAN_DFT_BACKWARD)) {
      ERROR("Error initializing delay-domain DFT plans");
      goto clean_exit;
    }
    srsran_dft_plan_set_norm(&q->dft_fwd, true);
    srsran_dft_plan_set_norm(&q->dft_bwd, true);
    q->dft_size = MAX_REFS_SYM;
```

### 2b. Per-grant selection (in `srsran_chest_ul_estimate_pusch`)

```c
// Delay-domain denoising engages for allocations of at least this many pilots (8 PRB): below it the
// kept-bin fraction is too large for the projection to beat the FIR tiers
#define PUSCH_DFT_MIN_NREFS 96
// The channel delay spread is physically bounded by the normal cyclic prefix
#define PUSCH_DFT_DELAY_SPAN_S 4.7e-6f
// Extra kept bins on each side, absorbing the sinc leakage of fractional-delay paths
#define PUSCH_DFT_MARGIN_BINS 3

  // Wide grants: project onto the delay bins a within-CP channel can occupy instead of FIR smoothing.
  // The mean delay was centered at bin 0 by the TA de-rotation, so a symmetric window of half the CP
  // (plus a leakage margin) covers the physical delay spread. The channel passes undistorted at any SNR,
  // so the projection replaces the FIR whenever it also removes more noise: its noise gain is the
  // kept-bin fraction, the FIR's is the squared norm of its taps.
  if (q->pusch_opts.dft_denoise && nrefs_sym >= PUSCH_DFT_MIN_NREFS) {
    uint32_t half_win =
        (uint32_t)ceilf(0.5f * PUSCH_DFT_DELAY_SPAN_S * 15e3f * (float)nrefs_sym) + PUSCH_DFT_MARGIN_BINS;
    float fir_gain = 0.0f;
    for (uint32_t i = 0; i < proc.filter_len; i++) {
      fir_gain += proc.filter[i] * proc.filter[i];
    }
    if ((float)(2 * half_win + 1) < fir_gain * (float)nrefs_sym) {
      if (q->dft_size != (uint32_t)nrefs_sym) {
        if (srsran_dft_replan_c(&q->dft_fwd, nrefs_sym) || srsran_dft_replan_c(&q->dft_bwd, nrefs_sym)) {
          ERROR("Error replanning delay-domain DFT to %d points", nrefs_sym);
          return SRSRAN_ERROR;
        }
        q->dft_size = (uint32_t)nrefs_sym;
      }
      proc.dft_half_win = half_win;
    }
  }
```

### 2c. Application (the DFT branch of `average_pilots`, per slot)

```c
    if (proc->dft_half_win > 0) {
      // Delay-domain projection: any within-CP channel lives in the kept bins (TA de-rotation centered the
      // mean delay at bin 0), so it passes undistorted while the noise of the discarded bins is removed
      cf_t* delay = q->tmp_noise; // scratch, free at this point
      srsran_dft_run_c(&q->dft_fwd, &input[i * nrefs], delay);
      srsran_vec_cf_zero(&delay[proc->dft_half_win + 1], nrefs - 2 * proc->dft_half_win - 1);
      srsran_dft_run_c(&q->dft_bwd, delay, out);
    }
```

### 2d. Exact noise bias (the DFT branch of `estimate_noise_pilots`)

```c
  if (proc->dft_half_win > 0) {
    float power = 0;
    for (int i = 0; i < nslots; i++) {
      power += srsran_chest_estimate_noise_pilots(
          &q->pilot_estimates[i * nrefs],
          &ce[SRSRAN_REFSIGNAL_UL_L(i, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE + n_prb[i] * SRSRAN_NRE],
          q->tmp_noise,
          nrefs);
    }
    power /= nslots;
    // The delay-domain smoother is an orthogonal projection, so the residual contains exactly the noise
    // that fell in the discarded bins: the bias factor is the discarded fraction, with no edge effects
    uint32_t keep = 2 * proc->dft_half_win + 1;
    if (keep < nrefs) {
      return power * (float)nrefs / (float)(nrefs - keep);
    }
    return power;
  }
```

---

## 3. Stage 3 — phase-aligned cross-slot averaging

When the channel is time-flat within the subframe, the two per-slot smoothed estimates
are redundant; averaging them halves the noise on every symbol. The measured cross-slot
phase is used for alignment (so small residual CFO does not degrade the average) and as
the time-flatness detector.

### 3a. The gate (in `srsran_chest_ul_estimate_pusch`)

```c
// Cross-slot DMRS averaging engages only when the pilot phase drift over 0.5 ms stays below a threshold,
// i.e. the channel is time-flat (residual CFO below ~64 Hz and low Doppler). Since the combining aligns
// with the measured phase, the gate only needs to reject genuine channel changes, not measurement noise:
// its width grows with the expected phase-measurement standard deviation 1/sqrt(nrefs*snr) so that low-SNR
// subframes (where the noise reduction matters most) are not rejected by the gate's own noise.
#define PUSCH_CROSS_SLOT_MAX_PHASE_RAD 0.2f
#define PUSCH_CROSS_SLOT_PHASE_NSTD 2.5f
#define PUSCH_CROSS_SLOT_PHASE_CAP_RAD 1.0f

    // Below the high tier the per-slot estimates are noisy and cross-slot averaging is worth more than
    // time tracking; at and above it, track the channel across the subframe instead (the <= 2.23x noise
    // amplification on the extrapolated edge symbols is cheap there, and the phase is measured accurately)
    if (q->pusch_opts.cross_slot_avg && (q->pusch_snr_prior < PUSCH_SMOOTH_SNR_HIGH)) {
      float phase_std           = 1.0f / sqrtf((float)nrefs_sym * SRSRAN_MAX(q->pusch_snr_prior, 0.01f));
      proc.cross_slot_max_phase = SRSRAN_MAX(
          PUSCH_CROSS_SLOT_MAX_PHASE_RAD,
          SRSRAN_MIN(PUSCH_CROSS_SLOT_PHASE_CAP_RAD, PUSCH_CROSS_SLOT_PHASE_NSTD * phase_std));
    }
```

### 3b. The phase measurement and combining (in `chest_ul_estimate`)

The cross-slot phase is measured once, up front, from the (flattened) pilots — it also
produces the CFO estimate:

```c
  // Calculate CFO
  float cross_phase = 0.0f;
  if (nslots == 2) {
    cross_phase = cargf(srsran_vec_dot_prod_conj_ccc(
        &q->pilot_estimates[0 * nrefs_sym], &q->pilot_estimates[1 * nrefs_sym], nrefs_sym));
    res->cfo_hz = cross_phase / (2.0f * (float)M_PI * 0.0005f);
  } else {
    res->cfo_hz = NAN;
  }
```

and applied to the *smoothed, re-rotated* rows in the time-processing step:

```c
      // When the channel is time-flat (small cross-slot phase, which doubles as a Doppler/residual-CFO
      // detector) and both slots sit at the same PRBs, averaging the two DMRS estimates halves the
      // estimation noise for every symbol of the subframe. The cross-slot phase is preserved so each
      // slot keeps its own mean phase.
      bool combined = false;
      if (proc->cross_slot_max_phase > 0.0f && nslots == 2 && !hopping &&
          fabsf(cross_phase) < proc->cross_slot_max_phase) {
        cf_t* h0 = &res->ce[SRSRAN_REFSIGNAL_UL_L(0, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE +
                            n_prb[0] * SRSRAN_NRE];
        cf_t* h1 = &res->ce[SRSRAN_REFSIGNAL_UL_L(1, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE +
                            n_prb[1] * SRSRAN_NRE];
        cf_t* c0 = q->tmp_noise; // scratch, free at this point
        cf_t* c1 = &q->tmp_noise[nrefs_sym];

        // c0 = (h0 + h1*e^{+j*phase})/2 (slot-1 estimate aligned to slot 0), c1 = c0*e^{-j*phase}
        cf_t rot = cexpf(I * cross_phase);
        srsran_vec_sc_prod_ccc(h1, rot, c0, nrefs_sym);
        srsran_vec_sum_ccc(h0, c0, c0, nrefs_sym);
        srsran_vec_sc_prod_cfc(c0, 0.5f, c0, nrefs_sym);
        srsran_vec_sc_prod_ccc(c0, conjf(rot), c1, nrefs_sym);

        for (int i = 0; i < SRSRAN_CP_NSYMB(q->cell.cp); i++) {
          srsran_vec_cf_copy(&res->ce[(i * q->cell.nof_prb + n_prb[0]) * SRSRAN_NRE], c0, nrefs_sym);
          srsran_vec_cf_copy(
              &res->ce[((i + SRSRAN_CP_NSYMB(q->cell.cp)) * q->cell.nof_prb + n_prb[1]) * SRSRAN_NRE],
              c1,
              nrefs_sym);
        }
        combined = true;
      }
```

If the gate rejects (or under hopping), the code falls through to time interpolation
(high SNR) or the per-slot hold.

---

## 4. How the three stages interact

### 4.1 De-rotation → projection: the window depends on the re-centering

The projection's keep-window is **symmetric, ±(CP/2 + margin) around delay 0**. That is
only valid because de-rotation removes the *mean* delay first: a channel with paths at
delays `0..τ_max` (≤ CP) becomes, after removing its mean slope, a channel with paths
at `−τ_max/2..+τ_max/2` — inside the window by construction. Without de-rotation, a
timing offset shifts every path by `ta` delay bins and can push the whole channel out
of the window, at which point the projection **deletes the channel itself**. Measured
at 100 PRB / 3 dB with a 3 µs offset (the channel lands at bin ~54, the window ends at
±46):

| Configuration | CE error, 3 µs offset |
|---|---|
| projection + cross-slot, **no de-rotation** | **2.03·N0 (worse than raw LS)** |
| projection + cross-slot + de-rotation | **0.043·N0** |

This is the strongest coupling in the pipeline: `dft_denoise` should never run without
`ta_derotation`. (The FIR filters degrade only gracefully under the same offset —
wide shallow response vs a hard window — which is why the legacy path never needed
this coupling.)

### 4.2 De-rotation → SNR prior → both gates

The second-difference SNR prior (`pusch_select_filter`) cancels *linear* channels
only; an un-flattened timing ramp reads as noise, dragging `snr_prior` down. Both
time-processing gates key off `snr_prior` (`< 13 dB` → averaging candidate, `≥ 13 dB`
→ interpolation candidate), and the projection-vs-FIR arbitration compares against the
FIR the tier selected — so an inflated noise reading distorts every downstream
decision. Flattening first keeps the prior honest.

### 4.3 Projection ↔ cross-slot averaging: independent axes, multiplicative gains

The projection reduces noise across **frequency** (within each slot); the averaging
reduces it across **time** (between slots). They are orthogonal linear operations on
independent noise dimensions, so their gains multiply: at 100 PRB / 3 dB, 0.078 (kept
fraction) × ½ (two-slot average) = 0.039 predicted, 0.039–0.043 measured. The ordering
is projection first, averaging second — averaging first would halve the noise the
projection sees but also halve the residual the noise estimate needs (4.4).

### 4.4 Everything → the noise estimate: the residual must stay in the flattened, pre-combined domain

`estimate_noise_pilots()` measures `|pilots − smoothed|²` and divides by the exact
expected residual fraction of the smoothing operator — `(nrefs−K)/nrefs` for the
projection. Two ordering rules keep that exact:

- **Re-rotation happens after the residual.** Both `pilot_estimates` and the smoothed
  rows are in the flattened domain when the residual is taken; re-rotating first would
  compare a rotated estimate against un-rotated pilots and turn the channel itself
  into "residual".
- **Combining happens after the residual.** The bias model describes the per-slot
  smoother; averaging the slots first would halve the CE noise without updating the
  model, biasing the noise estimate low by 3 dB.

The payoff of respecting both: the noise estimate stays within ±0.03 dB at 100 PRB
even with all three stages active.

### 4.5 De-rotation ↔ cross-slot phase: transparent by construction

The cross-slot phase is `carg(Σ p0[k]·conj(p1[k]))`. De-rotation multiplies both slots
by the *same* ramp `e^{+j2π·fe·k}`, which cancels exactly in each product term — so
the CFO estimate, the averaging gate and the alignment rotation are all identical with
or without de-rotation. Similarly, the combining runs on the re-rotated rows, so the
averaged estimates carry the true channel phase ramp that the equalizer needs.

### 4.6 Frequency hopping: each stage degrades independently and safely

Under intra-subframe hopping (`n_prb[0] != n_prb[1]`): the slope is still common to
both slots (only the constant offset differs, and that folds into each slot's channel
phase), so de-rotation and the projection keep working per slot; the cross-slot
average is hard-disabled (`!hopping` in the combine condition) because the two slots
see different channels; and the CFO is reported as NAN. No stage needs the others to
handle hopping.

### 4.7 Shared scratch, zero allocations

All three stages reuse `q->tmp_noise` (sized `MAX_REFS_SF`) strictly sequentially:
SNR-prior second differences → projection delay buffer → noise-residual scratch →
combining buffers `c0/c1`. No stage allocates at estimation time; the DFT plans are
created once at `max_prb` width and replanned (never reallocated) per grant.

### Measured bottom line (100 PRB, 3 dB, flat channel)

| Stages active | CE error vs N0 |
|---|---|
| none (legacy 3-tap + hold) | 0.335 |
| projection only | 0.077 |
| projection + cross-slot | 0.039 |
| **all three (default)** | **0.043 flat / 0.043 with 3 µs offset** |

The all-on configuration gives up ~0.4 dB on the idealized perfectly-timed flat test
channel (integer-bin artifact) and gains ~17 dB the moment a real timing offset
appears — which is why all three ship enabled together.
