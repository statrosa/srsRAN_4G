/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <complex.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "srsran/config.h"
#include "srsran/phy/ch_estimation/cedron_freq_estimator.h"
#include "srsran/phy/ch_estimation/chest_ul.h"
#include "srsran/phy/dft/dft_precoding.h"
#include "srsran/phy/utils/convolution.h"
#include "srsran/phy/utils/vector.h"
#include "srsran/srsran.h"

#define NOF_REFS_SYM (q->cell.nof_prb * SRSRAN_NRE)
#define NOF_REFS_SF (NOF_REFS_SYM * 2) // 2 reference symbols per subframe

#define MAX_REFS_SYM (max_prb * SRSRAN_NRE)
#define MAX_REFS_SF (max_prb * SRSRAN_NRE * 2) // 2 reference symbols per subframe

/** 3GPP LTE Downlink channel estimator and equalizer.
 * Estimates the channel in the resource elements transmitting references and interpolates for the rest
 * of the resource grid.
 *
 * The equalizer uses the channel estimates to produce an estimation of the transmitted symbol.
 *
 * This object depends on the srsran_refsignal_t object for creating the LTE CSR signal.
 */

int srsran_chest_ul_init(srsran_chest_ul_t* q, uint32_t max_prb)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;
  if (q != NULL) {
    bzero(q, sizeof(srsran_chest_ul_t));

    q->tmp_noise = srsran_vec_cf_malloc(MAX_REFS_SF);
    if (!q->tmp_noise) {
      perror("malloc");
      goto clean_exit;
    }
    q->pilot_estimates = srsran_vec_cf_malloc(MAX_REFS_SF);
    if (!q->pilot_estimates) {
      perror("malloc");
      goto clean_exit;
    }
    for (int i = 0; i < 4; i++) {
      q->pilot_estimates_tmp[i] = srsran_vec_cf_malloc(MAX_REFS_SF);
      if (!q->pilot_estimates_tmp[i]) {
        perror("malloc");
        goto clean_exit;
      }
    }
    q->pilot_recv_signal = srsran_vec_cf_malloc(MAX_REFS_SF + 1);
    if (!q->pilot_recv_signal) {
      perror("malloc");
      goto clean_exit;
    }

    q->pilot_known_signal = srsran_vec_cf_malloc(MAX_REFS_SF + 1);
    if (!q->pilot_known_signal) {
      perror("malloc");
      goto clean_exit;
    }

    if (srsran_interp_linear_vector_init(&q->srsran_interp_linvec, MAX_REFS_SYM)) {
      ERROR("Error initializing vector interpolator");
      goto clean_exit;
    }

    q->smooth_filter_len = 3;
    srsran_chest_set_smooth_filter3_coeff(q->smooth_filter, 0.3333);

    q->pusch_opts.adaptive_smoothing = true;
    q->pusch_opts.cross_slot_avg     = true;
    q->pusch_opts.ta_derotation      = true;
    q->pusch_opts.dft_denoise        = true;
    q->pusch_opts.time_interp        = true;

    q->dmrs_signal_configured = false;

    if (srsran_refsignal_dmrs_pusch_pregen_init(&q->dmrs_pregen, max_prb)) {
      ERROR("Error allocating memory for pregenerated signals");
      goto clean_exit;
    }

    if (srsran_cedron_freq_est_init(&q->srsran_cedron_freq_est, max_prb)) {
      ERROR("Error initializing cedron freq estimation algorithm.");
      goto clean_exit;
    }

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

    q->delay_profile = srsran_vec_f_malloc(MAX_REFS_SYM);
    if (!q->delay_profile) {
      perror("malloc");
      goto clean_exit;
    }
  }

  ret = SRSRAN_SUCCESS;

clean_exit:
  if (ret != SRSRAN_SUCCESS) {
    srsran_chest_ul_free(q);
  }
  return ret;
}

void srsran_chest_ul_free(srsran_chest_ul_t* q)
{
  srsran_refsignal_dmrs_pusch_pregen_free(&q->dmrs_signal, &q->dmrs_pregen);

  if (q->tmp_noise) {
    free(q->tmp_noise);
  }
  srsran_interp_linear_vector_free(&q->srsran_interp_linvec);
  srsran_cedron_freq_est_free(&q->srsran_cedron_freq_est);
  srsran_dft_plan_free(&q->dft_fwd);
  srsran_dft_plan_free(&q->dft_bwd);
  if (q->delay_profile) {
    free(q->delay_profile);
  }

  if (q->pilot_estimates) {
    free(q->pilot_estimates);
  }
  for (int i = 0; i < 4; i++) {
    if (q->pilot_estimates_tmp[i]) {
      free(q->pilot_estimates_tmp[i]);
    }
  }
  if (q->pilot_recv_signal) {
    free(q->pilot_recv_signal);
  }
  if (q->pilot_known_signal) {
    free(q->pilot_known_signal);
  }
  bzero(q, sizeof(srsran_chest_ul_t));
}

int srsran_chest_ul_res_init(srsran_chest_ul_res_t* q, uint32_t max_prb)
{
  bzero(q, sizeof(srsran_chest_ul_res_t));
  q->nof_re = SRSRAN_SF_LEN_RE(max_prb, SRSRAN_CP_NORM);
  q->ce     = srsran_vec_cf_malloc(q->nof_re);
  if (!q->ce) {
    perror("malloc");
    return -1;
  }
  return 0;
}

void srsran_chest_ul_res_set_identity(srsran_chest_ul_res_t* q)
{
  for (uint32_t i = 0; i < q->nof_re; i++) {
    q->ce[i] = 1.0;
  }
}

void srsran_chest_ul_res_free(srsran_chest_ul_res_t* q)
{
  if (q->ce) {
    free(q->ce);
  }
}

int srsran_chest_ul_set_cell(srsran_chest_ul_t* q, srsran_cell_t cell)
{
  int ret = SRSRAN_ERROR_INVALID_INPUTS;
  if (q != NULL && srsran_cell_isvalid(&cell)) {
    if (cell.id != q->cell.id || q->cell.nof_prb == 0) {
      q->cell = cell;
      ret     = srsran_refsignal_ul_set_cell(&q->dmrs_signal, cell);
      if (ret != SRSRAN_SUCCESS) {
        ERROR("Error initializing CSR signal (%d)", ret);
        return SRSRAN_ERROR;
      }

      if (srsran_interp_linear_vector_resize(&q->srsran_interp_linvec, NOF_REFS_SYM)) {
        ERROR("Error initializing vector interpolator");
        return SRSRAN_ERROR;
      }
    }
    ret = SRSRAN_SUCCESS;
  }
  return ret;
}

void srsran_chest_ul_pregen(srsran_chest_ul_t*                 q,
                            srsran_refsignal_dmrs_pusch_cfg_t* cfg,
                            srsran_refsignal_srs_cfg_t*        srs_cfg)
{
  srsran_refsignal_dmrs_pusch_pregen(&q->dmrs_signal, &q->dmrs_pregen, cfg);
  q->dmrs_signal_configured = true;

  if (srs_cfg) {
    srsran_refsignal_srs_pregen(&q->dmrs_signal, &q->srs_pregen, srs_cfg, cfg);
    q->srs_signal_configured = true;
  }
}

/**
 * Per-call description of the frequency- and time-domain processing chest_ul_estimate() applies. The PUSCH
 * path fills it from srsran_chest_ul_pusch_opts_t and the current grant; PUCCH has its own estimator and
 * SRS passes the legacy configuration (shared filter, extrapolated edges, everything else off).
 */
typedef struct {
  const float* filter;               // frequency-smoothing filter taps (zero filter_len disables smoothing)
  uint32_t     filter_len;           // number of filter taps
  bool         trunc_edges;          // true: truncate+renormalize at band edges; false: linear extrapolation
  uint32_t     dft_half_win;         // >0: smooth by projecting onto the delay bins |d| <= dft_half_win
                                     // (2*dft_half_win+1 bins total) instead of applying the FIR filter
  float        derot_cfo;            // pilot phase slope removed before smoothing, in normalized frequency
                                     // units (cycles/sample); re-applied to the estimates (0 disables)
  float        cross_slot_max_phase; // cross-slot averaging gate in radians (0 disables)
  float        time_interp_max_phase; // time-interpolation gate in radians (0 disables)
  float        cross_phase_ref;      // predicted cross-slot phase from a tracked CFO (radians): the gates
                                     // test the deviation from it (a predictable rotation is not channel
                                     // change) and the measured phase is unwrapped around it
} chest_ul_proc_t;

/// Wraps a phase to (-pi, pi]
static inline float wrap_phase(float x)
{
  return x - 2.0f * (float)M_PI * roundf(x / (2.0f * (float)M_PI));
}

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

/* Uses the difference between the averaged and non-averaged pilot estimates */
static float estimate_noise_pilots(srsran_chest_ul_t*     q,
                                   cf_t*                  ce,
                                   uint32_t               nslots,
                                   uint32_t               nrefs,
                                   uint32_t               n_prb[2],
                                   const chest_ul_proc_t* proc)
{
  const float* filter      = proc->filter;
  uint32_t     filter_len  = proc->filter_len;
  bool         trunc_edges = proc->trunc_edges;

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

  float power = 0;
  for (int i = 0; i < nslots; i++) {
    power += srsran_chest_estimate_noise_pilots(
        &q->pilot_estimates[i * nrefs],
        &ce[SRSRAN_REFSIGNAL_UL_L(i, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE + n_prb[i] * SRSRAN_NRE],
        q->tmp_noise,
        nrefs);
  }

  power /= nslots;

  // The smoothing filter attenuates part of the noise, so the raw-minus-smoothed residual measures only a
  // fraction of it. Divide by the exact fraction for this filter and allocation width (band edges
  // included) to get an unbiased noise estimate.
  if (q->noise_bias_filter_len != filter_len || q->noise_bias_nrefs != nrefs || q->noise_bias_trunc != trunc_edges ||
      q->noise_bias_tap0 != filter[0] || !isnormal(q->noise_bias)) {
    q->noise_bias            = srsran_chest_estimate_noise_bias(filter, filter_len, nrefs, !trunc_edges);
    q->noise_bias_filter_len = filter_len;
    q->noise_bias_nrefs      = nrefs;
    q->noise_bias_trunc      = trunc_edges;
    q->noise_bias_tap0       = filter[0];
  }

  if (isnormal(q->noise_bias)) {
    return power / q->noise_bias;
  }
  return power;
}

// The interpolator currently only supports same frequency allocation for each subframe: cesymb() indexes
// the grid by n_prb[0] only, so the disabled DO_LINEAR_INTERPOLATION block below would read/write the
// wrong REs whenever the two slots hop to different PRBs. It must stay disabled unless made per-slot.
// The copy fallback below is per-slot correct.
#define cesymb(i) ce[SRSRAN_RE_IDX(q->cell.nof_prb, i, n_prb[0] * SRSRAN_NRE)]
static void interpolate_pilots(srsran_chest_ul_t* q, cf_t* ce, uint32_t nslots, uint32_t nrefs, uint32_t n_prb[2])
{
#ifdef DO_LINEAR_INTERPOLATION
  uint32_t L1 = SRSRAN_REFSIGNAL_UL_L(0, q->cell.cp);
  uint32_t L2 = SRSRAN_REFSIGNAL_UL_L(1, q->cell.cp);
  uint32_t NL = 2 * SRSRAN_CP_NSYMB(q->cell.cp);

  /* Interpolate in the time domain between symbols */
  srsran_interp_linear_vector3(
      &q->srsran_interp_linvec, &cesymb(L2), &cesymb(L1), &cesymb(L1), &cesymb(L1 - 1), (L2 - L1), L1, false, nrefs);
  srsran_interp_linear_vector3(
      &q->srsran_interp_linvec, &cesymb(L1), &cesymb(L2), NULL, &cesymb(L1 + 1), (L2 - L1), (L2 - L1) - 1, true, nrefs);
  srsran_interp_linear_vector3(&q->srsran_interp_linvec,
                               &cesymb(L1),
                               &cesymb(L2),
                               &cesymb(L2),
                               &cesymb(L2 + 1),
                               (L2 - L1),
                               (NL - L2) - 1,
                               true,
                               nrefs);
#else
  // Instead of a linear interpolation, we just copy the estimates to all symbols in that subframe
  for (int s = 0; s < nslots; s++) {
    for (int i = 0; i < SRSRAN_CP_NSYMB(q->cell.cp); i++) {
      int src_symb = SRSRAN_REFSIGNAL_UL_L(s, q->cell.cp);
      int dst_symb = i + s * SRSRAN_CP_NSYMB(q->cell.cp);

      // skip the symbol with the estimates
      if (dst_symb != src_symb) {
        srsran_vec_cf_copy(&ce[(dst_symb * q->cell.nof_prb + n_prb[s]) * SRSRAN_NRE],
                           &ce[(src_symb * q->cell.nof_prb + n_prb[s]) * SRSRAN_NRE],
                           nrefs);
      }
    }
  }
#endif
}

static void average_pilots(srsran_chest_ul_t*     q,
                           cf_t*                  input,
                           cf_t*                  ce,
                           uint32_t               nslots,
                           uint32_t               nrefs,
                           uint32_t               n_prb[2],
                           const chest_ul_proc_t* proc)
{
  for (uint32_t i = 0; i < nslots; i++) {
    cf_t* out = &ce[SRSRAN_REFSIGNAL_UL_L(i, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE + n_prb[i] * SRSRAN_NRE];
    if (proc->dft_half_win > 0) {
      // Delay-domain projection: any within-CP channel lives in the kept bins (TA de-rotation centered the
      // mean delay at bin 0), so it passes undistorted while the noise of the discarded bins is removed
      cf_t* delay = q->tmp_noise; // scratch, free at this point
      srsran_dft_run_c(&q->dft_fwd, &input[i * nrefs], delay);
      srsran_vec_cf_zero(&delay[proc->dft_half_win + 1], nrefs - 2 * proc->dft_half_win - 1);
      srsran_dft_run_c(&q->dft_bwd, delay, out);
    } else if (proc->trunc_edges) {
      srsran_chest_smooth_pilots_trunc(&input[i * nrefs], out, proc->filter, nrefs, proc->filter_len);
    } else {
      srsran_chest_average_pilots(&input[i * nrefs], out, (float*)proc->filter, nrefs, 1, proc->filter_len);
    }
  }
}

// SNR tier limits for the adaptive PUSCH smoothing filter (linear power)
#define PUSCH_SMOOTH_SNR_HIGH 20.0f // ~13 dB: 16QAM+ operating region, keep the legacy short filter
#define PUSCH_SMOOTH_SNR_LOW 3.16f  // ~5 dB: below this, smooth as much as the channel plausibly allows

// Delay-domain denoising engages for allocations of at least this many pilots (8 PRB): below it the
// kept-bin fraction is too large for the projection to beat the FIR tiers
#define PUSCH_DFT_MIN_NREFS 96
// The channel delay spread is physically bounded by the normal cyclic prefix
#define PUSCH_DFT_DELAY_SPAN_S 4.7e-6f
// Extra kept bins on each side, absorbing the sinc leakage of fractional-delay paths
#define PUSCH_DFT_MARGIN_BINS 3

// Physical bound on the automatic timing de-rotation (~2 CP): a slope estimate beyond it (e.g. captured by
// receiver clipping products) saturates instead of being applied. Larger real offsets need the hypothesis
// sweep (srsran_chest_ul_rank_pusch) rather than a blind de-rotation.
#define PUSCH_DEROT_CLAMP_US 9.4f
// With an externally supplied window center, the measured slope may only fine-tune within this many delay
// bins of it: the energy-verified center is authoritative, the phase measurement is not
#define PUSCH_WIN_RESID_BINS 2.0f
// Energy-capture guard: minimum fraction of pilot energy inside the projection window (also floored at
// twice the noise-only expectation keep/nrefs). SNR-independent on purpose, so a clipping-skewed SNR
// pre-estimate cannot weaken it. A miss means the window does not contain the channel (or the pilots are
// too distorted to tell) and the projection must yield to a short FIR.
#define PUSCH_DFT_GUARD_MIN_FRAC 0.25f
// Spread-prior window narrowing pays only in the noise-limited regime: the saved noise scales with N0
// while the fractional-delay sinc leakage the narrower window cuts off scales with the channel power.
// Measured on flat channels: a clear win at <= 3 dB, a wash by 5 dB - so it engages with the same
// threshold as the widest smoothing tier, keeping "smooth as hard as physics allows" one consistent regime
#define PUSCH_SPREAD_NARROW_SNR_MAX PUSCH_SMOOTH_SNR_LOW

// Delay bins per micro-second of a nrefs-point pilot DFT (15 kHz subcarrier spacing)
#define PUSCH_BINS_PER_US(nrefs) (15e3f * 1e-6f * (float)(nrefs))

/// Half-width of the delay window a within-CP channel can occupy after its mean delay is centered at 0
static uint32_t pusch_dft_half_win(uint32_t nrefs)
{
  return (uint32_t)ceilf(0.5f * PUSCH_DFT_DELAY_SPAN_S * 15e3f * (float)nrefs) + PUSCH_DFT_MARGIN_BINS;
}

/// Replans the delay-domain transforms for this allocation width (no-op when already planned)
static int pusch_dft_replan(srsran_chest_ul_t* q, uint32_t nrefs)
{
  if (q->dft_size != nrefs) {
    if (srsran_dft_replan_c(&q->dft_fwd, nrefs) || srsran_dft_replan_c(&q->dft_bwd, nrefs)) {
      ERROR("Error replanning delay-domain DFT to %d points", nrefs);
      return SRSRAN_ERROR;
    }
    q->dft_size = nrefs;
  }
  return SRSRAN_SUCCESS;
}

/// Accumulates the delay-power profile of the LS pilot estimates into q->delay_profile (both slots summed;
/// hopping-safe, since the delay support is common to the slots). Returns the total profile power.
static float pusch_delay_profile(srsran_chest_ul_t* q, uint32_t nrefs, uint32_t nslots)
{
  cf_t* delay = q->tmp_noise; // scratch, free at this point
  srsran_vec_f_zero(q->delay_profile, nrefs);
  float total = 0.0f;
  for (uint32_t i = 0; i < nslots; i++) {
    srsran_dft_run_c(&q->dft_fwd, &q->pilot_estimates[i * nrefs], delay);
    for (uint32_t d = 0; d < nrefs; d++) {
      float p = __real__ delay[d] * __real__ delay[d] + __imag__ delay[d] * __imag__ delay[d];
      q->delay_profile[d] += p;
      total += p;
    }
  }
  return total;
}

/// Sums the delay-power profile over a window of +/-half_win bins around the signed center bin s
static float pusch_profile_window_power(const float* profile, uint32_t nrefs, int s, uint32_t half_win)
{
  float sum = 0.0f;
  for (int d = s - (int)half_win; d <= s + (int)half_win; d++) {
    sum += profile[((d % (int)nrefs) + (int)nrefs) % (int)nrefs];
  }
  return sum;
}

// Cross-slot DMRS averaging engages only when the pilot phase drift over 0.5 ms stays below a threshold,
// i.e. the channel is time-flat (residual CFO below ~64 Hz and low Doppler). Since the combining aligns
// with the measured phase, the gate only needs to reject genuine channel changes, not measurement noise:
// its width grows with the expected phase-measurement standard deviation 1/sqrt(nrefs*snr) so that low-SNR
// subframes (where the noise reduction matters most) are not rejected by the gate's own noise.
#define PUSCH_CROSS_SLOT_MAX_PHASE_RAD 0.2f
#define PUSCH_CROSS_SLOT_PHASE_NSTD 2.5f
#define PUSCH_CROSS_SLOT_PHASE_CAP_RAD 1.0f

// Above this cross-slot phase the channel changes too much within the subframe for a linear model, and a
// pure CFO this large (>318 Hz residual) points at a synchronization problem rather than something to track
#define PUSCH_TIME_INTERP_MAX_PHASE_RAD 1.0f

/**
 * Writes every symbol of the subframe as a linear interpolation (extrapolation outside the DMRS pair) of
 * the two smoothed slot estimates, with the measured cross-slot phase applied as a linear phase ramp in
 * time. Exact for a pure-CFO channel, where plain complex interpolation would shrink the magnitude
 * between the pilots; reduces to plain linear interpolation as the phase tends to zero. Model: the
 * channel rotates from h0 at the first DMRS to h1 = h0*e^{-j*phase} at the second while its aligned
 * amplitude moves linearly, so h(t) = [(1-t)*h0 + t*h1*e^{+j*phase}] * e^{-j*phase*t}.
 * Only valid without frequency hopping (n_prb[0] == n_prb[1]).
 */
static void
pusch_interp_pilots(srsran_chest_ul_t* q, cf_t* ce, uint32_t nrefs, uint32_t n_prb[2], float cross_phase)
{
  uint32_t L1         = SRSRAN_REFSIGNAL_UL_L(0, q->cell.cp);
  uint32_t L2         = SRSRAN_REFSIGNAL_UL_L(1, q->cell.cp);
  uint32_t nsymb_slot = SRSRAN_CP_NSYMB(q->cell.cp);
  cf_t*    h0         = &ce[(L1 * q->cell.nof_prb + n_prb[0]) * SRSRAN_NRE];
  cf_t*    h1         = &ce[(L2 * q->cell.nof_prb + n_prb[1]) * SRSRAN_NRE];

  for (uint32_t l = 0; l < 2 * nsymb_slot; l++) {
    if (l == L1 || l == L2) {
      continue; // the DMRS symbols keep their own estimates (t=0 and t=1 reproduce them exactly)
    }
    float t = (float)((int)l - (int)L1) / (float)(L2 - L1);
    cf_t  a = (1.0f - t) * cexpf(-I * cross_phase * t);
    cf_t  b = t * cexpf(I * cross_phase * (1.0f - t));

    cf_t* row = &ce[(l * q->cell.nof_prb + n_prb[l / nsymb_slot]) * SRSRAN_NRE];
    srsran_vec_sc_prod_ccc(h0, a, row, nrefs);
    srsran_vec_sc_prod_ccc(h1, b, q->tmp_noise, nrefs); // scratch, free at this point
    srsran_vec_sum_ccc(row, q->tmp_noise, row, nrefs);
  }
}

/**
 * Selects the PUSCH frequency-domain smoothing filter for the current grant from a cheap,
 * filter-independent SNR pre-estimate. Second differences of the LS pilot estimates cancel any locally
 * linear channel, so their average power is 6*N0 plus a residual channel-curvature term that is negligible
 * at 15 kHz subcarrier spacing. At low SNR a longer Gaussian filter trades a little frequency resolution
 * for a large noise reduction, which is where small QPSK grants operate; at high SNR the legacy short
 * filter preserves frequency selectivity. The result is written to q->pusch_filter/pusch_filter_len and
 * the pre-estimated SNR to q->pusch_snr_prior.
 */
static void pusch_select_filter(srsran_chest_ul_t* q, uint32_t nrefs_sym, uint32_t nslots, float n0_prior)
{
  float n0_raw = 0.0f;
  if (n0_prior > 0.0f) {
    // A tracked per-UE noise power replaces the per-subframe pre-estimate: same tiers, but the selection
    // and the time-processing gates stop flapping with the estimate's own noise
    n0_raw = n0_prior;
  } else {
    // Raw noise pre-estimate from pilot second differences, averaged over both slots
    for (uint32_t s = 0; s < nslots; s++) {
      const cf_t* p = &q->pilot_estimates[s * nrefs_sym];
      cf_t*       d = q->tmp_noise; // free at this point, used as scratch
      srsran_vec_sum_ccc(&p[0], &p[2], d, nrefs_sym - 2);
      srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
      srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
      // E{|n[k-1] - 2n[k] + n[k+1]|^2} = 6*N0 for white noise
      n0_raw += srsran_vec_avg_power_cf(d, nrefs_sym - 2) / 6.0f / (float)nslots;
    }
  }

  float epre      = srsran_vec_avg_power_cf(q->pilot_recv_signal, nslots * nrefs_sym);
  float snr_prior = isnormal(n0_raw) ? (epre / n0_raw) : INFINITY;
  q->pusch_snr_prior = snr_prior;

  uint32_t len;
  if (snr_prior >= PUSCH_SMOOTH_SNR_HIGH) {
    len = 3;
  } else if (snr_prior >= PUSCH_SMOOTH_SNR_LOW) {
    len = 5;
  } else {
    // Scale with the allocation width but never wider than 9 taps (135 kHz), which stays below the
    // coherence bandwidth of even long-delay-spread channels
    len = (nrefs_sym / 3) | 1;
    len = SRSRAN_MAX(5, SRSRAN_MIN(9, len));
  }

  if (len != q->pusch_filter_len) {
    if (len == 3) {
      // Same taps as the legacy filter; only the band-edge handling differs
      q->pusch_filter_len = srsran_chest_set_smooth_filter3_coeff(q->pusch_filter, 0.3333f);
    } else {
      q->pusch_filter_len = srsran_chest_set_smooth_filter_gauss(q->pusch_filter, len - 1, (float)len / 4.0f);
    }
  }
}

/**
 * Generic PUSCH and DMRS channel estimation. It assumes q->pilot_estimates has been populated with the Least Square
 * Estimates
 *
 * @param q Uplink Channel estimation instance
 * @param nslots number of slots (2 for DMRS, 1 for SRS)
 * @param nrefs_sym number of reference resource elements per symbols (depends on configuration)
 * @param stride sub-carrier distance between reference signal resource elements (1 for DMRS, 2 for SRS)
 * @param meas_ta_en enables or disables the Time Alignment error measurement
 * @param write_estimates Write channel estimation in res, (true for DMRS and false for SRS)
 * @param n_prb Resource block start for the grant, set to zero for Sounding Reference Signals
 * @param proc frequency- and time-domain processing descriptor (see chest_ul_proc_t)
 * @param res UL channel estimation result
 */
static void chest_ul_estimate(srsran_chest_ul_t*     q,
                              uint32_t               nslots,
                              uint32_t               nrefs_sym,
                              uint32_t               stride,
                              bool                   meas_ta_en,
                              bool                   use_cedron_alg,
                              bool                   write_estimates,
                              uint32_t               n_prb[SRSRAN_NOF_SLOTS_PER_SF],
                              const chest_ul_proc_t* proc,
                              srsran_chest_ul_res_t* res)
{
  // Calculate CFO. With a tracked-CFO prior the raw (-pi, pi] measurement is unwrapped around the
  // predicted cross-slot phase, extending the usable CFO range beyond +/-1 kHz; without one
  // (cross_phase_ref = 0) this is the identity.
  float cross_phase = 0.0f;
  if (nslots == 2) {
    float raw   = cargf(srsran_vec_dot_prod_conj_ccc(
        &q->pilot_estimates[0 * nrefs_sym], &q->pilot_estimates[1 * nrefs_sym], nrefs_sym));
    cross_phase = proc->cross_phase_ref + wrap_phase(raw - proc->cross_phase_ref);
    res->cfo_hz = cross_phase / (2.0f * (float)M_PI * 0.0005f);
  } else {
    res->cfo_hz = NAN;
  }

  // Calculate time alignment error in micro-seconds
  if (meas_ta_en) {
    res->ta_us = pilot_slope_to_ta_us(measure_pilot_slope(q, nslots, nrefs_sym, use_cedron_alg), stride);
  } else {
    res->ta_us = 0.0f;
  }

  // With intra-subframe frequency hopping the two slots sit at different PRBs, so the cross-slot pilot
  // phase measured above contains the channel difference between both frequency blocks and is not a CFO
  bool hopping = (nslots == 2) && (n_prb[0] != n_prb[1]);
  if (hopping) {
    res->cfo_hz = NAN;
  }

  if (res->ce != NULL) {
    if (proc->filter_len > 0) {
      average_pilots(q, q->pilot_estimates, res->ce, nslots, nrefs_sym, n_prb, proc);

      // If averaging, compute noise from difference between received and averaged estimates. Do it before
      // any further processing of the DMRS estimates so the residual matches the modeled filter bias.
      res->noise_estimate = estimate_noise_pilots(q, res->ce, nslots, nrefs_sym, n_prb, proc);
    } else {
      // Copy estimates to CE vector without averaging
      for (int i = 0; i < nslots; i++) {
        srsran_vec_cf_copy(
            &res->ce[SRSRAN_REFSIGNAL_UL_L(i, q->cell.cp) * q->cell.nof_prb * SRSRAN_NRE + n_prb[i] * SRSRAN_NRE],
            &q->pilot_estimates[i * nrefs_sym],
            nrefs_sym);
      }
      res->noise_estimate = 0;
    }

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

    if (write_estimates) {
      // When the channel is time-flat (small cross-slot phase, which doubles as a Doppler/residual-CFO
      // detector) and both slots sit at the same PRBs, averaging the two DMRS estimates halves the
      // estimation noise for every symbol of the subframe. The cross-slot phase is preserved so each
      // slot keeps its own mean phase.
      bool combined = false;
      if (proc->cross_slot_max_phase > 0.0f && nslots == 2 && !hopping &&
          fabsf(cross_phase - proc->cross_phase_ref) < proc->cross_slot_max_phase) {
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
      if (!combined) {
        // At high SNR (where combining is not engaged) and moderate cross-slot phase, tracking the
        // channel linearly across the subframe beats holding each slot's estimate; otherwise fall back to
        // the per-slot hold (also the SRS and frequency-hopping path)
        if (proc->time_interp_max_phase > 0.0f && nslots == 2 && !hopping &&
            fabsf(cross_phase - proc->cross_phase_ref) < proc->time_interp_max_phase) {
          pusch_interp_pilots(q, res->ce, nrefs_sym, n_prb, cross_phase);
        } else {
          interpolate_pilots(q, res->ce, nslots, nrefs_sym, n_prb);
        }
      }
    }
  }

  // Measure reference signal RE average power
  cf_t  corr     = srsran_vec_acc_cc(q->pilot_recv_signal, nslots * nrefs_sym) / (nslots * nrefs_sym);
  float rsrp_avg = __real__ corr * __real__ corr + __imag__ corr * __imag__ corr;

  // Measure EPRE
  float epre = srsran_vec_avg_power_cf(q->pilot_recv_signal, nslots * nrefs_sym);

  // RSRP shall not be greater than EPRE
  rsrp_avg = SRSRAN_MIN(rsrp_avg, epre);

  // Calculate SNR
  if (isnormal(res->noise_estimate)) {
    res->snr = epre / res->noise_estimate;
  } else {
    res->snr = NAN;
  }

  // Set EPRE and RSRP
  res->epre                = epre;
  res->epre_dBfs           = srsran_convert_power_to_dB(res->epre);
  res->rsrp                = rsrp_avg;
  res->rsrp_dBfs           = srsran_convert_power_to_dB(res->rsrp);
  res->snr_db              = srsran_convert_power_to_dB(res->snr);
  res->noise_estimate_dbFs = srsran_convert_power_to_dBm(res->noise_estimate);
}

int srsran_chest_ul_estimate_pusch_prior(srsran_chest_ul_t*             q,
                                         srsran_ul_sf_cfg_t*            sf,
                                         srsran_pusch_cfg_t*            cfg,
                                         cf_t*                          input,
                                         const srsran_chest_ul_prior_t* prior,
                                         srsran_chest_ul_res_t*         res)
{
  if (!q->dmrs_signal_configured) {
    ERROR("Error must call srsran_chest_ul_set_cfg() before using the UL estimator");
    return SRSRAN_ERROR;
  }

  float window_delay_us = (prior != NULL && prior->delay_valid) ? prior->delay_us : NAN;

  uint32_t nof_prb = cfg->grant.L_prb;

  if (!srsran_dft_precoding_valid_prb(nof_prb)) {
    ERROR("Error invalid nof_prb=%d", nof_prb);
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  int nrefs_sym = nof_prb * SRSRAN_NRE;
  int nrefs_sf  = nrefs_sym * SRSRAN_NOF_SLOTS_PER_SF;

  /* Get references from the input signal */
  srsran_refsignal_dmrs_pusch_get(&q->dmrs_signal, cfg, input, q->pilot_recv_signal);

  // Use the known DMRS signal to compute Least-squares estimates
  srsran_vec_prod_conj_ccc(q->pilot_recv_signal,
                           q->dmrs_pregen.r[cfg->grant.n_dmrs][sf->tti % SRSRAN_NOF_SF_X_FRAME][nof_prb],
                           q->pilot_estimates,
                           nrefs_sf);

  chest_ul_proc_t proc = {
      .filter      = q->smooth_filter,
      .filter_len  = q->smooth_filter_len,
      .trunc_edges = false,
  };

  // Measure and remove the pilot phase slope (i.e. the timing offset) before any frequency-domain
  // processing: a residual timing error rotates the pilots by e^{-j*2pi*15kHz*ta*k}, which smoothing
  // filters average across (biasing the estimates low, worst for the long low-SNR filters on
  // pre-TA-convergence grants such as msg3) and which inflates the second-difference SNR pre-estimate.
  // De-rotation is unitary, so the noise statistics and the modeled filter bias are unchanged; the slope
  // is re-applied to the smoothed estimates inside chest_ul_estimate.
  // With an externally supplied window center (the winning timing hypothesis of a sweeping decoder, see
  // srsran_chest_ul_rank_pusch) the center is authoritative: the measured slope may only fine-tune within
  // +/-PUSCH_WIN_RESID_BINS delay bins of it, so a phase measurement corrupted by clipping or interference
  // can never move the processing away from where the energy was actually found. Without a center, the
  // measured slope is applied whole, bounded by the physical +/-PUSCH_DEROT_CLAMP_US.
  bool  meas_ta_en   = cfg->meas_ta_en;
  float derot_ta_us  = 0.0f;
  bool  win_centered = !isnan(window_delay_us);
  if (q->pusch_opts.ta_derotation || win_centered) {
    float fe = 0.0f;
    if (q->pusch_opts.ta_derotation) {
      fe = measure_pilot_slope(q, SRSRAN_NOF_SLOTS_PER_SF, nrefs_sym, cfg->use_cedron_alg);
    }
    if (win_centered) {
      float fe_center = PUSCH_BINS_PER_US(1) * window_delay_us; // normalized frequency per us times us
      fe_center       = SRSRAN_MAX(-0.45f, SRSRAN_MIN(0.45f, fe_center));
      float resid_max = PUSCH_WIN_RESID_BINS / (float)nrefs_sym;
      float resid     = SRSRAN_MAX(-resid_max, SRSRAN_MIN(resid_max, fe - fe_center));
      fe              = fe_center + resid;
    } else {
      float fe_max = PUSCH_BINS_PER_US(1) * PUSCH_DEROT_CLAMP_US;
      fe           = SRSRAN_MAX(-fe_max, SRSRAN_MIN(fe_max, fe));
    }
    if (isnormal(fe)) {
      // Pilots carry e^{-j*2pi*fe*k}: multiply by the conjugate ramp (srsran_vec_apply_cfo applies
      // e^{+j*2pi*cfo*n}). The slope is common to both slots - even under hopping only the constant
      // phase offset differs, and that folds into the per-slot channel estimate.
      for (uint32_t i = 0; i < SRSRAN_NOF_SLOTS_PER_SF; i++) {
        srsran_vec_apply_cfo(&q->pilot_estimates[i * nrefs_sym], fe, &q->pilot_estimates[i * nrefs_sym], nrefs_sym);
      }
      proc.derot_cfo = fe;
      // The applied slope IS the (residual) time alignment error; the flattened pilots would measure ~0,
      // so take over the TA measurement from chest_ul_estimate
      derot_ta_us = pilot_slope_to_ta_us(fe, 1);
      meas_ta_en  = false;
    }
  }

  // A tracked CFO turns the cross-slot gates into deviation tests around its predicted phase: a UE with a
  // stable frequency offset no longer loses cross-slot averaging or time interpolation to a rotation that
  // is perfectly predictable
  if (prior != NULL && prior->cfo_valid && !isnan(prior->cfo_hz)) {
    proc.cross_phase_ref = 2.0f * (float)M_PI * prior->cfo_hz * 0.0005f;
  }

  // Select the frequency-domain smoothing for this grant. PUCCH and SRS keep the shared legacy filter.
  if (q->pusch_opts.adaptive_smoothing || q->pusch_opts.cross_slot_avg || q->pusch_opts.time_interp) {
    pusch_select_filter(
        q, nrefs_sym, SRSRAN_NOF_SLOTS_PER_SF, (prior != NULL && prior->n0_valid) ? prior->n0 : 0.0f);
    if (q->pusch_opts.adaptive_smoothing) {
      proc.filter      = q->pusch_filter;
      proc.filter_len  = q->pusch_filter_len;
      proc.trunc_edges = true;
    }
    // Below the high tier the per-slot estimates are noisy and cross-slot averaging is worth more than
    // time tracking; at and above it, track the channel across the subframe instead (the <= 2.23x noise
    // amplification on the extrapolated edge symbols is cheap there, and the phase is measured accurately)
    if (q->pusch_opts.cross_slot_avg && (q->pusch_snr_prior < PUSCH_SMOOTH_SNR_HIGH)) {
      float phase_std           = 1.0f / sqrtf((float)nrefs_sym * SRSRAN_MAX(q->pusch_snr_prior, 0.01f));
      proc.cross_slot_max_phase = SRSRAN_MAX(
          PUSCH_CROSS_SLOT_MAX_PHASE_RAD,
          SRSRAN_MIN(PUSCH_CROSS_SLOT_PHASE_CAP_RAD, PUSCH_CROSS_SLOT_PHASE_NSTD * phase_std));
    } else if (q->pusch_opts.time_interp && (q->pusch_snr_prior >= PUSCH_SMOOTH_SNR_HIGH)) {
      proc.time_interp_max_phase = PUSCH_TIME_INTERP_MAX_PHASE_RAD;
    }
  }

  // Wide grants: project onto the delay bins a within-CP channel can occupy instead of FIR smoothing.
  // The mean delay was centered at bin 0 by the TA de-rotation, so a symmetric window of half the CP
  // (plus a leakage margin) covers the physical delay spread. The channel passes undistorted at any SNR,
  // so the projection replaces the FIR whenever it also removes more noise: its noise gain is the
  // kept-bin fraction, the FIR's is the squared norm of its taps.
  res->delay_energy_frac = NAN;
  res->delay_centroid_us = NAN;
  res->delay_spread_us   = NAN;
  if (q->pusch_opts.dft_denoise && nrefs_sym >= PUSCH_DFT_MIN_NREFS) {
    uint32_t half_win = pusch_dft_half_win(nrefs_sym);
    // A tracked delay spread narrows the window below the blind CP/2 bound: most UEs live on channels
    // far shorter than the CP, and every excluded bin is excluded noise. Widening above the physical
    // bound is never allowed, and the floor keeps fractional-delay leakage covered. A wrong (too narrow)
    // prior is caught by the energy guard below and the tracker widens fast on the reported spread.
    // Narrowing engages only in the noise-limited regime (see PUSCH_SPREAD_NARROW_SNR_MAX).
    if (prior != NULL && prior->spread_valid && prior->spread_us >= 0.0f &&
        q->pusch_snr_prior < PUSCH_SPREAD_NARROW_SNR_MAX) {
      uint32_t hw = (uint32_t)ceilf(PUSCH_BINS_PER_US(nrefs_sym) * 0.5f * prior->spread_us) + PUSCH_DFT_MARGIN_BINS;
      half_win    = SRSRAN_MAX(PUSCH_DFT_MARGIN_BINS + 2, SRSRAN_MIN(half_win, hw));
    }
    float fir_gain = 0.0f;
    for (uint32_t i = 0; i < proc.filter_len; i++) {
      fir_gain += proc.filter[i] * proc.filter[i];
    }
    if ((float)(2 * half_win + 1) < fir_gain * (float)nrefs_sym) {
      if (pusch_dft_replan(q, (uint32_t)nrefs_sym)) {
        return SRSRAN_ERROR;
      }
      proc.dft_half_win = half_win;

      // Energy-capture guard: after de-rotation/centering the channel must sit inside the projection
      // window. If it does not (timing beyond the de-rotation clamp, a wrong externally-supplied center,
      // clipping products spread across the delay axis), the projection would delete channel energy - the
      // one catastrophic failure mode of this pipeline - so it yields to the shortest FIR instead, which
      // degrades gracefully under the same conditions.
      float total = pusch_delay_profile(q, nrefs_sym, SRSRAN_NOF_SLOTS_PER_SF);
      if (isnormal(total)) {
        float win_power        = pusch_profile_window_power(q->delay_profile, nrefs_sym, 0, half_win);
        float frac             = win_power / total;
        res->delay_energy_frac = frac;

        // Delay centroid and spread of the (noise-floor subtracted) in-window profile, for per-UE
        // tracking. The floor is taken from the excluded bins, which contain only noise when the guard
        // passes; the centroid is reported absolute (residual plus the de-rotation already applied).
        uint32_t keep   = 2 * half_win + 1;
        float    nfloor = (nrefs_sym > (int)keep) ? (total - win_power) / (float)(nrefs_sym - keep) : 0.0f;
        float    wsum = 0.0f, c1 = 0.0f, c2 = 0.0f;
        for (int o = -(int)half_win; o <= (int)half_win; o++) {
          float p = q->delay_profile[((o % nrefs_sym) + nrefs_sym) % nrefs_sym] - nfloor;
          if (p > 0.0f) {
            wsum += p;
            c1 += p * (float)o;
            c2 += p * (float)o * (float)o;
          }
        }
        if (wsum > 0.0f) {
          float cent_bins        = c1 / wsum;
          float var_bins         = SRSRAN_MAX(0.0f, c2 / wsum - cent_bins * cent_bins);
          float applied_us       = proc.derot_cfo / PUSCH_BINS_PER_US(1); // slope already removed, in us
          res->delay_centroid_us = applied_us - cent_bins / PUSCH_BINS_PER_US(nrefs_sym);
          res->delay_spread_us   = 4.0f * sqrtf(var_bins) / PUSCH_BINS_PER_US(nrefs_sym);
        }

        float keep_frac = (float)keep / (float)nrefs_sym;
        if (frac < SRSRAN_MAX(PUSCH_DFT_GUARD_MIN_FRAC, 2.0f * keep_frac)) {
          proc.dft_half_win = 0;
          proc.filter       = q->smooth_filter; // legacy 3-tap taps: least-assumption fallback
          proc.filter_len   = q->smooth_filter_len;
          proc.trunc_edges  = true;
        }
      } else {
        // No measurable pilot energy: nothing for the projection to protect, use the FIR path
        proc.dft_half_win      = 0;
        res->delay_energy_frac = 0.0f;
      }
    }
  }

  // Estimate. Use the post-hopping PRB positions (n_prb_tilde): DMRS extraction above and the PUSCH
  // decoder both index the grid by them, and they differ from n_prb when PUSCH frequency hopping is active
  chest_ul_estimate(q,
                    SRSRAN_NOF_SLOTS_PER_SF,
                    nrefs_sym,
                    1,
                    meas_ta_en,
                    cfg->use_cedron_alg,
                    true,
                    cfg->grant.n_prb_tilde,
                    &proc,
                    res);

  // TA measured by the de-rotation stage (chest_ul_estimate saw already-flattened pilots)
  if (cfg->meas_ta_en && proc.derot_cfo != 0.0f) {
    res->ta_us = derot_ta_us;
  }

  return 0;
}

int srsran_chest_ul_estimate_pusch_win(srsran_chest_ul_t*     q,
                                       srsran_ul_sf_cfg_t*    sf,
                                       srsran_pusch_cfg_t*    cfg,
                                       cf_t*                  input,
                                       float                  window_delay_us,
                                       srsran_chest_ul_res_t* res)
{
  srsran_chest_ul_prior_t prior = {};
  prior.delay_valid             = !isnan(window_delay_us);
  prior.delay_us                = window_delay_us;
  return srsran_chest_ul_estimate_pusch_prior(q, sf, cfg, input, &prior, res);
}

int srsran_chest_ul_estimate_pusch(srsran_chest_ul_t*     q,
                                   srsran_ul_sf_cfg_t*    sf,
                                   srsran_pusch_cfg_t*    cfg,
                                   cf_t*                  input,
                                   srsran_chest_ul_res_t* res)
{
  return srsran_chest_ul_estimate_pusch_prior(q, sf, cfg, input, NULL, res);
}

int srsran_chest_ul_rank_pusch(srsran_chest_ul_t*      q,
                               srsran_ul_sf_cfg_t*     sf,
                               srsran_pusch_cfg_t*     cfg,
                               cf_t*                   input,
                               float                   max_delay_us,
                               srsran_chest_ul_rank_t* rank)
{
  if (q == NULL || sf == NULL || cfg == NULL || input == NULL || rank == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  if (!q->dmrs_signal_configured) {
    ERROR("Error must call srsran_chest_ul_set_cfg() before using the UL estimator");
    return SRSRAN_ERROR;
  }

  rank->energy_frac = 0.0f;
  rank->delay_us    = 0.0f;
  rank->epre        = 0.0f;
  rank->reliable    = false;

  uint32_t nof_prb = cfg->grant.L_prb;
  if (!srsran_dft_precoding_valid_prb(nof_prb)) {
    ERROR("Error invalid nof_prb=%d", nof_prb);
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  uint32_t nrefs_sym = nof_prb * SRSRAN_NRE;
  uint32_t nrefs_sf  = nrefs_sym * SRSRAN_NOF_SLOTS_PER_SF;

  // Same LS estimation as the full estimate (shares the pilot scratch buffers)
  srsran_refsignal_dmrs_pusch_get(&q->dmrs_signal, cfg, input, q->pilot_recv_signal);
  srsran_vec_prod_conj_ccc(q->pilot_recv_signal,
                           q->dmrs_pregen.r[cfg->grant.n_dmrs][sf->tti % SRSRAN_NOF_SF_X_FRAME][nof_prb],
                           q->pilot_estimates,
                           nrefs_sf);

  rank->epre = srsran_vec_avg_power_cf(q->pilot_recv_signal, nrefs_sf);

  if (pusch_dft_replan(q, nrefs_sym)) {
    return SRSRAN_ERROR;
  }

  // NOTE: deliberately no timing de-rotation here - the metric must stay sensitive to the timing
  // hypothesis being ranked. The raw delay-power profile carries the full +/-33 us unambiguous span.
  float total = pusch_delay_profile(q, nrefs_sym, SRSRAN_NOF_SLOTS_PER_SF);
  if (!isnormal(total)) {
    return SRSRAN_SUCCESS; // dead air: frac 0, not reliable
  }

  // The physical-channel window, clamped so it can never cover most of a narrow allocation's profile
  // (which would flatten the metric and make every hypothesis look good)
  uint32_t half_win = SRSRAN_MIN(pusch_dft_half_win(nrefs_sym), nrefs_sym / 6);
  if (2 * half_win + 1 >= nrefs_sym) {
    rank->energy_frac = 1.0f;
    return SRSRAN_SUCCESS; // degenerate: no discrimination possible, not reliable
  }

  // Search the window center over +/-span bins around delay 0. The caller-supplied bound keeps the search
  // from locking onto another UE's cyclic-shift delay peak or an alias, and preserves the discrimination
  // between the caller's coarse candidates.
  int span = (int)(nrefs_sym / 2 - half_win - 1);
  if (max_delay_us > 0.0f) {
    int req = (int)ceilf(PUSCH_BINS_PER_US(nrefs_sym) * max_delay_us);
    span    = SRSRAN_MAX(1, SRSRAN_MIN(span, req));
  }

  float best_sum = -1.0f;
  int   best_s   = 0;
  float sum      = pusch_profile_window_power(q->delay_profile, nrefs_sym, -span, half_win);
  for (int s = -span; s <= span; s++) {
    if (s > -span) {
      // Slide the window one bin: add the new leading bin, drop the old trailing one (circular)
      int lead  = ((s + (int)half_win) % (int)nrefs_sym + (int)nrefs_sym) % (int)nrefs_sym;
      int trail = ((s - (int)half_win - 1) % (int)nrefs_sym + (int)nrefs_sym) % (int)nrefs_sym;
      sum += q->delay_profile[lead] - q->delay_profile[trail];
    }
    if (sum > best_sum) {
      best_sum = sum;
      best_s   = s;
    }
  }

  rank->energy_frac = best_sum / total;

  // The captured-energy function is flat wherever the whole channel fits inside the window, so the argmax
  // alone can sit anywhere on that plateau. The power centroid within the winning window recovers the
  // channel's actual center delay with sub-bin resolution (noise pulls it toward the window center by a
  // negligible amount whenever the guard/ranking thresholds are met).
  float wsum = 0.0f;
  float csum = 0.0f;
  for (int o = -(int)half_win; o <= (int)half_win; o++) {
    int d = (((best_s + o) % (int)nrefs_sym) + (int)nrefs_sym) % (int)nrefs_sym;
    wsum += q->delay_profile[d];
    csum += q->delay_profile[d] * (float)(best_s + o);
  }
  float center_s = (wsum > 0.0f) ? (csum / wsum) : (float)best_s;
  center_s       = SRSRAN_MAX(-(float)span, SRSRAN_MIN((float)span, center_s));

  // A ramp e^{-j*2pi*fe*k} (late UE, fe > 0) peaks at signed bin -fe*nrefs, so delay and bin have
  // opposite signs; report with the ta_us convention (positive = late)
  rank->delay_us = -center_s / PUSCH_BINS_PER_US(nrefs_sym);
  rank->reliable = (nrefs_sym >= 24);

  return SRSRAN_SUCCESS;
}

void srsran_chest_ul_set_pusch_opts(srsran_chest_ul_t* q, const srsran_chest_ul_pusch_opts_t* opts)
{
  q->pusch_opts = *opts;
}

// Per-UE tracker tuning. Attack/decay asymmetries always err toward the safe direction: noise and spread
// grow fast (believing an interference burst or new multipath immediately) and shrink slowly (narrowing
// the window / trusting a low noise floor must be earned). Slew limits bound the damage of any single
// accepted measurement.
#define TRACK_N0_ALPHA_UP 0.5f
#define TRACK_N0_ALPHA_DN 0.05f
#define TRACK_CFO_ALPHA 0.2f
#define TRACK_CFO_SLEW_HZ 30.0f
#define TRACK_DELAY_ALPHA 0.25f
#define TRACK_DELAY_SLEW_US 0.3f
#define TRACK_SPREAD_ALPHA_UP 0.5f
#define TRACK_SPREAD_ALPHA_DN 0.05f
// Without a CRC pass, signal-dependent tracks only accept measurements at or above ~6 dB SNR (a DTX or a
// deeply faded subframe must not steer them)
#define TRACK_MIN_SNR_NO_CRC 4.0f
// Updates before the signal-dependent priors are handed out
#define TRACK_WARMUP 4
// Priors expire when the UE has not been measured for this long (TTIs)
#define TRACK_MAX_AGE_TTI 500

/// EMA with independent attack/decay coefficients
static inline float track_ema_asym(float ema, float meas, float a_up, float a_dn)
{
  float a = (meas > ema) ? a_up : a_dn;
  return ema + a * (meas - ema);
}

/// EMA with a bound on the applied innovation
static inline float track_ema_slew(float ema, float meas, float alpha, float slew)
{
  float inc = alpha * (meas - ema);
  return ema + SRSRAN_MAX(-slew, SRSRAN_MIN(slew, inc));
}

/// TTI distance respecting the 10240 wrap-around
static inline uint32_t track_tti_age(uint32_t now, uint32_t last)
{
  return (now + 10240 - last) % 10240;
}

void srsran_chest_ul_track_reset(srsran_chest_ul_track_t* t)
{
  bzero(t, sizeof(srsran_chest_ul_track_t));
}

void srsran_chest_ul_track_update(srsran_chest_ul_track_t*     t,
                                  const srsran_chest_ul_res_t* res,
                                  uint32_t                     tti,
                                  bool                         crc_ok)
{
  if (t == NULL || res == NULL) {
    return;
  }

  // A long silence means the state is stale (the UE may have moved, retuned or re-synchronized): start
  // over rather than slew-filtering toward a possibly distant new operating point
  if (t->init && track_tti_age(tti, t->last_tti) > TRACK_MAX_AGE_TTI) {
    srsran_chest_ul_track_reset(t);
  }

  // Noise power updates regardless of CRC/DTX: the residual-based estimate measures the floor either way.
  // Fast attack protects against under-estimating a starting interference burst.
  if (isnormal(res->noise_estimate) && res->noise_estimate > 0.0f) {
    t->n0_ema = t->init ? track_ema_asym(t->n0_ema, res->noise_estimate, TRACK_N0_ALPHA_UP, TRACK_N0_ALPHA_DN)
                        : res->noise_estimate;
    t->init     = true;
    t->last_tti = tti;
  }

  // Signal-dependent tracks: require a CRC pass, or a comfortably detected signal. A missed grant (DTX)
  // leaves noise-only "measurements" that would otherwise poison these.
  bool signal_ok = crc_ok || (isnormal(res->snr) && res->snr >= TRACK_MIN_SNR_NO_CRC);
  if (!signal_ok) {
    return;
  }

  if (!isnan(res->cfo_hz)) {
    t->cfo_hz_ema =
        (t->nof_meas > 0) ? track_ema_slew(t->cfo_hz_ema, res->cfo_hz, TRACK_CFO_ALPHA, TRACK_CFO_SLEW_HZ)
                          : res->cfo_hz;
  }
  if (!isnan(res->delay_centroid_us)) {
    t->delay_us_ema =
        (t->nof_meas > 0)
            ? track_ema_slew(t->delay_us_ema, res->delay_centroid_us, TRACK_DELAY_ALPHA, TRACK_DELAY_SLEW_US)
            : res->delay_centroid_us;
  }
  if (!isnan(res->delay_spread_us)) {
    t->spread_us_ema =
        (t->nof_meas > 0)
            ? track_ema_asym(t->spread_us_ema, res->delay_spread_us, TRACK_SPREAD_ALPHA_UP, TRACK_SPREAD_ALPHA_DN)
            : res->delay_spread_us;
  }

  t->nof_meas++;
  t->last_tti = tti;
}

void srsran_chest_ul_track_notify_delay_shift(srsran_chest_ul_track_t* t, float delta_us)
{
  if (t != NULL && t->nof_meas > 0) {
    t->delay_us_ema += delta_us;
  }
}

void srsran_chest_ul_track_get_prior(const srsran_chest_ul_track_t* t, uint32_t tti, srsran_chest_ul_prior_t* prior)
{
  if (prior == NULL) {
    return;
  }
  bzero(prior, sizeof(srsran_chest_ul_prior_t));
  if (t == NULL || !t->init || track_tti_age(tti, t->last_tti) > TRACK_MAX_AGE_TTI) {
    return;
  }

  if (isnormal(t->n0_ema) && t->n0_ema > 0.0f) {
    prior->n0_valid = true;
    prior->n0       = t->n0_ema;
  }
  if (t->nof_meas >= TRACK_WARMUP) {
    prior->cfo_valid    = true;
    prior->cfo_hz       = t->cfo_hz_ema;
    prior->delay_valid  = true;
    prior->delay_us     = t->delay_us_ema;
    prior->spread_valid = true;
    prior->spread_us    = t->spread_us_ema;
  }
}

static float
estimate_noise_pilots_pucch(srsran_chest_ul_t* q, cf_t* ce, uint32_t n_rs, uint32_t n_prb[SRSRAN_NOF_SLOTS_PER_SF])
{
  float power = 0;
  for (int ns = 0; ns < SRSRAN_NOF_SLOTS_PER_SF; ns++) {
    for (int i = 0; i < n_rs; i++) {
      // All CE are the same, so pick the first symbol of the first slot always and compare with the noisy estimates
      power += srsran_chest_estimate_noise_pilots(
          &q->pilot_estimates[(i + ns * n_rs) * SRSRAN_NRE],
          &ce[SRSRAN_RE_IDX(q->cell.nof_prb, ns * SRSRAN_CP_NSYMB(q->cell.cp), n_prb[ns] * SRSRAN_NRE)],
          q->tmp_noise,
          SRSRAN_NRE);
    }
  }

  power /= (SRSRAN_NOF_SLOTS_PER_SF * n_rs);

  if (q->smooth_filter_len == 3) {
    // Calibrated for filter length 3
    float w = q->smooth_filter[0];
    float a = 7.419 * w * w + 0.1117 * w - 0.005387;
    return (power / (a * 0.8));
  } else {
    return power;
  }
}

int srsran_chest_ul_estimate_pucch(srsran_chest_ul_t*     q,
                                   srsran_ul_sf_cfg_t*    sf,
                                   srsran_pucch_cfg_t*    cfg,
                                   cf_t*                  input,
                                   srsran_chest_ul_res_t* res)
{
  int n_rs = srsran_refsignal_dmrs_N_rs(cfg->format, q->cell.cp);
  if (!n_rs) {
    ERROR("Error computing N_rs");
    return SRSRAN_ERROR;
  }
  res->delay_energy_frac = NAN; // PUSCH-only metrics, avoid stale values in reused results
  res->delay_centroid_us = NAN;
  res->delay_spread_us   = NAN;
  int nrefs_sf = SRSRAN_NRE * n_rs * SRSRAN_NOF_SLOTS_PER_SF;

  /* Get references from the input signal */
  srsran_refsignal_dmrs_pucch_get(&q->dmrs_signal, cfg, input, q->pilot_recv_signal);

  /* Generate known pilots */
  if (cfg->format == SRSRAN_PUCCH_FORMAT_2A || cfg->format == SRSRAN_PUCCH_FORMAT_2B) {
    float max   = -1e9;
    int   i_max = 0;

    int m = 0;
    if (cfg->format == SRSRAN_PUCCH_FORMAT_2A) {
      m = 2;
    } else {
      m = 4;
    }

    for (int i = 0; i < m; i++) {
      cfg->pucch2_drs_bits[0] = i % 2;
      cfg->pucch2_drs_bits[1] = i / 2;
      srsran_refsignal_dmrs_pucch_gen(&q->dmrs_signal, sf, cfg, q->pilot_known_signal);
      srsran_vec_prod_conj_ccc(q->pilot_recv_signal, q->pilot_known_signal, q->pilot_estimates_tmp[i], nrefs_sf);
      float x = cabsf(srsran_vec_acc_cc(q->pilot_estimates_tmp[i], nrefs_sf));
      if (x >= max) {
        max   = x;
        i_max = i;
      }
    }
    memcpy(q->pilot_estimates, q->pilot_estimates_tmp[i_max], nrefs_sf * sizeof(cf_t));
    cfg->pucch2_drs_bits[0] = i_max % 2;
    cfg->pucch2_drs_bits[1] = i_max / 2;

  } else {
    srsran_refsignal_dmrs_pucch_gen(&q->dmrs_signal, sf, cfg, q->pilot_known_signal);
    /* Use the known DMRS signal to compute Least-squares estimates */
    srsran_vec_prod_conj_ccc(q->pilot_recv_signal, q->pilot_known_signal, q->pilot_estimates, nrefs_sf);
  }

  // Measure reference signal RE average power
  cf_t corr = srsran_vec_acc_cc(q->pilot_estimates, SRSRAN_NOF_SLOTS_PER_SF * SRSRAN_NRE * n_rs) /
              (SRSRAN_NOF_SLOTS_PER_SF * SRSRAN_NRE * n_rs);
  float rsrp_avg = __real__ corr * __real__ corr + __imag__ corr * __imag__ corr;

  // Measure EPRE
  float epre = srsran_vec_avg_power_cf(q->pilot_estimates, SRSRAN_NOF_SLOTS_PER_SF * SRSRAN_NRE * n_rs);

  // RSRP shall not be greater than EPRE
  rsrp_avg = SRSRAN_MIN(rsrp_avg, epre);

  // Set EPRE and RSRP
  res->epre      = epre;
  res->epre_dBfs = srsran_convert_power_to_dB(res->epre);
  res->rsrp      = rsrp_avg;
  res->rsrp_dBfs = srsran_convert_power_to_dB(res->rsrp);

  // Estimate time alignment
  if (cfg->meas_ta_en) {
    float ta_err = 0.0f;
    for (int ns = 0; ns < SRSRAN_NOF_SLOTS_PER_SF; ns++) {
      for (int i = 0; i < n_rs; i++) {
        if (cfg->use_cedron_alg) {
          ta_err += srsran_cedron_freq_estimate(
                        &q->srsran_cedron_freq_est, &q->pilot_estimates[(i + ns * n_rs) * SRSRAN_NRE], SRSRAN_NRE) /
                    (float)(SRSRAN_NOF_SLOTS_PER_SF * n_rs);
        } else {
          ta_err += srsran_vec_estimate_frequency(&q->pilot_estimates[(i + ns * n_rs) * SRSRAN_NRE], SRSRAN_NRE) /
                    (float)(SRSRAN_NOF_SLOTS_PER_SF * n_rs);
        }
      }
    }

    // Calculate actual time alignment error in micro-seconds
    if (isnormal(ta_err)) {
      ta_err /= 15e3f;                             // Convert from normalized frequency to seconds
      ta_err *= 1e6f;                              // Convert to micro-seconds
      ta_err     = roundf(ta_err * 10.0f) / 10.0f; // Round to one tenth of micro-second
      res->ta_us = ta_err;
    } else {
      res->ta_us = 0.0f;
    }
  }

  if (res->ce != NULL) {
    uint32_t n_prb[2] = {};

    /* TODO: Currently averaging entire slot, performance good enough? */
    for (int ns = 0; ns < 2; ns++) {
      // Average all slot
      for (int i = 1; i < n_rs; i++) {
        srsran_vec_sum_ccc(&q->pilot_estimates[ns * n_rs * SRSRAN_NRE],
                           &q->pilot_estimates[(i + ns * n_rs) * SRSRAN_NRE],
                           &q->pilot_estimates[ns * n_rs * SRSRAN_NRE],
                           SRSRAN_NRE);
      }
      srsran_vec_sc_prod_ccc(&q->pilot_estimates[ns * n_rs * SRSRAN_NRE],
                             (float)1.0 / n_rs,
                             &q->pilot_estimates[ns * n_rs * SRSRAN_NRE],
                             SRSRAN_NRE);

      // Average in freq domain
      srsran_chest_average_pilots(&q->pilot_estimates[ns * n_rs * SRSRAN_NRE],
                                  &q->pilot_recv_signal[ns * n_rs * SRSRAN_NRE],
                                  q->smooth_filter,
                                  SRSRAN_NRE,
                                  1,
                                  q->smooth_filter_len);

      // Determine n_prb
      n_prb[ns] = srsran_pucch_n_prb(&q->cell, cfg, ns);

      // copy estimates to slot
      for (int i = 0; i < SRSRAN_CP_NSYMB(q->cell.cp); i++) {
        srsran_vec_cf_copy(
            &res->ce[SRSRAN_RE_IDX(q->cell.nof_prb, i + ns * SRSRAN_CP_NSYMB(q->cell.cp), n_prb[ns] * SRSRAN_NRE)],
            &q->pilot_recv_signal[ns * n_rs * SRSRAN_NRE],
            SRSRAN_NRE);
      }
    }

    // Estimate noise/interference
    res->noise_estimate = estimate_noise_pilots_pucch(q, res->ce, n_rs, n_prb);
    if (fpclassify(res->noise_estimate) == FP_ZERO) {
      res->noise_estimate = FLT_MIN;
    }
    res->noise_estimate_dbFs = srsran_convert_power_to_dBm(res->noise_estimate);

    // Estimate SINR
    if (isnormal(res->noise_estimate)) {
      res->snr    = res->epre / res->noise_estimate;
      res->snr_db = srsran_convert_power_to_dB(res->snr);
    } else {
      res->snr    = NAN;
      res->snr_db = NAN;
    }
  }

  return 0;
}

int srsran_chest_ul_estimate_srs(srsran_chest_ul_t*                 q,
                                 srsran_ul_sf_cfg_t*                sf,
                                 srsran_refsignal_srs_cfg_t*        cfg,
                                 srsran_refsignal_dmrs_pusch_cfg_t* pusch_cfg,
                                 cf_t*                              input,
                                 srsran_chest_ul_res_t*             res)
{
  if (q == NULL || sf == NULL || cfg == NULL || pusch_cfg == NULL || input == NULL || res == NULL) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }

  // Extract parameters
  uint32_t n_srs_re = srsran_refsignal_srs_M_sc(&q->dmrs_signal, cfg);

  // Extract Sounding Reference Signal
  if (srsran_refsignal_srs_get(&q->dmrs_signal, cfg, sf->tti, q->pilot_recv_signal, input) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR;
  }

  // Get Known pilots
  cf_t* known_pilots = q->pilot_known_signal;
  if (q->srs_signal_configured) {
    known_pilots = q->srs_pregen.r[sf->tti % SRSRAN_NOF_SF_X_FRAME];
  } else {
    srsran_refsignal_srs_gen(&q->dmrs_signal, cfg, pusch_cfg, sf->tti % SRSRAN_NOF_SF_X_FRAME, known_pilots);
  }

  // Compute least squares estimates
  srsran_vec_prod_conj_ccc(q->pilot_recv_signal, known_pilots, q->pilot_estimates, n_srs_re);

  res->delay_energy_frac = NAN; // PUSCH-only metrics, avoid stale values in reused results
  res->delay_centroid_us = NAN;
  res->delay_spread_us   = NAN;

  // Estimate with the legacy processing: shared filter, extrapolated edges, no PUSCH-only features
  uint32_t        n_prb[2] = {};
  chest_ul_proc_t proc     = {
          .filter      = q->smooth_filter,
          .filter_len  = q->smooth_filter_len,
          .trunc_edges = false,
  };
  chest_ul_estimate(q, 1, n_srs_re, 1, true, false, false, n_prb, &proc, res);

  return SRSRAN_SUCCESS;
}
