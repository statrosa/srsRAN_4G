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
  float        derot_cfo;            // pilot phase slope removed before smoothing, in normalized frequency
                                     // units (cycles/sample); re-applied to the estimates (0 disables)
  float        cross_slot_max_phase; // cross-slot averaging gate in radians (0 disables)
} chest_ul_proc_t;

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
    if (proc->trunc_edges) {
      srsran_chest_smooth_pilots_trunc(&input[i * nrefs], out, proc->filter, nrefs, proc->filter_len);
    } else {
      srsran_chest_average_pilots(&input[i * nrefs], out, (float*)proc->filter, nrefs, 1, proc->filter_len);
    }
  }
}

// SNR tier limits for the adaptive PUSCH smoothing filter (linear power)
#define PUSCH_SMOOTH_SNR_HIGH 20.0f // ~13 dB: 16QAM+ operating region, keep the legacy short filter
#define PUSCH_SMOOTH_SNR_LOW 3.16f  // ~5 dB: below this, smooth as much as the channel plausibly allows

// Cross-slot DMRS averaging engages only when the pilot phase drift over 0.5 ms stays below a threshold,
// i.e. the channel is time-flat (residual CFO below ~64 Hz and low Doppler). Since the combining aligns
// with the measured phase, the gate only needs to reject genuine channel changes, not measurement noise:
// its width grows with the expected phase-measurement standard deviation 1/sqrt(nrefs*snr) so that low-SNR
// subframes (where the noise reduction matters most) are not rejected by the gate's own noise.
#define PUSCH_CROSS_SLOT_MAX_PHASE_RAD 0.2f
#define PUSCH_CROSS_SLOT_PHASE_NSTD 2.5f
#define PUSCH_CROSS_SLOT_PHASE_CAP_RAD 1.0f

/**
 * Selects the PUSCH frequency-domain smoothing filter for the current grant from a cheap,
 * filter-independent SNR pre-estimate. Second differences of the LS pilot estimates cancel any locally
 * linear channel, so their average power is 6*N0 plus a residual channel-curvature term that is negligible
 * at 15 kHz subcarrier spacing. At low SNR a longer Gaussian filter trades a little frequency resolution
 * for a large noise reduction, which is where small QPSK grants operate; at high SNR the legacy short
 * filter preserves frequency selectivity. The result is written to q->pusch_filter/pusch_filter_len and
 * the pre-estimated SNR to q->pusch_snr_prior.
 */
static void pusch_select_filter(srsran_chest_ul_t* q, uint32_t nrefs_sym, uint32_t nslots)
{
  // Raw noise pre-estimate from pilot second differences, averaged over both slots
  float n0_raw = 0.0f;
  for (uint32_t s = 0; s < nslots; s++) {
    const cf_t* p = &q->pilot_estimates[s * nrefs_sym];
    cf_t*       d = q->tmp_noise; // free at this point, used as scratch
    srsran_vec_sum_ccc(&p[0], &p[2], d, nrefs_sym - 2);
    srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
    srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
    // E{|n[k-1] - 2n[k] + n[k+1]|^2} = 6*N0 for white noise
    n0_raw += srsran_vec_avg_power_cf(d, nrefs_sym - 2) / 6.0f / (float)nslots;
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
  // Calculate CFO
  float cross_phase = 0.0f;
  if (nslots == 2) {
    cross_phase = cargf(srsran_vec_dot_prod_conj_ccc(
        &q->pilot_estimates[0 * nrefs_sym], &q->pilot_estimates[1 * nrefs_sym], nrefs_sym));
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
      if (!combined) {
        interpolate_pilots(q, res->ce, nslots, nrefs_sym, n_prb);
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

int srsran_chest_ul_estimate_pusch(srsran_chest_ul_t*     q,
                                   srsran_ul_sf_cfg_t*    sf,
                                   srsran_pusch_cfg_t*    cfg,
                                   cf_t*                  input,
                                   srsran_chest_ul_res_t* res)
{
  if (!q->dmrs_signal_configured) {
    ERROR("Error must call srsran_chest_ul_set_cfg() before using the UL estimator");
    return SRSRAN_ERROR;
  }

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

  // Select the frequency-domain smoothing for this grant. PUCCH and SRS keep the shared legacy filter.
  if (q->pusch_opts.adaptive_smoothing || q->pusch_opts.cross_slot_avg) {
    pusch_select_filter(q, nrefs_sym, SRSRAN_NOF_SLOTS_PER_SF);
    if (q->pusch_opts.adaptive_smoothing) {
      proc.filter      = q->pusch_filter;
      proc.filter_len  = q->pusch_filter_len;
      proc.trunc_edges = true;
    }
    // At high SNR the per-slot estimates are already accurate and time tracking is worth more than the
    // remaining noise reduction
    if (q->pusch_opts.cross_slot_avg && (q->pusch_snr_prior < PUSCH_SMOOTH_SNR_HIGH)) {
      float phase_std           = 1.0f / sqrtf((float)nrefs_sym * SRSRAN_MAX(q->pusch_snr_prior, 0.01f));
      proc.cross_slot_max_phase = SRSRAN_MAX(
          PUSCH_CROSS_SLOT_MAX_PHASE_RAD,
          SRSRAN_MIN(PUSCH_CROSS_SLOT_PHASE_CAP_RAD, PUSCH_CROSS_SLOT_PHASE_NSTD * phase_std));
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

void srsran_chest_ul_set_pusch_opts(srsran_chest_ul_t* q, const srsran_chest_ul_pusch_opts_t* opts)
{
  q->pusch_opts = *opts;
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
