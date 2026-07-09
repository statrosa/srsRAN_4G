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

/**********************************************************************************************
 *  File:         chest_ul.h
 *
 *  Description:  3GPP LTE Uplink channel estimator and equalizer.
 *                Estimates the channel in the resource elements transmitting references and
 *                interpolates for the rest of the resource grid.
 *                The equalizer uses the channel estimates to produce an estimation of the
 *                transmitted symbol.
 *
 *  Reference:
 *********************************************************************************************/

#ifndef SRSRAN_CHEST_UL_H
#define SRSRAN_CHEST_UL_H

#include <stdio.h>

#include "srsran/config.h"

#include "srsran/phy/ch_estimation/cedron_freq_estimator.h"
#include "srsran/phy/ch_estimation/chest_common.h"
#include "srsran/phy/ch_estimation/refsignal_ul.h"
#include "srsran/phy/dft/dft.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/pucch_cfg.h"
#include "srsran/phy/phch/pusch_cfg.h"
#include "srsran/phy/resampling/interp.h"

typedef struct SRSRAN_API {
  cf_t*    ce;
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbFs;
  float    rsrp;
  float    rsrp_dBfs;
  float    epre;
  float    epre_dBfs;
  float    snr;
  float    snr_db;
  float    cfo_hz;
  float    ta_us;
} srsran_chest_ul_res_t;

/**
 * PUSCH-specific estimation options, all enabled by default. PUCCH and SRS estimation are unaffected by
 * every one of them.
 */
typedef struct SRSRAN_API {
  /// Select the frequency-smoothing filter per grant from the allocation width and a per-subframe SNR
  /// pre-estimate (with truncated band edges) instead of the fixed legacy 3-tap filter
  bool adaptive_smoothing;
  /// Average the two DMRS estimates (phase-aligned) when the channel is time-flat, the SNR is low and the
  /// slots are not frequency hopped
  bool cross_slot_avg;
  /// Measure the pilot phase slope (the timing offset) and flatten it before any frequency-domain
  /// processing, re-applying it to the final estimates. Keeps long smoothing filters unbiased and the SNR
  /// pre-estimate clean when the UE timing is not yet converged (e.g. msg3)
  bool ta_derotation;
  /// For wide grants, replace the FIR smoother with a delay-domain projection onto the bins a within-CP
  /// channel can occupy: large noise reduction with no penalty on frequency-selective channels
  bool dft_denoise;
  /// At high SNR, linearly interpolate/extrapolate the two smoothed slot estimates across the subframe
  /// symbols (with the cross-slot phase applied as a linear phase ramp in time) instead of holding each
  /// slot's estimate: tracks moderate CFO/Doppler that the per-slot hold cannot
  bool time_interp;
} srsran_chest_ul_pusch_opts_t;

typedef struct {
  srsran_cell_t cell;

  srsran_refsignal_ul_t             dmrs_signal;
  srsran_refsignal_ul_dmrs_pregen_t dmrs_pregen;
  bool                              dmrs_signal_configured;

  srsran_refsignal_srs_pregen_t srs_pregen;
  bool                          srs_signal_configured;

  cf_t* pilot_estimates;
  cf_t* pilot_estimates_tmp[4];
  cf_t* pilot_recv_signal;
  cf_t* pilot_known_signal;
  cf_t* tmp_noise;

#ifdef FREQ_SEL_SNR
  float snr_vector[12000];
  float pilot_power[12000];
#endif
  uint32_t smooth_filter_len;
  float    smooth_filter[SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN];

  // PUSCH-only estimation options and adaptive smoothing state; PUCCH and SRS keep using smooth_filter
  srsran_chest_ul_pusch_opts_t pusch_opts;
  float                        pusch_filter[SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN];
  uint32_t                     pusch_filter_len;
  float                        pusch_snr_prior;

  // Delay-domain denoising transforms for wide PUSCH grants; planned once at the maximum width and
  // replanned per allocation width
  srsran_dft_plan_t dft_fwd;
  srsran_dft_plan_t dft_bwd;
  uint32_t          dft_size;

  // Cached noise bias of the smoothing operator, recomputed when the filter or allocation width changes
  float    noise_bias;
  uint32_t noise_bias_filter_len;
  uint32_t noise_bias_nrefs;
  bool     noise_bias_trunc;
  float    noise_bias_tap0;

  srsran_interp_linsrsran_vec_t srsran_interp_linvec;

  srsran_cedron_freq_est_t srsran_cedron_freq_est;
} srsran_chest_ul_t;

SRSRAN_API int srsran_chest_ul_init(srsran_chest_ul_t* q, uint32_t max_prb);

SRSRAN_API void srsran_chest_ul_free(srsran_chest_ul_t* q);

SRSRAN_API int srsran_chest_ul_res_init(srsran_chest_ul_res_t* q, uint32_t max_prb);

SRSRAN_API void srsran_chest_ul_res_set_identity(srsran_chest_ul_res_t* q);

SRSRAN_API void srsran_chest_ul_res_free(srsran_chest_ul_res_t* q);

SRSRAN_API int srsran_chest_ul_set_cell(srsran_chest_ul_t* q, srsran_cell_t cell);

SRSRAN_API void srsran_chest_ul_pregen(srsran_chest_ul_t*                 q,
                                       srsran_refsignal_dmrs_pusch_cfg_t* cfg,
                                       srsran_refsignal_srs_cfg_t*        srs_cfg);

/**
 * Configures the PUSCH-specific estimation improvements (see srsran_chest_ul_pusch_opts_t; all enabled by
 * default). PUCCH and SRS estimation are unaffected either way.
 */
SRSRAN_API void srsran_chest_ul_set_pusch_opts(srsran_chest_ul_t* q, const srsran_chest_ul_pusch_opts_t* opts);

SRSRAN_API int srsran_chest_ul_estimate_pusch(srsran_chest_ul_t*     q,
                                              srsran_ul_sf_cfg_t*    sf,
                                              srsran_pusch_cfg_t*    cfg,
                                              cf_t*                  input,
                                              srsran_chest_ul_res_t* res);

SRSRAN_API int srsran_chest_ul_estimate_pucch(srsran_chest_ul_t*     q,
                                              srsran_ul_sf_cfg_t*    sf,
                                              srsran_pucch_cfg_t*    cfg,
                                              cf_t*                  input,
                                              srsran_chest_ul_res_t* res);

SRSRAN_API int srsran_chest_ul_estimate_srs(srsran_chest_ul_t*                 q,
                                            srsran_ul_sf_cfg_t*                sf,
                                            srsran_refsignal_srs_cfg_t*        cfg,
                                            srsran_refsignal_dmrs_pusch_cfg_t* pusch_cfg,
                                            cf_t*                              input,
                                            srsran_chest_ul_res_t*             res);

#endif // SRSRAN_CHEST_UL_H
