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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include "srsran/srsran.h"

srsran_cell_t cell = {
    6,              // nof_prb
    1,              // nof_ports
    1000,           // cell_id
    SRSRAN_CP_NORM, // cyclic prefix
    SRSRAN_PHICH_NORM,
    SRSRAN_PHICH_R_1, // PHICH length
    SRSRAN_FDD,

};

char* output_matlab = NULL;

// Quality-mode options (enabled with -L)
static uint32_t L_prb            = 0;        // allocation width in PRB; 0 keeps legacy mode
static float    snr_db           = NAN;      // AWGN SNR; NAN disables noise
static bool     intra_sf_hop     = false;    // slot 1 at a different PRB offset than slot 0
static uint32_t nof_sf           = 100;      // subframes to average metrics over
static bool     selective_chan   = false;    // frequency-selective, time-varying channel
static float    cfo_hz           = 0.0f;     // carrier frequency offset emulated per symbol
static float    timing_off_us    = 0.0f;     // timing offset emulated as a phase ramp across frequency
static float    p2_delay_us      = 0.0f;     // second propagation path: delay in micro-seconds
static float    p2_rel_db        = NAN;      // second propagation path: relative attenuation in dB (NAN=off)
static float    win_center_us    = NAN;      // explicit window center for the centered-estimate API
static bool     sweep_mode       = false;    // rank a coarse timing sweep, then centered-estimate the winner
static bool     track_mode       = false;    // maintain a per-UE tracker across subframes and feed priors back
static bool     dtx_mode         = false;    // with -T: every 4th subframe the UE misses its grant (noise only)

// Coarse sweep emulation: 64-sample FFT-window steps at 15.36 Msps (10 MHz LTE), i.e. the user-style
// [-64, +512] window expressed in micro-seconds, covering late arrivals up to ~33 us
#define SWEEP_STEP_US (64.0f / 15.36f)
#define SWEEP_K_MIN (-1)
#define SWEEP_K_MAX (8)
static float    max_nmse         = INFINITY; // pass threshold: CE MSE / N0 (or /avg|h|^2 without noise)
static float    max_noise_err_db = INFINITY; // pass threshold: |noise estimate error| in dB

void usage(char* prog)
{
  printf("Usage: %s [recovLsHNSfME]\n", prog);

  printf("\t-r nof_prb [Default %d]\n", cell.nof_prb);
  printf("\t-e extended cyclic prefix [Default normal]\n");

  printf("\t-c cell_id (1000 tests all). [Default %d]\n", cell.id);

  printf("\t-L L_prb: enable quality mode with this allocation width [Default disabled]\n");
  printf("\t-s snr_db: add AWGN at this SNR (quality mode) [Default no noise]\n");
  printf("\t-H intra-subframe frequency hopping (quality mode) [Default disabled]\n");
  printf("\t-N nof_subframes to average metrics over (quality mode) [Default %d]\n", nof_sf);
  printf("\t-S frequency-selective time-varying channel (quality mode) [Default flat]\n");
  printf("\t-f cfo_hz: emulate CFO (quality mode) [Default 0]\n");
  printf("\t-t to_us: emulate timing offset in micro-seconds (quality mode) [Default 0]\n");
  printf("\t-P delay_us,rel_db: add a second propagation path (quality mode) [Default off]\n");
  printf("\t-w center_us: use the centered-window estimate API with this center (quality mode)\n");
  printf("\t-W emulate a coarse timing sweep: rank candidates, centered-estimate the winner\n");
  printf("\t-T per-UE tracking: maintain link priors across subframes and feed them back\n");
  printf("\t-D with -T: inject DTX (missed grant) every 4th subframe, must not poison the tracker\n");
  printf("\t-M max CE NMSE (linear, vs N0 with noise, vs channel power without) [Default no check]\n");
  printf("\t-E max noise estimate error in dB (quality mode) [Default no check]\n");

  printf("\t-o output matlab file [Default %s]\n", output_matlab ? output_matlab : "None");
  printf("\t-v increase verbosity\n");
}

void parse_args(int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "r:ec:o:L:s:HN:Sf:t:P:w:WTDM:E:v")) != -1) {
    switch (opt) {
      case 'r':
        cell.nof_prb = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'e':
        cell.cp = SRSRAN_CP_EXT;
        break;
      case 'c':
        cell.id = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'L':
        L_prb = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 's':
        snr_db = strtof(optarg, NULL);
        break;
      case 'H':
        intra_sf_hop = true;
        break;
      case 'N':
        nof_sf = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'S':
        selective_chan = true;
        break;
      case 'f':
        cfo_hz = strtof(optarg, NULL);
        break;
      case 't':
        timing_off_us = strtof(optarg, NULL);
        break;
      case 'P':
        if (sscanf(optarg, "%f,%f", &p2_delay_us, &p2_rel_db) != 2) {
          usage(argv[0]);
          exit(-1);
        }
        break;
      case 'w':
        win_center_us = strtof(optarg, NULL);
        break;
      case 'W':
        sweep_mode = true;
        break;
      case 'T':
        track_mode = true;
        break;
      case 'D':
        dtx_mode = true;
        break;
      case 'M':
        max_nmse = strtof(optarg, NULL);
        break;
      case 'E':
        max_noise_err_db = strtof(optarg, NULL);
        break;
      case 'o':
        output_matlab = optarg;
        break;
      case 'v':
        increase_srsran_verbose_level();
        break;
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
}

// Channel at OFDM symbol l (0..2*nsymb-1) and subcarrier k (absolute, 0..nof_prb*12-1). The flat channel is
// constant within the subframe; the selective one varies smoothly in both time and frequency. Both get an
// extra per-symbol phase ramp when a CFO is emulated.
static cf_t channel_gain(uint32_t l, uint32_t k, float sf_phase)
{
  cf_t h;
  if (selective_chan) {
    float x = -1.0f + (float)l / SRSRAN_CP_NSYMB(cell.cp) + cosf(2.0f * M_PI * (float)k / cell.nof_prb / SRSRAN_NRE);
    h       = (3.0f + x) / 3.0f * cexpf(I * x);
  } else {
    h = cexpf(I * sf_phase);
  }
  if (cfo_hz != 0.0f) {
    // One subframe (1 ms) spans 2*nsymb symbols; approximate each symbol as equally spaced in time
    float t = (float)l * 1e-3f / (2.0f * SRSRAN_CP_NSYMB(cell.cp));
    h *= cexpf(I * 2.0f * M_PI * cfo_hz * t);
  }
  if (!isnan(p2_rel_db)) {
    // Two-path channel: second time-invariant path at p2_delay_us with relative attenuation p2_rel_db and
    // a fixed phase offset; total power normalized to keep the SNR definition unchanged
    float a2 = powf(10.0f, -p2_rel_db / 20.0f);
    cf_t  p2 = a2 * cexpf(I * (1.234f - 2.0f * M_PI * 15e3f * p2_delay_us * 1e-6f * (float)k));
    h        = (h + h * p2) / sqrtf(1.0f + a2 * a2);
  }
  if (timing_off_us != 0.0f) {
    // A propagation delay of to_us rotates subcarrier k by e^{-j*2pi*15kHz*to*k} (late UE, positive TA)
    h *= cexpf(-I * 2.0f * M_PI * 15e3f * timing_off_us * 1e-6f * (float)k);
  }
  return h;
}

static int run_quality_mode(void)
{
  int                   ret       = -1;
  cf_t*                 input     = NULL;
  cf_t*                 h         = NULL;
  cf_t*                 r_dmrs    = NULL;
  cf_t*                 grid_cand = NULL; // sweep mode: candidate-compensated copy of the grid
  srsran_chest_ul_t     est    = {};
  srsran_chest_ul_res_t res    = {};
  bool                  chest_initiated = false;
  bool                  res_initiated   = false;

  uint32_t nsymb     = SRSRAN_CP_NSYMB(cell.cp);
  uint32_t sf_len_re = 2 * cell.nof_prb * SRSRAN_NRE * nsymb;

  cell.id = 1;

  if (L_prb == 0 || L_prb > cell.nof_prb || (intra_sf_hop && 2 * L_prb > cell.nof_prb)) {
    ERROR("Invalid L_prb=%d for cell.nof_prb=%d (hopping=%d)", L_prb, cell.nof_prb, intra_sf_hop);
    goto quality_exit;
  }
  if (track_mode && (sweep_mode || !isnan(win_center_us))) {
    ERROR("-T cannot be combined with -W or -w");
    goto quality_exit;
  }
  if (dtx_mode && (!track_mode || isnan(snr_db))) {
    ERROR("-D requires -T and -s");
    goto quality_exit;
  }
  if (!srsran_dft_precoding_valid_prb(L_prb)) {
    ERROR("Invalid L_prb=%d for PUSCH", L_prb);
    goto quality_exit;
  }

  input  = srsran_vec_cf_malloc(sf_len_re);
  h      = srsran_vec_cf_malloc(sf_len_re);
  r_dmrs = srsran_vec_cf_malloc(2 * L_prb * SRSRAN_NRE);
  if (sweep_mode) {
    grid_cand = srsran_vec_cf_malloc(sf_len_re);
  }
  if (!input || !h || !r_dmrs || (sweep_mode && !grid_cand)) {
    perror("srsran_vec_malloc");
    goto quality_exit;
  }

  if (srsran_chest_ul_init(&est, cell.nof_prb)) {
    ERROR("Error initializing UL estimator");
    goto quality_exit;
  }
  chest_initiated = true;
  if (srsran_chest_ul_res_init(&res, cell.nof_prb)) {
    ERROR("Error initializing UL estimation result");
    goto quality_exit;
  }
  res_initiated = true;
  if (srsran_chest_ul_set_cell(&est, cell)) {
    ERROR("Error setting cell");
    goto quality_exit;
  }

  srsran_refsignal_dmrs_pusch_cfg_t dmrs_cfg;
  ZERO_OBJECT(dmrs_cfg);
  srsran_chest_ul_pregen(&est, &dmrs_cfg, NULL);

  srsran_pusch_cfg_t cfg;
  ZERO_OBJECT(cfg);
  cfg.meas_ta_en   = true;
  cfg.grant.L_prb  = L_prb;
  cfg.grant.n_dmrs = 0;
  // Pre-hopping position is PRB 0 for both slots; with intra-subframe hopping the second slot actually
  // transmits at the top of the band, which only n_prb_tilde reflects (as ra_ul.c computes it)
  cfg.grant.n_prb[0]       = 0;
  cfg.grant.n_prb[1]       = 0;
  cfg.grant.n_prb_tilde[0] = 0;
  cfg.grant.n_prb_tilde[1] = intra_sf_hop ? (cell.nof_prb - L_prb) : 0;

  srand(0x1234);

  double mse_acc       = 0.0;
  double sig_pow_acc   = 0.0;
  double noise_est_acc = 0.0;
  double n0_acc        = 0.0;
  double ta_acc        = 0.0;
  uint64_t mse_count   = 0;
  uint32_t nof_sf_meas = 0; // subframes contributing to the metrics (DTX subframes are excluded)
  bool   cfo_valid_seen = false;
  float  n0_dtx         = 0.0f; // noise power of the last transmitted subframe, reused on DTX injections

  srsran_chest_ul_track_t track;
  srsran_chest_ul_track_reset(&track);

  for (uint32_t sf_run = 0; sf_run < nof_sf; sf_run++) {
    srsran_ul_sf_cfg_t ul_sf;
    ZERO_OBJECT(ul_sf);
    ul_sf.tti = sf_run % 10240;

    float sf_phase = 2.0f * M_PI * ((float)rand() / RAND_MAX - 0.5f);
    bool  dtx_sf   = dtx_mode && (sf_run % 4 == 3); // the UE misses every 4th grant

    // Build transmit grid: DMRS + unit-power QPSK data on the allocated REs (nothing on DTX)
    srsran_vec_cf_zero(input, sf_len_re);
    srsran_vec_cf_zero(h, sf_len_re);
    float sig_pow = 0.0f;
    if (!dtx_sf) {
      if (srsran_refsignal_dmrs_pusch_gen(
              &est.dmrs_signal, &dmrs_cfg, L_prb, ul_sf.tti % 10, cfg.grant.n_dmrs, r_dmrs)) {
        ERROR("Error generating DMRS");
        goto quality_exit;
      }
      srsran_refsignal_dmrs_pusch_put(&est.dmrs_signal, &cfg, r_dmrs, input);

      for (uint32_t l = 0; l < 2 * nsymb; l++) {
        uint32_t slot = l / nsymb;
        if (l == SRSRAN_REFSIGNAL_UL_L(slot, cell.cp)) {
          continue; // DMRS already placed
        }
        uint32_t k0 = cfg.grant.n_prb_tilde[slot] * SRSRAN_NRE;
        for (uint32_t k = 0; k < L_prb * SRSRAN_NRE; k++) {
          float re                                        = (rand() & 1) ? M_SQRT1_2 : -M_SQRT1_2;
          float im                                        = (rand() & 1) ? M_SQRT1_2 : -M_SQRT1_2;
          input[l * cell.nof_prb * SRSRAN_NRE + k0 + k] = re + I * im;
        }
      }

      // Apply channel on the allocated REs and store the true gains for comparison
      for (uint32_t l = 0; l < 2 * nsymb; l++) {
        uint32_t slot = l / nsymb;
        uint32_t k0   = cfg.grant.n_prb_tilde[slot] * SRSRAN_NRE;
        for (uint32_t k = 0; k < L_prb * SRSRAN_NRE; k++) {
          uint32_t idx = l * cell.nof_prb * SRSRAN_NRE + k0 + k;
          h[idx]       = channel_gain(l, k0 + k, sf_phase);
          input[idx] *= h[idx];
          sig_pow += __real__(h[idx] * conjf(h[idx]));
        }
      }
      sig_pow /= (float)(2 * nsymb * L_prb * SRSRAN_NRE);
    }

    // AWGN. DTX subframes carry the same noise floor as the last transmitted one.
    float n0 = 0.0f;
    if (!isnan(snr_db)) {
      n0 = dtx_sf ? n0_dtx : sig_pow / srsran_convert_dB_to_power(snr_db);
      srsran_ch_awgn_c(input, input, n0, sf_len_re);
      if (!dtx_sf) {
        n0_dtx = n0;
      }
    }

    // Estimate the channel. In sweep mode, first rank the coarse timing candidates like a
    // window-sweeping decoder would: compensate the grid by each candidate delay, rank it, and run the
    // centered-window estimate on the winner. comp_us is the winner's compensation, which becomes part of
    // the effective channel the estimate is compared against.
    float comp_us = 0.0f;
    if (sweep_mode) {
      float                  best_frac = -1.0f;
      srsran_chest_ul_rank_t best_rank = {};
      for (int ck = SWEEP_K_MIN; ck <= SWEEP_K_MAX; ck++) {
        float tau_k = (float)ck * SWEEP_STEP_US;
        // Compensating a delay of tau_k multiplies subcarrier n by e^{+j*2pi*15kHz*tau_k*n}
        for (uint32_t l = 0; l < 2 * nsymb; l++) {
          srsran_vec_apply_cfo(&input[l * cell.nof_prb * SRSRAN_NRE],
                               15e3f * 1e-6f * tau_k,
                               &grid_cand[l * cell.nof_prb * SRSRAN_NRE],
                               cell.nof_prb * SRSRAN_NRE);
        }
        srsran_chest_ul_rank_t rank = {};
        if (srsran_chest_ul_rank_pusch(&est, &ul_sf, &cfg, grid_cand, SWEEP_STEP_US / 2.0f + 2.35f, &rank)) {
          ERROR("Error ranking timing hypothesis");
          goto quality_exit;
        }
        if (rank.energy_frac > best_frac) {
          best_frac = rank.energy_frac;
          best_rank = rank;
          comp_us   = tau_k;
        }
      }
      // Rebuild the winner's grid and run the estimate with the window centered on the ranked delay
      for (uint32_t l = 0; l < 2 * nsymb; l++) {
        srsran_vec_apply_cfo(&input[l * cell.nof_prb * SRSRAN_NRE],
                             15e3f * 1e-6f * comp_us,
                             &grid_cand[l * cell.nof_prb * SRSRAN_NRE],
                             cell.nof_prb * SRSRAN_NRE);
      }
      if (srsran_chest_ul_estimate_pusch_win(&est, &ul_sf, &cfg, grid_cand, best_rank.delay_us, &res)) {
        ERROR("Error running centered channel estimation");
        goto quality_exit;
      }
    } else if (track_mode) {
      // Per-UE tracking loop: priors in, measurements out, DTX subframes flagged as CRC failures
      srsran_chest_ul_prior_t prior;
      srsran_chest_ul_track_get_prior(&track, ul_sf.tti, &prior);
      if (srsran_chest_ul_estimate_pusch_prior(&est, &ul_sf, &cfg, input, &prior, &res)) {
        ERROR("Error running tracked channel estimation");
        goto quality_exit;
      }
      srsran_chest_ul_track_update(&track, &res, ul_sf.tti, !dtx_sf);
    } else if (!isnan(win_center_us)) {
      if (srsran_chest_ul_estimate_pusch_win(&est, &ul_sf, &cfg, input, win_center_us, &res)) {
        ERROR("Error running centered channel estimation");
        goto quality_exit;
      }
    } else if (srsran_chest_ul_estimate_pusch(&est, &ul_sf, &cfg, input, &res)) {
      ERROR("Error running channel estimation");
      goto quality_exit;
    }

    if (dtx_sf) {
      continue; // nothing transmitted: no metrics to accumulate, the tracker must simply survive it
    }

    // Accumulate CE error over the allocated REs of both slots. In sweep mode the winner's compensation
    // ramp is part of the effective channel the estimator saw.
    for (uint32_t l = 0; l < 2 * nsymb; l++) {
      uint32_t slot = l / nsymb;
      uint32_t k0   = cfg.grant.n_prb_tilde[slot] * SRSRAN_NRE;
      for (uint32_t k = 0; k < L_prb * SRSRAN_NRE; k++) {
        uint32_t idx  = l * cell.nof_prb * SRSRAN_NRE + k0 + k;
        cf_t     href = h[idx];
        if (comp_us != 0.0f) {
          href *= cexpf(I * 2.0f * M_PI * 15e3f * 1e-6f * comp_us * (float)(k0 + k));
        }
        cf_t err = res.ce[idx] - href;
        mse_acc += __real__(err * conjf(err));
        mse_count++;
      }
    }
    sig_pow_acc += sig_pow;
    noise_est_acc += res.noise_estimate;
    n0_acc += n0;
    // In sweep mode the recovered timing is the coarse candidate plus the estimator's residual TA
    ta_acc += (double)comp_us + res.ta_us;
    nof_sf_meas++;
    if (!isnan(res.cfo_hz)) {
      cfo_valid_seen = true;
    }
  }

  if (mse_count == 0 || nof_sf_meas == 0) {
    ERROR("No subframes measured");
    goto quality_exit;
  }
  float mse       = (float)(mse_acc / mse_count);
  float sig_pow   = (float)(sig_pow_acc / nof_sf_meas);
  float n0        = (float)(n0_acc / nof_sf_meas);
  float noise_est = (float)(noise_est_acc / nof_sf_meas);

  // Without noise, normalize the CE error by the channel power instead of N0
  float nmse         = isnan(snr_db) ? (mse / sig_pow) : (mse / n0);
  float noise_err_db = 10.0f * log10f(noise_est / n0);

  float ta_avg = (float)(ta_acc / nof_sf_meas);

  printf("L_prb=%d snr_db=%.1f hop=%d selective=%d cfo=%.0f to_us=%.1f nof_sf=%d\n",
         L_prb,
         snr_db,
         intra_sf_hop,
         selective_chan,
         cfo_hz,
         timing_off_us,
         nof_sf);
  printf("  CE MSE: %.6f, NMSE: %.4f (%.2f dB)%s\n",
         mse,
         nmse,
         10.0f * log10f(nmse),
         isnan(snr_db) ? " [vs channel power]" : " [vs N0; raw LS would be 1.0]");
  if (!isnan(snr_db)) {
    printf("  Noise: true %.6f, estimated %.6f, error %+.2f dB\n", n0, noise_est, noise_err_db);
  }
  if (timing_off_us != 0.0f) {
    printf("  TA: true %.2f us, estimated %.2f us\n", timing_off_us, ta_avg);
  }

  if (nmse > max_nmse) {
    ERROR("CE NMSE %.4f exceeds threshold %.4f", nmse, max_nmse);
    goto quality_exit;
  }
  if (!isnan(snr_db) && fabsf(noise_err_db) > max_noise_err_db) {
    ERROR("Noise estimate error %.2f dB exceeds threshold %.2f dB", noise_err_db, max_noise_err_db);
    goto quality_exit;
  }
  // With a timing-offset stimulus the recovered timing (coarse candidate + residual TA in sweep mode)
  // must match it. Skipped with an explicit -w center: a deliberately wrong center caps the reported TA.
  if (timing_off_us != 0.0f && isnan(win_center_us) && fabsf(ta_avg - timing_off_us) > 0.2f) {
    ERROR("TA estimate %.2f us deviates from the true %.2f us offset", ta_avg, timing_off_us);
    goto quality_exit;
  }
  // The cross-slot CFO estimate is meaningless when the two slots sit at different frequencies; the
  // estimator must flag it as invalid
  if (intra_sf_hop && cfo_valid_seen) {
    ERROR("CFO estimate must be invalid (NAN) under intra-subframe frequency hopping");
    goto quality_exit;
  }

  ret = 0;

quality_exit:
  if (chest_initiated) {
    srsran_chest_ul_free(&est);
  }
  if (res_initiated) {
    srsran_chest_ul_res_free(&res);
  }
  if (input) {
    free(input);
  }
  if (h) {
    free(h);
  }
  if (r_dmrs) {
    free(r_dmrs);
  }
  if (grid_cand) {
    free(grid_cand);
  }
  printf("%s\n", ret ? "Error" : "OK");
  return ret;
}

int main(int argc, char** argv)
{
  srsran_chest_ul_t est;
  cf_t *            input = NULL, *ce = NULL, *h = NULL;
  int               i, j, n_port = 0, sf_idx = 0, cid = 0;
  int               ret = -1;
  int               max_cid;
  FILE*             fmatlab = NULL;

  parse_args(argc, argv);

  if (L_prb > 0) {
    exit(run_quality_mode());
  }

  if (output_matlab) {
    fmatlab = fopen(output_matlab, "w");
    if (!fmatlab) {
      perror("fopen");
      goto do_exit;
    }
  }

  uint32_t num_re = 2U * cell.nof_prb * SRSRAN_NRE * SRSRAN_CP_NSYMB(cell.cp);

  input = srsran_vec_cf_malloc(num_re);
  if (!input) {
    perror("srsran_vec_malloc");
    goto do_exit;
  }
  h = srsran_vec_cf_malloc(num_re);
  if (!h) {
    perror("srsran_vec_malloc");
    goto do_exit;
  }
  ce = srsran_vec_cf_malloc(num_re);
  if (!ce) {
    perror("srsran_vec_malloc");
    goto do_exit;
  }
  srsran_vec_cf_zero(ce, num_re);

  if (cell.id == 1000) {
    cid     = 0;
    max_cid = 504;
  } else {
    cid     = cell.id;
    max_cid = cell.id;
  }
  printf("max_cid=%d, cid=%d, cell.id=%d\n", max_cid, cid, cell.id);
  if (srsran_chest_ul_init(&est, cell.nof_prb)) {
    ERROR("Error initializing equalizer");
    goto do_exit;
  }
  while (cid <= max_cid) {
    cell.id = cid;
    if (srsran_chest_ul_set_cell(&est, cell)) {
      ERROR("Error initializing equalizer");
      goto do_exit;
    }

    for (int n = 6; n <= cell.nof_prb; n += 5) {
      if (srsran_dft_precoding_valid_prb(n)) {
        for (int delta_ss = 29; delta_ss < SRSRAN_NOF_DELTA_SS; delta_ss++) {
          for (int cshift = 7; cshift < SRSRAN_NOF_CSHIFT; cshift++) {
            for (int t = 2; t < 3; t++) {
              /* Setup and pregen DMRS reference signals */
              srsran_refsignal_dmrs_pusch_cfg_t pusch_cfg;

              uint32_t nof_prb         = n;
              pusch_cfg.cyclic_shift   = cshift;
              pusch_cfg.delta_ss       = delta_ss;
              bool group_hopping_en    = false;
              bool sequence_hopping_en = false;

              if (!t) {
                group_hopping_en    = false;
                sequence_hopping_en = false;
              } else if (t == 1) {
                group_hopping_en    = false;
                sequence_hopping_en = true;
              } else if (t == 2) {
                group_hopping_en    = true;
                sequence_hopping_en = false;
              }
              pusch_cfg.group_hopping_en    = group_hopping_en;
              pusch_cfg.sequence_hopping_en = sequence_hopping_en;
              srsran_chest_ul_pregen(&est, &pusch_cfg, NULL);

              // Loop through subframe idx and cyclic shifts

              for (sf_idx = 0; sf_idx < 10; sf_idx += 3) {
                for (int cshift_dmrs = 0; cshift_dmrs < SRSRAN_NOF_CSHIFT; cshift_dmrs += 5) {
                  if (SRSRAN_VERBOSE_ISINFO()) {
                    printf("nof_prb: %d, ", nof_prb);
                    printf("cyclic_shift: %d, ", pusch_cfg.cyclic_shift);
                    printf("cyclic_shift_for_dmrs: %d, ", cshift_dmrs);
                    printf("delta_ss: %d, ", pusch_cfg.delta_ss);
                    printf("SF_idx: %d\n", sf_idx);
                  }

                  /* Generate random input */
                  srsran_vec_cf_zero(input, num_re);
                  for (i = 0; i < num_re; i++) {
                    input[i] = 0.5 - rand() / (float)RAND_MAX + I * (0.5 - rand() / (float)RAND_MAX);
                  }

                  /* Generate channel and pass input through channel */
                  for (i = 0; i < 2 * SRSRAN_CP_NSYMB(cell.cp); i++) {
                    for (j = 0; j < cell.nof_prb * SRSRAN_NRE; j++) {
                      float x = -1 + (float)i / SRSRAN_CP_NSYMB(cell.cp) +
                                cosf(2 * M_PI * (float)j / cell.nof_prb / SRSRAN_NRE);
                      h[i * cell.nof_prb * SRSRAN_NRE + j] = (3 + x) * cexpf(I * x);
                      input[i * cell.nof_prb * SRSRAN_NRE + j] *= h[i * cell.nof_prb * SRSRAN_NRE + j];
                    }
                  }

                  // Configure estimator
                  srsran_chest_ul_res_t res;
                  srsran_pusch_cfg_t    cfg;

                  ZERO_OBJECT(cfg);
                  res.ce                   = ce;
                  cfg.grant.L_prb          = n;
                  cfg.grant.n_prb_tilde[0] = 0;
                  cfg.grant.n_prb_tilde[1] = 0;
                  cfg.grant.n_dmrs         = cshift_dmrs;

                  srsran_ul_sf_cfg_t ul_sf;
                  ZERO_OBJECT(ul_sf);
                  ul_sf.tti = sf_idx;

                  // Estimate channel
                  srsran_chest_ul_estimate_pusch(&est, &ul_sf, &cfg, input, &res);

                  // Compute MSE
                  float mse = 0;
                  for (i = 0; i < num_re; i++) {
                    mse += cabsf(ce[i] - h[i]);
                  }
                  mse /= num_re;
                  INFO("MSE: %f", mse);
                  if (mse > 4) {
                    goto do_exit;
                  }
                }
              }
            }
          }
        }
      }
    }
    cid += 10;
    printf("cid=%d\n", cid);
  }

  srsran_chest_ul_free(&est);

  if (fmatlab) {
    fprintf(fmatlab, "input=");
    srsran_vec_fprint_c(fmatlab, input, num_re);
    fprintf(fmatlab, ";\n");
    fprintf(fmatlab, "h=");
    srsran_vec_fprint_c(fmatlab, h, num_re);
    fprintf(fmatlab, ";\n");
    fprintf(fmatlab, "ce=");
    srsran_vec_fprint_c(fmatlab, ce, num_re);
    fprintf(fmatlab, ";\n");
  }

  ret = 0;

do_exit:

  if (ce) {
    free(ce);
  }
  if (input) {
    free(input);
  }
  if (h) {
    free(h);
  }

  if (!ret) {
    printf("OK\n");
  } else {
    printf("Error at cid=%d, slot=%d, port=%d\n", cid, sf_idx, n_port);
  }

  exit(ret);
}
