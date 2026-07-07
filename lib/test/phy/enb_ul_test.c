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

/*
 * eNB uplink receive diversity test.
 *
 * A UE transmission (PUSCH or PUCCH) is fanned out onto N RX antennas through independent complex channel gains and
 * independent AWGN. The full eNB uplink chain (per-antenna FFT, channel estimation and MRC equalization) runs with 1
 * and 2 antennas and the test asserts that:
 *  - PUCCH is detected and decoded correctly with 1 and 2 RX antennas
 *  - PUSCH decodes at high SNR with 1 and 2 RX antennas, and the reported SNR gains ~3 dB with the second antenna
 *  - at an SNR where a single antenna mostly fails, two antennas decode reliably
 *  - MMSE-IRC: with a spatially-colored co-channel interferer IRC decodes where MRC fails; without interference
 *    IRC performs like MRC (no regression); with 1 antenna the IRC flag safely degrades to MRC
 */

#include <srsran/common/test_common.h>
#include <srsran/phy/utils/random.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "srsran/srsran.h"

static const srsran_cell_t test_cell = {
    25,                 // nof_prb
    1,                  // nof_ports
    1,                  // cell_id
    SRSRAN_CP_NORM,     // cyclic prefix
    SRSRAN_PHICH_NORM,  // PHICH length
    SRSRAN_PHICH_R_1_6, // PHICH resources
    SRSRAN_FDD,
};

// Unit-magnitude channel phases, one per RX antenna
static const float ant_phase[SRSRAN_MAX_PORTS] = {0.7f, -2.1f, 1.3f, -0.4f};

static int run_pusch_test(uint32_t  nof_rx_antennas,
                          uint32_t  mcs_idx,
                          float     snr_db,
                          uint32_t  nof_sf,
                          bool      irc_enable,
                          float     inr_db, // co-channel interferer power over noise; -INFINITY disables
                          uint32_t* nof_crc_ok,
                          float*    avg_snr_db)
{
  srsran_cell_t                     cell           = test_cell;
  srsran_refsignal_dmrs_pusch_cfg_t dmrs_pusch_cfg = {}; // Use default
  srsran_ue_ul_t                    ue_ul          = {};
  srsran_ue_ul_cfg_t                ue_ul_cfg      = {};
  srsran_enb_ul_t                   enb_ul         = {};
  srsran_ul_sf_cfg_t                ul_sf          = {};
  srsran_channel_awgn_t             awgn           = {};
  srsran_softbuffer_tx_t            softbuffer_tx  = {};
  srsran_softbuffer_rx_t            softbuffer_rx  = {};
  uint16_t                          rnti           = 0x1234;
  uint32_t                          sf_len         = SRSRAN_SF_LEN_PRB(cell.nof_prb);
  int                               ret            = SRSRAN_ERROR;

  srsran_random_t random_gen = srsran_random_init(0x1234);

  cf_t* tx_buffer                   = srsran_vec_cf_malloc(sf_len);
  cf_t* rx_buffer[SRSRAN_MAX_PORTS] = {};
  for (uint32_t a = 0; a < nof_rx_antennas; a++) {
    rx_buffer[a] = srsran_vec_cf_malloc(sf_len);
    TESTASSERT(rx_buffer[a]);
  }
  TESTASSERT(tx_buffer);

  TESTASSERT(!srsran_ue_ul_init(&ue_ul, tx_buffer, cell.nof_prb));
  TESTASSERT(!srsran_ue_ul_set_cell(&ue_ul, cell));

  TESTASSERT(!srsran_enb_ul_init(&enb_ul, rx_buffer, cell.nof_prb, nof_rx_antennas));
  TESTASSERT(!srsran_enb_ul_set_cell(&enb_ul, cell, &dmrs_pusch_cfg, NULL));
  srsran_enb_ul_set_irc(&enb_ul, irc_enable);

  TESTASSERT(!srsran_channel_awgn_init(&awgn, 0x5678));

  TESTASSERT(!srsran_softbuffer_tx_init(&softbuffer_tx, cell.nof_prb));
  TESTASSERT(!srsran_softbuffer_rx_init(&softbuffer_rx, cell.nof_prb));

  // Build the UL grant from a DCI: 20 PRB starting at PRB 0
  ue_ul_cfg.ul_cfg.pusch.rnti           = rnti;
  ue_ul_cfg.ul_cfg.pusch.softbuffers.tx = &softbuffer_tx;
  ue_ul_cfg.grant_available             = true;

  srsran_dci_ul_t dci = {};
  dci.freq_hop_fl     = SRSRAN_RA_PUSCH_HOP_DISABLED;
  dci.type2_alloc.riv = srsran_ra_type2_to_riv(20, 0, cell.nof_prb);
  dci.tb.mcs_idx      = mcs_idx;
  dci.tb.rv           = 0;
  TESTASSERT(!srsran_ue_ul_dci_to_pusch_grant(&ue_ul, &ul_sf, &ue_ul_cfg, &dci, &ue_ul_cfg.ul_cfg.pusch.grant));

  uint32_t tbs_bytes = (uint32_t)ue_ul_cfg.ul_cfg.pusch.grant.tb.tbs / 8;
  // Allocate some slack: the decoder writes the byte-aligned code block including the transport block CRC
  uint8_t* data    = srsran_vec_u8_malloc(tbs_bytes + 16);
  uint8_t* data_rx = srsran_vec_u8_malloc(tbs_bytes + 16);
  TESTASSERT(data && data_rx);

  *nof_crc_ok      = 0;
  float    snr_acc = 0.0f;
  uint32_t snr_cnt = 0;

  for (uint32_t sf = 0; sf < nof_sf; sf++) {
    ul_sf.tti = sf;

    srsran_softbuffer_tx_reset(&softbuffer_tx);
    srsran_softbuffer_rx_reset(&softbuffer_rx);

    for (uint32_t i = 0; i < tbs_bytes; i++) {
      data[i] = (uint8_t)srsran_random_uniform_int_dist(random_gen, 0, 255);
    }

    srsran_pusch_data_t pusch_data = {};
    pusch_data.ptr                 = data;
    TESTASSERT(srsran_ue_ul_encode(&ue_ul, &ul_sf, &ue_ul_cfg, &pusch_data) >= SRSRAN_SUCCESS);

    // Apply per-antenna channel gain and independent AWGN
    float n0_dBfs = srsran_convert_power_to_dB(srsran_vec_avg_power_cf(tx_buffer, sf_len)) - snr_db;
    TESTASSERT(!srsran_channel_awgn_set_n0(&awgn, n0_dBfs));
    for (uint32_t a = 0; a < nof_rx_antennas; a++) {
      srsran_vec_sc_prod_ccc(tx_buffer, cexpf(I * ant_phase[a]), rx_buffer[a], sf_len);
      srsran_channel_awgn_run_c(&awgn, rx_buffer[a], rx_buffer[a], sf_len);
    }

    // Add a rank-1 co-channel interferer: one white waveform with a fixed spatial signature
    // (different from the signal's), scaled to inr_db above the noise floor per antenna
    if (isfinite(inr_db)) {
      const cf_t interf_sig[SRSRAN_MAX_PORTS] = {1.0f, cexpf(I * 2.0f), cexpf(-I * 1.1f), cexpf(I * 0.4f)};
      float      interf_amp                   = powf(10.0f, (n0_dBfs + inr_db) / 20.0f);
      for (uint32_t i = 0; i < sf_len; i++) {
        cf_t v = srsran_random_gauss_dist(random_gen, 1.0f) + I * srsran_random_gauss_dist(random_gen, 1.0f);
        v      = v * (interf_amp * (float)M_SQRT1_2);
        for (uint32_t a = 0; a < nof_rx_antennas; a++) {
          rx_buffer[a][i] += interf_sig[a] * v;
        }
      }
    }

    // eNB receive chain
    srsran_enb_ul_fft(&enb_ul);

    srsran_pusch_cfg_t pusch_cfg = {};
    pusch_cfg.grant              = ue_ul_cfg.ul_cfg.pusch.grant;
    pusch_cfg.rnti               = rnti;
    pusch_cfg.softbuffers.rx     = &softbuffer_rx;
    pusch_cfg.max_nof_iterations = 10;
    pusch_cfg.meas_ta_en         = true;
    pusch_cfg.meas_epre_en       = true;

    srsran_pusch_res_t pusch_res = {};
    pusch_res.data               = data_rx;
    TESTASSERT(srsran_enb_ul_get_pusch(&enb_ul, &ul_sf, &pusch_cfg, &pusch_res) >= SRSRAN_SUCCESS);

    if (pusch_res.crc && memcmp(data, data_rx, tbs_bytes) == 0) {
      (*nof_crc_ok)++;
    }

    if (!isnan(enb_ul.chest_res.snr_db)) {
      snr_acc += enb_ul.chest_res.snr_db;
      snr_cnt++;
    }

    // TA shall be close to zero (no delay applied)
    TESTASSERT(fabsf(enb_ul.chest_res.ta_us) < 1.0f);
  }

  *avg_snr_db = (snr_cnt > 0) ? (snr_acc / snr_cnt) : NAN;

  ret = SRSRAN_SUCCESS;

  srsran_random_free(random_gen);
  srsran_channel_awgn_free(&awgn);
  srsran_softbuffer_tx_free(&softbuffer_tx);
  srsran_softbuffer_rx_free(&softbuffer_rx);
  srsran_ue_ul_free(&ue_ul);
  srsran_enb_ul_free(&enb_ul);
  free(tx_buffer);
  for (uint32_t a = 0; a < nof_rx_antennas; a++) {
    free(rx_buffer[a]);
  }
  free(data);
  free(data_rx);

  return ret;
}

static int run_pucch_test(uint32_t nof_rx_antennas, float snr_db)
{
  srsran_cell_t                     cell           = test_cell;
  srsran_refsignal_dmrs_pusch_cfg_t dmrs_pusch_cfg = {}; // Use default
  srsran_ue_ul_t                    ue_ul          = {};
  srsran_ue_ul_cfg_t                ue_ul_cfg      = {};
  srsran_enb_ul_t                   enb_ul         = {};
  srsran_ul_sf_cfg_t                ul_sf          = {};
  srsran_channel_awgn_t             awgn           = {};
  srsran_pucch_cfg_t                pucch_cfg      = {};
  uint16_t                          rnti           = 0x1234;
  uint32_t                          sf_len         = SRSRAN_SF_LEN_PRB(cell.nof_prb);

  // Basic PUCCH configuration: single carrier, one HARQ-ACK bit (format 1a)
  pucch_cfg.delta_pucch_shift              = 1;
  pucch_cfg.n_rb_2                         = 1;
  pucch_cfg.N_cs                           = 1;
  pucch_cfg.N_pucch_1                      = 1;
  pucch_cfg.ack_nack_feedback_mode         = SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL;
  pucch_cfg.uci_cfg.ack[0].grant_cc_idx    = 0;
  pucch_cfg.uci_cfg.ack[0].ncce[0]         = 1;
  pucch_cfg.uci_cfg.ack[0].nof_acks        = 1;
  pucch_cfg.rnti                           = rnti;
  pucch_cfg.threshold_data_valid_format1a  = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1A;

  cf_t* tx_buffer                   = srsran_vec_cf_malloc(sf_len);
  cf_t* rx_buffer[SRSRAN_MAX_PORTS] = {};
  for (uint32_t a = 0; a < nof_rx_antennas; a++) {
    rx_buffer[a] = srsran_vec_cf_malloc(sf_len);
    TESTASSERT(rx_buffer[a]);
  }
  TESTASSERT(tx_buffer);

  TESTASSERT(!srsran_ue_ul_init(&ue_ul, tx_buffer, cell.nof_prb));
  TESTASSERT(!srsran_ue_ul_set_cell(&ue_ul, cell));

  TESTASSERT(!srsran_enb_ul_init(&enb_ul, rx_buffer, cell.nof_prb, nof_rx_antennas));
  TESTASSERT(!srsran_enb_ul_set_cell(&enb_ul, cell, &dmrs_pusch_cfg, NULL));

  TESTASSERT(!srsran_channel_awgn_init(&awgn, 0x9abc));

  for (ul_sf.tti = 0; ul_sf.tti < 8; ul_sf.tti++) {
    srsran_pusch_data_t pusch_data  = {};
    pusch_data.uci.ack.valid        = true;
    pusch_data.uci.ack.ack_value[0] = ul_sf.tti & 1U;

    ue_ul_cfg.ul_cfg.pucch = pucch_cfg;

    TESTASSERT(srsran_ue_ul_encode(&ue_ul, &ul_sf, &ue_ul_cfg, &pusch_data) >= SRSRAN_SUCCESS);

    float n0_dBfs = srsran_convert_power_to_dB(srsran_vec_avg_power_cf(tx_buffer, sf_len)) - snr_db;
    TESTASSERT(!srsran_channel_awgn_set_n0(&awgn, n0_dBfs));
    for (uint32_t a = 0; a < nof_rx_antennas; a++) {
      srsran_vec_sc_prod_ccc(tx_buffer, cexpf(I * ant_phase[a]), rx_buffer[a], sf_len);
      srsran_channel_awgn_run_c(&awgn, rx_buffer[a], rx_buffer[a], sf_len);
    }

    srsran_enb_ul_fft(&enb_ul);

    srsran_pucch_res_t pucch_res = {};
    srsran_pucch_cfg_t cfg_rx    = pucch_cfg;
    TESTASSERT(!srsran_enb_ul_get_pucch(&enb_ul, &ul_sf, &cfg_rx, &pucch_res));

    TESTASSERT(pucch_res.detected);
    TESTASSERT(pucch_res.uci_data.ack.valid);
    TESTASSERT(pucch_res.uci_data.ack.ack_value[0] == pusch_data.uci.ack.ack_value[0]);
  }

  srsran_channel_awgn_free(&awgn);
  srsran_ue_ul_free(&ue_ul);
  srsran_enb_ul_free(&enb_ul);
  free(tx_buffer);
  for (uint32_t a = 0; a < nof_rx_antennas; a++) {
    free(rx_buffer[a]);
  }

  return SRSRAN_SUCCESS;
}

int main(int argc, char** argv)
{
  uint32_t crc_ok_1 = 0, crc_ok_2 = 0;
  float    snr_1 = 0.0f, snr_2 = 0.0f;

  // PUCCH decodes with 1 and 2 RX antennas
  TESTASSERT(!run_pucch_test(1, 10.0f));
  TESTASSERT(!run_pucch_test(2, 10.0f));

  // PUSCH at high SNR: both antenna configurations decode every subframe
  TESTASSERT(!run_pusch_test(1, 10, 30.0f, 4, false, -INFINITY, &crc_ok_1, &snr_1));
  TESTASSERT(!run_pusch_test(2, 10, 30.0f, 4, false, -INFINITY, &crc_ok_2, &snr_2));
  printf("high SNR: 1rx crc=%d/4 snr=%.1f dB; 2rx crc=%d/4 snr=%.1f dB\n", crc_ok_1, snr_1, crc_ok_2, snr_2);
  TESTASSERT(crc_ok_1 == 4);
  TESTASSERT(crc_ok_2 == 4);

  // The reported SNR shall gain ~3 dB from MRC over two equal-power branches
  TESTASSERT(snr_2 - snr_1 > 1.5f);
  TESTASSERT(snr_2 - snr_1 < 4.5f);

  // PUSCH at an SNR where a single antenna mostly fails: two antennas shall decode reliably
  TESTASSERT(!run_pusch_test(1, 16, 6.0f, 10, false, -INFINITY, &crc_ok_1, &snr_1));
  TESTASSERT(!run_pusch_test(2, 16, 6.0f, 10, false, -INFINITY, &crc_ok_2, &snr_2));
  printf("low SNR: 1rx crc=%d/10 snr=%.1f dB; 2rx crc=%d/10 snr=%.1f dB\n", crc_ok_1, snr_1, crc_ok_2, snr_2);
  TESTASSERT(crc_ok_2 >= 8);
  TESTASSERT(crc_ok_2 >= crc_ok_1 + 5);

  // --- MMSE-IRC ---

  // No regression without interference: IRC decodes like MRC at the same marginal SNR
  uint32_t crc_ok_mrc = 0, crc_ok_irc = 0;
  float    snr_mrc = 0.0f, snr_irc = 0.0f;
  TESTASSERT(!run_pusch_test(2, 16, 7.0f, 10, false, -INFINITY, &crc_ok_mrc, &snr_mrc));
  TESTASSERT(!run_pusch_test(2, 16, 7.0f, 10, true, -INFINITY, &crc_ok_irc, &snr_irc));
  printf("IRC parity (no interference): mrc crc=%d/10, irc crc=%d/10\n", crc_ok_mrc, crc_ok_irc);
  TESTASSERT(!isnan(snr_irc) && !isinf(snr_irc));
  TESTASSERT((int)crc_ok_irc >= (int)crc_ok_mrc - 1);

  // Directional interferer: MRC must mostly fail, IRC must decode reliably
  TESTASSERT(!run_pusch_test(2, 16, 20.0f, 10, false, 15.0f, &crc_ok_mrc, &snr_mrc));
  TESTASSERT(!run_pusch_test(2, 16, 20.0f, 10, true, 15.0f, &crc_ok_irc, &snr_irc));
  printf("IRC gain (INR 15 dB): mrc crc=%d/10, irc crc=%d/10\n", crc_ok_mrc, crc_ok_irc);
  TESTASSERT(crc_ok_irc >= 8);
  TESTASSERT(crc_ok_irc >= crc_ok_mrc + 5);

  // Flag with a single antenna safely degrades to MRC
  TESTASSERT(!run_pusch_test(1, 10, 30.0f, 4, true, -INFINITY, &crc_ok_irc, &snr_irc));
  printf("IRC 1rx fallback: crc=%d/4 snr=%.1f dB\n", crc_ok_irc, snr_irc);
  TESTASSERT(crc_ok_irc == 4);
  TESTASSERT(!isnan(snr_irc) && !isinf(snr_irc));

  printf("Ok\n");

  return SRSRAN_SUCCESS;
}
