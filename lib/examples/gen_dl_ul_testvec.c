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

/******************************************************************************
 *  File:         gen_dl_ul_testvec.c
 *
 *  Description:  Deterministic DL+UL IQ test-vector generator for
 *                dl_ul_capture_align. It uses srsRAN's real PHY (the same DSP
 *                srsENB/srsUE use) to emit two byte-aligned IQ files:
 *
 *                  dl.iq : full DL frames (PSS/SSS/CRS/PBCH/PCFICH via
 *                          srsran_enb_dl) with a PDCCH format-0 UL DCI for a
 *                          chosen C-RNTI in every subframe.
 *                  ul.iq : the matching PUSCH (srsran_ue_ul) at subframe n+4
 *                          for each DL grant; zeros elsewhere.
 *
 *                Because file-mode replay is subframe-sequential and skew-free,
 *                a grant the tool finds in DL subframe n is answered by the
 *                PUSCH placed at UL subframe n+4. Running
 *                  dl_ul_capture_align --dl-file dl.iq --ul-file ul.iq \
 *                     -c <cell_id> -p <nof_prb> -r <rnti>
 *                must then decode the PUSCH with CRC=OK. This validates the
 *                tool's decode path end-to-end with no radios and no core
 *                network, and doubles as a fast regression vector.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "srsran/srsran.h"

#define FDD_UL_DELAY_MS 4 // FDD: format-0 grant in subframe n -> PUSCH in subframe n+4

#define MAX_RNTIS 32

static uint32_t cell_id       = 1;
static uint32_t nof_prb       = 6;
static uint16_t rntis[MAX_RNTIS] = {0x46};
static uint32_t nof_rntis     = 1;
static uint32_t mcs_idx       = 10;
static int      L_rb          = -1; // default: full band
static uint32_t nof_subframes = 200;
static char*    dl_file       = NULL;
static char*    ul_file       = NULL;

// Parse a comma-separated hex C-RNTI list (e.g. "0x46,0x47,0x48").
static void parse_rnti_list(char* s)
{
  nof_rntis = 0;
  for (char* tok = strtok(s, ","); tok && nof_rntis < MAX_RNTIS; tok = strtok(NULL, ",")) {
    rntis[nof_rntis++] = (uint16_t)strtol(tok, NULL, 16);
  }
  if (nof_rntis == 0) {
    nof_rntis = 1;
  }
}

static void usage(char* prog)
{
  printf("Usage: %s -o dl.iq -O ul.iq [options]\n", prog);
  printf("\t-c cell_id [Default %d]\n", cell_id);
  printf("\t-p nof_prb [Default %d]\n", nof_prb);
  printf("\t-r Comma-separated C-RNTI list in hex, round-robin per subframe [Default 0x46]\n");
  printf("\t-m PUSCH MCS index [Default %d]\n", mcs_idx);
  printf("\t-L L_rb (PUSCH nof PRB) [Default full band]\n");
  printf("\t-n nof_subframes [Default %d]\n", nof_subframes);
  printf("\t-o DL output file\n");
  printf("\t-O UL output file\n");
}

static void parse_args(int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "c:p:r:m:L:n:o:O:h")) != -1) {
    switch (opt) {
      case 'c':
        cell_id = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'p':
        nof_prb = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'r':
        parse_rnti_list(optarg);
        break;
      case 'm':
        mcs_idx = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'L':
        L_rb = (int)strtol(optarg, NULL, 10);
        break;
      case 'n':
        nof_subframes = (uint32_t)strtol(optarg, NULL, 10);
        break;
      case 'o':
        dl_file = optarg;
        break;
      case 'O':
        ul_file = optarg;
        break;
      default:
        usage(argv[0]);
        exit(0);
    }
  }
  if (!dl_file || !ul_file) {
    usage(argv[0]);
    exit(-1);
  }
}

int main(int argc, char** argv)
{
  parse_args(argc, argv);

  if (L_rb < 0) {
    // Default to a valid, decodable non-full-band allocation (4 PRB when the
    // cell is wide enough, else 2). Full-band PUSCH is rejected by the encoder.
    L_rb = (nof_prb >= 4) ? 4 : 2;
  }

  srsran_cell_t cell = {};
  cell.nof_prb         = nof_prb;
  cell.nof_ports       = 1;
  cell.id              = cell_id;
  cell.cp              = SRSRAN_CP_NORM;
  cell.phich_length    = SRSRAN_PHICH_NORM;
  cell.phich_resources = SRSRAN_PHICH_R_1;
  cell.frame_type      = SRSRAN_FDD;

  uint32_t sf_len = SRSRAN_SF_LEN_PRB(nof_prb);

  // ---- DL transmitter (srsran_enb_dl) --------------------------------------
  cf_t*           dl_out[SRSRAN_MAX_PORTS] = {NULL};
  dl_out[0]                                = srsran_vec_cf_malloc(sf_len);
  srsran_enb_dl_t enb_dl                   = {};
  if (srsran_enb_dl_init(&enb_dl, dl_out, nof_prb)) {
    ERROR("Error initialising enb_dl");
    exit(-1);
  }
  if (srsran_enb_dl_set_cell(&enb_dl, cell)) {
    ERROR("Error setting cell on enb_dl");
    exit(-1);
  }

  // ---- UL transmitter (srsran_ue_ul) ---------------------------------------
  cf_t*          ul_out = srsran_vec_cf_malloc(sf_len);
  srsran_ue_ul_t ue_ul  = {};
  if (srsran_ue_ul_init(&ue_ul, ul_out, nof_prb)) {
    ERROR("Error initialising ue_ul");
    exit(-1);
  }
  if (srsran_ue_ul_set_cell(&ue_ul, cell)) {
    ERROR("Error setting cell on ue_ul");
    exit(-1);
  }

  srsran_ue_ul_cfg_t ue_ul_cfg = {};
  ue_ul_cfg.ul_cfg.pusch.rnti  = rntis[0];
  // DMRS defaults match sib.conf.example ul_rs and dl_ul_capture_align defaults.
  ue_ul_cfg.ul_cfg.dmrs.cyclic_shift        = 0;
  ue_ul_cfg.ul_cfg.dmrs.delta_ss            = 0;
  ue_ul_cfg.ul_cfg.dmrs.group_hopping_en    = false;
  ue_ul_cfg.ul_cfg.dmrs.sequence_hopping_en = false;
  ue_ul_cfg.ul_cfg.hopping.n_sb             = 1;
  ue_ul_cfg.ul_cfg.hopping.hopping_enabled  = false;
  ue_ul_cfg.grant_available                 = true;
  // No pregen: srsran_ue_ul_encode() generates the PUSCH DMRS on the fly.

  srsran_softbuffer_tx_t softbuffer = {};
  srsran_softbuffer_tx_init(&softbuffer, nof_prb);

  // Constant UL grant carried by the format-0 DCI in every DL subframe (the RNTI
  // is set per subframe below, cycling through the configured C-RNTI list).
  srsran_dci_ul_t dci_ul  = {};
  dci_ul.freq_hop_fl      = SRSRAN_RA_PUSCH_HOP_DISABLED;
  dci_ul.type2_alloc.riv  = srsran_ra_type2_to_riv((uint32_t)L_rb, 0, nof_prb);
  dci_ul.tb.mcs_idx       = mcs_idx;
  dci_ul.tb.rv            = 0;
  dci_ul.tb.ndi           = false;

  srsran_dci_cfg_t dci_cfg = {};

  // Output files and scratch buffers.
  srsran_filesink_t dl_sink = {}, ul_sink = {};
  srsran_filesink_init(&dl_sink, dl_file, SRSRAN_COMPLEX_FLOAT_BIN);
  srsran_filesink_init(&ul_sink, ul_file, SRSRAN_COMPLEX_FLOAT_BIN);
  cf_t*    zeros = srsran_vec_cf_malloc(sf_len);
  srsran_vec_cf_zero(zeros, sf_len);
  uint8_t* data = srsran_vec_u8_malloc(2000 * 8);
  for (int i = 0; i < 2000 * 8; i++) {
    data[i] = (uint8_t)(i & 0xff);
  }

  uint32_t nof_grants = 0, nof_pusch = 0;
  for (uint32_t i = 0; i < nof_subframes; i++) {
    // ---- DL subframe: base signals + a format-0 UL DCI on the PDCCH --------
    srsran_dl_sf_cfg_t dl_sf = {};
    dl_sf.tti                = i;
    dl_sf.cfi                = 3;
    dl_sf.sf_type            = SRSRAN_SF_NORM;

    srsran_enb_dl_put_base(&enb_dl, &dl_sf);

    // This subframe's grant targets one C-RNTI from the list (round-robin).
    uint16_t cur_rnti = rntis[i % nof_rntis];
    dci_ul.rnti       = cur_rnti;

    // Place the DCI at that C-RNTI's UE search space so find_ul_dci recovers it.
    srsran_dci_location_t locs[SRSRAN_MAX_CANDIDATES_UE] = {};
    uint32_t nloc = srsran_pdcch_ue_locations(&enb_dl.pdcch, &enb_dl.dl_sf, locs, SRSRAN_MAX_CANDIDATES_UE, cur_rnti);
    if (nloc > 0) {
      dci_ul.location = locs[0];
      if (srsran_enb_dl_put_pdcch_ul(&enb_dl, &dci_cfg, &dci_ul) == SRSRAN_SUCCESS) {
        nof_grants++;
      }
    }
    srsran_enb_dl_gen_signal(&enb_dl);
    srsran_filesink_write(&dl_sink, enb_dl.out_buffer[0], sf_len);

    // ---- UL subframe: PUSCH for the grant issued 4 subframes earlier -------
    if (i >= FDD_UL_DELAY_MS) {
      srsran_ul_sf_cfg_t ul_sf = {};
      ul_sf.tti                = i; // PUSCH TTI = (i-4) + 4

      srsran_pusch_grant_t grant = {};
      if (srsran_ue_ul_dci_to_pusch_grant(&ue_ul, &ul_sf, &ue_ul_cfg, &dci_ul, &grant)) {
        ERROR("Error computing PUSCH grant at sf %d", i);
        exit(-1);
      }
      // The PUSCH answers the grant issued 4 subframes earlier, so it is
      // scrambled with that grant's C-RNTI (round-robin, offset by the delay).
      ue_ul_cfg.ul_cfg.pusch.grant         = grant;
      ue_ul_cfg.ul_cfg.pusch.rnti          = rntis[(i - FDD_UL_DELAY_MS) % nof_rntis];
      srsran_softbuffer_tx_reset(&softbuffer);
      ue_ul_cfg.ul_cfg.pusch.softbuffers.tx = &softbuffer;

      srsran_pusch_data_t pusch_data = {};
      pusch_data.ptr                 = data;
      if (srsran_ue_ul_encode(&ue_ul, &ul_sf, &ue_ul_cfg, &pusch_data) < 0) {
        ERROR("Error encoding PUSCH at sf %d (L_prb=%d tbs=%d)", i, grant.L_prb, grant.tb.tbs);
        exit(-1);
      }
      srsran_filesink_write(&ul_sink, ue_ul.out_buffer, sf_len);
      nof_pusch++;
    } else {
      srsran_filesink_write(&ul_sink, zeros, sf_len);
    }
  }

  printf("Generated %d subframes: %d DL format-0 grants, %d UL PUSCH.\n", nof_subframes, nof_grants, nof_pusch);
  printf("  DL: %s\n  UL: %s\n", dl_file, ul_file);
  printf("Replay: dl_ul_capture_align --dl-file %s --ul-file %s -c %d -p %d -r 0x%x\n",
         dl_file,
         ul_file,
         cell_id,
         nof_prb,
         rntis[0]);

  srsran_filesink_free(&dl_sink);
  srsran_filesink_free(&ul_sink);
  srsran_softbuffer_tx_free(&softbuffer);
  srsran_ue_ul_free(&ue_ul);
  srsran_enb_dl_free(&enb_dl);
  free(dl_out[0]);
  free(ul_out);
  free(zeros);
  free(data);
  return SRSRAN_SUCCESS;
}
