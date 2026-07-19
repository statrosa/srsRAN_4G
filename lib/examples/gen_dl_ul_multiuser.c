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
 *  File:         gen_dl_ul_multiuser.c
 *
 *  Description:  Demanding, realistic DL+UL IQ test-vector generator for
 *                dl_ul_capture_align, using srsRAN's real PHY. Unlike
 *                gen_dl_ul_testvec (one C-RNTI, one grant per subframe), this
 *                models a multi-user cell with a RACH ramp-up and the full RRC
 *                connection ladder per UE:
 *
 *                  - RAR (RA-RNTI PDCCH + RAR PDSCH: Temporary C-RNTI + Msg3 grant)
 *                  - Msg3 PUSCH at n+6 (RRC Connection Request)
 *                  - Msg4 DL PDSCH (contention resolution / RRC Setup)
 *                  - Msg5 = the first dynamic DCI-0 UL grant (RRC Setup Complete)
 *                  - steady-state dynamic DCI-0 grants thereafter
 *
 *                UEs attach over time, so several ladders overlap. In the dynamic
 *                phase several UEs are scheduled in the SAME subframe (up to K per
 *                TTI) on non-overlapping PDCCH CCEs and PRBs, and their PUSCH are
 *                summed into one UL subframe at n+4 - so the tool must decode
 *                multiple overlapping-in-time PUSCH per subframe. The tool learns
 *                every C-RNTI from the RARs (no -r needed) and tags the timeline
 *                Msg3 / Msg4 / Msg5 / dyn.
 *
 *                Running dl_ul_capture_align on the result with NO -r exercises
 *                RAR auto-learn + the active C-RNTI pool + the parallel UL worker
 *                pool under realistic multi-user, multi-grant-per-TTI load.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "srsran/srsran.h"

#define FDD_UL_DELAY 4 // dynamic DCI-0 grant in sf n -> PUSCH in sf n+4
#define MSG3_UL_DELAY 6 // RAR in sf n -> Msg3 PUSCH in sf n+6

#define MAX_UE 32
#define MAX_CCE 256
#define SCHED_RING 16   // > MSG3_UL_DELAY; per-UL-subframe emission ring
#define MAX_EMIT 16     // max PUSCH summed into one UL subframe
#define CRNTI_BASE 0x46

static uint32_t cell_id       = 1;
static uint32_t nof_prb       = 25;
static uint32_t nof_ue        = 10;
static uint32_t k_per_tti     = 4; // dynamic grants scheduled per subframe
static uint32_t mcs_idx       = 8;
static uint32_t L_rb          = 3; // PRBs per PUSCH allocation
static uint32_t nof_subframes = 400;
static char*    dl_file       = NULL;
static char*    ul_file       = NULL;

// One scheduled PUSCH emission (a grant answered at a specific UL subframe).
typedef struct {
  uint16_t rnti;
  uint32_t riv;
  uint32_t mcs;
  int      is_msg3;
} emit_t;

static emit_t   ul_sched[SCHED_RING][MAX_EMIT];
static uint32_t ul_sched_n[SCHED_RING];

static void usage(char* prog)
{
  printf("Usage: %s -o dl.iq -O ul.iq [options]\n", prog);
  printf("\t-c cell_id [%d]\n", cell_id);
  printf("\t-p nof_prb [%d]\n", nof_prb);
  printf("\t-u nof_ue (C-RNTIs, 0x46..) [%d]\n", nof_ue);
  printf("\t-k dynamic grants per TTI [%d]\n", k_per_tti);
  printf("\t-m dynamic PUSCH MCS [%d]\n", mcs_idx);
  printf("\t-L PRBs per PUSCH allocation [%d]\n", L_rb);
  printf("\t-n nof_subframes [%d]\n", nof_subframes);
  printf("\t-o DL output file\n\t-O UL output file\n");
}

static void parse_args(int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "c:p:u:k:m:L:n:o:O:h")) != -1) {
    switch (opt) {
      case 'c': cell_id = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'p': nof_prb = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'u': nof_ue = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'k': k_per_tti = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'm': mcs_idx = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'L': L_rb = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'n': nof_subframes = (uint32_t)strtol(optarg, NULL, 10); break;
      case 'o': dl_file = optarg; break;
      case 'O': ul_file = optarg; break;
      default: usage(argv[0]); exit(0);
    }
  }
  if (!dl_file || !ul_file) {
    usage(argv[0]);
    exit(-1);
  }
  if (nof_ue > MAX_UE) {
    nof_ue = MAX_UE;
  }
}

// Place a DCI at a search-space location whose CCEs are still free this subframe.
static int place_dci_ul(srsran_enb_dl_t* enb, srsran_dci_cfg_t* cfg, srsran_dci_ul_t* dci, int* cce_used)
{
  srsran_dci_location_t locs[SRSRAN_MAX_CANDIDATES_UE];
  uint32_t n = srsran_pdcch_ue_locations(&enb->pdcch, &enb->dl_sf, locs, SRSRAN_MAX_CANDIDATES_UE, dci->rnti);
  for (uint32_t k = 0; k < n; k++) {
    uint32_t agg = 1u << locs[k].L, c0 = locs[k].ncce;
    int      free_slot = 1;
    for (uint32_t c = c0; c < c0 + agg; c++) {
      if (c >= MAX_CCE || cce_used[c]) {
        free_slot = 0;
        break;
      }
    }
    if (free_slot) {
      dci->location = locs[k];
      if (srsran_enb_dl_put_pdcch_ul(enb, cfg, dci) == SRSRAN_SUCCESS) {
        for (uint32_t c = c0; c < c0 + agg; c++) {
          cce_used[c] = 1;
        }
        return 0;
      }
    }
  }
  return -1;
}

static int place_locs(srsran_enb_dl_t*       enb,
                      srsran_dci_cfg_t*      cfg,
                      srsran_dci_dl_t*       dci,
                      int*                   cce_used,
                      srsran_dci_location_t* locs,
                      uint32_t               n)
{
  for (uint32_t k = 0; k < n; k++) {
    uint32_t agg = 1u << locs[k].L, c0 = locs[k].ncce;
    int      free_slot = 1;
    for (uint32_t c = c0; c < c0 + agg; c++) {
      if (c >= MAX_CCE || cce_used[c]) {
        free_slot = 0;
        break;
      }
    }
    if (free_slot) {
      dci->location = locs[k];
      if (srsran_enb_dl_put_pdcch_dl(enb, cfg, dci) == SRSRAN_SUCCESS) {
        for (uint32_t c = c0; c < c0 + agg; c++) {
          cce_used[c] = 1;
        }
        return 0;
      }
    }
  }
  return -1;
}

// Place a DL DCI for a C-RNTI in its UE-specific search space (used for Msg4).
static int place_dci_dl_ue(srsran_enb_dl_t* enb, srsran_dci_cfg_t* cfg, srsran_dci_dl_t* dci, int* cce_used)
{
  srsran_dci_location_t locs[SRSRAN_MAX_CANDIDATES_UE];
  uint32_t n = srsran_pdcch_ue_locations(&enb->pdcch, &enb->dl_sf, locs, SRSRAN_MAX_CANDIDATES_UE, dci->rnti);
  return place_locs(enb, cfg, dci, cce_used, locs, n);
}

static int place_dci_dl(srsran_enb_dl_t* enb, srsran_dci_cfg_t* cfg, srsran_dci_dl_t* dci, int* cce_used)
{
  srsran_dci_location_t locs[SRSRAN_MAX_CANDIDATES_COM];
  uint32_t n = srsran_pdcch_common_locations(&enb->pdcch, locs, SRSRAN_MAX_CANDIDATES_COM, enb->dl_sf.cfi);
  for (uint32_t k = 0; k < n; k++) {
    uint32_t agg = 1u << locs[k].L, c0 = locs[k].ncce;
    int      free_slot = 1;
    for (uint32_t c = c0; c < c0 + agg; c++) {
      if (c >= MAX_CCE || cce_used[c]) {
        free_slot = 0;
        break;
      }
    }
    if (free_slot) {
      dci->location = locs[k];
      if (srsran_enb_dl_put_pdcch_dl(enb, cfg, dci) == SRSRAN_SUCCESS) {
        for (uint32_t c = c0; c < c0 + agg; c++) {
          cce_used[c] = 1;
        }
        return 0;
      }
    }
  }
  return -1;
}

// Build a MAC RAR PDU with one RAR entry (Temp C-RNTI + 20-bit Msg3 UL grant).
static int build_rar_pdu(uint8_t* buf, uint16_t temp_crnti, uint32_t msg3_riv, uint32_t msg3_mcs)
{
  // 20-bit RAR grant (TS 36.213 6.2): hop(1)|rba(10)|trunc_mcs(4)|tpc(3)|ul_delay(1)|cqi(1).
  uint32_t g = ((0u & 0x1) << 19) | ((msg3_riv & 0x3ff) << 9) | ((msg3_mcs & 0xf) << 5) | ((0u & 0x7) << 2) |
               ((0u & 0x1) << 1) | (0u & 0x1);
  buf[0] = 0x40 | 0x01;                       // E=0,T=1(RAPID), RAPID=1
  buf[1] = 0x00;                              // R=0 | TA[10:4]=0
  buf[2] = (uint8_t)((g >> 16) & 0x0f);       // TA[3:0]=0 | grant[19:16]
  buf[3] = (uint8_t)((g >> 8) & 0xff);        // grant[15:8]
  buf[4] = (uint8_t)(g & 0xff);               // grant[7:0]
  buf[5] = (uint8_t)(temp_crnti >> 8);        // Temp C-RNTI MSB
  buf[6] = (uint8_t)(temp_crnti & 0xff);      // Temp C-RNTI LSB
  return 7;
}

int main(int argc, char** argv)
{
  parse_args(argc, argv);

  srsran_cell_t cell   = {};
  cell.nof_prb         = nof_prb;
  cell.nof_ports       = 1;
  cell.id              = cell_id;
  cell.cp              = SRSRAN_CP_NORM;
  cell.phich_length    = SRSRAN_PHICH_NORM;
  cell.phich_resources = SRSRAN_PHICH_R_1;
  cell.frame_type      = SRSRAN_FDD;

  uint32_t sf_len = SRSRAN_SF_LEN_PRB(nof_prb);

  cf_t*           dl_out[SRSRAN_MAX_PORTS] = {srsran_vec_cf_malloc(sf_len)};
  srsran_enb_dl_t enb_dl                   = {};
  if (srsran_enb_dl_init(&enb_dl, dl_out, nof_prb) || srsran_enb_dl_set_cell(&enb_dl, cell)) {
    ERROR("Error initialising enb_dl");
    exit(-1);
  }

  cf_t*          ul_out = srsran_vec_cf_malloc(sf_len);
  srsran_ue_ul_t ue_ul  = {};
  if (srsran_ue_ul_init(&ue_ul, ul_out, nof_prb) || srsran_ue_ul_set_cell(&ue_ul, cell)) {
    ERROR("Error initialising ue_ul");
    exit(-1);
  }

  srsran_ue_ul_cfg_t ue_ul_cfg              = {};
  ue_ul_cfg.ul_cfg.dmrs.cyclic_shift        = 0;
  ue_ul_cfg.ul_cfg.dmrs.delta_ss            = 0;
  ue_ul_cfg.ul_cfg.dmrs.group_hopping_en    = false;
  ue_ul_cfg.ul_cfg.dmrs.sequence_hopping_en = false;
  ue_ul_cfg.ul_cfg.hopping.n_sb             = 1;
  ue_ul_cfg.ul_cfg.hopping.hopping_enabled  = false;
  ue_ul_cfg.grant_available                 = true;

  srsran_softbuffer_tx_t ul_sb = {}, dl_sb = {};
  srsran_softbuffer_tx_init(&ul_sb, nof_prb);
  srsran_softbuffer_tx_init(&dl_sb, nof_prb);

  srsran_dci_cfg_t dci_cfg = {};

  srsran_filesink_t dl_sink = {}, ul_sink = {};
  srsran_filesink_init(&dl_sink, dl_file, SRSRAN_COMPLEX_FLOAT_BIN);
  srsran_filesink_init(&ul_sink, ul_file, SRSRAN_COMPLEX_FLOAT_BIN);

  cf_t* ul_acc = srsran_vec_cf_malloc(sf_len);
  uint8_t* data = srsran_vec_u8_malloc(4000 * 8);
  for (int i = 0; i < 4000 * 8; i++) {
    data[i] = (uint8_t)(i & 0xff);
  }
  uint8_t rar_pdu[64];

  // UE attach schedule: UE u sends RACH at sf (RAR_START + 3*u); active for
  // dynamic grants from its Msg3 onward. RARs start well after subframe 0 so the
  // sniffer has time to PSS/SSS-acquire and MIB-lock before the first RACH (a
  // real sniffer likewise only catches RACHes that occur after it acquires).
  // Ladder per UE: RAR (sf r) -> Msg3 PUSCH (r+6) -> Msg4 DL PDSCH (r+8) ->
  // first dynamic UL grant = Msg5 (from r+10). RARs are spaced so the ladders
  // interleave (several UEs attaching/active at once).
  const uint32_t RAR_START = 40;
  uint32_t ue_rar_sf[MAX_UE], ue_msg4_sf[MAX_UE], ue_active_from[MAX_UE];
  uint16_t ue_crnti[MAX_UE];
  for (uint32_t u = 0; u < nof_ue; u++) {
    ue_rar_sf[u]      = RAR_START + 4 * u;
    ue_crnti[u]       = (uint16_t)(CRNTI_BASE + u);
    ue_msg4_sf[u]     = ue_rar_sf[u] + MSG3_UL_DELAY + 2; // r+8, after Msg3
    ue_active_from[u] = ue_rar_sf[u] + MSG3_UL_DELAY + 4; // r+10, first UL = Msg5
  }
  // Msg3 uses the low PRBs; dynamic grants use PRBs above them (no overlap in a
  // shared UL subframe).
  uint32_t msg3_rb_start = 1;
  uint32_t dyn_rb0       = msg3_rb_start + L_rb; // first dynamic block
  uint32_t msg3_riv      = srsran_ra_type2_to_riv(L_rb, msg3_rb_start, nof_prb);

  uint32_t nof_rar = 0, nof_msg3 = 0, nof_msg4 = 0, nof_dyn = 0, nof_pusch = 0;
  uint32_t rr = 0; // round-robin cursor over UEs for dynamic scheduling

  for (uint32_t i = 0; i < nof_subframes; i++) {
    int cce_used[MAX_CCE] = {0};

    srsran_dl_sf_cfg_t dl_sf = {};
    dl_sf.tti                = i;
    dl_sf.cfi                = 3;
    dl_sf.sf_type            = SRSRAN_SF_NORM;
    srsran_enb_dl_put_base(&enb_dl, &dl_sf);
    enb_dl.dl_sf = dl_sf;

    // --- RAR: is any UE RACHing this subframe? ---
    for (uint32_t u = 0; u < nof_ue; u++) {
      if (ue_rar_sf[u] != i) {
        continue;
      }
      uint16_t ra_rnti = (uint16_t)(1 + (u % 10)); // valid RA-RNTI (1..10)

      // PDCCH: format-1A DL assignment for the RAR PDSCH (common search space).
      srsran_dci_dl_t dci_dl = {};
      dci_dl.rnti            = ra_rnti;
      dci_dl.format          = SRSRAN_DCI_FORMAT1A;
      dci_dl.alloc_type      = SRSRAN_RA_ALLOC_TYPE2;
      dci_dl.type2_alloc.riv = srsran_ra_type2_to_riv(4, nof_prb - 5, nof_prb);
      dci_dl.tb[0].mcs_idx   = 4;
      dci_dl.tb[0].rv        = 0;
      dci_dl.tb[0].ndi       = 0;
      dci_dl.tb[0].cw_idx    = 0;
      SRSRAN_DCI_TB_DISABLE(dci_dl.tb[1]);
      dci_dl.pid = 0;
      if (place_dci_dl(&enb_dl, &dci_cfg, &dci_dl, cce_used) != 0) {
        continue;
      }

      // PDSCH: encode the RAR MAC PDU.
      srsran_pdsch_grant_t grant = {};
      if (srsran_ra_dl_dci_to_grant(&cell, &enb_dl.dl_sf, SRSRAN_TM1, false, &dci_dl, &grant)) {
        continue;
      }
      build_rar_pdu(rar_pdu, ue_crnti[u], msg3_riv, 0);
      memset(data, 0, (grant.tb[0].tbs / 8) + 1);
      memcpy(data, rar_pdu, 7);
      srsran_pdsch_cfg_t pcfg = {};
      pcfg.grant              = grant;
      pcfg.rnti               = ra_rnti;
      srsran_softbuffer_tx_reset(&dl_sb);
      pcfg.softbuffers.tx[0] = &dl_sb;
      uint8_t* pdata[SRSRAN_MAX_CODEWORDS] = {data};
      if (srsran_enb_dl_put_pdsch(&enb_dl, &pcfg, pdata) == SRSRAN_SUCCESS) {
        nof_rar++;
        // Schedule the Msg3 PUSCH at n+6.
        uint32_t slot = (i + MSG3_UL_DELAY) % SCHED_RING;
        if (ul_sched_n[slot] < MAX_EMIT) {
          ul_sched[slot][ul_sched_n[slot]++] = (emit_t){ue_crnti[u], msg3_riv, 0, 1};
        }
      }
    }

    // --- Msg4: DL contention resolution / RRC Setup PDSCH for a C-RNTI. ---
    for (uint32_t u = 0; u < nof_ue; u++) {
      if (ue_msg4_sf[u] != i) {
        continue;
      }
      srsran_dci_dl_t dci_dl = {};
      dci_dl.rnti            = ue_crnti[u];
      dci_dl.format          = SRSRAN_DCI_FORMAT1A;
      dci_dl.alloc_type      = SRSRAN_RA_ALLOC_TYPE2;
      dci_dl.type2_alloc.riv = srsran_ra_type2_to_riv(4, nof_prb / 2, nof_prb);
      dci_dl.tb[0].mcs_idx   = 4;
      dci_dl.tb[0].rv        = 0;
      dci_dl.tb[0].ndi       = 1;
      dci_dl.tb[0].cw_idx    = 0;
      SRSRAN_DCI_TB_DISABLE(dci_dl.tb[1]);
      dci_dl.pid = 0;
      if (place_dci_dl_ue(&enb_dl, &dci_cfg, &dci_dl, cce_used) != 0) {
        continue;
      }
      srsran_pdsch_grant_t grant = {};
      if (srsran_ra_dl_dci_to_grant(&cell, &enb_dl.dl_sf, SRSRAN_TM1, false, &dci_dl, &grant)) {
        continue;
      }
      memset(data, 0x5a, (grant.tb[0].tbs / 8) + 1); // stand-in RRC Setup payload
      srsran_pdsch_cfg_t pcfg = {};
      pcfg.grant              = grant;
      pcfg.rnti               = ue_crnti[u];
      srsran_softbuffer_tx_reset(&dl_sb);
      pcfg.softbuffers.tx[0]               = &dl_sb;
      uint8_t* pdata[SRSRAN_MAX_CODEWORDS] = {data};
      if (srsran_enb_dl_put_pdsch(&enb_dl, &pcfg, pdata) == SRSRAN_SUCCESS) {
        nof_msg4++;
      }
    }

    // --- Dynamic DCI-0 grants: up to k_per_tti active UEs this subframe. ---
    // Stop issuing grants whose PUSCH would fall past the end of the file, so no
    // grant is left without its UL subframe.
    uint32_t placed = 0;
    for (uint32_t tries = 0; tries < nof_ue && placed < k_per_tti && i + FDD_UL_DELAY < nof_subframes; tries++) {
      uint32_t u = rr % nof_ue;
      rr++;
      if (i < ue_active_from[u]) {
        continue;
      }
      uint32_t rb_start = dyn_rb0 + placed * L_rb;
      if (rb_start + L_rb > nof_prb - 1) {
        break; // no PRBs left this subframe
      }
      srsran_dci_ul_t dci_ul = {};
      dci_ul.rnti            = ue_crnti[u];
      dci_ul.freq_hop_fl     = SRSRAN_RA_PUSCH_HOP_DISABLED;
      dci_ul.type2_alloc.riv = srsran_ra_type2_to_riv(L_rb, rb_start, nof_prb);
      dci_ul.tb.mcs_idx      = mcs_idx;
      dci_ul.tb.rv           = 0;
      dci_ul.tb.ndi          = false;
      if (place_dci_ul(&enb_dl, &dci_cfg, &dci_ul, cce_used) != 0) {
        continue;
      }
      nof_dyn++;
      placed++;
      uint32_t slot = (i + FDD_UL_DELAY) % SCHED_RING;
      if (ul_sched_n[slot] < MAX_EMIT) {
        ul_sched[slot][ul_sched_n[slot]++] = (emit_t){ue_crnti[u], dci_ul.type2_alloc.riv, mcs_idx, 0};
      }
    }

    srsran_enb_dl_gen_signal(&enb_dl);
    srsran_filesink_write(&dl_sink, enb_dl.out_buffer[0], sf_len);

    // --- UL subframe i: sum every PUSCH scheduled to land here. ---
    srsran_vec_cf_zero(ul_acc, sf_len);
    uint32_t slot = i % SCHED_RING;
    for (uint32_t e = 0; e < ul_sched_n[slot]; e++) {
      emit_t*            em    = &ul_sched[slot][e];
      srsran_ul_sf_cfg_t ul_sf = {};
      ul_sf.tti                = i;

      srsran_dci_ul_t dci = {};
      dci.rnti            = em->rnti;
      dci.freq_hop_fl     = SRSRAN_RA_PUSCH_HOP_DISABLED;
      dci.type2_alloc.riv = em->riv;
      dci.tb.mcs_idx      = em->mcs;
      dci.tb.rv           = 0;
      dci.tb.ndi          = false;

      srsran_pusch_grant_t grant = {};
      if (srsran_ue_ul_dci_to_pusch_grant(&ue_ul, &ul_sf, &ue_ul_cfg, &dci, &grant)) {
        continue;
      }
      ue_ul_cfg.ul_cfg.pusch.grant = grant;
      ue_ul_cfg.ul_cfg.pusch.rnti  = em->rnti;
      srsran_softbuffer_tx_reset(&ul_sb);
      ue_ul_cfg.ul_cfg.pusch.softbuffers.tx = &ul_sb;

      srsran_pusch_data_t pd = {};
      pd.ptr                 = data;
      if (srsran_ue_ul_encode(&ue_ul, &ul_sf, &ue_ul_cfg, &pd) < 0) {
        continue;
      }
      srsran_vec_sum_ccc(ul_acc, ue_ul.out_buffer, ul_acc, sf_len);
      nof_pusch++;
      if (em->is_msg3) {
        nof_msg3++;
      }
    }
    ul_sched_n[slot] = 0; // consume
    srsran_filesink_write(&ul_sink, ul_acc, sf_len);
  }

  printf("Generated %u sf, %u UEs: %u RARs, %u Msg3, %u Msg4, %u dynamic grants, %u PUSCH emitted\n",
         nof_subframes,
         nof_ue,
         nof_rar,
         nof_msg3,
         nof_msg4,
         nof_dyn,
         nof_pusch);
  printf("  DL: %s\n  UL: %s\n", dl_file, ul_file);
  printf("Replay (RAR auto-learn, no -r): dl_ul_capture_align --dl-file %s --ul-file %s -c %d -p %d -w 8\n",
         dl_file,
         ul_file,
         cell_id,
         nof_prb);

  srsran_filesink_free(&dl_sink);
  srsran_filesink_free(&ul_sink);
  srsran_softbuffer_tx_free(&ul_sb);
  srsran_softbuffer_tx_free(&dl_sb);
  srsran_ue_ul_free(&ue_ul);
  srsran_enb_dl_free(&enb_dl);
  free(dl_out[0]);
  free(ul_out);
  free(ul_acc);
  free(data);
  return SRSRAN_SUCCESS;
}
