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
 *  File:         dl_ul_capture_align.cc
 *
 *  Description:  Passive two-radio LTE diagnostic sniffer.
 *
 *                Radio 0 (DL B210) synchronises to a cell, decodes the PDCCH
 *                and detects UL grants (DCI format 0). Radio 1 (UL B210)
 *                continuously captures raw uplink IQ. The two captures are
 *                aligned on a common (PPS-referenced) time base so that, for a
 *                UL grant seen on the DL in subframe n, the corresponding PUSCH
 *                in subframe n+4 (FDD) is located in the UL capture, handed to a
 *                pool of UL decoder workers through a pull API, decoded with the
 *                eNB-side PUSCH receiver, and printed as a time-aligned
 *                DL-grant -> UL-response timeline.
 *
 *                An offline file mode (--dl-file / --ul-file) drives the same
 *                pipeline from two IQ recordings for testing without hardware.
 *
 *  NOTE:         This is a diagnostic tool for a network you are authorised to
 *                operate on. srsRAN's DCI search is RNTI-targeted, so the C-RNTI
 *                (or a blind range) must be supplied on the command line.
 *****************************************************************************/

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <signal.h>
#include <string>
#include <vector>

#include "srsran/common/common.h"
#include "srsran/common/threads.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/srsran.h"

// rf_utils.h has no extern "C" guard of its own; give it C linkage explicitly.
// (srsran.h already provides filesource/filesink with C linkage.)
extern "C" {
#include "srsran/phy/rf/rf_utils.h"
}

/**********************************************************************
 *  Program arguments
 **********************************************************************/
typedef struct {
  // RF
  std::string dl_rf_args;
  std::string ul_rf_args;
  std::string clock_src; // internal | external | gpsdo
  double      dl_freq;
  double      ul_freq;
  float       rf_gain;
  int         ul_offset_samples;

  // File mode (offline, no hardware)
  std::string dl_file;
  std::string ul_file;

  // Cell / decode
  int      force_n_id_2;
  uint32_t file_nof_prb;
  uint32_t file_nof_ports;
  uint32_t file_cell_id;
  bool     use_standard_lte_rate;

  // PUSCH DMRS configuration (from the cell's SIB2 PUSCH-Config)
  srsran_refsignal_dmrs_pusch_cfg_t dmrs_cfg;

  // RNTI search
  std::vector<uint16_t> rntis;
  bool                  blind;
  uint16_t              blind_start;
  uint16_t              blind_end;

  // Runtime
  uint32_t nof_workers;
  int      nof_subframes;
  int      verbose;
} prog_args_t;

static void args_default(prog_args_t* a)
{
  a->dl_rf_args            = "";
  a->ul_rf_args            = "";
  a->clock_src             = "external";
  a->dl_freq               = -1.0;
  a->ul_freq               = -1.0;
  a->rf_gain               = 40.0;
  a->ul_offset_samples     = 0;
  a->force_n_id_2          = -1;
  a->file_nof_prb          = 25;
  a->file_nof_ports        = 1;
  a->file_cell_id          = 0;
  a->use_standard_lte_rate = false;
  a->dmrs_cfg.cyclic_shift        = 0;
  a->dmrs_cfg.delta_ss            = 0;
  a->dmrs_cfg.group_hopping_en    = false;
  a->dmrs_cfg.sequence_hopping_en = false;
  a->blind                 = false;
  a->blind_start           = 0x0001;
  a->blind_end             = 0xffff;
  a->nof_workers           = 4;
  a->nof_subframes         = -1;
  a->verbose               = 0;
}

static void usage(const char* prog)
{
  printf("Usage: %s [options]\n", prog);
  printf("  Live two-B210 mode:  -f <DL Hz> -F <UL Hz> -a <DL rf args> -A <UL rf args> -r <rnti,..>\n");
  printf("  Offline file mode:   --dl-file <iq> --ul-file <iq> -c <cell_id> -p <nof_prb> -r <rnti,..>\n\n");
  printf("  -f DL centre frequency in Hz\n");
  printf("  -F UL centre frequency in Hz\n");
  printf("  -a DL RF args (e.g. \"serial=ABC123\"); clock source appended from -k\n");
  printf("  -A UL RF args (e.g. \"serial=DEF456\")\n");
  printf("  -k Clock/time source for both radios: internal|external|gpsdo [Default external]\n");
  printf("  -g RX gain in dB applied to both radios [Default 40]\n");
  printf("  -o Fixed UL capture offset in samples (residual skew / timing advance) [Default 0]\n");
  printf("  -r Comma-separated C-RNTIs to search, hex (e.g. 0x46,0x47). SI/RA-RNTI may be included\n");
  printf("  -b Blind RNTI scan range \"start:end\" in hex (e.g. 0x0001:0x00ff). Heavy CPU\n");
  printf("  -w Number of UL decoder workers [Default 4]\n");
  printf("  -d PUSCH DMRS cfg \"cyclic_shift,delta_ss,group_hop,seq_hop\" [Default 0,0,0,0]\n");
  printf("  -n Number of subframes to process (-1 = run until Ctrl+C) [Default -1]\n");
  printf("  -l Force N_id_2 during cell search [Default auto]\n");
  printf("  --dl-file DL IQ recording (SRSRAN_COMPLEX_FLOAT_BIN) for offline mode\n");
  printf("  --ul-file UL IQ recording (SRSRAN_COMPLEX_FLOAT_BIN) for offline mode\n");
  printf("  -c Cell id (file mode)     -p nof_prb (file mode)     -P nof_ports (file mode)\n");
  printf("  -e Use standard LTE sample rates\n");
  printf("  -v Increase verbosity (repeatable)\n");
}

static void parse_rnti_list(const char* s, std::vector<uint16_t>* out)
{
  std::string str(s);
  size_t      pos = 0;
  while (pos < str.size()) {
    size_t comma = str.find(',', pos);
    std::string tok = str.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
    if (!tok.empty()) {
      out->push_back((uint16_t)strtol(tok.c_str(), nullptr, 16));
    }
    if (comma == std::string::npos) {
      break;
    }
    pos = comma + 1;
  }
}

static void parse_args(prog_args_t* args, int argc, char** argv)
{
  args_default(args);

  // Long options for file mode are handled by a manual first pass so we can keep getopt simple.
  static std::vector<char*> filtered;
  filtered.push_back(argv[0]);
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--dl-file") == 0 && i + 1 < argc) {
      args->dl_file = argv[++i];
    } else if (strcmp(argv[i], "--ul-file") == 0 && i + 1 < argc) {
      args->ul_file = argv[++i];
    } else {
      filtered.push_back(argv[i]);
    }
  }

  int   fargc = (int)filtered.size();
  char** fargv = filtered.data();
  int   opt;
  while ((opt = getopt(fargc, fargv, "f:F:a:A:k:g:o:r:b:w:d:n:l:c:p:P:evh")) != -1) {
    switch (opt) {
      case 'f':
        args->dl_freq = strtod(optarg, nullptr);
        break;
      case 'F':
        args->ul_freq = strtod(optarg, nullptr);
        break;
      case 'a':
        args->dl_rf_args = optarg;
        break;
      case 'A':
        args->ul_rf_args = optarg;
        break;
      case 'k':
        args->clock_src = optarg;
        break;
      case 'g':
        args->rf_gain = strtof(optarg, nullptr);
        break;
      case 'o':
        args->ul_offset_samples = (int)strtol(optarg, nullptr, 10);
        break;
      case 'r':
        parse_rnti_list(optarg, &args->rntis);
        break;
      case 'b': {
        args->blind      = true;
        char* colon      = strchr(optarg, ':');
        args->blind_start = (uint16_t)strtol(optarg, nullptr, 16);
        args->blind_end   = colon ? (uint16_t)strtol(colon + 1, nullptr, 16) : args->blind_start;
        break;
      }
      case 'w':
        args->nof_workers = (uint32_t)strtol(optarg, nullptr, 10);
        break;
      case 'd':
        sscanf(optarg,
               "%u,%u,%u,%u",
               &args->dmrs_cfg.cyclic_shift,
               &args->dmrs_cfg.delta_ss,
               (unsigned*)&args->dmrs_cfg.group_hopping_en,
               (unsigned*)&args->dmrs_cfg.sequence_hopping_en);
        break;
      case 'n':
        args->nof_subframes = (int)strtol(optarg, nullptr, 10);
        break;
      case 'l':
        args->force_n_id_2 = (int)strtol(optarg, nullptr, 10);
        break;
      case 'c':
        args->file_cell_id = (uint32_t)strtol(optarg, nullptr, 10);
        break;
      case 'p':
        args->file_nof_prb = (uint32_t)strtol(optarg, nullptr, 10);
        break;
      case 'P':
        args->file_nof_ports = (uint32_t)strtol(optarg, nullptr, 10);
        break;
      case 'e':
        args->use_standard_lte_rate = true;
        break;
      case 'v':
        increase_srsran_verbose_level();
        args->verbose = get_srsran_verbose_level();
        break;
      case 'h':
      default:
        usage(fargv[0]);
        exit(0);
    }
  }

  bool file_mode = !args->dl_file.empty() && !args->ul_file.empty();
  if (args->nof_workers < 1) {
    args->nof_workers = 1;
  }
  if (args->rntis.empty() && !args->blind) {
    fprintf(stderr, "Error: at least one -r RNTI or a -b blind range is required.\n\n");
    usage(fargv[0]);
    exit(-1);
  }
  if (!file_mode && (args->dl_freq < 0 || args->ul_freq < 0)) {
    fprintf(stderr, "Error: live mode needs both -f (DL) and -F (UL) frequencies.\n\n");
    usage(fargv[0]);
    exit(-1);
  }
}

/**********************************************************************
 *  Globals / signal handling
 **********************************************************************/
static std::atomic<bool> go_exit{false};

static void sig_int_handler(int signo)
{
  if (signo == SIGINT) {
    go_exit = true;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

// Monotonic subframe index shared by both radios. Because both B210s are reset
// to a common PPS-referenced device time, the absolute device timestamp of a
// 1 ms subframe maps to the same integer index on the DL and UL sides.
static inline uint64_t sf_mono_from_ts(const srsran_timestamp_t& ts)
{
  return (uint64_t)llround(srsran_timestamp_real(&ts) / 1.0e-3);
}

/**********************************************************************
 *  Timeline record (a decoded DL-grant -> UL-response result)
 **********************************************************************/
struct timeline_record_t {
  uint64_t dl_mono;
  uint32_t dl_tti;
  uint32_t ul_tti;
  uint16_t rnti;
  uint32_t mcs;
  int      tbs;
  uint32_t L_prb;
  uint32_t rb_start;
  bool     crc;
  float    snr_db;
  float    ul_present; // 1 if the UL subframe was available, 0 if missed
  double   decode_ms;
};

/**********************************************************************
 *  ul_capture_store: the pull API the workers query for the exact subframe
 **********************************************************************/
struct pending_grant_t {
  uint64_t        dl_mono;
  uint32_t        dl_tti;
  uint64_t        ul_mono;
  uint32_t        ul_tti;
  uint16_t        rnti;
  srsran_dci_ul_t dci;
};

struct ul_work_t {
  pending_grant_t    grant;
  bool               iq_valid;
  std::vector<cf_t>  iq;
  srsran_timestamp_t ul_ts;
};

class ul_capture_store
{
public:
  ul_capture_store(uint32_t sf_len_, uint32_t ring_depth_) :
    sf_len(sf_len_), ring_depth(ring_depth_), ring(ring_depth_)
  {
    for (auto& s : ring) {
      s.iq.resize(sf_len);
    }
  }

  // Called by the UL capture producer.
  void push_ul_subframe(uint64_t mono, const srsran_timestamp_t& ts, const cf_t* iq)
  {
    std::unique_lock<std::mutex> lock(mtx);
    ring_slot_t&                 slot = ring[mono % ring_depth];
    slot.mono                         = mono;
    slot.valid                        = true;
    slot.ts                           = ts;
    memcpy(slot.iq.data(), iq, sizeof(cf_t) * sf_len);
    if (mono >= latest_mono || !have_latest) {
      latest_mono = mono;
      have_latest = true;
    }
    lock.unlock();
    cv.notify_all();
  }

  // Called by the DL producer for every detected UL grant.
  void push_grant(const pending_grant_t& g)
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      grants.push(g);
      total_grants++;
    }
    cv.notify_all();
  }

  // Called by a free worker: block until the next pending grant's UL subframe is
  // captured (or provably missed), then hand back the grant plus the exact IQ.
  // Returns false only when stopped and fully drained.
  bool acquire_work(ul_work_t& out)
  {
    std::unique_lock<std::mutex> lock(mtx);
    while (true) {
      if (grants.empty()) {
        if (!running) {
          return false;
        }
        cv.wait(lock);
        continue;
      }

      pending_grant_t g    = grants.front();
      ring_slot_t&    slot = ring[g.ul_mono % ring_depth];

      if (slot.valid && slot.mono == g.ul_mono) {
        // Exact subframe present: hand it to the worker.
        grants.pop();
        out.grant    = g;
        out.iq_valid = true;
        out.ul_ts    = slot.ts;
        out.iq.assign(slot.iq.begin(), slot.iq.end());
        return true;
      }

      // Determine whether the subframe has been missed (capture already moved past
      // it, or it was evicted from the ring) versus not yet captured.
      bool passed  = have_latest && latest_mono > g.ul_mono;
      bool evicted = have_latest && latest_mono >= g.ul_mono + ring_depth;
      if (evicted || (passed && !(slot.valid && slot.mono == g.ul_mono))) {
        grants.pop();
        missed_grants++;
        out.grant    = g;
        out.iq_valid = false;
        return true; // report the miss so the timeline still shows the grant
      }

      if (!running) {
        // Draining: give up waiting on future captures.
        grants.pop();
        missed_grants++;
        out.grant    = g;
        out.iq_valid = false;
        return true;
      }

      cv.wait(lock);
    }
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      running = false;
    }
    cv.notify_all();
  }

  uint64_t get_total_grants()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return total_grants;
  }
  uint64_t get_missed_grants()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return missed_grants;
  }

private:
  struct ring_slot_t {
    uint64_t           mono  = 0;
    bool               valid = false;
    srsran_timestamp_t ts    = {};
    std::vector<cf_t>  iq;
  };

  uint32_t                    sf_len;
  uint32_t                    ring_depth;
  std::vector<ring_slot_t>    ring;
  std::queue<pending_grant_t> grants;
  uint64_t                    latest_mono   = 0;
  bool                        have_latest   = false;
  uint64_t                    total_grants  = 0;
  uint64_t                    missed_grants = 0;
  bool                        running       = true;
  std::mutex                  mtx;
  std::condition_variable     cv;
};

/**********************************************************************
 *  Ordered timeline printer
 **********************************************************************/
class timeline_printer
{
public:
  explicit timeline_printer(uint32_t reorder_hold_) : reorder_hold(reorder_hold_) {}

  void submit(const timeline_record_t& r)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (r.dl_mono > max_dl_mono) {
      max_dl_mono = r.dl_mono;
    }
    pending.emplace(r.dl_mono, r);
    flush_ready(false);
  }

  void drain()
  {
    std::unique_lock<std::mutex> lock(mtx);
    flush_ready(true);
  }

private:
  void print_one(const timeline_record_t& r)
  {
    if (!r.ul_present) {
      printf("DL#%-5u grant rnti=0x%04x mcs=%2u L_prb=%2u rb_start=%2u  ->  UL#%-5u  [UL subframe missed]\n",
             r.dl_tti,
             r.rnti,
             r.mcs,
             r.L_prb,
             r.rb_start,
             r.ul_tti);
    } else {
      printf("DL#%-5u grant rnti=0x%04x mcs=%2u L_prb=%2u rb_start=%2u  ->  UL#%-5u  CRC=%-3s tbs=%5db snr=%5.1fdB "
             "(decode %.2fms)\n",
             r.dl_tti,
             r.rnti,
             r.mcs,
             r.L_prb,
             r.rb_start,
             r.ul_tti,
             r.crc ? "OK" : "NOK",
             r.tbs,
             r.snr_db,
             r.decode_ms);
    }
    fflush(stdout);
  }

  // Flush records whose dl_mono is safely behind the watermark (in-order output).
  void flush_ready(bool all)
  {
    while (!pending.empty()) {
      auto it = pending.begin();
      if (!all && it->first + reorder_hold > max_dl_mono) {
        break;
      }
      print_one(it->second);
      pending.erase(it);
    }
  }

  uint32_t                              reorder_hold;
  uint64_t                              max_dl_mono = 0;
  std::multimap<uint64_t, timeline_record_t> pending;
  std::mutex                            mtx;
};

/**********************************************************************
 *  UL decoder worker: owns its own eNB-side PUSCH receiver
 **********************************************************************/
class ul_decoder_worker : public srsran::thread
{
public:
  ul_decoder_worker(uint32_t          id_,
                    srsran_cell_t     cell_,
                    ul_capture_store* store_,
                    timeline_printer* printer_,
                    const srsran_refsignal_dmrs_pusch_cfg_t& dmrs_cfg_) :
    srsran::thread("ul_dec_" + std::to_string(id_)),
    id(id_),
    cell(cell_),
    store(store_),
    printer(printer_),
    dmrs_cfg(dmrs_cfg_)
  {
    sf_len    = SRSRAN_SF_LEN_PRB(cell.nof_prb);
    in_buffer = srsran_vec_cf_malloc(sf_len);
    data      = srsran_vec_u8_malloc(2000 * 8);

    if (srsran_enb_ul_init(&enb_ul, in_buffer, cell.nof_prb)) {
      ERROR("Worker %u: error initialising enb_ul", id);
      init_ok = false;
      return;
    }
    if (srsran_enb_ul_set_cell(&enb_ul, cell, &dmrs_cfg, nullptr)) {
      ERROR("Worker %u: error setting cell on enb_ul", id);
      init_ok = false;
      return;
    }
    srsran_softbuffer_rx_init(&softbuffer, cell.nof_prb);
    init_ok = true;
  }

  ~ul_decoder_worker()
  {
    if (init_ok) {
      srsran_enb_ul_free(&enb_ul);
      srsran_softbuffer_rx_free(&softbuffer);
    }
    if (in_buffer) {
      free(in_buffer);
    }
    if (data) {
      free(data);
    }
  }

  bool ok() const { return init_ok; }

protected:
  void run_thread() override
  {
    ul_work_t work;
    work.iq.reserve(sf_len);
    while (store->acquire_work(work)) {
      timeline_record_t rec = {};
      rec.dl_mono           = work.grant.dl_mono;
      rec.dl_tti            = work.grant.dl_tti;
      rec.ul_tti            = work.grant.ul_tti;
      rec.rnti              = work.grant.rnti;
      rec.mcs               = work.grant.dci.tb.mcs_idx;

      if (!work.iq_valid) {
        rec.ul_present = 0.0f;
        printer->submit(rec);
        continue;
      }

      decode(work, rec);
      printer->submit(rec);
    }
  }

private:
  void decode(const ul_work_t& work, timeline_record_t& rec)
  {
    struct timeval t[3];
    gettimeofday(&t[1], nullptr);

    // Load the exact captured UL subframe and transform to the frequency grid.
    memcpy(in_buffer, work.iq.data(), sizeof(cf_t) * sf_len);
    srsran_enb_ul_fft(&enb_ul);

    // Build the PUSCH grant from the recovered UL DCI.
    srsran_ul_sf_cfg_t ul_sf = {};
    ul_sf.tti                = work.grant.ul_tti;

    srsran_pusch_hopping_cfg_t hopping = {};
    hopping.hopping_enabled            = false;

    srsran_pusch_cfg_t   pusch_cfg = {};
    srsran_pusch_grant_t grant     = {};
    srsran_dci_ul_t      dci       = work.grant.dci;
    if (srsran_ra_ul_dci_to_grant(&cell, &ul_sf, &hopping, &dci, &grant)) {
      ERROR("Worker %u: error building PUSCH grant for UL#%u", id, work.grant.ul_tti);
      rec.ul_present = 1.0f;
      rec.crc        = false;
      rec.tbs        = 0;
      return;
    }

    pusch_cfg.grant        = grant;
    pusch_cfg.rnti         = work.grant.rnti;
    pusch_cfg.softbuffers.rx = &softbuffer;
    pusch_cfg.meas_time_en   = false;
    srsran_softbuffer_rx_reset_tbs(&softbuffer, (uint32_t)grant.tb.tbs);

    srsran_pusch_res_t res = {};
    res.data               = data;

    int ret = srsran_enb_ul_get_pusch(&enb_ul, &ul_sf, &pusch_cfg, &res);

    gettimeofday(&t[2], nullptr);
    get_time_interval(t);

    rec.ul_present = 1.0f;
    rec.tbs        = grant.tb.tbs;
    rec.L_prb      = grant.L_prb;
    rec.rb_start   = grant.n_prb[0];
    rec.snr_db     = enb_ul.chest_res.snr_db;
    rec.crc        = (ret == SRSRAN_SUCCESS) ? res.crc : false;
    rec.decode_ms  = (double)(t[0].tv_sec * 1e6 + t[0].tv_usec) / 1e3;
  }

  uint32_t                          id;
  srsran_cell_t                     cell;
  ul_capture_store*                 store;
  timeline_printer*                 printer;
  srsran_refsignal_dmrs_pusch_cfg_t dmrs_cfg;

  uint32_t               sf_len     = 0;
  cf_t*                  in_buffer  = nullptr;
  uint8_t*               data       = nullptr;
  srsran_enb_ul_t        enb_ul     = {};
  srsran_softbuffer_rx_t softbuffer = {};
  bool                   init_ok    = false;
};

/**********************************************************************
 *  UL capture producer thread (live RF)
 **********************************************************************/
class ul_rf_capture : public srsran::thread
{
public:
  ul_rf_capture(srsran_rf_t* rf_, uint32_t sf_len_, int offset_samples_, ul_capture_store* store_) :
    srsran::thread("ul_capture"), rf(rf_), sf_len(sf_len_), offset_samples(offset_samples_), store(store_)
  {
    buffer = srsran_vec_cf_malloc(sf_len);
  }
  ~ul_rf_capture()
  {
    if (buffer) {
      free(buffer);
    }
  }

  void stop()
  {
    running = false;
    wait_thread_finish();
  }

protected:
  void run_thread() override
  {
    // Discard a fixed sample offset once to absorb residual inter-device skew /
    // timing advance, so every captured 1 ms window aligns to the DL subframe grid.
    if (offset_samples > 0) {
      std::vector<cf_t> skip(offset_samples);
      void*             p = skip.data();
      srsran_rf_recv_with_time_multi(rf, &p, (uint32_t)offset_samples, true, nullptr, nullptr);
    }

    while (running && !go_exit) {
      srsran_timestamp_t ts = {};
      void*              p  = buffer;
      int n = srsran_rf_recv_with_time_multi(rf, &p, sf_len, true, &ts.full_secs, &ts.frac_secs);
      if (n < 0) {
        ERROR("UL capture: error receiving samples");
        break;
      }
      store->push_ul_subframe(sf_mono_from_ts(ts), ts, buffer);
    }
  }

private:
  srsran_rf_t*      rf;
  uint32_t          sf_len;
  int               offset_samples;
  ul_capture_store* store;
  cf_t*             buffer  = nullptr;
  std::atomic<bool> running{true};
};

/**********************************************************************
 *  DL receive wrapper for ue_sync
 **********************************************************************/
static int srsran_rf_recv_wrapper(void* h, cf_t* data_[SRSRAN_MAX_PORTS], uint32_t nsamples, srsran_timestamp_t* t)
{
  void* ptr[SRSRAN_MAX_PORTS];
  for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
    ptr[i] = data_[i];
  }
  return srsran_rf_recv_with_time_multi(
      (srsran_rf_t*)h, ptr, nsamples, true, t ? &t->full_secs : nullptr, t ? &t->frac_secs : nullptr);
}

/**********************************************************************
 *  DL processing: detect UL grants and push them to the store
 **********************************************************************/
static void process_dl_subframe(srsran_ue_dl_t*     ue_dl,
                                srsran_ue_dl_cfg_t* ue_dl_cfg,
                                srsran_cell_t*      cell,
                                const prog_args_t&  args,
                                uint32_t            tti,
                                uint64_t            dl_mono,
                                ul_capture_store*   store,
                                uint64_t*           nof_grants_seen)
{
  srsran_dl_sf_cfg_t dl_sf = {};
  dl_sf.tti                = tti;
  dl_sf.sf_type            = SRSRAN_SF_NORM;

  if (srsran_ue_dl_decode_fft_estimate(ue_dl, &dl_sf, ue_dl_cfg) < 0) {
    return;
  }

  auto search_rnti = [&](uint16_t rnti) {
    srsran_dci_dl_t dci_dl[SRSRAN_MAX_DCI_MSG] = {};
    // find_dl_dci runs the blind PDCCH search for this rnti and, as a side effect,
    // stashes the format-0 (UL) DCI candidates for find_ul_dci to unpack.
    srsran_ue_dl_find_dl_dci(ue_dl, &dl_sf, ue_dl_cfg, rnti, dci_dl);

    srsran_dci_ul_t dci_ul[SRSRAN_MAX_DCI_MSG] = {};
    int             nof_ul = srsran_ue_dl_find_ul_dci(ue_dl, &dl_sf, ue_dl_cfg, rnti, dci_ul);
    for (int i = 0; i < nof_ul; i++) {
      pending_grant_t g = {};
      g.dl_mono         = dl_mono;
      g.dl_tti          = tti;
      // FDD: a format-0 grant in subframe n schedules PUSCH in subframe n+4.
      g.ul_mono = dl_mono + FDD_HARQ_DELAY_UL_MS;
      g.ul_tti  = TTI_ADD(tti, FDD_HARQ_DELAY_UL_MS);
      g.rnti    = rnti;
      g.dci     = dci_ul[i];
      store->push_grant(g);
      (*nof_grants_seen)++;
    }
  };

  for (uint16_t rnti : args.rntis) {
    search_rnti(rnti);
  }
  if (args.blind) {
    for (uint32_t r = args.blind_start; r <= args.blind_end; r++) {
      search_rnti((uint16_t)r);
    }
  }
}

/**********************************************************************
 *  RF setup helpers
 **********************************************************************/
static std::string build_rf_args(const std::string& base, const std::string& clock_src)
{
  std::string args = base;
  if (clock_src != "internal" && args.find("clock=") == std::string::npos) {
    if (!args.empty()) {
      args += ",";
    }
    args += "clock=" + clock_src;
  }
  return args;
}

/**********************************************************************
 *  main
 **********************************************************************/
int main(int argc, char** argv)
{
  prog_args_t args;
  parse_args(&args, argc, argv);

  srsran_use_standard_symbol_size(args.use_standard_lte_rate);

  signal(SIGINT, sig_int_handler);
  signal(SIGSEGV, sig_int_handler);

  bool          file_mode = !args.dl_file.empty() && !args.ul_file.empty();
  srsran_cell_t cell      = {};

  srsran_rf_t      dl_rf     = {};
  srsran_rf_t      ul_rf     = {};
  bool             dl_rf_open = false, ul_rf_open = false;
  srsran_ue_sync_t ue_sync   = {};

  cell_search_cfg_t cell_detect_config = {};
  cell_detect_config.max_frames_pbch      = SRSRAN_DEFAULT_MAX_FRAMES_PBCH;
  cell_detect_config.max_frames_pss       = SRSRAN_DEFAULT_MAX_FRAMES_PSS;
  cell_detect_config.nof_valid_pss_frames = SRSRAN_DEFAULT_NOF_VALID_PSS_FRAMES;
  cell_detect_config.init_agc             = 0;

  if (file_mode) {
    cell.id              = args.file_cell_id;
    cell.cp              = SRSRAN_CP_NORM;
    cell.phich_length    = SRSRAN_PHICH_NORM;
    cell.phich_resources = SRSRAN_PHICH_R_1;
    cell.nof_ports       = args.file_nof_ports;
    cell.nof_prb         = args.file_nof_prb;

    if (srsran_ue_sync_init_file_multi(&ue_sync, cell.nof_prb, (char*)args.dl_file.c_str(), 0, 0, 1)) {
      ERROR("Error initialising DL file ue_sync");
      exit(-1);
    }
    printf("Offline mode: DL=%s UL=%s, cell_id=%d, nof_prb=%d\n",
           args.dl_file.c_str(),
           args.ul_file.c_str(),
           cell.id,
           cell.nof_prb);
  } else {
    std::string dl_args = build_rf_args(args.dl_rf_args, args.clock_src);
    std::string ul_args = build_rf_args(args.ul_rf_args, args.clock_src);

    printf("Opening DL radio (%s)...\n", dl_args.c_str());
    if (srsran_rf_open_multi(&dl_rf, (char*)dl_args.c_str(), 1)) {
      ERROR("Error opening DL radio");
      exit(-1);
    }
    dl_rf_open = true;
    printf("Opening UL radio (%s)...\n", ul_args.c_str());
    if (srsran_rf_open_multi(&ul_rf, (char*)ul_args.c_str(), 1)) {
      ERROR("Error opening UL radio");
      exit(-1);
    }
    ul_rf_open = true;

    srsran_rf_set_rx_gain(&dl_rf, args.rf_gain);
    srsran_rf_set_rx_gain(&ul_rf, args.rf_gain);
    printf("Tuning DL to %.3f MHz, UL to %.3f MHz\n", args.dl_freq / 1e6, args.ul_freq / 1e6);
    srsran_rf_set_rx_freq(&dl_rf, 1, args.dl_freq);
    srsran_rf_set_rx_freq(&ul_rf, 1, args.ul_freq);

    // Reset both device clocks to a common PPS edge so their timestamps share one
    // base. Requires a shared 10 MHz + PPS (clock=external) or GPSDO on both.
    if (args.clock_src != "internal") {
      printf("Aligning both radios to PPS (clock=%s)...\n", args.clock_src.c_str());
      srsran_rf_sync(&dl_rf);
      srsran_rf_sync(&ul_rf);
    } else {
      printf("WARNING: clock=internal - the two radios are NOT time-aligned; UL matching will be unreliable.\n");
    }

    // Cell search + MIB on the DL radio.
    float cfo = 0;
    int   ret = 0;
    do {
      ret = rf_search_and_decode_mib(&dl_rf, 1, &cell_detect_config, args.force_n_id_2, &cell, &cfo);
      if (ret < 0) {
        ERROR("Error searching for cell");
        exit(-1);
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found, retrying... (Ctrl+C to exit)\n");
      }
    } while (ret == 0 && !go_exit);
    if (go_exit) {
      srsran_rf_close(&dl_rf);
      srsran_rf_close(&ul_rf);
      exit(0);
    }

    int srate = srsran_sampling_freq_hz(cell.nof_prb);
    if (srate < 0) {
      ERROR("Invalid nof_prb %d", cell.nof_prb);
      exit(-1);
    }
    printf("Setting sampling rate %.2f MHz on both radios\n", srate / 1e6);
    srsran_rf_set_rx_srate(&dl_rf, (double)srate);
    srsran_rf_set_rx_srate(&ul_rf, (double)srate);

    if (srsran_ue_sync_init_multi_decim(
            &ue_sync, cell.nof_prb, cell.id == 1000, srsran_rf_recv_wrapper, 1, (void*)&dl_rf, 0)) {
      ERROR("Error initialising DL ue_sync");
      exit(-1);
    }
    ue_sync.cfo_current_value       = cfo / 15000;
    ue_sync.cfo_is_copied           = true;
    ue_sync.cfo_correct_enable_find = true;
  }

  // In file mode srsran_ue_sync_init_file_multi() already configures the cell; a
  // subsequent set_cell() would fail its "nof_prb lower than initialized" check.
  if (!file_mode) {
    if (srsran_ue_sync_set_cell(&ue_sync, cell)) {
      ERROR("Error setting cell on ue_sync");
      exit(-1);
    }
  }

  srsran_cell_fprint(stdout, &cell, 0);

  // DL PDCCH decoder objects.
  uint32_t max_num_samples          = 3 * SRSRAN_SF_LEN_PRB(cell.nof_prb);
  cf_t*    sf_buffer[SRSRAN_MAX_PORTS] = {nullptr};
  sf_buffer[0]                       = srsran_vec_cf_malloc(max_num_samples);

  srsran_ue_dl_t ue_dl = {};
  if (srsran_ue_dl_init(&ue_dl, sf_buffer, cell.nof_prb, 1)) {
    ERROR("Error initialising ue_dl");
    exit(-1);
  }
  if (srsran_ue_dl_set_cell(&ue_dl, cell)) {
    ERROR("Error setting cell on ue_dl");
    exit(-1);
  }
  srsran_ue_dl_cfg_t ue_dl_cfg    = {};
  ue_dl_cfg.cfg.tm                = SRSRAN_TM1;
  ue_dl_cfg.chest_cfg.estimator_alg = SRSRAN_ESTIMATOR_ALG_INTERPOLATE;

  const uint32_t sf_len     = SRSRAN_SF_LEN_PRB(cell.nof_prb);
  const uint32_t ring_depth = 256; // >> n+4 latency + worker backlog

  ul_capture_store store(sf_len, ring_depth);
  timeline_printer printer(16);

  // UL decoder worker pool.
  std::vector<std::unique_ptr<ul_decoder_worker>> workers;
  for (uint32_t i = 0; i < args.nof_workers; i++) {
    auto w = std::unique_ptr<ul_decoder_worker>(
        new ul_decoder_worker(i, cell, &store, &printer, args.dmrs_cfg));
    if (!w->ok()) {
      ERROR("Failed to initialise UL decoder worker %u", i);
      exit(-1);
    }
    w->start(-1);
    workers.push_back(std::move(w));
  }
  printf("Started %u UL decoder workers\n", args.nof_workers);

  // UL capture producer (live RF) or file source (offline).
  std::unique_ptr<ul_rf_capture> ul_capture;
  srsran_filesource_t            ul_filesrc = {};
  std::vector<cf_t>              ul_file_buf;
  bool                           ul_file_open = false;

  if (file_mode) {
    if (srsran_filesource_init(&ul_filesrc, (char*)args.ul_file.c_str(), SRSRAN_COMPLEX_FLOAT_BIN)) {
      ERROR("Error opening UL file %s", args.ul_file.c_str());
      exit(-1);
    }
    ul_file_open = true;
    ul_file_buf.resize(sf_len);
    // Calibration: advance the UL file by -o samples to line it up with the DL
    // stream (e.g. two independently-recorded captures with a constant start
    // offset). Applied once, before the lockstep read loop.
    if (args.ul_offset_samples > 0) {
      std::vector<cf_t> skip(args.ul_offset_samples);
      void*             p = skip.data();
      srsran_filesource_read_multi(&ul_filesrc, &p, args.ul_offset_samples, 1);
      printf("Skipped %d UL samples for DL/UL alignment\n", args.ul_offset_samples);
    }
  } else {
    srsran_rf_start_rx_stream(&dl_rf, false);
    srsran_rf_start_rx_stream(&ul_rf, false);
    ul_capture.reset(new ul_rf_capture(&ul_rf, sf_len, args.ul_offset_samples, &store));
    ul_capture->start(-1);
  }

  printf("\n---- DL-grant -> UL-response timeline ----\n");

  // Main DL loop.
  uint64_t dl_mono         = 0; // monotonic subframe counter for file mode
  uint64_t nof_grants_seen = 0;
  uint32_t sfn             = 0;
  bool     mib_locked      = file_mode; // in file mode we trust the preset cell
  int      processed       = 0;
  srsran_ue_mib_t ue_mib   = {};
  if (!file_mode) {
    if (srsran_ue_mib_init(&ue_mib, sf_buffer[0], cell.nof_prb)) {
      ERROR("Error initialising UE MIB");
      exit(-1);
    }
    srsran_ue_mib_set_cell(&ue_mib, cell);
  }

  while (!go_exit && (args.nof_subframes < 0 || processed < args.nof_subframes)) {
    cf_t* buffers[SRSRAN_MAX_PORTS] = {sf_buffer[0]};
    int   ret                       = srsran_ue_sync_zerocopy(&ue_sync, buffers, max_num_samples);
    if (ret < 0) {
      ERROR("Error in ue_sync");
      break;
    }
    if (ret != 1) {
      continue;
    }

    uint32_t sf_idx = srsran_ue_sync_get_sfidx(&ue_sync);

    if (!mib_locked) {
      if (sf_idx == 0) {
        uint8_t bch[SRSRAN_BCH_PAYLOAD_LEN];
        int     sfn_offset = 0;
        int     n          = srsran_ue_mib_decode(&ue_mib, bch, nullptr, &sfn_offset);
        if (n == SRSRAN_UE_MIB_FOUND) {
          srsran_pbch_mib_unpack(bch, &cell, &sfn);
          sfn        = (sfn + sfn_offset) % 1024;
          mib_locked = true;
          printf("Decoded MIB. SFN=%d\n", sfn);
        }
      }
      continue;
    }

    uint32_t tti = sfn * 10 + sf_idx;

    // Establish the monotonic subframe index shared with the UL producer.
    if (file_mode) {
      // Offline: both files are read from sample 0 in lockstep, so the DL loop
      // counter is the shared index; advance the UL file by one subframe too.
      void* p = ul_file_buf.data();
      int   r = srsran_filesource_read_multi(&ul_filesrc, &p, (int)sf_len, 1);
      if (r > 0) {
        srsran_timestamp_t ts = {};
        srsran_timestamp_init(&ts, (time_t)(dl_mono / 1000), (double)(dl_mono % 1000) * 1e-3);
        store.push_ul_subframe(dl_mono, ts, ul_file_buf.data());
      }
    } else {
      srsran_timestamp_t dl_ts = {};
      srsran_ue_sync_get_last_timestamp(&ue_sync, &dl_ts);
      dl_mono = sf_mono_from_ts(dl_ts);
    }

    process_dl_subframe(&ue_dl, &ue_dl_cfg, &cell, args, tti, dl_mono, &store, &nof_grants_seen);

    if (file_mode) {
      dl_mono++;
    }
    if (sf_idx == 9) {
      sfn = (sfn + 1) % 1024;
    }
    processed++;
  }

  printf("\nShutting down...\n");
  go_exit = true;

  if (ul_capture) {
    ul_capture->stop();
  }
  store.stop();
  for (auto& w : workers) {
    w->wait_thread_finish();
  }
  printer.drain();

  printf("\n---- Summary ----\n");
  printf("DL subframes processed : %d\n", processed);
  printf("UL grants detected     : %" PRIu64 "\n", store.get_total_grants());
  printf("UL subframes missed    : %" PRIu64 "\n", store.get_missed_grants());

  // Cleanup.
  workers.clear();
  srsran_ue_dl_free(&ue_dl);
  srsran_ue_sync_free(&ue_sync);
  if (!file_mode) {
    srsran_ue_mib_free(&ue_mib);
  }
  if (ul_file_open) {
    srsran_filesource_free(&ul_filesrc);
  }
  if (sf_buffer[0]) {
    free(sf_buffer[0]);
  }
  if (dl_rf_open) {
    srsran_rf_close(&dl_rf);
  }
  if (ul_rf_open) {
    srsran_rf_close(&ul_rf);
  }

  printf("Done.\n");
  return 0;
}
