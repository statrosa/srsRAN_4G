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
 *  File:         zmq_dl_ul_record.c
 *
 *  Description:  Coherent two-channel DL+UL recorder for the dl_ul_capture_align
 *                ZeroMQ end-to-end test.
 *
 *                Opens ONE radio handle with two RX channels (ch0 = DL from the
 *                eNB, ch1 = UL from the UE), synchronises to the DL cell on
 *                channel 0 with srsran_ue_sync, and writes both channels to two
 *                SEPARATE SRSRAN_COMPLEX_FLOAT_BIN files, subframe-aligned.
 *
 *                Because both channels are pulled through a single handle they
 *                share one sample clock, and writing starts on a subframe
 *                boundary, so dl.iq[k] and ul.iq[k] correspond to the same TTI.
 *                Replaying the two files through dl_ul_capture_align's
 *                --dl-file/--ul-file mode then aligns a DL grant at subframe n
 *                with the PUSCH at subframe n+4 with zero extra offset.
 *
 *                Although written for the ZMQ test, it works with any RF backend
 *                that presents two coherent RX channels (e.g. one B210 with two
 *                RX antennas). Modelled on usrp_capture_sync.c.
 *****************************************************************************/

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "srsran/phy/rf/rf.h"
#include "srsran/srsran.h"

static bool           keep_running   = true;
static char*          dl_file_name   = NULL;
static char*          ul_file_name   = NULL;
static char           rf_args[256]   = "auto";
static float          rf_gain        = 40.0;
static float          rf_freq        = 1e9; // ZMQ ignores the actual value
static int            nof_prb        = 6;
static int            nof_subframes  = -1;
static int            N_id_2         = -1;
static uint32_t       nof_channels   = 2;
static bool           use_std_rates  = false;
static srsran_ue_sync_mode_t sync_mode = SYNC_MODE_PSS;

static void int_handler(int dummy)
{
  keep_running = false;
}

static void usage(char* prog)
{
  printf("Usage: %s [agpnvAe] -l N_id_2 -o dl_out_file -O ul_out_file\n", prog);
  printf("\t-a RF args [Default %s]\n", rf_args);
  printf("\t-g RF gain [Default %.2f dB]\n", rf_gain);
  printf("\t-f RF freq (ignored by ZMQ) [Default %.1f]\n", rf_freq);
  printf("\t-p nof_prb [Default %d]\n", nof_prb);
  printf("\t-l force N_id_2 (0..2). For pci=1 use 1 [Default auto]\n");
  printf("\t-A nof channels: ch0=DL, ch1=UL [Default %d]\n", nof_channels);
  printf("\t-n nof_subframes to record [Default %d = until Ctrl+C]\n", nof_subframes);
  printf("\t-o DL output file (channel 0)\n");
  printf("\t-O UL output file (channel 1)\n");
  printf("\t-e use standard LTE sample rates\n");
  printf("\t-v verbose\n");
}

static void parse_args(int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "agfpnlAoOev")) != -1) {
    switch (opt) {
      case 'a':
        strncpy(rf_args, argv[optind], 255);
        rf_args[255] = '\0';
        break;
      case 'g':
        rf_gain = strtof(argv[optind], NULL);
        break;
      case 'f':
        rf_freq = strtof(argv[optind], NULL);
        break;
      case 'p':
        nof_prb = (int)strtol(argv[optind], NULL, 10);
        break;
      case 'n':
        nof_subframes = (int)strtol(argv[optind], NULL, 10);
        break;
      case 'l':
        N_id_2 = (int)strtol(argv[optind], NULL, 10);
        break;
      case 'A':
        nof_channels = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'o':
        dl_file_name = argv[optind];
        break;
      case 'O':
        ul_file_name = argv[optind];
        break;
      case 'e':
        use_std_rates = true;
        break;
      case 'v':
        increase_srsran_verbose_level();
        break;
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
  if (N_id_2 == -1 || dl_file_name == NULL || ul_file_name == NULL || nof_channels < 2) {
    usage(argv[0]);
    exit(-1);
  }
}

static int srsran_rf_recv_wrapper(void* h, cf_t* data[SRSRAN_MAX_PORTS], uint32_t nsamples, srsran_timestamp_t* t)
{
  void* ptr[SRSRAN_MAX_PORTS];
  for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
    ptr[i] = data[i];
  }
  return srsran_rf_recv_with_time_multi(h, ptr, nsamples, true, &t->full_secs, &t->frac_secs);
}

int main(int argc, char** argv)
{
  cf_t*             buffer[SRSRAN_MAX_CHANNELS] = {NULL};
  int               n                           = 0;
  srsran_rf_t       rf                          = {};
  srsran_filesink_t dl_sink                     = {};
  srsran_filesink_t ul_sink                     = {};
  srsran_ue_sync_t  ue_sync                     = {};
  srsran_cell_t     cell                        = {};

  signal(SIGINT, int_handler);
  parse_args(argc, argv);

  srsran_use_standard_symbol_size(use_std_rates);

  srsran_filesink_init(&dl_sink, dl_file_name, SRSRAN_COMPLEX_FLOAT_BIN);
  srsran_filesink_init(&ul_sink, ul_file_name, SRSRAN_COMPLEX_FLOAT_BIN);

  printf("Opening RF device with %d channels (ch0=DL, ch1=UL)...\n", nof_channels);
  if (srsran_rf_open_multi(&rf, rf_args, nof_channels)) {
    ERROR("Error opening rf");
    exit(-1);
  }

  uint32_t max_num_samples = 3 * SRSRAN_SF_LEN_MAX;
  for (uint32_t i = 0; i < nof_channels; i++) {
    buffer[i] = srsran_vec_cf_malloc(max_num_samples);
  }

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);

  srsran_rf_set_rx_gain(&rf, rf_gain);
  printf("Set RX freq: %.6f MHz\n", srsran_rf_set_rx_freq(&rf, nof_channels, rf_freq) / 1000000);
  printf("Set RX gain: %.1f dB\n", srsran_rf_get_rx_gain(&rf));

  int srate = srsran_sampling_freq_hz(nof_prb);
  if (srate < 0) {
    ERROR("Invalid number of PRB %d", nof_prb);
    exit(-1);
  }
  printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
  float srate_rf = srsran_rf_set_rx_srate(&rf, (double)srate);
  if ((int)srate_rf != srate) {
    ERROR("Could not set sampling rate (wanted %d got %f)", srate, srate_rf);
    exit(-1);
  }
  srsran_rf_start_rx_stream(&rf, false);

  cell.cp        = SRSRAN_CP_NORM;
  cell.id        = N_id_2;
  cell.nof_prb   = nof_prb;
  cell.nof_ports = 1;

  // Synchronise on channel 0 (DL). Channels are pulled coherently through the
  // single handle, so channel 1 (UL) is captured on the same sample clock.
  if (srsran_ue_sync_init_multi_decim_mode(
          &ue_sync, cell.nof_prb, cell.id == 1000, srsran_rf_recv_wrapper, nof_channels, (void*)&rf, 1, sync_mode)) {
    ERROR("Error initiating ue_sync");
    exit(-1);
  }
  if (srsran_ue_sync_set_cell(&ue_sync, cell)) {
    ERROR("Error initiating ue_sync");
    exit(-1);
  }

  uint32_t sf_count      = 0;
  bool     start_capture = false;
  bool     stop_capture  = false;
  uint32_t sf_len        = SRSRAN_SF_LEN_PRB(nof_prb);

  printf("Waiting for DL subframe #9 boundary to start aligned capture...\n");
  while ((sf_count < (uint32_t)nof_subframes || nof_subframes == -1) && !stop_capture) {
    n = srsran_ue_sync_zerocopy(&ue_sync, buffer, max_num_samples);
    if (n < 0) {
      ERROR("Error receiving samples");
      exit(-1);
    }
    if (n == 1) {
      if (!start_capture) {
        // Begin on the sf9->sf0 boundary so file sample 0 is a subframe boundary.
        if (srsran_ue_sync_get_sfidx(&ue_sync) == 9) {
          start_capture = true;
        }
      } else {
        // Write each channel to its own file (no interleaving).
        srsran_filesink_write(&dl_sink, buffer[0], sf_len);
        srsran_filesink_write(&ul_sink, buffer[1], sf_len);
        sf_count++;
        if ((sf_count % 100) == 0) {
          printf("Recorded %6d subframes...\r", sf_count);
          fflush(stdout);
        }
      }
    }
    if (!keep_running) {
      if (!start_capture || (start_capture && srsran_ue_sync_get_sfidx(&ue_sync) == 9)) {
        stop_capture = true;
      }
    }
  }

  srsran_filesink_free(&dl_sink);
  srsran_filesink_free(&ul_sink);
  srsran_rf_close(&rf);
  srsran_ue_sync_free(&ue_sync);
  for (uint32_t i = 0; i < nof_channels; i++) {
    if (buffer[i]) {
      free(buffer[i]);
    }
  }

  printf("\nDone - wrote %d subframes to %s (DL) and %s (UL)\n", sf_count, dl_file_name, ul_file_name);
  return SRSRAN_SUCCESS;
}
