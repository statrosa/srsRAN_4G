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
 *  File:         zmq_reqrep_relay.c
 *
 *  Description:  Transparent ZeroMQ REQ/REP recording relay for the
 *                dl_ul_capture_align over-the-air-less test.
 *
 *                srsRAN's ZMQ RF driver uses a request/reply handshake: the
 *                receiver (ZMQ_REQ) sends a 1-byte request and the transmitter
 *                (ZMQ_REP) replies with one IQ block. That link is strictly
 *                point-to-point, so a third consumer cannot tap it. This relay
 *                inserts itself between srsENB and srsUE on BOTH directions,
 *                faithfully forwarding each request/reply (so the request-paced
 *                timing that keeps eNB and UE locked is preserved) while writing
 *                a copy of every IQ block to a file. The two recordings can then
 *                be replayed through dl_ul_capture_align.
 *
 *                Per direction it binds a REP socket the real consumer connects
 *                to, and connects a REQ socket to the real producer:
 *
 *                  consumer(REQ) -> [REP relay REQ] -> producer(REP)
 *                                        |
 *                                        +-> append IQ block to file
 *
 *                With base_srate == cell_rate on both eNB and UE, the blocks are
 *                already at the cell sample rate, so the files replay directly.
 *****************************************************************************/

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <zmq.h>

#define RELAY_MAX_BLOCK (8 * 1024 * 1024) // >> one subframe at any supported rate

static volatile bool keep_running = true;

static void sig_handler(int signo)
{
  (void)signo;
  keep_running = false;
}

typedef struct {
  void*       ctx;
  const char* name;
  const char* rep_bind;    // consumer side: real RX (ZMQ_REQ) connects here
  const char* req_connect; // producer side: real TX (ZMQ_REP) is bound here
  const char* out_file;
  uint64_t    nblocks;
  uint64_t    nbytes;
} relay_dir_t;

// One direction: shuttle request/reply and record each reply block.
static void* relay_thread(void* arg)
{
  relay_dir_t* d = (relay_dir_t*)arg;

  void* rep = zmq_socket(d->ctx, ZMQ_REP); // faces the consumer (real RX)
  void* req = zmq_socket(d->ctx, ZMQ_REQ); // faces the producer (real TX)
  if (!rep || !req) {
    fprintf(stderr, "[%s] error creating sockets\n", d->name);
    return NULL;
  }

  // Outer REP recv times out so we can notice shutdown; inner REQ transaction
  // blocks (the producer is request-paced and replies promptly).
  int rcvtimeo = 500;
  zmq_setsockopt(rep, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
  int linger = 0;
  zmq_setsockopt(rep, ZMQ_LINGER, &linger, sizeof(linger));
  zmq_setsockopt(req, ZMQ_LINGER, &linger, sizeof(linger));

  if (zmq_bind(rep, d->rep_bind) != 0) {
    fprintf(stderr, "[%s] error binding %s: %s\n", d->name, d->rep_bind, zmq_strerror(zmq_errno()));
    goto out;
  }
  if (zmq_connect(req, d->req_connect) != 0) {
    fprintf(stderr, "[%s] error connecting %s: %s\n", d->name, d->req_connect, zmq_strerror(zmq_errno()));
    goto out;
  }

  FILE* f = fopen(d->out_file, "wb");
  if (!f) {
    fprintf(stderr, "[%s] error opening %s\n", d->name, d->out_file);
    goto out;
  }

  uint8_t* block = malloc(RELAY_MAX_BLOCK);
  uint8_t  reqbuf[16];
  uint8_t  dummy = 0xff;

  while (keep_running) {
    // 1. Receive the consumer's request (blocking with timeout for shutdown).
    int rc = zmq_recv(rep, reqbuf, sizeof(reqbuf), 0);
    if (rc < 0) {
      if (zmq_errno() == EAGAIN || zmq_errno() == EINTR) {
        continue; // no request yet - check keep_running and retry
      }
      break;
    }

    // 2. Forward a request to the producer.
    if (zmq_send(req, &dummy, sizeof(dummy), 0) < 0) {
      break;
    }

    // 3. Receive the producer's IQ block.
    int n = zmq_recv(req, block, RELAY_MAX_BLOCK, 0);
    if (n < 0) {
      break;
    }

    // 4. Record it (flush so a killed relay still leaves a valid file).
    if (n > 0) {
      fwrite(block, 1, (size_t)n, f);
      d->nblocks++;
      d->nbytes += (uint64_t)n;
      if ((d->nblocks & 0x3f) == 0) {
        fflush(f);
      }
    }

    // 5. Serve the block back to the consumer.
    if (zmq_send(rep, block, (size_t)n, 0) < 0) {
      break;
    }
  }

  fflush(f);
  fclose(f);
  free(block);

out:
  zmq_close(rep);
  zmq_close(req);
  printf("[%s] relayed %lu blocks (%lu bytes) -> %s\n",
         d->name,
         (unsigned long)d->nblocks,
         (unsigned long)d->nbytes,
         d->out_file);
  return NULL;
}

static void usage(const char* prog)
{
  printf("Usage: %s [options]\n", prog);
  printf("  DL (eNB TX -> UE RX), UL (UE TX -> eNB RX). Ports 2000/2001 real, 3000/3001 relay.\n");
  printf("  -a DL producer endpoint (eNB TX, REQ connect) [tcp://localhost:2000]\n");
  printf("  -b DL consumer endpoint (to UE RX, REP bind)   [tcp://*:3000]\n");
  printf("  -c UL producer endpoint (UE TX, REQ connect)   [tcp://localhost:2001]\n");
  printf("  -e UL consumer endpoint (to eNB RX, REP bind)  [tcp://*:3001]\n");
  printf("  -o DL output IQ file [dl.iq]\n");
  printf("  -O UL output IQ file [ul.iq]\n");
}

int main(int argc, char** argv)
{
  const char* dl_req = "tcp://localhost:2000";
  const char* dl_rep = "tcp://*:3000";
  const char* ul_req = "tcp://localhost:2001";
  const char* ul_rep = "tcp://*:3001";
  const char* dl_out = "dl.iq";
  const char* ul_out = "ul.iq";

  int opt;
  while ((opt = getopt(argc, argv, "a:b:c:e:o:O:h")) != -1) {
    switch (opt) {
      case 'a':
        dl_req = optarg;
        break;
      case 'b':
        dl_rep = optarg;
        break;
      case 'c':
        ul_req = optarg;
        break;
      case 'e':
        ul_rep = optarg;
        break;
      case 'o':
        dl_out = optarg;
        break;
      case 'O':
        ul_out = optarg;
        break;
      default:
        usage(argv[0]);
        exit(0);
    }
  }

  signal(SIGINT, sig_handler);
  signal(SIGTERM, sig_handler);

  void* ctx = zmq_ctx_new();
  if (!ctx) {
    fprintf(stderr, "Error creating ZMQ context\n");
    exit(-1);
  }

  relay_dir_t dl = {.ctx = ctx, .name = "DL", .rep_bind = dl_rep, .req_connect = dl_req, .out_file = dl_out};
  relay_dir_t ul = {.ctx = ctx, .name = "UL", .rep_bind = ul_rep, .req_connect = ul_req, .out_file = ul_out};

  printf("Relay DL: %s (eNB) -> record %s -> %s (UE)\n", dl_req, dl_out, dl_rep);
  printf("Relay UL: %s (UE)  -> record %s -> %s (eNB)\n", ul_req, ul_out, ul_rep);

  pthread_t dl_t, ul_t;
  pthread_create(&dl_t, NULL, relay_thread, &dl);
  pthread_create(&ul_t, NULL, relay_thread, &ul);

  pthread_join(dl_t, NULL);
  pthread_join(ul_t, NULL);

  zmq_ctx_destroy(ctx);
  printf("Relay stopped.\n");
  return 0;
}
