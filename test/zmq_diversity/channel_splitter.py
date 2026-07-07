#!/usr/bin/env python3
"""Per-antenna uplink channel emulator for srsRAN ZMQ setups.

Duplicates a srsRAN ZMQ UL sample stream (UE transmitter, REQ/REP protocol)
onto one or two eNB RX channels, applying an independent impairment chain to
each antenna in physically correct order:

    fixed gain/phase -> integer-sample delay -> fading -> AWGN

Noise is added AFTER fading so that fades genuinely reduce the SNR, and it is
also added to idle (zero) chunks, like a real receiver front-end.

The srsRAN ZMQ RF device uses a REQ/REP pair per channel: the receiver (REQ)
sends a 1-byte dummy request and the transmitter (REP) replies with a chunk of
cf32 samples. This script REQ-fetches from the UE transmitter and serves one
REP endpoint per eNB RX antenna.
"""
import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import zmq

SRATE = 5.76e6  # must match base_srate in the eNB/UE configs

# EPA tap profile (TS 36.104 B.2), delays in ns / powers in dB
EPA_DELAYS_NS = [0, 30, 70, 90, 110, 190, 410]
EPA_POWERS_DB = [0.0, -1.0, -1.0, -2.0, -3.0, -8.0, -20.8]


class AntennaChain:
    """Impairment chain for one RX antenna."""

    def __init__(self, idx, gain_db, phase_deg, delay_samp, fading, doppler, noise_dbfs, seed):
        self.idx  = idx
        self.rng  = np.random.default_rng(seed)
        self.gain = 0.0 if gain_db is None else 10.0 ** (gain_db / 20.0)
        self.gain = complex(self.gain * math.cos(math.radians(phase_deg)),
                            self.gain * math.sin(math.radians(phase_deg)))
        self.delay_buf = np.zeros(delay_samp, dtype=np.complex64) if delay_samp > 0 else None
        self.noise_std = 10.0 ** (noise_dbfs / 20.0) if noise_dbfs is not None else 0.0
        self.fading    = fading
        self.doppler   = max(0.01, doppler)

        if fading == "rayleigh":
            self.taps_delay = np.array([0])
            self.taps_pwr   = np.array([1.0])
        elif fading == "epa":
            # Bucket the EPA taps into whole samples at SRATE
            delays = np.round(np.array(EPA_DELAYS_NS) * 1e-9 * SRATE).astype(int)
            pwr    = 10.0 ** (np.array(EPA_POWERS_DB) / 10.0)
            nmax   = delays.max() + 1
            bucket = np.zeros(nmax)
            for d, p in zip(delays, pwr):
                bucket[d] += p
            self.taps_delay = np.nonzero(bucket)[0]
            self.taps_pwr   = bucket[self.taps_delay]
            self.taps_pwr  /= self.taps_pwr.sum()  # unit average channel power
        else:
            self.taps_delay = None

        if self.taps_delay is not None:
            n = len(self.taps_delay)
            self.h = (self.rng.standard_normal(n) + 1j * self.rng.standard_normal(n)) / math.sqrt(2)
            self.h = self.h * np.sqrt(self.taps_pwr)
            self.conv_state = np.zeros(int(self.taps_delay.max()), dtype=np.complex64)

    def _evolve_taps(self, dt):
        """First-order Gauss-Markov tap evolution; correlation set by Doppler."""
        rho = math.exp(-((math.pi * self.doppler * dt) ** 2) / 2.0)
        n   = len(self.taps_delay)
        w   = (self.rng.standard_normal(n) + 1j * self.rng.standard_normal(n)) / math.sqrt(2)
        w   = w * np.sqrt(self.taps_pwr)
        return rho * self.h + math.sqrt(1.0 - rho * rho) * w

    def process(self, x):
        """Apply the impairment chain to one chunk of complex64 samples."""
        y = x * self.gain

        if self.delay_buf is not None and len(self.delay_buf) > 0:
            d = len(self.delay_buf)
            y = np.concatenate((self.delay_buf, y))
            self.delay_buf = y[-d:].copy()
            y = y[:-d]

        if self.taps_delay is not None:
            h_next = self._evolve_taps(len(x) / SRATE)
            if len(self.taps_delay) == 1 and self.taps_delay[0] == 0:
                # Flat fading: interpolate the single tap across the chunk
                ramp = np.linspace(0.0, 1.0, len(y), endpoint=False)
                hvec = self.h * (1.0 - ramp) + h_next * ramp
                y = y * hvec
            else:
                # Sparse FIR with per-chunk taps (block fading within a chunk)
                L   = int(self.taps_delay.max())
                inp = np.concatenate((self.conv_state, y))
                out = np.zeros(len(y), dtype=np.complex128)
                for tap, delay in zip(self.h, self.taps_delay):
                    out += tap * inp[L - delay : L - delay + len(y)]
                self.conv_state = inp[-L:].copy() if L > 0 else self.conv_state
                y = out
            self.h = h_next

        if self.noise_std > 0.0:
            n = (self.rng.standard_normal(len(y)) + 1j * self.rng.standard_normal(len(y)))
            y = y + n * (self.noise_std / math.sqrt(2.0))

        return y.astype(np.complex64)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", choices=["1rx", "2rx"], default="2rx")
    ap.add_argument("--ue-tx", default="tcp://localhost:2101")
    ap.add_argument("--ports", type=int, nargs="+", default=[2201, 2202])
    for a in (0, 1):
        ap.add_argument(f"--gain{a}", type=float, default=0.0, help=f"antenna {a} gain in dB (use 'off' via --off{a})")
        ap.add_argument(f"--off{a}", action="store_true", help=f"antenna {a} carries no signal (noise only)")
        ap.add_argument(f"--phase{a}", type=float, default=0.0, help="phase rotation in degrees")
        ap.add_argument(f"--delay{a}", type=int, default=0, help="delay in samples")
        ap.add_argument(f"--noise{a}", type=float, default=None, help="noise power in dBfs (absolute)")
    ap.add_argument("--fading", choices=["none", "rayleigh", "epa"], default="none")
    ap.add_argument("--doppler", type=float, default=5.0, help="max Doppler in Hz")
    ap.add_argument("--stats-every", type=int, default=20000, help="chunks between stats prints")
    args = ap.parse_args()

    nof_ant = 1 if args.mode == "1rx" else 2
    chains = []
    for a in range(nof_ant):
        gain = None if getattr(args, f"off{a}") else getattr(args, f"gain{a}")
        chains.append(AntennaChain(a,
                                   gain,
                                   getattr(args, f"phase{a}"),
                                   getattr(args, f"delay{a}"),
                                   args.fading,
                                   args.doppler,
                                   getattr(args, f"noise{a}"),
                                   seed=1000 + a))

    ctx = zmq.Context()
    poller = zmq.Poller()

    def fresh_ue_socket(old=None):
        # Lazy-pirate reconnect: a strict REQ socket that lost its peer mid-request
        # (e.g. the UE was restarted) must be recreated to leave the wait-for-reply state.
        if old is not None:
            poller.unregister(old)
            old.close(linger=0)
        sock = ctx.socket(zmq.REQ)
        sock.connect(args.ue_tx)
        poller.register(sock, zmq.POLLIN)
        return sock

    ue = fresh_ue_socket()

    reps = []
    for port in args.ports[:nof_ant]:
        s = ctx.socket(zmq.REP)
        s.bind(f"tcp://*:{port}")
        reps.append(s)

    queues  = [deque() for _ in reps]
    pending = [False] * len(reps)

    for s in reps:
        poller.register(s, zmq.POLLIN)

    fetch_outstanding = False
    fetch_sent_at     = 0.0
    chunks       = 0
    active       = 0
    active_pwr   = deque(maxlen=512)  # dBfs of recent active chunks

    print(f"channel_splitter: {args.ue_tx} -> {args.ports[:nof_ant]} "
          f"fading={args.fading} doppler={args.doppler}", flush=True)
    while True:
        events = dict(poller.poll(timeout=100))

        for i, s in enumerate(reps):
            if s in events and not pending[i]:
                s.recv()
                pending[i] = True

        if ue in events and fetch_outstanding:
            data = ue.recv()
            fetch_outstanding = False
            x = np.frombuffer(data, dtype=np.complex64)
            p = float(np.mean(np.abs(x) ** 2)) if len(x) else 0.0
            if p > 1e-9:
                active += 1
                active_pwr.append(10.0 * math.log10(p))
            for i, q in enumerate(queues):
                q.append(chains[i].process(x).tobytes())
            chunks += 1
            if chunks % args.stats_every == 0:
                med = np.median(active_pwr) if active_pwr else float("nan")
                print(f"channel_splitter: chunks={chunks} active={active} "
                      f"active_pwr_med={med:.1f} dBfs", flush=True)

        for i, s in enumerate(reps):
            if pending[i] and queues[i]:
                s.send(queues[i].popleft())
                pending[i] = False

        now = time.monotonic()
        if fetch_outstanding and now - fetch_sent_at > 2.0:
            # The UE likely restarted and the request was lost; rebuild the socket and re-send
            ue = fresh_ue_socket(ue)
            ue.send(b"\xff")
            fetch_sent_at = now

        if not fetch_outstanding and min(len(q) for q in queues) < 8:
            ue.send(b"\xff")
            fetch_outstanding = True
            fetch_sent_at     = now


if __name__ == "__main__":
    sys.exit(main())
