#!/usr/bin/env python3
"""UL RX diversity test campaign over ZMQ.

Runs srsenb (1 or 2 RX antennas) + srsue with the per-antenna channel
emulator (channel_splitter.py) in between, across a matrix of real-world
impairment scenarios. Without a core network every connection attempt ends in
RRCConnectionReject after Msg3, so each attempt exercises exactly the paths
the diversity feature changed: multi-antenna PRACH detection and MRC PUSCH
(Msg3) decoding. srsue is restarted every cycle because NAS enters a 12-min
back-off after 5 attach attempts.

Usage:
  ./run_campaign.py --binaries <build_dir> [--calibrate] [--scenario NAME]...
"""
import argparse
import json
import os
import re
import signal
import socket
import statistics
import subprocess
import sys
import time

HERE     = os.path.dirname(os.path.abspath(__file__))
RESULTS  = os.path.join(HERE, "results")
PORTS    = [2000, 2101, 2201, 2202]

RE_PUSCH = re.compile(r"PUSCH: .*rnti=0x([0-9a-f]+).*crc=(OK|KO).*snr=([\-\d.]+) dB.*ta=([\-\d.]+) us")
RE_PRACH = re.compile(r"PRACH: .*preamble=(\d+), offset=([\-\d.]+) us, peak2avg=([\-\d.]+)")


def ports_free():
    for p in PORTS:
        s = socket.socket()
        try:
            s.bind(("127.0.0.1", p))
        except OSError:
            return False
        finally:
            s.close()
    return True


class Procs:
    def __init__(self):
        self.procs = []

    def start(self, cmd, logfile, cwd=HERE):
        f = open(logfile, "w")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=cwd, start_new_session=True)
        self.procs.append(p)
        return p

    def kill(self, p):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            p.wait(timeout=5)
        except Exception:
            pass
        if p in self.procs:
            self.procs.remove(p)

    def kill_all(self):
        for p in list(self.procs):
            self.kill(p)
        deadline = time.time() + 10
        while not ports_free() and time.time() < deadline:
            time.sleep(0.5)


def parse_enb_log(path):
    out = {"prach": [], "pusch": []}
    if not os.path.exists(path):
        return out
    with open(path, errors="replace") as f:
        for line in f:
            m = RE_PUSCH.search(line)
            if m:
                out["pusch"].append({"rnti": m.group(1), "crc": m.group(2) == "OK",
                                     "snr": float(m.group(3)), "ta": float(m.group(4))})
                continue
            m = RE_PRACH.search(line)
            if m:
                out["prach"].append({"preamble": int(m.group(1)), "offset_us": float(m.group(2)),
                                     "peak2avg": float(m.group(3))})
    return out


def count_ue_console(path):
    tx = comp = 0
    if os.path.exists(path):
        with open(path, errors="replace") as f:
            for line in f:
                if "Random Access Transmission" in line:
                    tx += 1
                elif "Random Access Complete" in line:
                    comp += 1
    return tx, comp


def run_scenario(name, enb_conf, splitter_args, binaries, ue_cycles, ue_cycle_s, tag_extra=""):
    print(f"\n=== scenario {name} ({enb_conf}, splitter: {' '.join(splitter_args)}) ===", flush=True)
    procs = Procs()
    enb_log     = os.path.join(RESULTS, f"{name}_enb.log")
    enb_console = os.path.join(RESULTS, f"{name}_enb_console.log")
    split_log   = os.path.join(RESULTS, f"{name}_splitter.log")
    try:
        if not ports_free():
            print("ports busy, waiting...", flush=True)
            time.sleep(5)
            if not ports_free():
                raise RuntimeError("ports still busy")
        procs.start([sys.executable, os.path.join(HERE, "channel_splitter.py")] + splitter_args, split_log)
        time.sleep(1)
        procs.start([os.path.join(binaries, "srsenb/src/srsenb"), os.path.join(HERE, enb_conf),
                     "--log.filename=" + enb_log], enb_console)
        time.sleep(6)

        ue_tx_total = ue_comp_total = 0
        for cyc in range(ue_cycles):
            ue_console = os.path.join(RESULTS, f"{name}_ue{cyc}.log")
            ue = procs.start([os.path.join(binaries, "srsue/src/srsue"), os.path.join(HERE, "ue.conf"),
                              "--log.filename=" + os.path.join(RESULTS, f"{name}_ue{cyc}_stack.log")],
                             ue_console)
            time.sleep(ue_cycle_s)
            procs.kill(ue)
            time.sleep(2)
            tx, comp = count_ue_console(ue_console)
            ue_tx_total += tx
            ue_comp_total += comp
            print(f"  cycle {cyc + 1}/{ue_cycles}: ra_tx={tx} ra_complete={comp}", flush=True)
    finally:
        procs.kill_all()

    enb = parse_enb_log(enb_log)
    ok  = [p for p in enb["pusch"] if p["crc"]]
    res = {
        "scenario": name,
        "enb_conf": enb_conf,
        "splitter_args": splitter_args,
        "ra_transmissions": ue_tx_total,
        "ra_completions": ue_comp_total,
        "prach_detections": len(enb["prach"]),
        "prach_peak2avg_mean": statistics.fmean(p["peak2avg"] for p in enb["prach"]) if enb["prach"] else None,
        "pusch_total": len(enb["pusch"]),
        "pusch_crc_ok": len(ok),
        "pusch_snr_mean": statistics.fmean(p["snr"] for p in ok) if ok else None,
        "pusch_snr_min": min((p["snr"] for p in ok), default=None),
        "pusch_ta_absmax": max((abs(p["ta"]) for p in ok), default=None),
    }
    with open(os.path.join(RESULTS, f"{name}.json"), "w") as f:
        json.dump(res, f, indent=2)
    print(f"  -> ra_tx={res['ra_transmissions']} prach_det={res['prach_detections']} "
          f"msg3={res['pusch_crc_ok']}/{res['pusch_total']} "
          f"snr_mean={res['pusch_snr_mean']}", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binaries", default="/home/user/srsRAN_4G/build")
    ap.add_argument("--noise-ref", type=float, required=True,
                    help="active UE burst power at the splitter in dBfs (from calibration)")
    ap.add_argument("--snr0", type=float, required=True,
                    help="calibrated SNR (dB) where 1rx Msg3 fails ~50%%")
    ap.add_argument("--scenario", action="append", help="run only these scenarios")
    ap.add_argument("--ue-cycles", type=int, default=3)
    ap.add_argument("--ue-cycle-s", type=int, default=140)
    args = ap.parse_args()

    os.makedirs(RESULTS, exist_ok=True)

    def noise_for(snr_db):
        return round(args.noise_ref - snr_db, 1)

    s0 = args.snr0
    scenarios = [
        # (name, enb_conf, splitter args)
        ("s1_benign_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--noise0", str(noise_for(s0 + 8)), "--noise1", str(noise_for(s0 + 8))]),
        ("s1_benign_1rx", "enb_1rx.conf",
         ["--mode", "1rx", "--noise0", str(noise_for(s0 + 8))]),
        ("s2_lowsnr_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--noise0", str(noise_for(s0)), "--noise1", str(noise_for(s0))]),
        ("s2_lowsnr_1rx", "enb_1rx.conf",
         ["--mode", "1rx", "--noise0", str(noise_for(s0))]),
        ("s3_imbalance_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--noise0", str(noise_for(s0 + 8)), "--noise1", str(noise_for(s0 - 2))]),
        ("s4_deadant_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--noise0", str(noise_for(s0 + 8)), "--off1", "--noise1", str(noise_for(s0 + 8))]),
        ("s5_rayleigh_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--fading", "rayleigh", "--doppler", "5",
          "--noise0", str(noise_for(s0 + 6)), "--noise1", str(noise_for(s0 + 6))]),
        ("s5_rayleigh_1rx", "enb_1rx.conf",
         ["--mode", "1rx", "--fading", "rayleigh", "--doppler", "5", "--noise0", str(noise_for(s0 + 6))]),
        ("s6_epa_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--fading", "epa", "--doppler", "5",
          "--noise0", str(noise_for(s0 + 6)), "--noise1", str(noise_for(s0 + 6))]),
        ("s7_skew_2rx", "enb_2rx.conf",
         ["--mode", "2rx", "--delay1", "3", "--phase1", "120", "--gain1", "-3",
          "--noise0", str(noise_for(s0 + 6)), "--noise1", str(noise_for(s0 + 6))]),
        ("s8_etu300_2rx", "enb_2rx_etu.conf",
         ["--mode", "2rx"]),
    ]

    results = []
    for name, conf, sargs in scenarios:
        if args.scenario and name not in args.scenario:
            continue
        results.append(run_scenario(name, conf, sargs, args.binaries, args.ue_cycles, args.ue_cycle_s))

    print("\n=== campaign done ===")
    for r in results:
        print(json.dumps(r))


if __name__ == "__main__":
    main()
