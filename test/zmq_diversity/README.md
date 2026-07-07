# UL RX diversity — ZMQ test campaign

End-to-end validation of srsenb uplink receive diversity (MRC) under
realistic per-antenna channel conditions, using the ZMQ RF device.
No RF hardware required.

## Topology

```
srsue ---DL(clean)--------------------------------- srsenb tx_port0 :2000
srsue tx_port0 :2101 --> channel_splitter.py --+--> srsenb rx_port0 :2201
                        (per-antenna channels) +--> srsenb rx_port1 :2202
```

`channel_splitter.py` duplicates the UE uplink onto both eNB RX antennas and
applies an independent impairment chain per antenna in physically correct
order: fixed gain/phase -> sample delay -> fading (flat Rayleigh or EPA
multipath, independent realizations) -> AWGN at per-antenna noise power
(noise after fading, so fades genuinely reduce SNR).

The downlink is left clean on purpose: it isolates the uplink receive path,
which is what the diversity feature changes.

## Why Msg3 is the measurement

Without a core network (or on kernels without SCTP), every UE connection
attempt ends with RRCConnectionReject after Msg3. Each attempt still
exercises the full new UL chain: multi-antenna PRACH detection and MRC PUSCH
decoding. The campaign therefore counts, per scenario: PRACH detections,
Msg3 PUSCH CRC outcomes, reported (combined) SNR and timing advance.
srsue re-attempts every ~25 s (NAS T3410/T3411) and backs off for 12 min
after 5 attempts (T3402), so the orchestrator restarts srsue each cycle.

With srsepc on an SCTP-capable kernel the same setup completes full attach
and the campaign can be extended with iperf-driven PUSCH traffic.

## Running

```sh
# 1. Calibrate: measure the UE burst power at the splitter (clean run),
#    then find the SNR where single-antenna Msg3 starts failing:
./calib_run.sh calib0 enb_2rx.conf --mode 2rx     # read active_pwr_med from results/calib0_splitter.log
RUNTIME=140 ./calib_run.sh snrX enb_1rx.conf --mode 1rx --noise0 <active_pwr_med - SNR>

# 2. Full campaign (about 2 h):
./run_campaign.py --noise-ref <active_pwr_med> --snr0 <calibrated SNR>
```

Results land in `results/<scenario>.json` (+ raw eNB/UE/splitter logs);
the campaign summary is in `RESULTS.md`.

Configs assume 25 PRB, `base_srate=5.76e6`, `nof_ports=1`, `tm=1` and
`nof_rx_ant=2` (or 1 for the comparison arms).
