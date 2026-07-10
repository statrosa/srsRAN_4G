#!/bin/bash
#
# Copyright 2013-2023 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

# End-to-end test for lib/examples/dl_ul_capture_align over the ZeroMQ virtual
# radio, with a REAL srsenb transmitting to a real srsue.
#
#   srsepc + srsenb + srsue run over ZMQ with the transmitters in PUB mode and
#   all receivers in SUB mode, so a coherent 2-channel recorder
#   (zmq_dl_ul_record) can tap BOTH the eNB DL (port 2000) and the UE UL
#   (port 2001) without starving the UE. Uplink traffic (ping) forces the eNB
#   to issue UL grants / schedule PUSCH. The recorder writes two subframe-aligned
#   IQ files, which are replayed through dl_ul_capture_align's offline mode. The
#   test PASSES if the tool decodes at least one PUSCH with CRC=OK.
#
# Usage: sudo ./dl_ul_capture_align_zmq_test.sh [build_path] [nof_prb]
#   build_path defaults to the repo's ./build, nof_prb defaults to 6.

set -u

BUILD_PATH="${1:-$(cd "$(dirname "$0")/.." && pwd)/build}"
# Use the default 50-PRB cell: enb.conf.example / sib.conf.example PRACH config
# (prach_freq_offset=4) requires >= ~15 PRB, so n_prb=6 would fail PRACH.
NOF_PRB="${2:-50}"
SRC_PATH="$(cd "$(dirname "$0")/.." && pwd)"

BASE_SRATE="23.04e6"
CELL_ID=1          # rr.conf.example cell_id/pci = 1
N_ID_2=1           # pci % 3
UE_NETNS="ue1"
REC_SUBFRAMES=3000 # ~3 s of capture (11.52 Msps at 50 PRB)
RNTI_LIST="0x46,0x47,0x48,0x49,0x4a"

# ---- preflight: this test needs a full LTE stack that the sandbox may lack --
# srsEPC's S1AP requires kernel SCTP; without it the UE is rejected at RRC and
# never reaches RRC-CONNECTED, so no decodable format-0 UL grants are issued.
# (Run lib/examples/../test/dl_ul_capture_align_selftest.sh for a radio-free,
#  core-free decode check that works anywhere.)
if ! python3 -c "import socket,sys; socket.socket(socket.AF_INET,socket.SOCK_STREAM,132)" 2>/dev/null; then
  echo "SKIP: kernel has no SCTP support (needed by srsEPC S1AP)."
  echo "      Run this on a Linux host with SCTP; here, use dl_ul_capture_align_selftest.sh instead."
  exit 77
fi
if [ ! -x "$BUILD_PATH/srsepc/src/srsepc" ]; then
  echo "SKIP: srsEPC/srsENB/srsUE not built (configure with -DENABLE_SRSEPC=ON etc.)."
  exit 77
fi

WORK=$(mktemp -d /tmp/dlul_zmq.XXXXXX)
EPC_LOG="$WORK/epc.log"
ENB_LOG="$WORK/enb.log"
UE_LOG="$WORK/ue.log"
REC_LOG="$WORK/rec.log"
TOOL_LOG="$WORK/tool.log"
DL_IQ="$WORK/dl.iq"
UL_IQ="$WORK/ul.iq"

epc_pid=0; enb_pid=0; ue_pid=0; rec_pid=0; ping_pid=0

cleanup() {
  echo "== Tearing down =="
  [ "$ping_pid" -ne 0 ] && kill -9 "$ping_pid" 2>/dev/null
  [ "$rec_pid" -ne 0 ]  && kill -SIGINT "$rec_pid" 2>/dev/null; sleep 1
  [ "$ue_pid" -ne 0 ]   && kill -SIGTERM "$ue_pid" 2>/dev/null; sleep 2
  [ "$enb_pid" -ne 0 ]  && kill -SIGTERM "$enb_pid" 2>/dev/null; sleep 2
  [ "$epc_pid" -ne 0 ]  && kill -SIGTERM "$epc_pid" 2>/dev/null; sleep 2
  for p in "$rec_pid" "$ue_pid" "$enb_pid" "$epc_pid"; do
    [ "$p" -ne 0 ] && ps -p "$p" >/dev/null 2>&1 && kill -9 "$p" 2>/dev/null
  done
  ip netns delete "$UE_NETNS" 2>/dev/null
}
trap cleanup EXIT

fail() { echo "FAIL: $1"; exit 1; }

# --- binaries -------------------------------------------------------------
EPC_BIN="$BUILD_PATH/srsepc/src/srsepc"
ENB_BIN="$BUILD_PATH/srsenb/src/srsenb"
UE_BIN="$BUILD_PATH/srsue/src/srsue"
REC_BIN="$BUILD_PATH/lib/examples/zmq_dl_ul_record"
TOOL_BIN="$BUILD_PATH/lib/examples/dl_ul_capture_align"
for b in "$EPC_BIN" "$ENB_BIN" "$UE_BIN" "$REC_BIN" "$TOOL_BIN"; do
  [ -x "$b" ] || fail "binary not found/executable: $b"
done

# --- ZMQ device args (PUB transmitters, SUB receivers = fan-out) ----------
# PUB/SUB is the only native way a 3rd consumer (the recorder) can tap both
# links, but it drops the REQ/REP flow control that keeps eNB<->UE lock-stepped;
# on a slow/shared host the eNB may miss the UE uplink. If the UE cannot hold
# sync, use an external GNU Radio ZMQ broker for the tap and keep eNB<->UE on
# default REQ/REP.
ENB_RF="tx_type=pub,rx_type=sub,tx_port=tcp://*:2000,rx_port=tcp://localhost:2001,id=enb,base_srate=${BASE_SRATE}"
UE_RF="tx_type=pub,rx_type=sub,tx_port=tcp://*:2001,rx_port=tcp://localhost:2000,id=ue,base_srate=${BASE_SRATE}"
REC_RF="rx_type=sub,rx_port0=tcp://localhost:2000,rx_port1=tcp://localhost:2001,id=rec,base_srate=${BASE_SRATE}"

# --- netns ----------------------------------------------------------------
ip netns delete "$UE_NETNS" 2>/dev/null
ip netns add "$UE_NETNS" || fail "cannot create netns $UE_NETNS (need root)"

echo "== Starting srsEPC =="
nohup "$EPC_BIN" "$SRC_PATH/srsepc/epc.conf.example" \
  --hss.db_file="$SRC_PATH/srsepc/user_db.csv.example" \
  --log.filename="$EPC_LOG" >"$WORK/epc.stdout" 2>&1 &
epc_pid=$!
sleep 3
ps -p "$epc_pid" >/dev/null || fail "srsEPC did not start (see $WORK/epc.stdout)"

echo "== Starting srsENB (n_prb=$NOF_PRB, PUB/SUB) =="
nohup "$ENB_BIN" "$SRC_PATH/srsenb/enb.conf.example" \
  --enb_files.sib_config="$SRC_PATH/srsenb/sib.conf.example" \
  --enb_files.rr_config="$SRC_PATH/srsenb/rr.conf.example" \
  --enb_files.rb_config="$SRC_PATH/srsenb/rb.conf.example" \
  --enb.n_prb="$NOF_PRB" \
  --rf.device_name=zmq --rf.device_args="$ENB_RF" \
  --log.all_level=info --log.filename="$ENB_LOG" >"$WORK/enb.stdout" 2>&1 &
enb_pid=$!
sleep 3
ps -p "$enb_pid" >/dev/null || fail "srsENB did not start (see $WORK/enb.stdout)"

echo "== Starting srsUE (netns=$UE_NETNS, PUB/SUB) =="
nohup "$UE_BIN" "$SRC_PATH/srsue/ue.conf.example" \
  --rf.device_name=zmq --rf.device_args="$UE_RF" --rf.rx_gain=60 \
  --gw.netns="$UE_NETNS" \
  --log.all_level=info --log.filename="$UE_LOG" >"$WORK/ue.stdout" 2>&1 &
ue_pid=$!
sleep 3
ps -p "$ue_pid" >/dev/null || fail "srsUE did not start (see $WORK/ue.stdout)"

# --- wait for attach (tun_srsue gets an IP inside the netns) --------------
echo "== Waiting for UE attach (tun_srsue) =="
UE_IP=""
for i in $(seq 1 30); do
  UE_IP=$(ip netns exec "$UE_NETNS" ip addr show tun_srsue 2>/dev/null | grep -oP 'inet \K[0-9.]+')
  [ -n "$UE_IP" ] && break
  sleep 1
done
[ -n "$UE_IP" ] || fail "UE did not attach within 30s (see $UE_LOG)"
GW_IP="$(echo "$UE_IP" | cut -d. -f1-3).1"
echo "UE attached: $UE_IP, gateway $GW_IP"

# --- start the coherent 2-channel recorder --------------------------------
echo "== Starting recorder (tap DL:2000 + UL:2001) =="
nohup "$REC_BIN" -l "$N_ID_2" -p "$NOF_PRB" -A 2 -n "$REC_SUBFRAMES" \
  -o "$DL_IQ" -O "$UL_IQ" -a "$REC_RF" >"$REC_LOG" 2>&1 &
rec_pid=$!
sleep 2
ps -p "$rec_pid" >/dev/null || fail "recorder did not start (see $REC_LOG)"

# --- force uplink traffic (BSR -> UL grants -> PUSCH) ---------------------
echo "== Generating uplink traffic =="
ip netns exec "$UE_NETNS" ping -i 0.05 -c 200 "$GW_IP" >"$WORK/ping.log" 2>&1 &
ping_pid=$!

# --- let the recorder capture, then wait for it to finish -----------------
echo "== Recording... =="
for i in $(seq 1 20); do
  ps -p "$rec_pid" >/dev/null 2>&1 || break
  sleep 1
done

# grab the assigned C-RNTI(s) from the eNB log if present
CRNTI=$(grep -oiE "rnti=0x4[0-9a-f]+" "$ENB_LOG" 2>/dev/null | head -n1 | cut -d= -f2)
[ -n "$CRNTI" ] && RNTI_LIST="$CRNTI,$RNTI_LIST"
echo "Searching RNTIs: $RNTI_LIST"

# stop the live stack before the (offline) decode
cleanup
trap - EXIT

# --- replay through the tool ----------------------------------------------
case "$NOF_PRB" in
  6) SF_LEN=1920;; 15) SF_LEN=3840;; 25) SF_LEN=5760;;
  50) SF_LEN=11520;; 75) SF_LEN=15360;; 100) SF_LEN=23040;;
  *) SF_LEN=1920;;
esac
DL_BYTES=$(stat -c%s "$DL_IQ" 2>/dev/null || echo 0)
DL_SF=$(( DL_BYTES / 8 / SF_LEN ))
echo "== Recorded DL subframes: ~$DL_SF =="
[ "$DL_SF" -gt 10 ] || fail "recorder captured too few subframes ($DL_SF); check ZMQ sync (see $REC_LOG)"

echo "== Replaying capture through dl_ul_capture_align =="
"$TOOL_BIN" --dl-file "$DL_IQ" --ul-file "$UL_IQ" -c "$CELL_ID" -p "$NOF_PRB" \
  -r "$RNTI_LIST" -w 4 -n 1000000 2>&1 | tee "$TOOL_LOG"

# --- assertion ------------------------------------------------------------
GRANTS=$(grep -c "grant rnti=" "$TOOL_LOG")
CRC_OK=$(grep -c "CRC=OK" "$TOOL_LOG")
echo "== Result: grants detected=$GRANTS, PUSCH CRC=OK=$CRC_OK =="
if [ "$CRC_OK" -gt 0 ]; then
  echo "PASS: decoded $CRC_OK PUSCH transmission(s) with CRC=OK"
  echo "Artifacts in $WORK"
  exit 0
fi
echo "Artifacts kept in $WORK for inspection (enb=$ENB_LOG ue=$UE_LOG rec=$REC_LOG)"
fail "no PUSCH decoded with CRC=OK (grants seen: $GRANTS)"
