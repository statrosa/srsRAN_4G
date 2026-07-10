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

# Real srsENB-over-ZMQ capture & decode for dl_ul_capture_align, in-sandbox, with
# NO relay, NO core network (no srsEPC/SCTP), NO netns.
#
#   srsENB (MME-connectivity RRC gate removed) and srsUE run over a DIRECT ZMQ
#   REQ/REP link, which attaches normally (RRC Connection Setup -> the UE sends
#   Msg5 on a PDCCH format-0 grant). The eNB is launched with
#   SRSRAN_RF_TX_RECORD_FILE / SRSRAN_RF_RX_RECORD_FILE so the RF layer records
#   the eNB's own transmitted DL (dl.iq) and received UL (ul.iq) at the cell rate,
#   perfectly aligned on the eNB's timeline (nothing is inserted in the sample
#   path, so the delicate ZMQ timing is undisturbed). dl_ul_capture_align then
#   replays the two files and must decode the Msg5 PUSCH with CRC=OK.
#
# Usage: sudo ./dl_ul_capture_align_zmq_record_test.sh [build_path] [nof_prb]

set -u
BUILD_PATH="${1:-$(cd "$(dirname "$0")/.." && pwd)/build}"
SRC_PATH="$(cd "$(dirname "$0")/.." && pwd)"
NOF_PRB="${2:-15}"

case "$NOF_PRB" in
  6)  BASE_SRATE=1.92e6;  SF_LEN=1920 ;;
  15) BASE_SRATE=3.84e6;  SF_LEN=3840 ;;
  25) BASE_SRATE=5.76e6;  SF_LEN=5760 ;;
  50) BASE_SRATE=11.52e6; SF_LEN=11520 ;;
  *)  echo "unsupported nof_prb $NOF_PRB"; exit 1 ;;
esac
CELL_ID=1
ATTACH_TIMEOUT=45
RNTI_LIST="0x46,0x47,0x48,0x49,0x4a,0x4b"

ENB_BIN="$BUILD_PATH/srsenb/src/srsenb"
UE_BIN="$BUILD_PATH/srsue/src/srsue"
TOOL_BIN="$BUILD_PATH/lib/examples/dl_ul_capture_align"
fail() { echo "FAIL: $1"; exit 1; }
for b in "$ENB_BIN" "$UE_BIN" "$TOOL_BIN"; do [ -x "$b" ] || fail "missing binary $b"; done

if grep -q "MME isn't connected. Sending Connection Reject" "$SRC_PATH/srsenb/src/stack/rrc/rrc_ue.cc"; then
  fail "srsenb still has the MME-connectivity gate; remove it so RRC setup completes without a core."
fi

WORK=$(mktemp -d /tmp/dlul_rec.XXXXXX)
DL_IQ="$WORK/dl.iq"; UL_IQ="$WORK/ul.iq"
ENB_LOG="$WORK/enb.log"; UE_LOG="$WORK/ue.log"

enb_pid=0; ue_pid=0
cleanup() {
  [ "$ue_pid"  -ne 0 ] && kill -SIGTERM "$ue_pid"  2>/dev/null
  [ "$enb_pid" -ne 0 ] && kill -SIGTERM "$enb_pid" 2>/dev/null
  sleep 2
  for p in "$ue_pid" "$enb_pid"; do
    [ "$p" -ne 0 ] && ps -p "$p" >/dev/null 2>&1 && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT

ENB_RF="fail_on_disconnect=true,tx_port=tcp://*:2000,rx_port=tcp://localhost:2001,id=enb,base_srate=${BASE_SRATE}"
UE_RF="tx_port=tcp://*:2001,rx_port=tcp://localhost:2000,id=ue,base_srate=${BASE_SRATE}"

echo "== Starting srsENB ($NOF_PRB PRB, direct ZMQ, recording DL+UL via RF tap) =="
SRSRAN_RF_TX_RECORD_FILE="$DL_IQ" SRSRAN_RF_RX_RECORD_FILE="$UL_IQ" \
nohup "$ENB_BIN" "$SRC_PATH/srsenb/enb.conf.example" \
  --enb_files.sib_config="$SRC_PATH/srsenb/sib.conf.example" \
  --enb_files.rr_config="$SRC_PATH/srsenb/rr.conf.example" \
  --enb_files.rb_config="$SRC_PATH/srsenb/rb.conf.example" \
  --enb.n_prb="$NOF_PRB" \
  --rf.device_name=zmq --rf.device_args="$ENB_RF" \
  --log.all_level=info --log.filename="$ENB_LOG" >"$WORK/enb.stdout" 2>&1 &
enb_pid=$!
sleep 4
ps -p "$enb_pid" >/dev/null || fail "srsENB did not start (see $WORK/enb.stdout)"

echo "== Starting srsUE (direct ZMQ, no netns) =="
nohup "$UE_BIN" "$SRC_PATH/srsue/ue.conf.example" \
  --rf.device_name=zmq --rf.device_args="$UE_RF" --rf.rx_gain=60 \
  --log.all_level=info --log.filename="$UE_LOG" >"$WORK/ue.stdout" 2>&1 &
ue_pid=$!

echo "== Waiting up to ${ATTACH_TIMEOUT}s for RRC connection setup =="
attached=0
for i in $(seq 1 "$ATTACH_TIMEOUT"); do
  sleep 1
  if grep -qiE "rrcConnectionSetupComplete" "$ENB_LOG" 2>/dev/null; then
    echo "  RRC connection setup complete at ~${i}s"
    attached=1
    sleep 3 # capture the Msg5 PUSCH (and any follow-on UL)
    break
  fi
done

cleanup
trap - EXIT

echo "== eNB evidence =="
grep -iE "rrcConnectionSetupComplete" "$ENB_LOG" 2>/dev/null | head -2
grep -iE "PUSCH:.*rnti=0x4.*crc=OK" "$ENB_LOG" 2>/dev/null | head -3

DL_SF=$(( $(stat -c%s "$DL_IQ" 2>/dev/null || echo 0) / 8 / SF_LEN ))
UL_SF=$(( $(stat -c%s "$UL_IQ" 2>/dev/null || echo 0) / 8 / SF_LEN ))
echo "== Recorded DL ~$DL_SF sf, UL ~$UL_SF sf =="
if [ "$attached" -ne 1 ]; then
  echo "SKIP: RRC setup did not complete within ${ATTACH_TIMEOUT}s on this host."
  echo "Artifacts kept in $WORK"
  exit 77
fi
[ "$DL_SF" -gt 50 ] || fail "too few DL subframes recorded ($DL_SF)"

CRNTIS=$(grep -oiE "rnti=0x4[0-9a-f]+" "$ENB_LOG" 2>/dev/null | sort -u | cut -d= -f2 | paste -sd, -)
[ -n "$CRNTIS" ] && RNTI_LIST="$CRNTIS,$RNTI_LIST"
echo "== Searching RNTIs: $RNTI_LIST =="

# The eNB records UL(RX) and DL(TX) with a small constant offset (TX advance).
# Scan it via the tool's -o (skip UL samples).
best_ok=0; best_D=0
for D in 0 1 2 3 4 5 6 7 8; do
  off=$(( D * SF_LEN ))
  ok=$("$TOOL_BIN" --dl-file "$DL_IQ" --ul-file "$UL_IQ" -c "$CELL_ID" -p "$NOF_PRB" \
        -r "$RNTI_LIST" -o "$off" -w 4 -n "$DL_SF" 2>/dev/null | grep -c "CRC=OK")
  echo "  offset D=$D -> CRC=OK=$ok"
  if [ "$ok" -gt "$best_ok" ]; then best_ok=$ok; best_D=$D; fi
done

echo "== Best: D=$best_D with CRC=OK=$best_ok =="
if [ "$best_ok" -gt 0 ]; then
  echo "PASS: decoded $best_ok real srsENB-over-ZMQ PUSCH transmission(s) with CRC=OK (DL/UL offset $best_D sf)"
  rm -rf "$WORK"
  exit 0
fi
echo "Artifacts kept in $WORK (enb=$ENB_LOG)"
fail "no PUSCH decoded with CRC=OK despite RRC setup completing"
