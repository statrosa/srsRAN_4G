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

# Demanding, realistic regression for lib/examples/dl_ul_capture_align: RAR
# auto-learn + active C-RNTI pool + multiple RNTIs per TTI.
#
# gen_dl_ul_multiuser emits (via srsRAN's real PHY) a multi-user cell where UEs
# attach over time: each UE gets an RA-RNTI RAR (Temp C-RNTI + Msg3 grant) and a
# Msg3 PUSCH at n+6, then dynamic DCI-0 grants - several UEs per subframe on
# non-overlapping PRBs, their PUSCH summed into one UL subframe at n+4.
#
# dl_ul_capture_align is run with NO -r: it must learn every C-RNTI from the RARs,
# decode each Msg3, and decode all the dynamic PUSCH (including several distinct
# RNTIs sharing one UL subframe) with CRC=OK.
#
# Usage: ./dl_ul_capture_align_rar_multiuser_test.sh [build_path] [nof_prb] [nof_ue] [k_per_tti]

set -u
BUILD_PATH="${1:-$(cd "$(dirname "$0")/.." && pwd)/build}"
NOF_PRB="${2:-25}"
NOF_UE="${3:-10}"
K="${4:-4}"
NSF=400

GEN="$BUILD_PATH/lib/examples/gen_dl_ul_multiuser"
TOOL="$BUILD_PATH/lib/examples/dl_ul_capture_align"
for b in "$GEN" "$TOOL"; do
  [ -x "$b" ] || { echo "FAIL: missing binary $b (build gen_dl_ul_multiuser + dl_ul_capture_align)"; exit 1; }
done

WORK=$(mktemp -d /tmp/dlul_rarmu.XXXXXX)
trap 'rm -rf "$WORK"' EXIT
DL="$WORK/dl.iq"; UL="$WORK/ul.iq"; LOG="$WORK/tool.log"

echo "== Generating multi-user vector (prb=$NOF_PRB, $NOF_UE UEs, $K grants/TTI) =="
"$GEN" -c 1 -p "$NOF_PRB" -u "$NOF_UE" -k "$K" -m 8 -L 3 -n "$NSF" -o "$DL" -O "$UL" \
  || { echo "FAIL: generator error"; exit 1; }

echo "== Replaying with RAR auto-learn (no -r) =="
"$TOOL" --dl-file "$DL" --ul-file "$UL" -c 1 -p "$NOF_PRB" -w 8 -n "$NSF" >"$LOG" 2>&1

LEARNED=$(grep -c "Learned C-RNTI" "$LOG")
MSG3_OK=$(grep "Msg3" "$LOG" | grep -c "CRC=OK")
CRC_OK=$(grep -c "CRC=OK" "$LOG")
CRC_NOK=$(grep -c "CRC=NOK" "$LOG")
# Busiest UL subframe: how many distinct PUSCH decoded into one subframe.
MAX_PER_SF=$(grep "CRC=OK" "$LOG" | grep -oE "UL#[0-9]+" | sort | uniq -c | sort -rn | head -1 | awk '{print $1}')

grep "Active C-RNTIs" "$LOG"
echo "== Learned=$LEARNED/$NOF_UE, Msg3 CRC=OK=$MSG3_OK/$NOF_UE, CRC=OK=$CRC_OK, CRC=NOK=$CRC_NOK, max PUSCH/UL-sf=$MAX_PER_SF =="

if [ "$LEARNED" -eq "$NOF_UE" ] && [ "$MSG3_OK" -eq "$NOF_UE" ] && [ "$CRC_NOK" -eq 0 ] &&
   [ "$CRC_OK" -gt 500 ] && [ "${MAX_PER_SF:-0}" -ge 3 ]; then
  echo "PASS: learned all $NOF_UE C-RNTIs from RAR, decoded all Msg3, and $CRC_OK PUSCH with CRC=OK"
  echo "      (up to $MAX_PER_SF distinct PUSCH decoded in a single UL subframe)"
  exit 0
fi
echo "FAIL: expected $NOF_UE learned + $NOF_UE Msg3, no CRC=NOK, and >=3 PUSCH in some UL subframe"
exit 1
