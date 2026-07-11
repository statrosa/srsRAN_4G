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

# Multi-user regression test for lib/examples/dl_ul_capture_align.
#
# gen_dl_ul_testvec emits a DL stream that cycles a PDCCH format-0 UL DCI through
# a list of C-RNTIs (one grant per subframe, round-robin) plus the matching PUSCH
# at n+4 for each. dl_ul_capture_align is then run with all those C-RNTIs in its
# active pool; it must keep every C-RNTI active and decode each one's PUSCH with
# CRC=OK. This exercises the active-C-RNTI pool + parallel UL decoder workers at
# scale, with no radios and no core network.
#
# Usage: ./dl_ul_capture_align_multiuser_test.sh [build_path] [nof_prb] [nof_rntis]

set -u
BUILD_PATH="${1:-$(cd "$(dirname "$0")/.." && pwd)/build}"
NOF_PRB="${2:-15}"
NOF_RNTIS="${3:-10}"
NSF=300               # subframes to generate
RUN_SF=$((NSF - 5))   # replay short of the file end to avoid wrap-around misses

GEN="$BUILD_PATH/lib/examples/gen_dl_ul_testvec"
TOOL="$BUILD_PATH/lib/examples/dl_ul_capture_align"
for b in "$GEN" "$TOOL"; do
  [ -x "$b" ] || { echo "FAIL: missing binary $b (build gen_dl_ul_testvec + dl_ul_capture_align)"; exit 1; }
done

# Build the C-RNTI list 0x46, 0x47, ... (NOF_RNTIS entries).
RNTIS=""
for i in $(seq 0 $((NOF_RNTIS - 1))); do
  printf -v hx '0x%x' $((0x46 + i))
  RNTIS="${RNTIS:+$RNTIS,}$hx"
done

WORK=$(mktemp -d /tmp/dlul_multi.XXXXXX)
trap 'rm -rf "$WORK"' EXIT
DL="$WORK/dl.iq"; UL="$WORK/ul.iq"; LOG="$WORK/tool.log"

echo "== Generating $NOF_RNTIS-C-RNTI vector (prb=$NOF_PRB): $RNTIS =="
"$GEN" -c 1 -p "$NOF_PRB" -r "$RNTIS" -m 10 -n "$NSF" -o "$DL" -O "$UL" \
  || { echo "FAIL: generator error"; exit 1; }

echo "== Replaying through dl_ul_capture_align (all $NOF_RNTIS C-RNTIs in the pool) =="
"$TOOL" --dl-file "$DL" --ul-file "$UL" -c 1 -p "$NOF_PRB" -r "$RNTIS" -R -w 4 -n "$RUN_SF" >"$LOG" 2>&1

grep "Active C-RNTIs" "$LOG"
CRC_OK=$(grep -c "CRC=OK" "$LOG")
CRC_NOK=$(grep -c "CRC=NOK" "$LOG")
# Count how many distinct C-RNTIs produced at least one CRC=OK.
DISTINCT_OK=$(grep "CRC=OK" "$LOG" | grep -oE "rnti=0x[0-9a-f]+" | sort -u | wc -l)
echo "== Result: CRC=OK=$CRC_OK, CRC=NOK=$CRC_NOK, C-RNTIs decoded=$DISTINCT_OK/$NOF_RNTIS =="

if [ "$CRC_NOK" -eq 0 ] && [ "$DISTINCT_OK" -eq "$NOF_RNTIS" ]; then
  echo "PASS: all $NOF_RNTIS active C-RNTIs decoded PUSCH with CRC=OK ($CRC_OK total)"
  exit 0
fi
echo "FAIL: expected all $NOF_RNTIS C-RNTIs decoded and no CRC=NOK"
exit 1
