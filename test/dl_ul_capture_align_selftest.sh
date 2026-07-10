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

# Self-contained decode regression test for lib/examples/dl_ul_capture_align.
#
# gen_dl_ul_testvec uses srsRAN's real PHY (the same DSP srsENB/srsUE use) to
# emit a DL stream with a PDCCH format-0 UL DCI plus the matching PUSCH at n+4,
# then dl_ul_capture_align replays it and must decode the PUSCH with CRC=OK.
# Needs no radios and no core network, so it runs anywhere the library builds.
#
# Usage: ./dl_ul_capture_align_selftest.sh [build_path] [nof_prb] [rnti_hex] [mcs]

set -u
BUILD_PATH="${1:-$(cd "$(dirname "$0")/.." && pwd)/build}"
NOF_PRB="${2:-6}"
RNTI="${3:-0x46}"
MCS="${4:-10}"
NSF=100          # subframes to generate
RUN_SF=$((NSF - 5))  # replay short of the file end to avoid wrap-around misses

GEN="$BUILD_PATH/lib/examples/gen_dl_ul_testvec"
TOOL="$BUILD_PATH/lib/examples/dl_ul_capture_align"
for b in "$GEN" "$TOOL"; do
  [ -x "$b" ] || { echo "FAIL: missing binary $b (build gen_dl_ul_testvec + dl_ul_capture_align)"; exit 1; }
done

WORK=$(mktemp -d /tmp/dlul_self.XXXXXX)
trap 'rm -rf "$WORK"' EXIT
DL="$WORK/dl.iq"; UL="$WORK/ul.iq"; LOG="$WORK/tool.log"

echo "== Generating test vectors (prb=$NOF_PRB rnti=$RNTI mcs=$MCS) =="
"$GEN" -c 1 -p "$NOF_PRB" -r "$RNTI" -m "$MCS" -n "$NSF" -o "$DL" -O "$UL" || { echo "FAIL: generator error"; exit 1; }

echo "== Replaying through dl_ul_capture_align =="
"$TOOL" --dl-file "$DL" --ul-file "$UL" -c 1 -p "$NOF_PRB" -r "$RNTI" -w 4 -n "$RUN_SF" >"$LOG" 2>&1

CRC_OK=$(grep -c "CRC=OK" "$LOG")
CRC_NOK=$(grep -c "CRC=NOK" "$LOG")
echo "== Result: CRC=OK=$CRC_OK, CRC=NOK=$CRC_NOK =="
if [ "$CRC_OK" -gt 0 ] && [ "$CRC_NOK" -eq 0 ]; then
  echo "PASS: decoded $CRC_OK srsRAN-generated PUSCH transmission(s) with CRC=OK"
  exit 0
fi
echo "FAIL: expected CRC=OK decodes and no CRC=NOK (ok=$CRC_OK nok=$CRC_NOK)"
exit 1
