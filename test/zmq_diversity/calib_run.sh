#!/bin/bash
# One measurement run: $1=tag $2=enb_conf $3...=splitter args
cd "$(dirname "$0")"
TAG=$1; CONF=$2; shift 2
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -9 $p 2>/dev/null; done; sleep 2; }
trap cleanup EXIT
python3 channel_splitter.py --stats-every 2000 "$@" > results/${TAG}_splitter.log 2>&1 & PIDS+=($!)
sleep 1
/home/user/srsRAN_4G/build/srsenb/src/srsenb $CONF --log.filename=$PWD/results/${TAG}_enb.log > results/${TAG}_enb_console.log 2>&1 & PIDS+=($!)
sleep 6
/home/user/srsRAN_4G/build/srsue/src/srsue ue.conf --log.filename=$PWD/results/${TAG}_ue_stack.log > results/${TAG}_ue.log 2>&1 & PIDS+=($!)
sleep ${RUNTIME:-90}
cleanup
grep -c "Random Access Transmission" results/${TAG}_ue.log 2>/dev/null || echo 0
