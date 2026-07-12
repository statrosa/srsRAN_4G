# Near-ML Decoding of Short Blocks: CRC-Aided Flip List Decoding

## Context

For short transport blocks — the cell-edge QPSK grants this branch has been optimizing
end to end — the LTE turbo decoder is 1–1.5 dB away from maximum-likelihood performance.
After the channel-estimation work reduced CE error to ~5% of the noise floor, the
decoder's own gap became the largest remaining implementation loss at the noise floor.
This feature recovers part of that gap by spending compute: when a decode fails, the
receiver *hypothesizes* corrections of the least-reliable bits and lets the CRC pick the
survivor — the same hypothesize-and-test philosophy as the timing sweep, applied to the
FEC layer.

Sources: `lib/src/phy/fec/turbo/turbodecoder.{h,c}` (`srsran_tdec_get_app`),
`lib/src/phy/phch/sch.c` (flip engine), `srsran_pusch_cfg_t.max_flip_attempts` (knob).
Commits `3a959d0` + `da59687`.

## How it works

1. **Standard decode first.** The usual iteration loop with CRC early stopping runs to
   its budget. If it passes, nothing changes — the flip engine costs zero on the
   success path.
2. **Reliability ranking.** On failure (and only for code blocks ≤ 1024 bits, where the
   ML gap is large and re-decodes are cheap), the new `srsran_tdec_get_app()` exports
   the a-posteriori LLRs of the information bits — reading exactly the buffer the hard
   decision was sliced from (`app1`/`ext1` by iteration parity, both 8- and 16-bit
   paths). The `SCH_FLIP_NOF_POS = 32` least-reliable positions become flip candidates
   (filler bits skipped).
3. **Pinned re-decodes.** For each candidate — single flips in ascending reliability
   order, then pairs — the systematic channel LLR at position `3*i` of the
   deratematched buffer is saturated toward the *opposite* of the failed decode's
   decision (`APP > 0 ⇒ bit 1` convention), and the full iteration loop re-runs. The
   pin dominates the channel evidence for that bit, so the trellis is forced through
   the alternative hypothesis while every other bit stays soft.
4. **CRC selects.** The first candidate that passes CRC wins. All candidates fail →
   the block fails exactly as it would have without the feature.

## Why it is robust

- **Undetected-error probability is controlled.** Every accepted candidate is a full
  trellis-consistent decode followed by the CRC-24 check — the identical acceptance
  test as a plain decode. The budget bounds the total false-accept probability to
  ≤ attempts × 2⁻²⁴ per failed block (4×10⁻⁶ at 64 attempts), inside the HARQ
  NACK→ACK budget. The test harness verifies this *empirically*: in BLER mode a CRC
  pass with a payload that mismatches the transmitted ground truth is always fatal —
  zero occurrences across all measurement runs.
- **HARQ-safe.** The deratematched buffer holds soft-combined LLRs across
  retransmissions; each pin is saved and restored exactly, so a failed flip session
  leaves the buffer bit-identical for the next redundancy version.
- **Deterministic.** No randomness in the candidate schedule — results reproduce
  run-to-run, which is what makes the ctest thresholds meaningful.
- **Off by default.** `max_flip_attempts = 0` (the default everywhere) keeps the
  decoder bit-exact with the previous behavior; the entire pre-existing test suite
  runs with the feature disabled and is unchanged.
- **Bounded worst case.** Latency adds at most
  `attempts × max_iterations × t_decode(cb_len)` on *failed* short blocks only.
  Suggested budgets: 8–16 where real-time margins are tight, 64 for
  compute-rich/lab receivers.

## Measured results

4-PRB QPSK MCS 4 (TBS 256 + CRC, single code block), real DMRS channel estimation,
AWGN, 200 subframes per point, deterministic seed:

| SNR | Baseline BLER | flip 16 | flip 64 |
|---|---|---|---|
| 0 dB | 0.385 | 0.280 | 0.260 |
| 0.5 dB | 0.105 | 0.060 | **0.050** |
| 1 dB | 0.020 | 0.010 | **0.005** |

- **10% BLER point: ~0.54 dB → ~0.31 dB (≈ 0.23 dB coding gain)** at flip 64.
- BLER roughly halved at fixed SNR on the waterfall; 4× reduced at its bottom edge.
- The gain concentrates exactly where HARQ retransmissions are decided, so the
  real-world reading is ~2× fewer retransmissions for cell-edge/IoT UEs in this band —
  battery and latency, like the CFO-tracking row of the UE-tracking work.

The measured gain is below the literature's upper range for flip decoding (0.4–0.9 dB)
for an honest reason: at these operating points many failures carry more than two
channel-symbol errors, which a single/pair flip schedule cannot repair. Deeper
schedules (triples, adaptive ordering by pairwise APP) or order-2+ OSD would close
more of the gap at correspondingly higher budgets — the engine's schedule is the
natural extension point.

## Test coverage

| ctest | Asserts |
|---|---|
| `pusch_test_flip_stress` | flip-64 BLER ≤ 0.08 at 0.5 dB (measured 0.050; the baseline's 0.105 fails this bound — regression guard in both directions) |
| `pusch_test_bler_baseline` | baseline BLER ≤ 0.16 at the same point (pins the waterfall position against estimator regressions) |
| entire pre-existing suite | runs with `max_flip_attempts = 0`: bit-exact baseline preserved |

The `-p bler_max <frac>` tolerant mode added to `pusch_test` counts block errors
instead of failing on the first one, and treats any CRC-pass-with-wrong-payload as an
immediate failure regardless of the threshold — the standing empirical check on the
false-accept analysis.

## Relation to the rest of the branch

The channel-estimation stages determine the LLR *quality* entering the decoder; this
feature determines how much of the LLRs' information the decoder *extracts*. They
compound: the estimator work moved the whole BLER waterfall left by reducing effective
noise, and flip decoding moves it left again by ~0.23 dB at the same LLRs. It also
pairs naturally with two earlier proposals — iterative CE↔decode would improve the
LLRs between flip attempts, and the hypothesis-sweep architecture can afford flip
budgets per timing candidate since the CRC oracle is shared.
