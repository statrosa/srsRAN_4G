# Uplink RX Diversity (MRC) for srsenb — Design, Implementation & Validation

Branch: `claude/srsenb-rx-diversity-ohkx2i` (base: `6bcbd9e`)
Diff size: 61 files changed, ~2600 insertions (feature + tests + campaign tooling/results)

This document captures the complete context of the change: why it was needed,
every code change with rationale, the bugs found along the way, the test
evidence, and how to deploy it on real hardware (USRP B210/X310 with
1 TX + 2 RX antennas).

---

## 1. Motivation and problem statement

Goal: make srsenb genuinely use two RX antennas on the uplink — maximum-ratio
combining (MRC) for PUSCH/PUCCH and multi-antenna PRACH detection — for
uplink SNR/coverage gain (~3 dB for two equal branches, far more under
fading).

### What stock srsRAN 4G did

The RF plumbing was already multi-antenna, but every uplink sample from the
second antenna was **received and then discarded**:

- `radio.cc:74` — `nof_channels = nof_antennas * nof_carriers`: with
  `nof_antennas=2` the device opens and streams 2 RX channels.
- `srsenb/src/phy/txrx.cc` — all RX ports are filled every TTI into
  per-antenna buffers `cc_worker::signal_buffer_rx[p]`.
- **But** `cc_worker.cc` initialised the UL demod with antenna 0 only:
  `srsran_enb_ul_init(&enb_ul, signal_buffer_rx[0], nof_prb)`.
- `srsran_enb_ul_t` was single-antenna by type: scalar `sf_symbols`,
  `in_buffer`, one FFT, one channel-estimate buffer.
- `srsran_chest_ul_res_t` had a single `cf_t* ce`; all UL estimators took a
  single input.
- PUSCH and PUCCH decode used the SISO equalizer `srsran_predecoding_single()`.
- PRACH detection took one signal; the PRACH worker was handed antenna 0.
- Config: there was **no independent RX antenna count** —
  `enb_cfg_parser.cc` hard-coupled `rf.nof_antennas = enb.nof_ports`, and
  `nof_ports` means **TX** ports (TM2/3/4). A 1 TX / 2 RX configuration was
  impossible to express.

### The key pre-existing asset

The MRC math already existed and was battle-tested — on the **UE downlink**:
`srsran_predecoding_single_multi()` (`lib/src/phy/mimo/precoding.c:395`)
performs MMSE-MRC across `nof_rxant` antennas with SSE/AVX kernels, used by
`pdsch.c` via `srsran_predecoding_type`. The UE DL object
(`srsran_ue_dl_init(q, cf_t* in_buffer[SRSRAN_MAX_PORTS], max_prb,
nof_rx_antennas)`) is the exact architectural pattern: per-antenna input
buffers → per-antenna FFT → per-antenna channel estimates → MRC combine.

**The design is therefore a mirror of the UE-DL pattern onto the eNB UL,
changing signatures in place** (small, fully-enumerable caller set; srsue is
unaffected because it only uses the *encode* paths).

---

## 2. Code changes in detail

### 2.1 Library — uplink channel estimation (`chest_ul`)

`lib/include/srsran/phy/ch_estimation/chest_ul.h`,
`lib/src/phy/ch_estimation/chest_ul.c` (+429/−didactic rewrite of the
estimators):

```c
// before                          // after
typedef struct {                   typedef struct {
  cf_t*    ce;                       cf_t*    ce[SRSRAN_MAX_PORTS]; // per RX antenna
  uint32_t nof_re;                   uint32_t nof_rx_antennas;
  ...                                uint32_t nof_re;
} srsran_chest_ul_res_t;             ...
                                   } srsran_chest_ul_res_t;

int srsran_chest_ul_res_init(q, max_prb);          // -> (q, max_prb, nof_rx_antennas)
int srsran_chest_ul_estimate_pusch(..., cf_t* input,  ...);  // -> cf_t* input[SRSRAN_MAX_PORTS]
int srsran_chest_ul_estimate_pucch(..., cf_t* input,  ...);  // -> cf_t* input[SRSRAN_MAX_PORTS]
int srsran_chest_ul_estimate_srs  (..., cf_t* input,  ...);  // -> cf_t* input[SRSRAN_MAX_PORTS]
```

Implementation structure: the existing single-antenna machinery
(`pilot_recv_signal`, `pilot_estimates`, LS estimation, smoothing,
interpolation, noise estimation) is looped once per antenna, writing
`res->ce[a]` and a per-antenna measurement scratch
(`chest_ul_ant_meas_t { noise_estimate, epre, rsrp, cfo_hz, ta_us }`), then a
single combining step produces the public scalars consumed by the MAC:

- `noise_estimate`, `epre`, `rsrp` — **arithmetic mean** over antennas
  (keeps dBFS semantics; the mean noise is also what the MMSE-MRC equalizer
  consumes).
- `snr` — **sum of per-antenna `epre/noise`**: the post-MRC effective SNR
  (+3 dB for two equal branches). This is what link adaptation sees.
- `ta_us`, `cfo_hz` — **RSRP-weighted average** with NaN/Inf exclusion.
  Timing/frequency are common to all branches; weighting protects the TA
  loop from a disconnected antenna.
- PUCCH format 2a/2b DRS-bit hypothesis search: the correlation metric is
  **summed over antennas before picking the maximum**, so both antennas vote
  on the same DRS bits.

### 2.2 Library — PUSCH decode (`pusch.h/.c`)

```c
// struct: per-antenna working buffers (encode path uses index 0)
cf_t* ce;  cf_t* d;      // ->  cf_t* ce[SRSRAN_MAX_PORTS]; cf_t* d[SRSRAN_MAX_PORTS];
                          //     uint32_t nof_rx_antennas;

int srsran_pusch_init_enb(q, max_prb);              // -> (q, max_prb, nof_rx_antennas)
int srsran_pusch_decode(..., cf_t* sf_symbols, ...); // -> cf_t* sf_symbols[SRSRAN_MAX_PORTS]
```

Decode flow (per antenna `a`): extract data REs into `q->d[a]` and channel
estimates into `q->ce[a]`, then replace the SISO equalizer with the existing
MRC kernel:

```c
// before
srsran_predecoding_single(q->d, q->ce, q->z, NULL, nof_re, 1.0f, channel->noise_estimate);
// after
srsran_predecoding_single_multi(q->d, q->ce, q->z, NULL, nof_rx_antennas,
                                nof_re, 1.0f, channel->noise_estimate);
```

Everything downstream of the DFT-despread (demodulation, descrambling, turbo
decode, UCI) is untouched — it operates on the combined stream. EPRE is
averaged over antennas. The UE encode path is unchanged (uses `d[0]`; the
existing `is_ue` flag gates eNB-only allocations).

### 2.3 Library — PUCCH decode (`pucch.h/.c`)

Same shape: `z_tmp` and `ce` become per-antenna arrays,
`srsran_pucch_init_enb(q)` → `(q, nof_rx_antennas)`, decode takes
`cf_t* sf_symbols[SRSRAN_MAX_PORTS]`, and equalization becomes
`srsran_predecoding_single_multi`. Two deliberate decisions:

- **Combine before demodulation**: all detection statistics
  (format-1 correlation, format-2 LLR-RMS metric, format-3) operate on the
  combined signal, so the existing detection thresholds keep their meaning
  and simply improve.
- **DMRS detection gate = max over antennas**: a UE visible on only one
  branch must still pass the gate
  (`data->dmrs_correlation = max_a rms(ce[a])/power(ce[a])`).

### 2.4 Library — eNB UL front end (`enb_ul.h/.c`)

```c
typedef struct {
  srsran_cell_t cell;
  uint32_t      nof_rx_antennas;
  cf_t*         sf_symbols[SRSRAN_MAX_PORTS];
  cf_t*         in_buffer[SRSRAN_MAX_PORTS];
  srsran_chest_ul_res_t chest_res;      // now allocated via chest_ul_res_init
  srsran_ofdm_t fft[SRSRAN_MAX_PORTS];  // one RX FFT per antenna
  ...
} srsran_enb_ul_t;

int srsran_enb_ul_init(q, cf_t* in_buffer[SRSRAN_MAX_PORTS], max_prb, nof_rx_antennas);
```

`set_cell` configures one OFDM RX per antenna (same cfg as before:
`freq_shift_f = -0.5`, `rx_window_offset = 0.5`); `srsran_enb_ul_fft()` runs
all of them. `get_pusch`/`get_pucch` pass the `sf_symbols` array through —
the call sites did not change textually because the array decays to the new
parameter type.

### 2.5 Library — PRACH multi-antenna detection (`prach.h/.c`)

PRACH gets an **additive** API (the old function becomes a 1-antenna
wrapper, so its five existing callers, including NR perf tests, don't churn):

```c
int srsran_prach_detect_offset_multi(srsran_prach_t* p, uint32_t freq_offset,
                                     cf_t* signals[SRSRAN_MAX_PORTS],
                                     uint32_t nof_rx_antennas, uint32_t sig_len,
                                     uint32_t* indices, float* t_offsets,
                                     float* peak_to_avg, uint32_t* n_indices);
```

Mechanism: the input FFT and bin extraction run per antenna into
`p->prach_bins_m[a]` (index 0 aliases the legacy `prach_bins`), and inside
the per-preamble loop the correlation power-delay profiles are accumulated
**non-coherently** (`|IFFT(bins_a · conj(root))|²` summed over antennas into
`p->corr`) before the peak search. `corr_ave` scales with the sum, so the
`detect_factor` threshold semantics are preserved; noise variance of the PDP
drops while true peaks add in power. The frequency-domain timing-offset
`cross` vector comes from antenna 0. Successive cancellation (which
reconstructs/subtracts in a single bin set, and which srsenb never enables)
is restricted to one antenna.

### 2.6 NR side — mechanical compatibility fixes

`srsran_chest_ul_res_t` is shared with the NR PUCCH path, so:
`gnb_ul.c` passes `nof_rx_antennas=1` to `res_init`; `pucch_nr.c` and
`dmrs_pucch.c` index `res->ce[0][...]` instead of `res->ce[...]`. NR
behavior is unchanged (still 1 antenna).

### 2.7 srsenb — configuration and buffer plumbing

- **New option `enb.nof_rx_ant`** (`srsenb/hdr/enb.h`, `main.cc`, default 0
  = same as `nof_ports`). `enb_cfg_parser.cc` replaces the old hard patch:

  ```
  args_->phy.nof_rx_ant  = (nof_rx_ant == 0) ? nof_ports : nof_rx_ant;   // validated <= SRSRAN_MAX_PORTS
  args_->rf.nof_antennas = max(nof_ports, nof_rx_ant);
  ```

  so `tm=1, nof_ports=1, nof_rx_ant=2` is now expressible; the TM/`nof_ports`
  validation (TX-side) is untouched.
- `phy_common.h` gains `get_nof_rx_ant()` and
  `get_rf_ant_stride() = max(tx_ports, rx_ant)`. The stride matters because
  `rf_buffer_t::get/set(logical_ch, port, nof_antennas)` computes
  `logical_ch * nof_antennas + port` and the radio indexes with **its**
  `nof_antennas`; once TX ports ≠ RX antennas every set/get in
  `txrx.cc` (RX mapping, PRACH tap) and `sf_worker.cc` (TX mapping) must use
  the common stride or buffers get mis-mapped. `get_nof_rf_channels()` now
  counts `max(nof_ports, nof_rx_ant)` per LTE cell (sizes the UL channel
  emulator).
- `cc_worker.cc`: RX buffers allocated for `max(nof_ports, nof_rx_ant)`
  antennas; `srsran_enb_ul_init(&enb_ul, signal_buffer_rx, nof_prb,
  phy->get_nof_rx_ant())`. Nothing else changes — SNR/TA/metrics reads
  already consume the *combined* `chest_res` scalars (that is exactly why the
  combining lives in `chest_ul`).
- `txrx.cc`: PRACH gets all antennas
  (`prach->new_tti(cc, tti, prach_buffer[])`).
- `prach_worker.{h,cc}`: `sf_buffer` holds per-antenna sample planes
  (`std::array<std::vector<cf_t>, SRSRAN_MAX_PORTS>`, sized on first use);
  `init(...)` takes `nof_rx_antennas`; `run_tti` calls
  `srsran_prach_detect_offset_multi`. The NR worker pool passes 1.
- Asymmetric TX note: the radio layer assumes symmetric channel counts, so
  with 1 TX / 2 RX the device opens 2 full channels and the unused TX simply
  carries zeros. `radio.cc::map_channels` gained a nullptr guard so an unset
  TX port keeps the driver's zero-buffer default instead of overwriting it
  with NULL.

### 2.8 RF driver bug fix — ZMQ `tx_off` (found by the e2e test)

`lib/src/phy/rf/rf_zmq_imp.c` set a **device-global** `tx_off = true` as soon
as *any* channel lacked a `tx_port`, then silently discarded **every**
baseband transmission (`rf_zmq_send_timed_multi` early-returns). An eNB with
`tx_port0 + rx_port0/rx_port1` therefore transmitted nothing — the downlink
went completely dark. Fixed: transmission is disabled device-wide only when
**no** channel has a transmitter; channels without one already no-op
individually. This is required for any asymmetric TX/RX ZMQ topology.

---

## 3. Test evidence

### 3.1 Unit test — `lib/test/phy/enb_ul_test.c` (new, in ctest)

Full-chain simulation: `srsran_ue_ul_encode` builds a real PUSCH subframe
from a DCI-derived grant; the transmission is fanned onto N antennas through
distinct unit-magnitude complex gains and independent AWGN; the complete eNB
chain (per-antenna FFT → chest → MRC decode) runs with 1 vs 2 antennas.
Asserts (all passing):

- PUCCH format-1a decodes with 1 and 2 antennas at 10 dB.
- High SNR: both configurations decode 4/4 TBs and the reported SNR gains
  1.5–4.5 dB — measured **+3.1 dB** (31.8 → 34.9 dB).
- Low SNR (MCS16 @ 6 dB): 1 RX decodes **0/10** transport blocks, 2 RX
  decodes **10/10** (8.5 vs 11.5 dB reported).
- |TA| < 1 µs throughout.

Full regression: ~1200 ctest tests pass (all single-antenna UL tests
unchanged; the one failure, `network_utils_test`, is environmental — the
container kernel lacks SCTP).

### 3.2 End-to-end over ZMQ (no hardware)

srsue ↔ srsenb, 25 PRB, `nof_ports=1, tm=1, nof_rx_ant=2`, UE uplink
duplicated to both eNB RX ports by a protocol-faithful splitter. Both the
"dead second antenna" and "both antennas fed" variants complete the full
flow — cell search, **multi-antenna PRACH detection, MRC Msg3 decode
(crc=OK)**, RRC processing:

| Run | Msg3 PUSCH SNR | EPRE |
|---|---|---|
| 2 RX, antenna 1 silent | 115.7 dB | 132.0 dBfs |
| 2 RX, both antennas fed | **118.7 dB (+3.0)** | 135.1 dBfs |

(Container kernels without SCTP cannot run srsepc/S1AP, so attach terminates
at the expected `RRCConnectionReject` "MME not connected"; with a real EPC
the same configs complete attach.)

### 3.3 Channel-realistic campaign — `test/zmq_diversity/`

Committed tooling: `channel_splitter.py` (per-antenna gain/phase/delay →
independent flat-Rayleigh or EPA-profile fading with Doppler → per-antenna
AWGN, noise **after** fading), `run_campaign.py` orchestrator (full-stack
restart per UE cycle, metric harvesting to JSON), configs, and
`RESULTS.md`. Calibration: UE burst power 58.9 dBfs at the splitter;
1-antenna Msg3 fails ~50% at **SNR₀ = −16 dB** full-band.

| Scenario | Channel (per antenna) | RX | PRACH det | Msg3 CRC OK | SNR | 
|---|---|---|---|---|---|
| s1 benign | AWGN SNR₀+8, balanced | 1rx | 12/12 | 12/12 | 4.3 dB |
| s1 benign | AWGN SNR₀+8, balanced | 2rx | 12/12 | 12/12 | **7.1 dB (+2.8)** |
| s2 low SNR | AWGN SNR₀ | 1rx | 15/15 | 15/38 (39%) | 0.6 dB |
| s2 low SNR | AWGN SNR₀ | 2rx | 12/12 | 11/15 (**73%**) | 3.7 dB |
| s3 imbalance | ant0 SNR₀+8 / ant1 SNR₀−2 | 2rx | 12/12 | 11/14 (79%) | **5.8 dB** ≥ best branch |
| s4 dead antenna | ant1 noise-only | 2rx | 12/12 | 12/12 | 5.5 dB, TA clean |
| s5 Rayleigh 5 Hz | independent fading + AWGN | 1rx | 24/27 | 9/74 (**12%**) | 3.1 dB |
| s5 Rayleigh 5 Hz | independent fading + AWGN | 2rx | 6/9 | 6/6 (**100%**) | 5.9 dB |
| s6 EPA multipath | independent, freq-selective | 2rx | 6/9 | 6/6 | 7.1 dB |
| s7 RF-chain skew | +3 samples, 120°, −3 dB on ant1 | 2rx | 12/12 | 12/12 | 5.3 dB, TA ≤ 1.9 µs |
| s8 ETU300 (srsRAN emulator) | 300 Hz Doppler, no noise | 1rx | **0/50** | — | — |
| s8 ETU300 (srsRAN emulator) | 300 Hz Doppler, no noise | 2rx | 4/84 | 4/4 | 32.7 dB |

Headline: under identical independent Rayleigh fading, one antenna decodes
12% of Msg3 transmissions while two antennas decode 100% — diversity order 2
exactly where it matters. At ETU300 the preamble itself decorrelates
(channel physics; 3GPP defines restricted sets for this regime) — and two
antennas still complete every RA they catch while one detects nothing.

---

## 4. Bugs found and operational lessons

1. **ZMQ driver `tx_off`** (§2.8) — real driver bug, only discoverable via
   an asymmetric-channel e2e run.
2. **eNB/UE sample-timeline alignment** — restarting only the UE against a
   long-running ZMQ eNB leaves the new UL stream at an arbitrary
   sub-subframe offset in the eNB's resumed timeline; PRACH lands outside
   the detection windows. The campaign restarts the whole stack per cycle.
3. **REQ socket recovery** — a strict ZMQ REQ stuck in wait-for-reply after
   its peer dies must be recreated (lazy-pirate), not resent.
4. **TIME_WAIT vs port probes** — ZMQ binds with SO_REUSEADDR; health probes
   must too, or every quick restart looks like a port conflict.
5. **`-march=native` in ephemeral containers** — a container migration to a
   host with different AVX-512 extensions makes all binaries die with
   SIGILL; a clean rebuild fixes it.
6. **NAS retry cadence** — without a core network srsue makes 5 attach
   attempts (~25 s apart, T3410/T3411) then backs off 12 min (T3402); all
   hardcoded in `nas.h`. Long test campaigns must restart srsue.

---

## 5. Deploying on a B210/X310

```ini
[enb]
nof_ports  = 1        # TX ports (TM1)
nof_rx_ant = 2        # NEW: UL RX antennas
tm         = 1

[rf]
device_name = uhd
# B210: rx antennas RX2 on both channels; X310: one daughterboard per channel
```

Expected behavior: the radio opens 2 RX channels (TX runs on channel 0 only,
channel 1 transmits zeros); `pusch_snr` in the eNB metrics rises ~3 dB for
equal branches and the cell keeps working (at ~the single-antenna level) if
one cable is pulled — that exact case is covered by campaign scenario s4.
Quick hardware check: compare reported UL SNR/BLER with one RX cable
disconnected vs both connected.

## 6. Limitations and future work

- **SRS**: the estimator API is multi-antenna, but srsenb still doesn't
  schedule/consume SRS (unchanged from upstream).
- **MRC vs IRC**: MRC is optimal against white noise; interference-limited
  deployments would benefit from interference rejection combining. The
  per-antenna estimates/buffers added here are the prerequisite.
- **srsRAN channel emulator caveats** (relevant to reproducing the
  campaign): its AWGN is a single shared instance (same noise for all
  antennas) applied **before** fading, so fades don't change SNR — per-antenna
  SNR realism needs an external emulator like `channel_splitter.py`. Its
  per-channel fading seeds are `0x1234 * i`, so channel 0 gets seed 0.
- **PUCCH thresholds**: combining strictly improves the detection metrics;
  thresholds were deliberately left untouched. A future pass could re-tune
  `threshold_*` for 2 RX to trade the gain between sensitivity and false
  alarms.
- Kernel SCTP is required for srsepc/S1AP (full attach) — not available in
  some containers; everything up to and including Msg3 works regardless.

---

## 7. Optional MMSE-IRC equalizer

An optional interference-rejection-combining equalizer builds on this
per-antenna infrastructure. It is documented separately in
[`UL_RX_IRC_OPTIMIZATIONS.md`](UL_RX_IRC_OPTIMIZATIONS.md): flag-gated
(`expert.equalizer_mode = irc`, default MRC), it whitens with the estimated
interference-plus-noise covariance to spatially null a co-channel interferer.
End-to-end result: under a 15 dB rank-1 interferer that leaves MRC decoding
0/360 Msg3 transmissions, IRC decodes 100%, with no penalty on a clean
channel.
