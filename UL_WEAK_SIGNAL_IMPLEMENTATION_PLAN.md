# Implementation Plan: srsenb UL Receive-Chain Optimizations for Faraway / Low-SNR UEs

This document is the detailed implementation plan for improving srsenb reception of distant
UEs transmitting at low power and low SNR. It covers the receiver-side changes only —
detection thresholds, feedback gates, decoder settings, and channel estimation. Power-control
changes (the TPC sample floor and closed-loop power-control configuration) are intentionally
out of scope of this document.

All defaults preserve existing behavior bit-for-bit; every improvement is opt-in via
`enb.conf`.

---

## 1. Background: how the UL receive chain limits weak-signal reception

The srsenb uplink receive chain is:

```
RF samples ──► OFDM FFT ──► channel estimation (chest_ul) ──► MMSE equalizer ──► decoders
                 │                                                                 │
                 └──► PRACH worker (preamble correlation)              PUSCH: turbo decode
                                                                       PUCCH: correlation detect
                                                                          │
                                                            SNR/TA reports ──► MAC scheduler
```

Analysis of this chain found four hardcoded values tuned for strong-signal lab conditions,
plus one expert config option that is parsed but never applied:

| # | Location | Hardcoded value | Effect on faraway UEs |
|---|----------|-----------------|------------------------|
| 1 | `srsenb/src/phy/prach_worker.cc:50` | PRACH detect factor = 60 (lib default 18) | Weak RACH preambles rejected → UE never attaches |
| 2 | `srsenb/src/phy/phy_ue_db.cc:105-109` | PUCCH thresholds 0.5 / DMRS 0.4 | Missed HARQ ACK/SR at cell edge → DL throughput collapse, RACH fallback |
| 3 | `srsenb/hdr/phy/lte/cc_worker.h:67` | PUSCH SNR-report gate = 1.0 dB | SNR/TA feedback to scheduler cut for marginal UEs → stale, optimistic MCS |
| 4 | `lib/src/phy/ch_estimation/chest_ul.c:93-94` | 3-tap smoothing filter w = 1/3 | Not harmful (already optimal), but the `estimator_fil_w` config that claims to control it is a silent no-op |
| 5 | `expert.pusch_max_its` (already configurable) | default 8 half-iterations | Decoder stops short of its low-SNR potential; config/docs only |

### Measured-SNR calibration (anchors items 3 and all SNR-based tuning)

Verified in `lib/src/phy/ch_estimation/chest_ul.c:377-394`: the reported UL "SNR" is
`EPRE / noise_estimate`, where EPRE is the average power of the received pilot resource
elements — i.e. **signal + noise**. Therefore:

```
measured_snr_dB = 10 * log10(1 + SINR_linear)
```

Consequences:

- A UE that did **not** transmit on its grant (DTX) measures ≈ **0 dB**, not −∞.
- True SINR 0 dB measures ≈ +3 dB; true −6 dB measures ≈ +1 dB.
- The existing 1.0 dB gate therefore corresponds to a true SINR of roughly −6 dB and is
  partly a legitimate DTX filter. Tuned values must stay **above 0 dB** or DTX subframes
  contaminate the scheduler's SNR average.

### Value analysis per change

**PRACH detection factor (highest impact).** Detection in `lib/src/phy/phch/prach.c:928`
requires the preamble correlation peak to exceed `detect_factor × correlation_average`.
srsenb overrides the library default of 18 with 60 — 3.3× stricter. For a distant UE, the
peak-to-average ratio shrinks with received power, so genuine preambles are rejected; in the
field the UE power-ramps through all `preamble_trans_max` attempts and never receives a RAR.
Since RACH is the entry point to everything, this factor sets the attach range of the cell.
Cost of lowering: occasional false RACH detections (a wasted RAR + Msg3 grant each) — benign
on a private network with few UEs.

**PUCCH thresholds (high impact).** PUCCH format 1/1a detection is normalized correlation vs
threshold (`lib/src/phy/phch/pucch.c:644,659,680,828-859`) plus a DMRS gate. At cell-edge
SNR the correlation of a genuinely transmitted PUCCH routinely falls below 0.5. A missed ACK
is treated as DTX/NACK → needless DL retransmissions; a missed SR means the UE cannot request
UL grants and falls back to RACH. Unlike PUSCH, PUCCH format 1/1a has no HARQ soft-combining,
so its effective range is *shorter* than the data channel's unless the thresholds come down.
Risk: a false ACK makes the eNB believe a DL TB was delivered when it wasn't (RLC AM recovers
it, with latency) — lower conservatively and watch retransmission metrics.

**PUSCH SNR-report gate (moderate impact).** `cc_worker.cc` forwards SNR/TA to the MAC only
when measured SNR ≥ 1.0 dB. Given the compressed scale above, this is not a catastrophic
feedback cut, but it harms weak UEs two ways: (a) *censoring bias* — below-gate samples
vanish during fades instead of dragging the scheduler's exponentially-smoothed SNR average
down, so MCS stays too aggressive and BLER stays high; (b) any consumer of the SNR feed stops
receiving exactly the samples that indicate the UE is struggling. Recommended tuned value:
0.2–0.5 dB measured (never ≤ 0). TA reports deliberately stay inside the same gate: timing
estimates from near-noise DMRS are unreliable, and a wrong timing-advance command is worse
than none. Side effect to know about: the gate (1.0 dB) exceeds the default
`rlf_min_ul_snr_estim` (−2 dB), so the SNR-based UL radio-link-failure check in
`mac.cc` can never fire today; it remains inert with the recommended values (−2 < 0 ≤ gate),
and the operator now controls that trade-off explicitly.

**`estimator_fil_w` wiring (hygiene, not a sensitivity gain).** The option is parsed in
`main.cc` into a field nobody reads. The hardcoded filter value w = 1/3 makes the 3-tap
smoother `[w, 1−2w, w]` uniform — already the maximum noise averaging a 3-tap filter can do,
i.e. optimal for the flat, low-Doppler channels of stationary faraway UEs. Wiring the option
fixes a config trap and enables tuning for frequency-selective channels. Two constraints:
the parsed default must change from 0.1 to 0.3333 (otherwise wiring it would silently
*degrade* low-SNR estimation), and w must be clamped because the noise-estimate calibration
polynomial `a = 7.419w² + 0.1117w − 0.005387` (see `estimate_noise_pilots()`) crosses zero
near w ≈ 0.02 and the center tap goes negative above w = 1/3.

**Turbo iterations (small, free).** `expert.pusch_max_its` is already fully plumbed
(`main.cc` → `phy_ue_db.cc` → `sch.c`). At cell-edge SNR the decoder operates at the turbo
waterfall where extra iterations are worth a few tenths of a dB, and per-codeblock CRC early
stopping means extra iterations only cost CPU on decodes that are currently *failing*.
Config/docs change only: recommend 16, and fix the stale "(default: 4)" comment (real
default: 8).

### Examined and rejected

- **`equalizer_mode` wiring**: the UL already always uses MMSE equalization
  (`srsran_predecoding_single(..., noise_estimate)` in `pusch.c` / `pucch.c`) — the best
  choice at low SNR. Plumbing the option could only select something worse ("zf"). Left
  unwired.
- **Time-domain interpolation** (`DO_LINEAR_INTERPOLATION` in `chest_ul.c`, dead code):
  helps only under Doppler; stationary faraway UEs gain nothing.
- **Longer / Gaussian smoothing filters**: the UL noise-estimate correction is calibrated
  only for the 3-tap case; a longer filter would bias the SNR that feeds MCS selection and
  RLF systemically. Future work requiring recalibration.
- **2-antenna receive diversity (MRC)**: ~3 dB gain and `srsran_predecoding_single_multi()`
  already exists in the library, but the UL path (`enb_ul`, `chest_ul`, `cc_worker` buffers)
  is single-port end-to-end and the target radio has one RX port. Future work.

---

## 2. Code changes

### 2.1 Library: channel-estimator smoothing-coefficient setter

New public setter on the UL channel estimator, with the calibration clamp, plus a thin
wrapper at the eNB UL object level. A post-init setter (rather than extending
`srsran_enb_ul_set_cell()`) matches the existing style of post-init tweaks such as
`pusch_8bit_decoder`.

`lib/include/srsran/phy/ch_estimation/chest_ul.h`:

```diff
 SRSRAN_API int srsran_chest_ul_set_cell(srsran_chest_ul_t* q, srsran_cell_t cell);

+SRSRAN_API void srsran_chest_ul_set_smooth_filter3_coeff(srsran_chest_ul_t* q, float w);
+
 SRSRAN_API void srsran_chest_ul_pregen(srsran_chest_ul_t*                 q,
                                        srsran_refsignal_dmrs_pusch_cfg_t* cfg,
                                        srsran_refsignal_srs_cfg_t*        srs_cfg);
```

`lib/src/phy/ch_estimation/chest_ul.c` (after `srsran_chest_ul_set_cell()`):

```diff
+void srsran_chest_ul_set_smooth_filter3_coeff(srsran_chest_ul_t* q, float w)
+{
+  // The noise estimation correction in estimate_noise_pilots() is calibrated for a 3-tap filter and
+  // becomes singular as w approaches ~0.02; w above 1/3 makes the center tap negative
+  w                    = SRSRAN_MAX(0.05f, SRSRAN_MIN(w, 1.0f / 3.0f));
+  q->smooth_filter_len = srsran_chest_set_smooth_filter3_coeff(q->smooth_filter, w);
+}
```

`lib/include/srsran/phy/enb/enb_ul.h`:

```diff
                                       srsran_refsignal_srs_cfg_t*        srs_cfg);

+SRSRAN_API void srsran_enb_ul_set_smooth_filter3_coeff(srsran_enb_ul_t* q, float w);
+
 SRSRAN_API void srsran_enb_ul_fft(srsran_enb_ul_t* q);
```

`lib/src/phy/enb/enb_ul.c` (after `srsran_enb_ul_set_cell()`):

```diff
+void srsran_enb_ul_set_smooth_filter3_coeff(srsran_enb_ul_t* q, float w)
+{
+  srsran_chest_ul_set_smooth_filter3_coeff(&q->chest, w);
+}
```

### 2.2 srsenb: new expert argument fields

`srsenb/hdr/phy/phy_interfaces.h`, struct `phy_args_t` (the `SRSRAN_PUCCH_DEFAULT_THRESHOLD_*`
macros are visible via the existing `#include "srsran/srsran.h"`):

```diff
   float                   rx_gain_offset      = 62;
   float                   max_prach_offset_us = 10;
+  float                   prach_detect_factor = 60.0f;
   uint32_t                pusch_max_its       = 10;
   uint32_t                nr_pusch_max_its    = 10;
   bool                    pusch_8bit_decoder  = false;
+  float                   pusch_min_snr_info_db = 1.0f;
   float                   tx_amplitude        = 1.0f;
   ...
   uint32_t                nof_prach_threads   = 1;
   bool                    extended_cp         = false;
+  float                   pucch_threshold_format1             = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1;
+  float                   pucch_threshold_data_valid_format1a = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1A;
+  float                   pucch_threshold_data_valid_format2  = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT2;
+  float                   pucch_threshold_data_valid_format3  = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT3;
+  float                   pucch_threshold_dmrs                = SRSRAN_PUCCH_DEFAULT_THRESHOLD_DMRS;
```

### 2.3 srsenb: command-line / config option parsing

`srsenb/src/main.cc`, in the `[expert]` option block. Note the `estimator_fil_w` default
change from 0.1 to 0.3333 — this *preserves* runtime behavior, because 0.3333 is the value
the estimator has always used while the parsed 0.1 was ignored:

```diff
     ("expert.max_prach_offset_us", bpo::value<float>(&args->phy.max_prach_offset_us)->default_value(30), "Maximum allowed RACH offset (in us).")
+    ("expert.prach_detect_factor", bpo::value<float>(&args->phy.prach_detect_factor)->default_value(60), "PRACH detection threshold as a factor over the correlation average. Lower values detect weaker preambles at the cost of more false detections.")
+    ("expert.pusch_min_snr_info_db", bpo::value<float>(&args->phy.pusch_min_snr_info_db)->default_value(1.0), "Minimum estimated PUSCH SNR (in dB) for reporting SNR/TA measurements to the stack. The measurement includes noise power, so a missed grant (DTX) reads approx. 0 dB; keep this above 0.")
+    ("expert.pucch_threshold_format1", bpo::value<float>(&args->phy.pucch_threshold_format1)->default_value(SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1), "PUCCH SR/ACK detection correlation threshold. Lower values detect weaker UEs at the cost of false detections.")
+    ("expert.pucch_threshold_data_valid_format1a", bpo::value<float>(&args->phy.pucch_threshold_data_valid_format1a)->default_value(SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1A), "PUCCH format 1a/1b ACK/NACK validity correlation threshold.")
+    ("expert.pucch_threshold_data_valid_format2", bpo::value<float>(&args->phy.pucch_threshold_data_valid_format2)->default_value(SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT2), "PUCCH format 2 CQI validity correlation threshold.")
+    ("expert.pucch_threshold_data_valid_format3", bpo::value<float>(&args->phy.pucch_threshold_data_valid_format3)->default_value(SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT3), "PUCCH format 3 validity correlation threshold.")
+    ("expert.pucch_threshold_dmrs", bpo::value<float>(&args->phy.pucch_threshold_dmrs)->default_value(SRSRAN_PUCCH_DEFAULT_THRESHOLD_DMRS), "PUCCH DMRS detection gate threshold. Set to 0 to disable the DMRS gate.")
     ("expert.equalizer_mode", bpo::value<string>(&args->phy.equalizer_mode)->default_value("mmse"), "Equalizer mode.")
-    ("expert.estimator_fil_w", bpo::value<float>(&args->phy.estimator_fil_w)->default_value(0.1), "Chooses the coefficients for the 3-tap channel estimator centered filter.")
+    ("expert.estimator_fil_w", bpo::value<float>(&args->phy.estimator_fil_w)->default_value(0.3333), "Coefficient w of the 3-tap [w, 1-2w, w] UL channel estimator smoothing filter. 0.3333 is uniform averaging (maximum noise smoothing); reduce only for highly frequency-selective channels.")
```

### 2.4 srsenb: PRACH detection-factor plumbing

`srsenb/hdr/phy/prach_worker.h` — mirror the existing `set_max_prach_offset_us` pattern in
both the worker and the per-carrier pool:

```diff
   int  new_tti(uint32_t tti, cf_t* buffer);
   void set_max_prach_offset_us(float delay_us);
+  void set_detect_factor(float factor);
   void stop();
```

```diff
   // in class prach_worker_pool
+  void set_detect_factor(float factor)
+  {
+    for (auto& prach : prach_vec) {
+      prach->set_detect_factor(factor);
+    }
+  }
```

`srsenb/src/phy/prach_worker.cc` — the library setter already exists
(`srsran_prach_set_detect_factor()`, `lib/src/phy/phch/prach.c:795`); the hardcoded
`srsran_prach_set_detect_factor(&prach, 60)` in `init()` stays as the initial value and the
new setter overrides it at startup:

```diff
 void prach_worker::set_max_prach_offset_us(float delay_us)
 {
   max_prach_offset_us = delay_us;
 }

+void prach_worker::set_detect_factor(float factor)
+{
+  srsran_prach_set_detect_factor(&prach, factor);
+}
```

`srsenb/src/phy/phy.cc`, in `phy::init_lte()` (only the LTE PRACH pool exists here; the NR
path is untouched):

```diff
   prach.set_max_prach_offset_us(args.max_prach_offset_us);
+  prach.set_detect_factor(args.prach_detect_factor);
```

### 2.5 srsenb: PUCCH threshold plumbing

`srsenb/src/phy/phy_ue_db.cc`, `_set_common_config_rnti()` — replace the hardcoded macros
with the parsed values (the `phy_args` pointer already exists as a member):

```diff
-  phy_cfg.ul_cfg.pucch.threshold_format1             = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1;
-  phy_cfg.ul_cfg.pucch.threshold_data_valid_format1a = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT1A;
-  phy_cfg.ul_cfg.pucch.threshold_data_valid_format2  = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT2;
-  phy_cfg.ul_cfg.pucch.threshold_data_valid_format3  = SRSRAN_PUCCH_DEFAULT_THRESHOLD_FORMAT3;
-  phy_cfg.ul_cfg.pucch.threshold_dmrs_detection      = SRSRAN_PUCCH_DEFAULT_THRESHOLD_DMRS;
+  phy_cfg.ul_cfg.pucch.threshold_format1             = phy_args->pucch_threshold_format1;
+  phy_cfg.ul_cfg.pucch.threshold_data_valid_format1a = phy_args->pucch_threshold_data_valid_format1a;
+  phy_cfg.ul_cfg.pucch.threshold_data_valid_format2  = phy_args->pucch_threshold_data_valid_format2;
+  phy_cfg.ul_cfg.pucch.threshold_data_valid_format3  = phy_args->pucch_threshold_data_valid_format3;
+  phy_cfg.ul_cfg.pucch.threshold_dmrs_detection      = phy_args->pucch_threshold_dmrs;
```

Setting `pucch_threshold_dmrs = 0` disables the DMRS gate entirely: the library skips the
check when the value is not `isnormal()` (`pucch.c:828`).

### 2.6 srsenb: PUSCH SNR-report gate + estimator wiring in cc_worker

`srsenb/hdr/phy/lte/cc_worker.h` — remove the hardcoded gate and the never-referenced
`PUCCH_RL_CORR_TH` constant:

```diff
 private:
-  constexpr static float PUSCH_RL_SNR_DB_TH = 1.0f;
-  constexpr static float PUCCH_RL_CORR_TH   = 0.15f;
-
   int  encode_pdsch(stack_interface_phy_lte::dl_sched_grant_t* grants, uint32_t nof_grants);
```

`srsenb/src/phy/lte/cc_worker.cc` — apply the estimator coefficient once at init (right
after `srsran_enb_ul_set_cell()`), and make the report gate read the config:

```diff
   if (srsran_enb_ul_set_cell(&enb_ul, cell, &phy->dmrs_pusch_cfg, nullptr)) {
     ERROR("Error initiating ENB UL");
     return;
   }

+  srsran_enb_ul_set_smooth_filter3_coeff(&enb_ul, phy->params.estimator_fil_w);
+
   /* Setup SI-RNTI in PHY */
```

```diff
   float snr_db = enb_ul.chest_res.snr_db;

   // Notify MAC of RL status
-  if (snr_db >= PUSCH_RL_SNR_DB_TH) {
+  if (snr_db >= phy->params.pusch_min_snr_info_db) {
     // Notify MAC UL channel quality
     phy->stack->snr_info(ul_sf.tti, rnti, cc_idx, snr_db, mac_interface_phy_lte::PUSCH);
```

The `ta_info` call remains inside the same gate (unchanged code below the diff context) —
timing-advance estimates from near-noise DMRS are unreliable, so they are only forwarded
when the SNR gate passes. For better TA/CFO estimation at low SNR, the already-plumbed
`expert.use_cedron_f_est_alg = true` is the companion knob.

### 2.7 enb.conf.example documentation

Fix the stale comment (real default is 8, not 4):

```diff
-# pusch_max_its:        Maximum number of turbo decoder iterations (default: 4)
+# pusch_max_its:        Maximum number of turbo decoder half-iterations (default: 8)
```

Document the new options in the `[expert]` comment block:

```
# max_prach_offset_us:  Maximum allowed RACH offset (in us). Limits the cell radius (1 us ~ 150 m); raise for large cells
# prach_detect_factor:  PRACH detection threshold as a factor over the correlation average (default: 60).
#                       Lower values (e.g. 25-35) detect weaker preambles from faraway UEs at the cost of
#                       occasional false RACH detections
# pusch_min_snr_info_db: Minimum estimated PUSCH SNR (in dB) for reporting SNR/TA measurements to the
#                       stack (default: 1.0). The estimate includes noise power, so a missed grant (DTX)
#                       reads approx. 0 dB: keep this above 0 or DTX subframes contaminate power control.
#                       Lower towards 0.2-0.5 so power control and link adaptation keep tracking cell-edge UEs
# pucch_threshold_format1: PUCCH SR/ACK detection correlation threshold (default: 0.5). Lower values
#                       (e.g. 0.2-0.3) detect PUCCH from weaker UEs; too low causes false ACK/SR detections
# pucch_threshold_data_valid_format1a: PUCCH format 1a/1b ACK/NACK validity threshold (default: 0.5)
# pucch_threshold_data_valid_format2: PUCCH format 2 CQI validity threshold (default: 0.5)
# pucch_threshold_data_valid_format3: PUCCH format 3 validity threshold (default: 0.5)
# pucch_threshold_dmrs: PUCCH DMRS detection gate (default: 0.4). Lower (e.g. 0.2) for weak UEs; 0 disables the gate
# estimator_fil_w:      Coefficient w of the 3-tap [w, 1-2w, w] UL channel estimator smoothing filter
#                       (default: 0.3333 = uniform averaging = maximum noise smoothing, best at low SNR).
#                       Reduce only for highly frequency-selective channels
```

And the commented defaults in the `[expert]` section body:

```
#max_prach_offset_us  = 30
#prach_detect_factor  = 60
#pusch_min_snr_info_db = 1.0
#pucch_threshold_format1 = 0.5
#pucch_threshold_data_valid_format1a = 0.5
#pucch_threshold_data_valid_format2 = 0.5
#pucch_threshold_data_valid_format3 = 0.5
#pucch_threshold_dmrs = 0.4
#estimator_fil_w      = 0.3333
```

---

## 3. Recommended tuned values (receiver side)

Starting point for a coverage-limited deployment; adjust gradually while watching metrics:

```ini
[expert]
prach_detect_factor   = 30     # detect weaker RACH preambles (watch for false RACH)
max_prach_offset_us   = 60     # for cells larger than ~4 km radius
pusch_max_its         = 16     # more turbo half-iterations; CRC early-stop keeps average cost low
pusch_min_snr_info_db = 0.3    # keep link adaptation tracking cell-edge UEs (never <= 0)
pucch_threshold_format1 = 0.25 # detect weaker SR/ACK (lower conservatively: false ACKs corrupt DL HARQ)
pucch_threshold_data_valid_format1a = 0.3
pucch_threshold_data_valid_format2  = 0.3
pucch_threshold_dmrs  = 0.2
estimator_fil_w       = 0.3333 # keep: already maximum smoothing
use_cedron_f_est_alg  = true   # better TA/CFO estimation at low SNR
```

Rollout order: (1) PRACH settings until distant UEs attach reliably; (2) the SNR-report
gate, confirming with `metrics_csv_enable = true` that `ul_snr` tracks the weak UE and UL
MCS settles low and stable; (3) PUCCH thresholds last, watching the DL retransmission rate
and data integrity; (4) `pusch_max_its` any time — it is low-risk.

---

## 4. Build and verification

Build (out of tree):

```sh
mkdir -p build && cd build && cmake ../ && make -j"$(nproc)" srsenb
```

Unit tests covering the touched code (all must pass unmodified, since library defaults are
unchanged):

```sh
ctest -R "prach_test|pucch_test$|chest_test_ul"
```

- `lib/src/phy/phch/test/prach_test.c`, `prach_test_multi.c` — exercise
  `srsran_prach_detect_offset()` and the detection factor.
- `lib/src/phy/phch/test/pucch_test.c` — exercises the threshold fields of
  `srsran_pucch_cfg_t`.
- `lib/src/phy/ch_estimation/test/chest_test_ul.c` — covers
  `srsran_chest_ul_estimate_pusch()` with the smoothing filter and noise calibration.

Config regression:

```sh
./build/srsenb/src/srsenb --help | grep -E "prach_detect_factor|pusch_min_snr_info_db|pucch_threshold|estimator_fil_w"
```

must show every new option at its behavior-preserving default (60 / 1.0 / 0.5 / 0.5 / 0.5 /
0.5 / 0.4 / 0.3333), and a stock `enb.conf` must produce identical startup values.

Field validation (weak-signal bench: UE behind ~90–100 dB attenuation, or the built-in
channel emulator with `channel.ul.awgn.enable = true` and a low SNR setting):

1. **Defaults regression** — run the stock config; attach and traffic behavior must be
   identical, and `PUSCH: ... snr=...` info-log lines (with `log.phy_level = info`) must
   appear as before.
2. **PRACH** — set `prach_detect_factor = 30`; the weak UE's preamble should now appear in
   the log (`RACH: ... p2avg=...`) and the UE should attach. Leave the eNB running
   overnight with no UE transmitting to measure the false-RACH rate.
3. **SNR gate** — set `pusch_min_snr_info_db = 0.3`; with `metrics_csv_enable = true`,
   confirm `ul_snr` in the CSV tracks the weak UE (instead of sticking near the scheduler's
   initial value) and UL MCS settles low and stable instead of oscillating.
4. **PUCCH** — lower the thresholds; confirm the DL retransmission rate for the weak UE
   drops and iperf DL throughput recovers; verify data integrity end-to-end (false-ACK
   check).
5. **Estimator knob** — set `estimator_fil_w = 0.2` and confirm the reported `ul_snr` /
   noise metrics shift (proves the option is now live); return it to 0.3333 for operation.

---

## 5. Out of scope of this document

- **Power control**: the TPC minimum-SNR sample floor (`scheduler.tpc_min_snr_db`) and the
  closed-loop power-control configuration (`target_pusch_sinr`, `target_pucch_sinr`,
  `enable_phr_handling` in rr.conf; `p0_nominal_pusch` / `alpha` in sib.conf).
- **Future work**: 2-antenna UL receive diversity (MRC), longer smoothing filters with
  recalibrated noise estimation, time-domain channel interpolation for high-Doppler UEs.
