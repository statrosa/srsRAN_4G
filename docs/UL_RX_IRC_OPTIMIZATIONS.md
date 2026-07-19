# MMSE-IRC for the srsenb Uplink — Design, Robustness & Results

Companion to `UL_RX_DIVERSITY.md`. This document covers the optional
interference-rejection-combining (IRC) equalizer added on top of the base
MRC receive-diversity work. It is **flag-gated, off by default, and
regression-proof by construction**.

Branch: `claude/srsenb-rx-diversity-ohkx2i`.

---

## 1. Why IRC on top of MRC

MRC is optimal against **white** noise: it maximizes SNR by combining
antennas in proportion to their channel gains. But real cellular uplinks are
frequently **interference-limited** — a neighbouring-cell UE (or any
co-channel emitter) arrives with a definite *spatial signature*, i.e. its
energy is correlated across the two RX antennas. MRC treats that coloured
interference as if it were white noise and combines right into it.

IRC (MMSE-IRC) instead **whitens** with the interference-plus-noise
covariance `R` before combining: `w ∝ R⁻¹h`. With two antennas and one
dominant interferer this places a spatial null on the interferer while still
collecting the wanted signal — the classic 2-antenna "one signal, one null"
degree of freedom.

The base diversity work already produced everything IRC needs — per-antenna
channel estimates (`res->ce[a]`), per-antenna symbol buffers, and per-antenna
DMRS residuals — so IRC is an *additive* equalizer choice, not a rework.

---

## 2. Design

### 2.1 Covariance estimation (`chest_ul.c`)

`srsran_chest_ul_res_t` gains `cf_t noise_cov[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS]`
and `bool noise_cov_valid`. Estimation runs only when
`chest.est_noise_cov` is set (zero cost otherwise):

- During the existing per-antenna PUSCH loop, each antenna's **DMRS residual**
  (LS pilot estimate minus the smoothed/averaged estimate at the same pilot
  REs) is persisted into the previously-idle `pilot_estimates_tmp[a]` scratch
  via `srsran_vec_sub_ccc`.
- After the loop, `chest_ul_estimate_noise_cov()` builds a wideband-per-grant
  `R`:
  - **diagonal** = the calibrated per-antenna noise powers already computed
    for the scalar path (identical smoothing-filter calibration factor, so
    the IRC diagonal is numerically consistent with today's
    `noise_estimate`);
  - **off-diagonal** = `E[r_i · conj(r_j)]` over the grant's pilot REs
    (`srsran_vec_dot_prod_conj_ccc`), with the same calibration factor.
- **Positive-definiteness is enforced**: cross terms are clamped to
  `0.95·√(R_ii·R_jj)`, then the matrix is diagonally loaded by
  `0.05·tr(R)/N`. Any NaN/Inf or negative diagonal sets
  `noise_cov_valid = false`.

### 2.2 IRC equalizer kernel (`precoding.c`)

```c
int srsran_predecoding_single_multi_cov(cf_t* y[SRSRAN_MAX_PORTS],
                                        cf_t* h[SRSRAN_MAX_PORTS],
                                        cf_t* x, int nof_rxant, int nof_symbols,
                                        float scaling,
                                        cf_t  cov[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS]);
```

For `nof_rxant == 2` it validates `R` (finite, positive trace, determinant
above a condition-number floor), inverts it **once per call** with the
existing `srsran_mat_2x2_inv_gen` (`lib/include/srsran/phy/utils/mat.h:36`),
then per RE:

```
w   = R⁻¹ h                       (h = [h0[i], h1[i]])
x[i] = (hᴴ R⁻¹ y) / ((hᴴ R⁻¹ h + 1) · scaling)
```

The `+1` is the MMSE normalization in the whitened domain — it keeps the
post-equalizer scale consistent with the MRC path's
`|h|² + noise` form, so downstream LLR scaling and turbo decoding are
unchanged. No new matrix-library code was required for the 2-antenna,
single-layer case; the loop mirrors `srsran_predecoding_single_gen`
(`precoding.c:287`).

### 2.3 Dispatch (`pusch.c`) — the robustness contract

```c
if (q->irc_enable && nof_rx_antennas == 2 && channel->noise_cov_valid) {
    srsran_predecoding_single_multi_cov(q->d, q->ce, q->z, 2, nof_re, 1.0f, channel->noise_cov);
} else {
    // rate-limited INFO if IRC was requested but no usable covariance
    srsran_predecoding_single_multi(q->d, q->ce, q->z, NULL, nof_rx_antennas, nof_re, 1.0f,
                                    channel->noise_estimate);   // <-- unchanged MRC line
}
```

Four layers of "never silently fail, never regress":

1. **Off by default** — `expert.equalizer_mode` defaults to `"mmse"` (MRC);
   IRC only engages on `"irc"`. The MRC code path is textually identical to
   before, so with the flag off the binary behaves bit-for-bit as the base
   diversity build (proven by the full ctest suite).
2. **Guarded engagement** — IRC runs only with exactly 2 RX antennas *and* a
   validated covariance. 1 antenna, >2 antennas, or an unusable `R` → MRC.
3. **In-kernel fallback** — even inside `_cov`, a covariance that fails the
   condition check falls back to MRC using `mean(diag R)`. The fallback is
   logged (rate-limited INFO), never silent.
4. **No stale covariance** (hardened in commit `bdc41c7`) —
   `srsran_chest_ul_estimate_pusch` invalidates `noise_cov_valid` as its
   very first statement, and `srsran_enb_ul_get_pusch` returns an error
   instead of decoding when the estimation fails, so combining one UE's
   symbols with a *previous* grant's `R` (or stale channel estimates) is
   structurally impossible.

### 2.4 Plumbing

`srsran_enb_ul_set_irc(&enb_ul, bool)` sets `chest.est_noise_cov` and
`pusch.irc_enable` (both only if `nof_rx_antennas == 2`); `cc_worker::init`
calls it when `phy->params.equalizer_mode == "irc"` and logs the decision.
Config: `expert.equalizer_mode = irc` in enb.conf (the option already parsed
but was previously consumed nowhere on the eNB — a genuinely dead knob until
now).

---

## 3. Validation

### 3.1 Unit test (`enb_ul_test.c`, in ctest)

The test injects a **rank-1 co-channel interferer** — one white waveform with
a fixed per-antenna spatial signature distinct from the wanted signal, scaled
to a target INR — and asserts:

| Check | Result |
|---|---|
| **No regression** (no interferer, marginal SNR): IRC ≈ MRC | MRC 10/10, IRC 10/10 |
| **Rejection** (15 dB directional interferer): IRC ≫ MRC | **MRC 0/10, IRC 10/10** |
| **1-antenna fallback** (flag on): decodes, finite metrics | 4/4, no NaN/Inf |
| **Stale-covariance guard**: invalid grant (`L_prb=7`) after a good IRC decode | estimate errors, `noise_cov_valid == false` |

Full ctest suite passes with the flag off (the two SCTP tests and their
kin fail only because the container kernel lacks SCTP — unrelated to this
change and present before it).

### 3.2 End-to-end ZMQ campaign (`test/zmq_diversity/`, scenario s9)

`channel_splitter.py` gained `--interf-inr`: a shared rank-1 interference
waveform added with a distinct per-antenna gain (spatially coloured
interference), after fading, before the receiver noise. Three arms at
`SNR₀+8` with a 15 dB interferer (and a clean-channel bias check):

| Arm | Equalizer | Interferer | PRACH det | Msg3 CRC OK |
|---|---|---|---|---|
| s9 MRC | MRC | 15 dB rank-1 | 90/90 | **0/360 (0%)** |
| s9 IRC | MMSE-IRC | 15 dB rank-1 | 90/90 | **87/87 (100%)** |
| s9 parity | MMSE-IRC | none | (clean) | 100% — matches MRC benign |

The separation is total: under an interferer that MRC cannot escape (0 of 360
Msg3 transmissions decode), IRC nulls it and decodes **every** transmission,
while on a clean channel IRC shows no penalty. PRACH detection is unaffected
in both arms (PRACH stays MRC/non-coherent and has enough processing gain
here) — the interference cost lands entirely on the PUSCH payload, which is
exactly where IRC recovers it.

---

## 4. Scope and limitations

- **PUSCH only.** PUCCH and PRACH remain MRC. Interference hurts the payload
  channel most, and PUSCH is where the DMRS residuals needed for `R` are
  readily available per grant.
- **2 RX antennas** for the closed-form path; other counts fall back to MRC.
- **Wideband-per-grant covariance** — `R` is estimated once per grant across
  its DMRS REs, assuming the interference spatial statistics are roughly
  stationary across the allocation. Per-RB (or per-subband) covariance is the
  natural refinement for strongly frequency-selective interference, and would
  slot into the same kernel by passing an `R` per RB.
- **Plain-C kernel.** The per-RE 2×2 matvec is inexpensive at ≤50 PRB; a SIMD
  variant (mirroring `srsran_mat_2x2_mmse_csi_simd`) is a straightforward
  future optimization if profiling on a live eNB shows it matters.
- The covariance estimate is only as good as the DMRS residual: at very high
  wanted-signal SNR with no interference, `R` approaches a scaled identity and
  IRC converges to MRC (confirmed by the parity arm).
- **Cross-term calibration is a heuristic.** The off-diagonal terms reuse the
  smoothing-filter compensation factor that was derived for the (white-noise)
  diagonal. That is accurate when the interference looks spectrally white-ish
  after de-rotation by the conjugate DMRS — the common case for a
  non-DMRS-matched interferer, and the reason the s9 nulling works — but it
  is not derived from first principles for strongly colored interference
  spectra.
- **Diagonal loading caps nulling depth.** The 5% loading that guarantees
  invertibility also bounds how deep the spatial null can get: at very high
  INR (tens of dB) suppression saturates before the interferer is fully
  removed. This is the standard robustness-versus-depth tradeoff; the
  validated 15 dB INR regime sits comfortably inside it, and lowering the
  loading factor is the knob if deeper nulls are ever needed.

## 5. Enabling it

```ini
[expert]
equalizer_mode = irc     ; default "mmse" (MRC); "irc" needs nof_rx_ant = 2
```

At startup the eNB logs `UL equalizer: MMSE-IRC enabled (2 RX antennas, MRC
fallback on unusable covariance)`, or a notice that IRC was requested but
`nof_rx_ant != 2` so MRC is used. Runtime fallbacks (covariance unusable for a
given grant) are logged rate-limited, so a persistently-failing estimator is
visible in the logs rather than silently degrading.
