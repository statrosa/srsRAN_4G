# PUSCH Gaussian Frequency Smoothing — Code and Selection Logic

This document covers one stage of the PUSCH uplink channel estimator in isolation: the
**FIR frequency-domain smoothing of the least-squares (LS) DMRS estimates**, i.e. the
adaptive tap-length Gaussian filters and the rule that picks between them. The other
pipeline stages (TA de-rotation, delay-domain denoising, cross-slot averaging, time
interpolation) are deliberately out of scope here; see
`pusch_channel_estimation_enhancements.md` for those.

Source files:

- Filter construction and application: `lib/src/phy/ch_estimation/chest_common.c`
- Per-grant selection: `lib/src/phy/ch_estimation/chest_ul.c` (`pusch_select_filter`)
- Enabled by `pusch_opts.adaptive_smoothing` (default on), settable via
  `srsran_chest_ul_set_pusch_opts()`. When disabled, PUSCH falls back to the legacy
  fixed 3-tap filter with extrapolated edges (the one PUCCH and SRS always use).

---

## 1. What the smoother does

After LS estimation, each pilot subcarrier carries one noisy channel sample:

```
p[k] = H[k] + n[k],   k = 0 .. nrefs-1   (nrefs = 12 * L_prb per slot)
```

Adjacent subcarriers of a physical channel are highly correlated (delay spread ≤ CP
bounds how fast `H[k]` can change), while the noise is independent per subcarrier.
Convolving with a normalized low-pass window `w` therefore keeps the channel and
averages the noise:

```
ĥ[k] = Σ_m w[m] · p[k + m - h]        (h = filter_len / 2)
```

- **Noise gain** of the interior outputs: `Σ w²` (fraction of N0 that survives).
- **Channel distortion**: a channel component that rotates by α radians per subcarrier
  (equivalently, a path at delay τ = α/(2π·15 kHz)) is attenuated by the filter's
  frequency response `H(α)` — the price paid for the noise reduction.

Choosing the tap length is exactly this trade: longer filter → less noise, more risk of
attenuating genuine frequency selectivity.

## 2. Filter construction code

Both constructors live in `lib/src/phy/ch_estimation/chest_common.c` and fill a
caller-provided tap array, returning the filter length.

### 2.1 Gaussian window (5- and 9-tap tiers)

```c
uint32_t srsran_chest_set_smooth_filter_gauss(float* filter, uint32_t order, float std_dev)
{
  const uint32_t filterlen = order + 1;
  const int      center    = (filterlen - 1) / 2;

  if (!filterlen) {
    return 0;
  }

  for (int i = 0; i < filterlen; i++) {
    filter[i] = expf(-powf(i - center, 2) / (2.0f * powf(std_dev, 2)));
  }

  // Calculate average for normalization
  const float norm = srsran_vec_acc_ff(filter, filterlen);

  // Avoids NAN, INF or ZERO division
  if (!isnormal(norm)) {
    return 0;
  }

  // Normalize filter
  srsran_vec_sc_prod_fff(filter, 1.0f / norm, filter, filterlen);

  return filterlen;
}
```

The taps are a sampled, unit-sum Gaussian: `w[i] = exp(-(i-c)²/2σ²) / Σ`. The estimator
calls it with `order = len - 1` and `std_dev = len / 4.0f`, so the window's ±2σ points
land at the filter edges — wide enough that no tap is wasted on negligible weights,
narrow enough that the window tapers instead of acting like a boxcar (a taper has lower
sidelobes, so a selective channel leaks less bias into distant subcarriers).

### 2.2 Three-tap flat filter (high-SNR tier — not a Gaussian)

```c
uint32_t srsran_chest_set_smooth_filter3_coeff(float* smooth_filter, float w)
{
  smooth_filter[0] = w;
  smooth_filter[2] = w;
  smooth_filter[1] = 1 - 2 * w;
  return 3;
}
```

Called with `w = 0.3333f`, i.e. taps `[⅓ ⅓ ⅓]`. These are the same taps the legacy
estimator has always used; in the adaptive path only the band-edge handling differs
(section 4). It is kept for the high-SNR tier precisely because it is short.

### 2.3 The taps, numerically

| Filter | σ | Taps (center outward) | Noise gain Σw² | Noise reduction | Half-power delay* |
|---|---|---|---|---|---|
| 3-tap flat | — | 0.3333, 0.3333 | 0.3333 | 4.8 dB | ~10.9 µs |
| 5-tap Gaussian | 1.25 | 0.3324, 0.2414, 0.0924 | 0.2441 | 6.1 dB | ~8.2 µs |
| 9-tap Gaussian | 2.25 | 0.1854, 0.1680, 0.1249, 0.0762, 0.0382 | 0.1366 | 8.7 dB | ~4.9 µs |

\* The path delay at which the filter's response has dropped 3 dB. All three pass a
within-CP (≤ 4.7 µs) channel with at most ~3 dB attenuation of its fastest component;
the 9-tap sits right at that limit, which is why it is the cap (9 taps × 15 kHz =
135 kHz footprint, below the coherence bandwidth of even long-delay-spread channels).

**A 7-tap filter is never produced in practice.** The width-scaling rule below computes
`(nrefs/3) | 1`, and `nrefs` is always a multiple of 12: 1 PRB gives `(12/3)|1 = 5`,
2 PRB gives `(24/3)|1 = 9`, wider grants clamp at 9. Only 18–23 pilots would yield 7,
and no LTE allocation has that many.

## 3. Selection logic — when each length is used

From `lib/src/phy/ch_estimation/chest_ul.c`. The selection runs once per grant, before
smoothing, and must not depend on the smoothing filter itself — so the SNR is
pre-estimated from **second differences** of the LS pilots, which cancel any locally
linear channel and leave `6·N0` of noise power per sample:

```c
// SNR tier limits for the adaptive PUSCH smoothing filter (linear power)
#define PUSCH_SMOOTH_SNR_HIGH 20.0f // ~13 dB: 16QAM+ operating region, keep the legacy short filter
#define PUSCH_SMOOTH_SNR_LOW 3.16f  // ~5 dB: below this, smooth as much as the channel plausibly allows

/**
 * Selects the PUSCH frequency-domain smoothing filter for the current grant from a cheap,
 * filter-independent SNR pre-estimate. Second differences of the LS pilot estimates cancel any locally
 * linear channel, so their average power is 6*N0 plus a residual channel-curvature term that is negligible
 * at 15 kHz subcarrier spacing. At low SNR a longer Gaussian filter trades a little frequency resolution
 * for a large noise reduction, which is where small QPSK grants operate; at high SNR the legacy short
 * filter preserves frequency selectivity. The result is written to q->pusch_filter/pusch_filter_len and
 * the pre-estimated SNR to q->pusch_snr_prior.
 */
static void pusch_select_filter(srsran_chest_ul_t* q, uint32_t nrefs_sym, uint32_t nslots)
{
  // Raw noise pre-estimate from pilot second differences, averaged over both slots
  float n0_raw = 0.0f;
  for (uint32_t s = 0; s < nslots; s++) {
    const cf_t* p = &q->pilot_estimates[s * nrefs_sym];
    cf_t*       d = q->tmp_noise; // free at this point, used as scratch
    srsran_vec_sum_ccc(&p[0], &p[2], d, nrefs_sym - 2);
    srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
    srsran_vec_sub_ccc(d, &p[1], d, nrefs_sym - 2);
    // E{|n[k-1] - 2n[k] + n[k+1]|^2} = 6*N0 for white noise
    n0_raw += srsran_vec_avg_power_cf(d, nrefs_sym - 2) / 6.0f / (float)nslots;
  }

  float epre      = srsran_vec_avg_power_cf(q->pilot_recv_signal, nslots * nrefs_sym);
  float snr_prior = isnormal(n0_raw) ? (epre / n0_raw) : INFINITY;
  q->pusch_snr_prior = snr_prior;

  uint32_t len;
  if (snr_prior >= PUSCH_SMOOTH_SNR_HIGH) {
    len = 3;
  } else if (snr_prior >= PUSCH_SMOOTH_SNR_LOW) {
    len = 5;
  } else {
    // Scale with the allocation width but never wider than 9 taps (135 kHz), which stays below the
    // coherence bandwidth of even long-delay-spread channels
    len = (nrefs_sym / 3) | 1;
    len = SRSRAN_MAX(5, SRSRAN_MIN(9, len));
  }

  if (len != q->pusch_filter_len) {
    if (len == 3) {
      // Same taps as the legacy filter; only the band-edge handling differs
      q->pusch_filter_len = srsran_chest_set_smooth_filter3_coeff(q->pusch_filter, 0.3333f);
    } else {
      q->pusch_filter_len = srsran_chest_set_smooth_filter_gauss(q->pusch_filter, len - 1, (float)len / 4.0f);
    }
  }
}
```

### The resulting tier table

| Pre-estimated SNR | 1 PRB | ≥ 2 PRB | Why |
|---|---|---|---|
| ≥ ~13 dB (`≥ 20` linear) | 3-tap flat | 3-tap flat | 16QAM/64QAM region: estimation noise is already small relative to the data SNR, so preserving frequency selectivity (equalizer accuracy on selective channels) is worth more than 1–4 dB extra denoising. |
| ~5 – 13 dB | 5-tap Gaussian | 5-tap Gaussian | QPSK/16QAM boundary: 6 dB noise reduction with a half-power passband (~8 µs) that still comfortably covers any within-CP channel. |
| < ~5 dB (`< 3.16` linear) | 5-tap Gaussian | 9-tap Gaussian | Cell-edge QPSK: estimation noise dominates the link, so smooth as hard as physics allows. The 9-tap cap keeps the footprint (135 kHz) below the coherence bandwidth; 1-PRB grants stay at 5 taps because 9 taps would span 75% of the allocation and the truncated edges would dominate. |

The filter is rebuilt only when the length actually changes (`len !=
q->pusch_filter_len`), and it lives in PUSCH-only state (`q->pusch_filter`), so the
shared `q->smooth_filter` used by PUCCH and SRS is never touched.

## 4. Applying the taps — truncated band edges

The taps are applied by `srsran_chest_smooth_pilots_trunc()`
(`lib/src/phy/ch_estimation/chest_common.c`) rather than the legacy
`srsran_conv_same_cf()` path. The difference is only at the band edges, but for narrow
grants the edges are a large fraction of the outputs (2 of 12 subcarriers on 1 PRB):
the legacy convolution *linearly extrapolates* the input beyond the allocation, and
those extrapolated edge outputs carry up to **1.89× the input noise — worse than no
smoothing at all**. Truncating the filter to the taps that fall inside the allocation
and renormalizing them keeps every edge output unbiased (for a locally flat channel)
with noise at or below the interior level.

```c
void srsran_chest_smooth_pilots_trunc(const cf_t* input,
                                      cf_t*       output,
                                      const float* filter,
                                      uint32_t    nrefs,
                                      uint32_t    filter_len)
{
  uint32_t M = filter_len;
  uint32_t h = M / 2;

  if (M == 0 || nrefs == 0 || M > SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN) {
    return;
  }

  uint32_t first_interior = SRSRAN_MIN(h, nrefs);
  uint32_t last_interior  = nrefs > h ? nrefs - h : first_interior;
  if (last_interior < first_interior) {
    last_interior = first_interior;
  }

  // Band edges: drop the taps that fall outside the allocation and renormalize the rest, so edge outputs
  // stay unbiased for a flat channel and their noise never exceeds the interior's (unlike linear
  // extrapolation, which amplifies noise at the edge samples)
  for (uint32_t i = 0; i < first_interior; i++) {
    cf_t  acc  = 0.0f;
    float norm = 0.0f;
    for (uint32_t m = (h > i) ? (h - i) : 0; m < M; m++) {
      uint32_t k = i + m - h;
      if (k >= nrefs) {
        break;
      }
      acc += filter[m] * input[k];
      norm += filter[m];
    }
    output[i] = (norm > 0.0f) ? (acc / norm) : input[i];
  }

  // Interior: full filter, same as srsran_conv_same_cf
  for (uint32_t i = first_interior; i < last_interior; i++) {
    output[i] = srsran_vec_dot_prod_cfc(&input[i - h], filter, M);
  }

  for (uint32_t i = last_interior; i < nrefs; i++) {
    cf_t  acc  = 0.0f;
    float norm = 0.0f;
    for (uint32_t m = (h > i) ? (h - i) : 0; m < M; m++) {
      uint32_t k = i + m - h;
      if (k >= nrefs) {
        break;
      }
      acc += filter[m] * input[k];
      norm += filter[m];
    }
    output[i] = (norm > 0.0f) ? (acc / norm) : input[i];
  }
}
```

The matching noise-estimate correction (`srsran_chest_estimate_noise_bias()`) models
these truncated edge outputs exactly, so the reported noise power stays unbiased for
every tap length and allocation width — but that belongs to the noise-estimation stage,
not this document.

## 5. When to use it — and when not to

**Use the adaptive taps** (the default) whenever the estimator serves ordinary PUSCH
traffic. The measured effect on a flat channel (CE error as a fraction of N0, this
stage alone, no cross-slot combining):

| Case | Fixed legacy 3-tap | Adaptive taps |
|---|---|---|
| 1 PRB @ 0 dB | 0.58 | 0.28 |
| 2 PRB @ 0 dB | 0.46 | 0.15 |
| 4 PRB @ 0 dB | 0.40 | 0.15 |
| 1 PRB @ 20 dB | 0.58 | 0.37 |
| Selective @ 30 dB | 0.1440 (MSE) | 0.1440 (bit-identical) |

The low-SNR rows are the point: small QPSK grants at the cell edge are
estimation-noise limited, and the longer taps convert directly into decoding margin.
The last row is the safety property: at high SNR the selection falls back to the same
3 taps as before, so frequency-selective performance cannot regress.

**Caveats / when to turn it off** (`srsran_chest_ul_set_pusch_opts` with
`adaptive_smoothing = false`):

- **Uncorrected timing offsets.** The 5/9-tap filters average across the pilot phase
  ramp a timing error creates and bias the estimate low. In this codebase that is
  solved upstream (the de-rotation stage flattens the pilots first — out of scope
  here), but if this smoothing code is reused elsewhere *without* that stage, long
  taps should not be applied when the residual timing error can exceed ~1–2 µs.
- **Extreme delay spread at low SNR.** The 9-tap half-power point sits at ~4.9 µs of
  path delay. Channels whose energy genuinely extends to the CP edge lose up to 3 dB
  of their longest paths; if a deployment lives on such channels, capping the low-SNR
  tier at 5–7 taps is the first knob to try.
- **Strong narrowband interference.** The second-difference SNR prior counts
  interference as noise, which drags the selection toward longer taps. That is usually
  the desired behavior (more smoothing when the pilots are unreliable), but the tier
  choice is then driven by SINR, not thermal SNR.
- **Tier flapping.** There is no hysteresis on the ~5/13 dB thresholds; a UE hovering
  at a boundary alternates filter lengths between subframes. Each subframe is
  self-consistent (the noise bias is recomputed per selection), so this is cosmetic,
  but worth knowing when reading logs.
