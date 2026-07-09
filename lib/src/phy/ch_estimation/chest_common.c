/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "srsran/phy/ch_estimation/chest_common.h"
#include "srsran/phy/utils/convolution.h"
#include "srsran/phy/utils/vector.h"

uint32_t srsran_chest_set_triangle_filter(float* fil, int filter_len)
{
  for (int i = 0; i < filter_len / 2; i++) {
    fil[i]                      = i + 1;
    fil[i + filter_len / 2 + 1] = filter_len / 2 - i;
  }
  fil[filter_len / 2] = filter_len / 2 + 1;

  float s = 0;
  for (int i = 0; i < filter_len; i++) {
    s += fil[i];
  }
  for (int i = 0; i < filter_len; i++) {
    fil[i] /= s;
  }
  return filter_len;
}

/* Uses the difference between the averaged and non-averaged pilot estimates */
float srsran_chest_estimate_noise_pilots(cf_t* noisy, cf_t* noiseless, cf_t* noise_vec, uint32_t nof_pilots)
{
  /* Substract noisy pilot estimates */
  srsran_vec_sub_ccc(noiseless, noisy, noise_vec, nof_pilots);

  /* Compute average power */
  float power = srsran_vec_avg_power_cf(noise_vec, nof_pilots);
  return power;
}

float srsran_chest_estimate_noise_bias(const float* filter, uint32_t filter_len, uint32_t nrefs, bool extrapolate_edges)
{
  uint32_t M = filter_len;
  uint32_t h = M / 2;

  if (M == 0 || nrefs == 0) {
    return 1.0f;
  }

  // Interior outputs see the full filter centered on the output sample: the residual weight vector is
  // (filter - unit impulse) and its squared norm is the residual-to-noise ratio for white noise
  double bias_interior = 0.0;
  for (uint32_t m = 0; m < M; m++) {
    double w = (double)filter[m] - ((m == h) ? 1.0 : 0.0);
    bias_interior += w * w;
  }

  if (nrefs <= 2 * h || M > SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN) {
    // Fewer samples than the two edge regions is not a supported operating point
    return (float)bias_interior;
  }

  double total = (double)(nrefs - 2 * h) * bias_interior;

  // The h outputs at each band edge have different effective weights. Note that with extrapolation the two
  // edges of srsran_conv_same_cf are NOT mirror images of each other, so both are modeled explicitly.

  // First h outputs: effective weights over input[0 .. i+h]
  for (uint32_t i = 0; i < h; i++) {
    double w[SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN] = {};
    double wsum                               = 0.0;
    for (uint32_t m = 0; m < M; m++) {
      uint32_t t = i + m;
      if (t < h) {
        if (extrapolate_edges) {
          // srsran_conv_same_cf builds samples beyond the edge as (2+h-t)*input[1] - (1+h-t)*input[0]
          w[1] += (double)filter[m] * (double)(2 + h - t);
          w[0] -= (double)filter[m] * (double)(1 + h - t);
        }
        // truncated edges drop these taps
      } else {
        w[t - h] += (double)filter[m];
        wsum += (double)filter[m];
      }
    }
    if (!extrapolate_edges && wsum > 0.0) {
      for (uint32_t j = 0; j <= i + h; j++) {
        w[j] /= wsum;
      }
    }
    w[i] -= 1.0;
    for (uint32_t j = 0; j <= i + h; j++) {
      total += w[j] * w[j];
    }
  }

  // Last h outputs: effective weights over the last samples, index o counted backwards from the band edge
  // (o=0 is the last sample). Output nrefs-h+j sits at o = h-1-j.
  for (uint32_t j = 0; j < h; j++) {
    double   w[SRSRAN_CHEST_MAX_SMOOTH_FIL_LEN] = {};
    double   wsum                               = 0.0;
    uint32_t o_self                             = h - 1 - j;
    for (uint32_t m = 0; m < M; m++) {
      uint32_t t = j + m;
      if (extrapolate_edges) {
        if (t >= M - 1) {
          // srsran_conv_same_cf builds samples beyond the edge as (2+t-h)*input[N-1] - (1+t-h)*input[N-2]
          w[0] += (double)filter[m] * (double)(2 + t - h);
          w[1] -= (double)filter[m] * (double)(1 + t - h);
        } else {
          w[M - 2 - t] += (double)filter[m];
        }
      } else {
        if (m <= o_self + h) {
          w[o_self + h - m] += (double)filter[m];
          wsum += (double)filter[m];
        }
      }
    }
    if (!extrapolate_edges && wsum > 0.0) {
      for (uint32_t o = 0; o <= o_self + h; o++) {
        w[o] /= wsum;
      }
    }
    w[o_self] -= 1.0;
    for (uint32_t o = 0; o <= o_self + h; o++) {
      total += w[o] * w[o];
    }
  }

  return (float)(total / nrefs);
}

uint32_t srsran_chest_set_smooth_filter3_coeff(float* smooth_filter, float w)
{
  smooth_filter[0] = w;
  smooth_filter[2] = w;
  smooth_filter[1] = 1 - 2 * w;
  return 3;
}

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

void srsran_chest_average_pilots(cf_t*    input,
                                 cf_t*    output,
                                 float*   filter,
                                 uint32_t nof_ref,
                                 uint32_t nof_symbols,
                                 uint32_t filter_len)
{
  for (int l = 0; l < nof_symbols; l++) {
    srsran_conv_same_cf(&input[l * nof_ref], filter, &output[l * nof_ref], nof_ref, filter_len);
  }
}
