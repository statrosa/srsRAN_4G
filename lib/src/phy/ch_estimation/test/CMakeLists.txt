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

set(CTEST_LABELS "lib;phy;chest")

########################################################################
# Downlink Channel Estimation TEST  
########################################################################

add_executable(chest_test_dl chest_test_dl.c)
target_link_libraries(chest_test_dl srsran_phy)

add_lte_test(chest_test_dl_cellid0 chest_test_dl -c 0)
add_lte_test(chest_test_dl_cellid1 chest_test_dl -c 1)
add_lte_test(chest_test_dl_cellid2 chest_test_dl -c 2)

add_lte_test(chest_test_dl_cellid0_50prb chest_test_dl -c 0 -r 50)
add_lte_test(chest_test_dl_cellid1_50prb chest_test_dl -c 1 -r 50)
add_lte_test(chest_test_dl_cellid2_50prb chest_test_dl -c 2 -r 50)


########################################################################
# Uplink Channel Estimation TEST  
########################################################################

add_executable(chest_test_ul chest_test_ul.c)
target_link_libraries(chest_test_ul srsran_phy srsran_common)

add_executable(refsignal_ul_test_all refsignal_ul_test.c)
target_link_libraries(refsignal_ul_test_all srsran_phy srsran_common)

add_lte_test(chest_test_ul_cellid0 chest_test_ul -c 0 -r 50)
add_lte_test(chest_test_ul_cellid1 chest_test_ul -c 1 -r 50)
add_lte_test(chest_test_ul_cellid2 chest_test_ul -c 2 -r 50)

# Estimation quality on narrow allocations: -M bounds the channel estimate NMSE (relative to N0, raw LS
# equals 1.0) and -E bounds the noise estimate error in dB
add_lte_test(chest_test_ul_quality_prb1_snr0 chest_test_ul -r 50 -L 1 -s 0 -N 200 -M 0.25 -E 0.5)
add_lte_test(chest_test_ul_quality_prb1_snr10 chest_test_ul -r 50 -L 1 -s 10 -N 200 -M 0.25 -E 0.5)
add_lte_test(chest_test_ul_quality_prb1_snr20 chest_test_ul -r 50 -L 1 -s 20 -N 200 -M 0.45 -E 0.5)
add_lte_test(chest_test_ul_quality_prb2_snr0 chest_test_ul -r 50 -L 2 -s 0 -N 200 -M 0.13 -E 0.5)
add_lte_test(chest_test_ul_quality_prb4_snr0 chest_test_ul -r 50 -L 4 -s 0 -N 200 -M 0.12 -E 0.5)
add_lte_test(chest_test_ul_quality_prb4_snr10 chest_test_ul -r 50 -L 4 -s 10 -N 200 -M 0.17 -E 0.5)
# Cross-slot DMRS averaging under CFO: small CFO is phase-aligned and combined; large CFO falls back to
# per-slot estimates
add_lte_test(chest_test_ul_quality_cfo50 chest_test_ul -r 50 -L 1 -s 0 -f 50 -N 200 -M 0.28 -E 0.5)
add_lte_test(chest_test_ul_quality_cfo300 chest_test_ul -r 50 -L 1 -s 0 -f 300 -N 200 -M 0.45 -E 0.5)
add_lte_test(chest_test_ul_quality_nonoise chest_test_ul -r 50 -L 2 -N 20 -M 0.0001)
# Intra-subframe frequency hopping: CE must be written at each slot's actual (post-hopping) PRBs and the
# cross-slot CFO estimate flagged invalid
add_lte_test(chest_test_ul_quality_hop chest_test_ul -r 50 -L 4 -H -N 20 -M 0.0001)
add_lte_test(chest_test_ul_quality_hop_snr10 chest_test_ul -r 50 -L 4 -H -s 10 -N 200 -M 0.32 -E 0.5)
# Timing offset (-t, micro-seconds): without TA de-rotation the pilot phase ramp biases the smoothed
# estimates and the noise estimate (at 4 us / 20 dB the CE gets worse than raw LS and the noise estimate
# +3.4 dB). Every -t case also asserts the reported ta_us within 0.2 us of the stimulus.
add_lte_test(chest_test_ul_quality_to4_prb4_snr0 chest_test_ul -r 50 -L 4 -s 0 -t 4.0 -N 200 -M 0.13 -E 0.5)
add_lte_test(chest_test_ul_quality_to3_prb1_snr20 chest_test_ul -r 50 -L 1 -s 20 -t 3.0 -N 200 -M 0.55 -E 1.0)
add_lte_test(chest_test_ul_quality_to1_nonoise chest_test_ul -r 50 -L 2 -t 1.0 -N 20 -M 0.0001)
# Wide grants use delay-domain denoising: the noise gain is the kept-bin fraction of the delay window
# (mid/high SNR: 0.124 -> 0.072 at 25 PRB), the channel passes undistorted (noiseless case exact) and it
# stacks with the TA de-rotation
add_lte_test(chest_test_ul_quality_prb12_snr0 chest_test_ul -r 50 -L 12 -s 0 -N 200 -M 0.12 -E 0.5)
add_lte_test(chest_test_ul_quality_prb25_snr10 chest_test_ul -r 50 -L 25 -s 10 -N 200 -M 0.10 -E 0.5)
add_lte_test(chest_test_ul_quality_prb25_nonoise chest_test_ul -r 50 -L 25 -N 20 -M 0.0001)
add_lte_test(chest_test_ul_quality_prb25_to3 chest_test_ul -r 50 -L 25 -s 10 -t 3.0 -N 200 -M 0.10 -E 0.5)
add_lte_test(chest_test_ul_quality_prb12_selective chest_test_ul -r 50 -L 12 -s 20 -S -N 200 -M 6)
# Phase-corrected time interpolation at high SNR: a 200 Hz CFO leaves the per-slot hold with NMSE 10.5
# (phase-error dominated); interpolating with the cross-slot phase ramp restores the no-CFO quality
add_lte_test(chest_test_ul_quality_interp_cfo200 chest_test_ul -r 50 -L 4 -s 25 -f 200 -N 200 -M 0.55 -E 0.5)
add_lte_test(chest_test_ul_quality_interp_nocfo chest_test_ul -r 50 -L 4 -s 25 -N 200 -M 0.55 -E 0.5)
add_lte_test(chest_test_ul_quality_interp_cfo_to2 chest_test_ul -r 50 -L 4 -s 25 -f 200 -t 2.0 -N 200 -M 0.55 -E 0.5)
# Coarse/fine timing sweep (-W): rank 10 window candidates (64-sample steps) with
# srsran_chest_ul_rank_pusch, centered-estimate the winner; offsets far beyond the blind de-rotation
# range decode at well-timed quality and coarse+residual TA recovers the true offset within 0.2 us
add_lte_test(chest_test_ul_sweep_to20_snr5 chest_test_ul -r 50 -L 25 -s 5 -t 20 -W -N 100 -M 0.10 -E 0.5)
add_lte_test(chest_test_ul_sweep_to25_prb8_snr0 chest_test_ul -r 50 -L 8 -s 0 -t 25 -W -N 100 -M 0.13 -E 0.5)
add_lte_test(chest_test_ul_sweep_to12_nonoise chest_test_ul -r 50 -L 12 -t 12 -W -N 20 -M 0.0001)
# Centered-window estimate (-w): a correct external center matches auto quality; a center wrong by 5 us
# trips the energy-capture guard and degrades gracefully to the FIR path (without the guard the
# projection deletes the channel: NMSE 9.9, noise estimate +10.8 dB)
add_lte_test(chest_test_ul_center_correct chest_test_ul -r 50 -L 25 -s 10 -t 5.0 -w 5.0 -N 100 -M 0.10 -E 0.5)
add_lte_test(chest_test_ul_center_guard chest_test_ul -r 50 -L 25 -s 10 -t 5.0 -w 0 -N 100 -M 0.30 -E 1.0)
# Frequency-selective channel at high SNR: regression guard against over-smoothing (the time-varying
# stimulus is also tracked by the high-SNR time interpolation, hence the tight bound)
add_lte_test(chest_test_ul_quality_selective chest_test_ul -r 50 -L 6 -s 30 -S -N 200 -M 60)

########################################################################
# Uplink Sounding Reference Signals Channel Estimation TEST
########################################################################

add_executable(chest_test_srs chest_test_srs.c)
target_link_libraries(chest_test_srs srsran_phy srsran_common)

foreach (cell_n_prb 6 15 25 50 75 100)
  add_lte_test(chest_test_srs_${cell_n_prb} chest_test_srs -c 2 -r ${cell_n_prb})
endforeach(cell_n_prb 6 15 25 50 75 100)


########################################################################
# Downlink Channel Estimation for NB-IoT TEST
########################################################################

add_executable(chest_nbiot_test_dl chest_nbiot_test_dl.c)
target_link_libraries(chest_nbiot_test_dl srsran_phy)

add_lte_test(chest_nbiot_test_dl chest_nbiot_test_dl)


########################################################################
# Sidelink Channel Estimation TEST
########################################################################

add_executable(chest_test_sl chest_test_sl.c)
target_link_libraries(chest_test_sl srsran_phy)

add_lte_test(chest_test_sl_psbch chest_test_sl)


########################################################################
# NR PDSCH DMRS Channel Estimation TEST
########################################################################

add_executable(dmrs_pdsch_test dmrs_pdsch_test.c)
target_link_libraries(dmrs_pdsch_test srsran_phy)

add_nr_test(dmrs_pdsch_test dmrs_pdsch_test)


########################################################################
# NR PDSCH DMRS Channel Estimation TEST
########################################################################

add_executable(dmrs_pdcch_test dmrs_pdcch_test.c)
target_link_libraries(dmrs_pdcch_test srsran_phy)

add_nr_test(dmrs_pdcch_test_non_interleaved dmrs_pdcch_test)
add_nr_test(dmrs_pdcch_test_interleaved dmrs_pdcch_test -I)


########################################################################
# NR CSI RS Measurement test
########################################################################

add_executable(csi_rs_test csi_rs_test.c)
target_link_libraries(csi_rs_test srsran_phy)

add_nr_test(csi_rs_test csi_rs_test -o 3 -S 0 -L 150 -f 3 -p 15)


########################################################################
# NR CSI RS Pattern test
########################################################################

add_executable(csi_rs_pattern_test csi_rs_pattern_test.c)
target_link_libraries(csi_rs_pattern_test srsran_phy)

add_nr_test(csi_rs_pattern_test csi_rs_pattern_test)

