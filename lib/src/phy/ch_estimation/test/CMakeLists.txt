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
add_lte_test(chest_test_ul_quality_prb1_snr0 chest_test_ul -r 50 -L 1 -s 0 -N 200 -M 0.35 -E 0.5)
add_lte_test(chest_test_ul_quality_prb1_snr10 chest_test_ul -r 50 -L 1 -s 10 -N 200 -M 0.35 -E 0.5)
add_lte_test(chest_test_ul_quality_prb1_snr20 chest_test_ul -r 50 -L 1 -s 20 -N 200 -M 0.45 -E 0.5)
add_lte_test(chest_test_ul_quality_prb2_snr0 chest_test_ul -r 50 -L 2 -s 0 -N 200 -M 0.20 -E 0.5)
add_lte_test(chest_test_ul_quality_prb4_snr0 chest_test_ul -r 50 -L 4 -s 0 -N 200 -M 0.20 -E 0.5)
add_lte_test(chest_test_ul_quality_prb4_snr10 chest_test_ul -r 50 -L 4 -s 10 -N 200 -M 0.32 -E 0.5)
add_lte_test(chest_test_ul_quality_nonoise chest_test_ul -r 50 -L 2 -N 20 -M 0.0001)
# Intra-subframe frequency hopping: CE must be written at each slot's actual (post-hopping) PRBs and the
# cross-slot CFO estimate flagged invalid
add_lte_test(chest_test_ul_quality_hop chest_test_ul -r 50 -L 4 -H -N 20 -M 0.0001)
add_lte_test(chest_test_ul_quality_hop_snr10 chest_test_ul -r 50 -L 4 -H -s 10 -N 200 -M 0.32 -E 0.5)
# Frequency-selective channel at high SNR: regression guard against over-smoothing
add_lte_test(chest_test_ul_quality_selective chest_test_ul -r 50 -L 6 -s 30 -S -N 200 -M 95)

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

