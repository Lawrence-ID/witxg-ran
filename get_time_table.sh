#!/bin/bash
echo -n -e "nr_segmentation: \t\t"              ; cat log.txt | grep nr_segmentation              | ./time_statistic.sh
echo -n -e "nrLDPC_encoder: \t\t"               ; cat log.txt | grep nrLDPC_encoder               | ./time_statistic.sh
echo -n -e "nr_rate_matching_ldpc: \t\t"        ; cat log.txt | grep nr_rate_matching_ldpc        | ./time_statistic.sh
echo -n -e "nr_interleaving_ldpc: \t\t"         ; cat log.txt | grep nr_interleaving_ldpc         | ./time_statistic.sh
echo -n -e "nr_pdsch_codeword_scrambling: \t"   ; cat log.txt | grep nr_pdsch_codeword_scrambling | ./time_statistic.sh
echo -n -e "nr_modulation: \t\t\t"              ; cat log.txt | grep nr_modulation                | ./time_statistic.sh
echo -n -e "nr_layer_mapping: \t\t"             ; cat log.txt | grep nr_layer_mapping             | ./time_statistic.sh
echo -n -e "dlsch_resource_mapping_stats: \t"   ; cat log.txt | grep dlsch_resource_mapping_stats | ./time_statistic.sh
echo -n -e "dlsch_precoding_stats: \t\t"        ; cat log.txt | grep dlsch_precoding_stats        | ./time_statistic.sh
echo -n -e "PHY_ofdm_mod:idft: \t\t"            ; cat log.txt | grep PHY_ofdm_mod:idft            | ./time_statistic.sh
