/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file PHY/NR_TRANSPORT/dlsch_decoding.c
* \brief Top-level routines for transmission of the PDSCH 38211 v 15.2.0
* \author Guy De Souza
* \date 2018
* \version 0.1
* \company Eurecom
* \email: desouza@eurecom.fr
* \note
* \warning
*/

#include "nr_dlsch.h"
#include "nr_dci.h"
#include "nr_sch_dmrs.h"
#include "PHY/MODULATION/nr_modulation.h"
#include "PHY/NR_REFSIG/dmrs_nr.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

//#define DEBUG_DLSCH
//#define DEBUG_DLSCH_MAPPING

void nr_pdsch_codeword_scrambling(uint8_t *in,
                                  uint32_t size,
                                  uint8_t q,
                                  uint32_t Nid,
                                  uint32_t n_RNTI,
                                  uint32_t* out) {

  uint8_t reset, b_idx;
  uint32_t x1, x2, s=0;
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_PDSCH_CODEWORD_SCRAMBLING, 1);
  reset = 1;
  x2 = (n_RNTI<<15) + (q<<14) + Nid;

  for (int i=0; i<size; i++) {
    b_idx = i&0x1f;
    if (b_idx==0) {
      s = lte_gold_generic(&x1, &x2, reset);
      reset = 0;
      if (i)
        out++;
    }
    *out ^= (((in[i])&1) ^ ((s>>b_idx)&1))<<b_idx;
    //printf("i %d b_idx %d in %d s 0x%08x out 0x%08x\n", i, b_idx, in[i], s, *out);
  }
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_PDSCH_CODEWORD_SCRAMBLING, 0);
}

void nr_pdsch_codeword_scrambling_optim(uint8_t *in,
					uint32_t size,
					uint8_t q,
					uint32_t Nid,
					uint32_t n_RNTI,
					uint32_t* out) {
  
  uint32_t x1, x2, s=0,in32;

  x2 = (n_RNTI<<15) + (q<<14) + Nid;

  s=lte_gold_generic(&x1, &x2, 1);

#if defined(__AVX2__)
  for (int i=0; i<((size>>5)+((size&0x1f) > 0 ? 1 : 0)); i++) {
    in32=_mm256_movemask_epi8(_mm256_slli_epi16(((__m256i*)in)[i],7));
    out[i]=(in32^s);
    //    printf("in[%d] %x => %x\n",i,in32,out[i]);
    s=lte_gold_generic(&x1, &x2, 0);
  }
#elif defined(__SSE4__)
  _m128i *in128;
  for (int i=0; i<((size>>5)+((size&0x1f) > 0 ? 1 : 0)); i++) {
    in128=&((__m128i*)in)[i<<1];
    ((uint16_t*)&in32)[0] = _mm128_movemask_epi8(_mm256_slli_epi16(in128[0],7));
    ((uint16_t*)&in32)[1] = _mm128_movemask_epi8(_mm256_slli_epi16(in128[1],7));
    out[i]=(in32^s);
    s=lte_gold_generic(&x1, &x2, 0);
  }
  //#elsif defined(__arm__) || defined(__aarch64)
  
#else 
  nr_pdsch_codeword_scrambling(in,
			       size,
			       q,
			       Nid,
			       n_RNTI,
			       out);
#endif
}


uint8_t nr_generate_pdsch(PHY_VARS_gNB *gNB,
			  int frame,
			  int slot) {

  NR_gNB_DLSCH_t *dlsch;
  uint32_t ***pdsch_dmrs = gNB->nr_gold_pdsch_dmrs[slot];
  int32_t** txdataF = gNB->common_vars.txdataF;
  int16_t amp = AMP;
  NR_DL_FRAME_PARMS *frame_parms = &gNB->frame_parms;
  int xOverhead = 0;
  time_stats_t *dlsch_encoding_stats=&gNB->dlsch_encoding_stats;
  time_stats_t *dlsch_scrambling_stats=&gNB->dlsch_scrambling_stats;
  time_stats_t *dlsch_modulation_stats=&gNB->dlsch_modulation_stats;
  time_stats_t *tinput=&gNB->tinput;
  time_stats_t *tprep=&gNB->tprep;
  time_stats_t *tparity=&gNB->tparity;
  time_stats_t *toutput=&gNB->toutput;
  time_stats_t *dlsch_rate_matching_stats=&gNB->dlsch_rate_matching_stats;
  time_stats_t *dlsch_interleaving_stats=&gNB->dlsch_interleaving_stats;
  time_stats_t *dlsch_segmentation_stats=&gNB->dlsch_segmentation_stats;

  for (int dlsch_id=0;dlsch_id<NUMBER_OF_NR_DLSCH_MAX;dlsch_id++) {
    dlsch = gNB->dlsch[dlsch_id][0];
    if (dlsch->slot_tx[slot] == 0) continue;

    int harq_pid = dlsch->harq_ids[frame%2][slot];
    NR_DL_gNB_HARQ_t *harq = dlsch->harq_processes[harq_pid];
    nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15 = &harq->pdsch_pdu.pdsch_pdu_rel15;
    uint32_t scrambled_output[NR_MAX_NB_CODEWORDS][NR_MAX_PDSCH_ENCODED_LENGTH>>5];
    int16_t **mod_symbs = (int16_t**)dlsch->mod_symbs;
    int16_t **tx_layers = (int16_t**)dlsch->txdataF;
    int8_t Wf[2], Wt[2], l0, l_prime, l_overline, delta;
    uint8_t dmrs_Type = rel15->dmrsConfigType;
    int nb_re_dmrs;
    uint16_t n_dmrs;
    if (rel15->dmrsConfigType==NFAPI_NR_DMRS_TYPE1) {
      nb_re_dmrs = 6*rel15->numDmrsCdmGrpsNoData;
      n_dmrs = ((rel15->rbSize+rel15->rbStart)*6)<<1;
    }
    else {
      nb_re_dmrs = 4*rel15->numDmrsCdmGrpsNoData;
      n_dmrs = ((rel15->rbSize+rel15->rbStart)*4)<<1;
    }
    uint16_t nb_re;
    nb_re = ((12*rel15->NrOfSymbols)-nb_re_dmrs-xOverhead)*rel15->rbSize*rel15->nrOfLayers;
    uint8_t Qm = rel15->qamModOrder[0];
    uint32_t encoded_length = nb_re*Qm;
    int16_t mod_dmrs[14][n_dmrs] __attribute__ ((aligned(16)));
    
    
    /// CRC, coding, interleaving and rate matching
    AssertFatal(harq->pdu!=NULL,"harq->pdu is null\n");
    start_meas(dlsch_encoding_stats);
    nr_dlsch_encoding(gNB,
		      harq->pdu, frame, slot, dlsch, frame_parms,tinput,tprep,tparity,toutput,
		      dlsch_rate_matching_stats,
		      dlsch_interleaving_stats,
		      dlsch_segmentation_stats);
    stop_meas(dlsch_encoding_stats);
#ifdef DEBUG_DLSCH
    printf("PDSCH encoding:\nPayload:\n");
    for (int i=0; i<harq->B>>7; i++) {
      for (int j=0; j<16; j++)
	printf("0x%02x\t", harq->pdu[(i<<4)+j]);
      printf("\n");
    }
    printf("\nEncoded payload:\n");
    for (int i=0; i<encoded_length>>3; i++) {
      for (int j=0; j<8; j++)
	printf("%d", harq->f[(i<<3)+j]);
      printf("\t");
    }
    printf("\n");
#endif
    
    
    
    /// scrambling
    start_meas(dlsch_scrambling_stats);
    for (int q=0; q<rel15->NrOfCodewords; q++)
      memset((void*)scrambled_output[q], 0, (encoded_length>>5)*sizeof(uint32_t));
    for (int q=0; q<rel15->NrOfCodewords; q++)
      nr_pdsch_codeword_scrambling_optim(harq->f,
					 encoded_length,
					 q,
					 rel15->dlDmrsScramblingId,
					 rel15->rnti,
					 scrambled_output[q]);
    
    stop_meas(dlsch_scrambling_stats);
#ifdef DEBUG_DLSCH
    printf("PDSCH scrambling:\n");
    for (int i=0; i<encoded_length>>8; i++) {
      for (int j=0; j<8; j++)
	printf("0x%08x\t", scrambled_output[0][(i<<3)+j]);
      printf("\n");
    }
#endif
    
    /// Modulation
    start_meas(dlsch_modulation_stats);
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_PDSCH_MODULATION, 1);
    for (int q=0; q<rel15->NrOfCodewords; q++)
      nr_modulation(scrambled_output[q],
		    encoded_length,
		    Qm,
		    mod_symbs[q]);
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_PDSCH_MODULATION, 0);
    stop_meas(dlsch_modulation_stats);
#ifdef DEBUG_DLSCH
    printf("PDSCH Modulation: Qm %d(%d)\n", Qm, nb_re);
    for (int i=0; i<nb_re>>3; i++) {
      for (int j=0; j<8; j++) {
	printf("%d %d\t", mod_symbs[0][((i<<3)+j)<<1], mod_symbs[0][(((i<<3)+j)<<1)+1]);
      }
      printf("\n");
    }
#endif
    
    
    /// Layer mapping
    nr_layer_mapping(mod_symbs,
		     rel15->nrOfLayers,
		     nb_re,
		     tx_layers);
#ifdef DEBUG_DLSCH
    printf("Layer mapping (%d layers):\n", rel15->nrOfLayers);
    for (int l=0; l<rel15->nrOfLayers; l++)
      for (int i=0; i<(nb_re/rel15->nrOfLayers)>>3; i++) {
	printf("layer %d, Re %d..%d : ",l,i<<3,(i<<3)+7);
	for (int j=0; j<8; j++) {
	  printf("l%d %d\t", tx_layers[l][((i<<3)+j)<<1], tx_layers[l][(((i<<3)+j)<<1)+1]);
	}
	printf("\n");
      }
#endif
    
    /// Antenna port mapping
    //to be moved to init phase potentially, for now tx_layers 1-8 are mapped on antenna ports 1000-1007

    /// DMRS QPSK modulation
    for (int l=rel15->StartSymbolIndex; l<rel15->StartSymbolIndex+rel15->NrOfSymbols; l++) {
      if (rel15->dlDmrsSymbPos & (1 << l))
        nr_modulation(pdsch_dmrs[l][0], n_dmrs, DMRS_MOD_ORDER, mod_dmrs[l]); // currently only codeword 0 is modulated. Qm = 2 as DMRS is QPSK modulated
    }

#ifdef DEBUG_DLSCH
    l0 = get_l0(rel15->dlDmrsSymbPos);
    printf("DMRS modulation (single symbol %d, %d symbols, type %d):\n", l0, n_dmrs>>1, dmrs_Type);
    for (int i=0; i<n_dmrs>>4; i++) {
      for (int j=0; j<8; j++) {
        printf("%d %d\t", mod_dmrs[((i<<3)+j)<<1], mod_dmrs[(((i<<3)+j)<<1)+1]);
      }
      printf("\n");
    }
#endif
    
    
    /// Resource mapping
    
    // Non interleaved VRB to PRB mapping
    uint16_t start_sc = frame_parms->first_carrier_offset + rel15->rbStart*NR_NB_SC_PER_RB;
    if (start_sc >= frame_parms->ofdm_symbol_size)
      start_sc -= frame_parms->ofdm_symbol_size;
    
#ifdef DEBUG_DLSCH_MAPPING
    printf("PDSCH resource mapping started (start SC %d\tstart symbol %d\tN_PRB %d\tnb_re %d,nb_layers %d)\n",
	   start_sc, rel15->StartSymbolIndex, rel15->rbSize, nb_re,rel15->nrOfLayers);
#endif
    for (int ap=0; ap<rel15->nrOfLayers; ap++) {
      
      // DMRS params for this ap
      get_Wt(Wt, ap, dmrs_Type);
      get_Wf(Wf, ap, dmrs_Type);
      delta = get_delta(ap, dmrs_Type);
      l_prime = 0; // single symbol ap 0
      l0 = get_l0(rel15->dlDmrsSymbPos);
      l_overline = l0;

#ifdef DEBUG_DLSCH_MAPPING
      uint8_t dmrs_symbol = l0+l_prime;
      printf("DMRS Type %d params for ap %d: Wt %d %d \t Wf %d %d \t delta %d \t l_prime %d \t l0 %d\tDMRS symbol %d\n",
	     1+dmrs_Type,ap, Wt[0], Wt[1], Wf[0], Wf[1], delta, l_prime, l0, dmrs_symbol);
#endif
      uint8_t k_prime=0;
      uint16_t m=0, n=0, dmrs_idx=0, k=0;
      
      int txdataF_offset = (slot%2)*frame_parms->samples_per_slot_wCP;

      for (int l=rel15->StartSymbolIndex; l<rel15->StartSymbolIndex+rel15->NrOfSymbols; l++) {
        k = start_sc;
        n = 0;
        k_prime = 0;
        if (dmrs_Type == NFAPI_NR_DMRS_TYPE1) // another if condition to be included to check pdsch config type (reference of k)
          dmrs_idx = rel15->rbStart*6;
        else
          dmrs_idx = rel15->rbStart*4;
        for (int i=0; i<rel15->rbSize*NR_NB_SC_PER_RB; i++) {

          if ((rel15->dlDmrsSymbPos & (1 << l)) && (k == ((start_sc+get_dmrs_freq_idx(n, k_prime, delta, dmrs_Type))%(frame_parms->ofdm_symbol_size)))) {

            if (l==(l_overline+1)) //take into account the double DMRS symbols
              l_prime = 1;
            else if (l>(l_overline+1)) {//new DMRS pair
              l_overline = l;
              l_prime = 0;
            }

            ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + (2*txdataF_offset)] = (Wt[l_prime]*Wf[k_prime]*amp*mod_dmrs[l][dmrs_idx<<1]) >> 15;
            ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + 1 + (2*txdataF_offset)] = (Wt[l_prime]*Wf[k_prime]*amp*mod_dmrs[l][(dmrs_idx<<1) + 1]) >> 15;
#ifdef DEBUG_DLSCH_MAPPING
            printf("dmrs_idx %d\t l %d \t k %d \t k_prime %d \t n %d \t txdataF: %d %d\n",
                   dmrs_idx, l, k, k_prime, n, ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + (2*txdataF_offset)],
                   ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + 1 + (2*txdataF_offset)]);
#endif
            dmrs_idx++;
            k_prime++;
            k_prime&=1;
            n+=(k_prime)?0:1;

          } else {

            if( (!(rel15->dlDmrsSymbPos & (1 << l))) || allowed_xlsch_re_in_dmrs_symbol(k,start_sc,frame_parms->ofdm_symbol_size,rel15->numDmrsCdmGrpsNoData,dmrs_Type)) {
              ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + (2*txdataF_offset)] = (amp * tx_layers[ap][m<<1]) >> 15;
              ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + 1 + (2*txdataF_offset)] = (amp * tx_layers[ap][(m<<1) + 1]) >> 15;
#ifdef DEBUG_DLSCH_MAPPING
              printf("m %d\t l %d \t k %d \t txdataF: %d %d\n",
                     m, l, k, ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + (2*txdataF_offset)],
                     ((int16_t*)txdataF[ap])[((l*frame_parms->ofdm_symbol_size + k)<<1) + 1 + (2*txdataF_offset)]);
#endif
              m++;
            }
          }
          if (++k >= frame_parms->ofdm_symbol_size)
            k -= frame_parms->ofdm_symbol_size;
        } //RE loop
      } // symbol loop
    }// layer loop


    dlsch->slot_tx[slot]=0;
  }// dlsch loop
  return 0;
}

void dump_pdsch_stats(PHY_VARS_gNB *gNB) {

  for (int i=0;i<NUMBER_OF_NR_SCH_STATS_MAX;i++)
    if (gNB->dlsch_stats[i].rnti > 0)
      LOG_I(PHY,"DLSCH RNTI %x: round_trials %d(%1.1e):%d(%1.1e):%d(%1.1e):%d, current_Qm %d, current_RI %d, total_bytes TX %d\n",
	    gNB->dlsch_stats[i].rnti,
	    gNB->dlsch_stats[i].round_trials[0],
	    (double)gNB->dlsch_stats[i].round_trials[1]/gNB->dlsch_stats[i].round_trials[0],
	    gNB->dlsch_stats[i].round_trials[1],
	    (double)gNB->dlsch_stats[i].round_trials[2]/gNB->dlsch_stats[i].round_trials[0],
	    gNB->dlsch_stats[i].round_trials[2],
	    (double)gNB->dlsch_stats[i].round_trials[3]/gNB->dlsch_stats[i].round_trials[0],
	    gNB->dlsch_stats[i].round_trials[3],
	    gNB->dlsch_stats[i].current_Qm,
	    gNB->dlsch_stats[i].current_RI,
	    gNB->dlsch_stats[i].total_bytes_tx);

}

void clear_pdsch_stats(PHY_VARS_gNB *gNB) {

  for (int i=0;i<NUMBER_OF_NR_DLSCH_MAX;i++)
    memset((void*)&gNB->dlsch_stats[i],0,sizeof(gNB->dlsch_stats[i]));
}
