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

#ifndef __PHY_VARS_UE__H__
#define __PHY_VARS_UE__H__

#include "PHY/types.h"
#include "PHY/defs_UE.h"

#include "common/ran_context.h"

char *namepointer_chMag ;
char fmageren_name2[512];
char *namepointer_log2;


#include "PHY/LTE_REFSIG/primary_synch.h"

PHY_VARS_UE ***PHY_vars_UE_g;

unsigned short rev[2048],rev_times4[8192],rev_half[1024];
unsigned short rev256[256],rev512[512],rev1024[1024],rev4096[4096],rev2048[2048],rev8192[8192];


const short conjugate[8]__attribute__((aligned(16))) = {-1,1,-1,1,-1,1,-1,1};
const short conjugate2[8]__attribute__((aligned(16))) = {1,-1,1,-1,1,-1,1,-1};


#include "SIMULATION/ETH_TRANSPORT/vars.h"


#ifndef OPENAIR2
  unsigned char NB_eNB_INST=0;
  uint16_t NB_UE_INST=0;
  unsigned char NB_RN_INST=0;
  unsigned char NB_INST=0;
#endif


int number_of_cards;


int flag_LA=0;
int flagMag;
//extern  channel_desc_t *eNB2UE[NUMBER_OF_eNB_MAX][NUMBER_OF_UE_MAX];
//extern  double ABS_SINR_eff_BLER_table[MCS_COUNT][9][9];
//extern  double ABS_beta[MCS_COUNT];odi
double sinr_bler_map[MCS_COUNT][2][MCS_TABLE_LENGTH_MAX];
int table_length[MCS_COUNT];
//double sinr_bler_map_up[MCS_COUNT][2][16];

//for MU-MIMO abstraction using MIESM
//this 2D arrarays contains SINR, MI and RBIR in rows 1, 2, and 3 respectively
double MI_map_4qam[3][162];
double MI_map_16qam[3][197];
double MI_map_64qam[3][227];

// here the first index is for transmission mode 1, 2, 5 and 6 whereas the second index is for the 16 sinr vaues corresponding to 16 CQIs
const double sinr_to_cqi[4][16]= { {-2.5051, -2.5051, -1.7451, -0.3655, 1.0812, 2.4012, 3.6849, 6.6754, 8.3885, 8.7970, 12.0437, 14.4709, 15.7281,  17.2424,  17.2424, 17.2424},
  {-2.2360, -2.2360, -1.3919, -0.0218, 1.5319,  2.9574,  4.3234, 6.3387, 8.9879, 9.5096, 12.6609, 14.0116, 16.4984, 18.1572, 18.1572, 18.1572},
  {-1, -1.0000, -0.4198, -0.0140, 1.0362,  2.3520, 3.5793, 6.1136, 8.4836, 9.0858, 12.4723, 13.9128, 16.2054, 17.7392, 17.7392, 17.7392},
  { -4.1057, -4.1057, -3.3768, -2.2916, -1.1392, 0.1236, 1.2849, 3.1933, 5.9298, 6.4052, 9.6245, 10.9414, 13.5166, 14.9545, 14.9545, 14.9545}
};

//int cqi_to_mcs[16]={0, 0, 1, 3, 5, 7, 9, 13, 15, 16, 20, 23, 25, 27, 27, 27};
const int cqi_to_mcs[16]= {0, 0, 1, 2, 4, 6, 8, 11, 13, 16, 18, 20, 23, 25, 27, 27};

//for SNR to MI conversion 7 th order Polynomial coeff
const double q_qam16[8]= {3.21151853033897e-10,5.55435952230651e-09,-2.30760065362117e-07,-6.25587743817859e-06,4.62251036452795e-06,0.00224150813158937,0.0393723140344367,0.245486379182639};
const double q_qpsk[8]= {1.94491167814437e-09,8.40494123817774e-08,4.75527131198034e-07,-2.48946285301621e-05,-0.000347614016158364,0.00209252225437100,0.0742986115462510,0.488297879889425};
const double q_qam64[8]= {2.25934026232206e-11,-1.45992206328306e-10,-3.70861183071900e-08,-1.22206071019319e-06,6.49115500399637e-06,0.00129828997837433,0.0259669554914859,0.166602901214898};

//for MI to SNR conversion 7 th order Polynomial coeff
const double p_qpsk[8]= {5982.42405670359,-21568.1135917693,31293.9987036905,-23394.6795043871,9608.34750585489,-2158.15802349899,267.731968719036,-20.6145324295965};
const double p_qam16[8]= {7862.12690694170,-28510.3207048338,41542.2150287122,-31088.3036957379,12690.1982361016,-2785.66604739984,326.595462489375,-18.9911849872089};
const double p_qam64[8]= {8832.57933013696,-32119.1802555952,46914.2578990397,-35163.8150557183,14343.7419388853,-3126.61025510092,360.954930562237,-18.0358548533343};

// ideal CE MIESM

const double beta1_dlsch_MI[6][MCS_COUNT] = { {1.1188, 0.3720, 0.3755, 0.9453, 0.5799, 0.5256, 0.5485, 0.5340, 0.5165, 0.5300, 0.6594, 0.5962, 0.4884, 0.4927, 0.3687, 0.4614, 0.4081, 0.2639,0.2935,0.2520,0.3709,0.2906,0.2612,0.2390}, {0.7138, 0.5533, 0.5533, 0.4828, 0.4998, 0.4843, 0.4942, 0.5323, 0.5142, 0.4756, 0.5792, 0.4167, 0.4445, 0.3942, 0.3789, 0.2756, 0.4456, 0.1650, 0.2254, 0.2353, 0.2097,0.2517,0.3242,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1},  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1},{1.808065416202085, 1.754544945430673,1.902272019362616, 1.790054645392961, 1.563204092967629, 1.585258289348813, 1.579349443720310, 1.570650121437345, 1.545055626608596, 1.362229442426877, 1.85, 1.79, 1.65, 1.54, 1.46, 1.39, 1.33, 1,1,1,1,1,1,1},{0.7146, 0.4789, 0.5392, 0.5556, 0.4975, 0.4847, 0.4691, 0.5261, 0.5278, 0.4962, 0.4468, 0.4113, 0.4622, 0.4609, 0.3946, 0.3991, 0.3532, 0.2439, 0.1898, 0.2929, 0.2712, 0.3367, 0.3591, 0.2571}};
const double beta2_dlsch_MI[6][MCS_COUNT] = { {1.1293, 0.3707, 0.3722, 0.9310, 0.5808, 0.5265, 0.5404, 0.5279, 0.5210, 0.5226, 0.6438, 0.5827, 0.4804, 0.4830, 0.3638, 0.4506, 0.4107, 0.2547, 0.2797, 0.2413, 0.3351, 0.2750, 0.2568, 0.2273}, {0.7028, 0.5503, 0.5503, 0.4815, 0.5006, 0.4764, 0.4810, 0.5124, 0.4964, 0.4485, 0.5497, 0.3971, 0.4239, 0.3701, 0.3494, 0.2630, 0.4053, 0.1505, 0.2001,0.2024,0.1788,0.2124,0.2668,1}, {1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1},  {1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1},{1.079518113138858, 1.105622953570353, 1.031337449900606, 1.073342032668810, 1.242636589110353, 1.255054927783647, 1.291463834317768, 1.317048698347491, 1.354485054187984, 0.338534029291017, 1.85, 1.79, 1.65, 1.54, 1.46, 1.39, 1.33,1, 1,1,1,1,1,1},{0.6980, 0.4694, 0.5379, 0.5483, 0.4982, 0.4737, 0.4611, 0.5051, 0.5020, 0.4672, 0.4357, 0.3957, 0.4389, 0.4344, 0.3645, 0.3661, 0.3301, 0.2179, 0.1730, 0.2536, 0.2389,0.2884,0.2936,0.2226}};

//real CE MIESM
/*
double beta1_dlsch_MI[6][MCS_COUNT] = { {1.32955, 0.59522, 0.54024, 0.98698, 0.81305, 0.76976, 0.69258, 0.69713, 0.70546, 0.69111, 0.81904, 0.72664, 0.79491, 0.72562, 0.53980, 0.33134, 0.50550, 0.40602,0.40281,0.47012,0.50510,0.23540,0.32045,1}, {0.59632, 1.08475, 1.02431, 1.07020, 0.90170, 0.97719, 0.95464, 0.92764, 0.86721, 0.85986, 0.64558, 0.80631, 0.82673, 0.82888, 0.87122, 0.77245, 0.29771, 0.43477, 0.55321, 0.61027, 0.56111, 0.57292, 0.39737,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1},  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1},{1.808065416202085, 1.754544945430673,1.902272019362616, 1.790054645392961, 1.563204092967629, 1.585258289348813, 1.579349443720310, 1.570650121437345, 1.545055626608596, 1.362229442426877, 1.85, 1.79, 1.65, 1.54, 1.46, 1.39, 1.33, 1,1,1,1,1,1,1},{0.77532, 1.07544, 1.10571, 1.04099, 0.91638, 0.88644, 0.96405, 0.86709, 0.94066, 0.84430, 1.24478, 1.09665, 1.42604, 0.79541, 0.71847, 0.71604, 0.74561, 0.36431, 0.41536, 0.52175, 0.47096, 0.49977, 0.59728,1}};
double beta2_dlsch_MI[6][MCS_COUNT] = { {1.36875, 0.59304, 0.53870, 0.98239, 0.81637, 0.76847, 0.69842, 0.69885, 0.69967, 0.69826, 0.82660, 0.70559, 0.78404, 0.70670, 0.55393, 0.36893, 0.52225, 0.39752, 0.40494, 0.46239, 0.49247,0.26900,0.34504,1}, {0.43775, 0.78208, 0.72875, 0.77458, 0.64485, 0.69174, 0.66097, 0.63289, 0.59652, 0.61175, 0.44551, 0.56047, 0.57314, 0.57553, 0.58849, 0.52159, 0.21241, 0.30139, 0.37373, 0.32029, 0.37067, 0.36706, 0.27118,1}, {1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1},  {1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1},{1.079518113138858, 1.105622953570353, 1.031337449900606, 1.073342032668810, 1.242636589110353, 1.255054927783647, 1.291463834317768, 1.317048698347491, 1.354485054187984, 0.338534029291017, 1.85, 1.79, 1.65, 1.54, 1.46, 1.39, 1.33,1, 1,1,1,1,1,1},{0.54448, 0.73731, 0.79165, 0.74407, 0.68042, 0.64906, 0.71349, 0.62109, 0.65815, 0.60940, 0.90549, 0.78708, 1.03176, 0.58431, 0.53379, 0.51224, 0.52767, 0.26848, 0.29642, 0.36879, 0.34148, 0.35279,0.40633,1}};
*/
//ideal channel estimation values
//double beta1_dlsch[6][MCS_COUNT] = { {2.3814, 0.4956, 0.5273, 1.1708, 0.8014, 0.7889, 0.8111, 0.8139, 0.8124, 0.8479, 1.9280, 1.9664, 2.3857, 2.5147, 2.4511, 3.0158, 2.8643, 5.3013, 5.8594, 6.5372, 7.8073, 7.8030, 7.5295, 7.1320}, {0.5146, 0.5549, 0.7405, 0.6913, 0.7349, 0.7000, 0.7539, 0.7955, 0.8074, 0.7760, 1.8150, 1.6561, 1.9280, 2.3563, 2.6699, 2.3086, 3.1601, 4.5316, 5.2870, 6.0983, 6.5635, 7.7024, 9.9592, 6.6173}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {1.79358, 1.17908, 2.02600, 1.72040, 1.58618, 1.59039, 1.68111, 1.67062, 1.64911, 1.33274, 4.87800, 3.58797, 3.72338, 5.35700, 2.81752, 1.93472, 2.23259, 1,1,1,1,1,1,1}, {0.4445, 0.5918, 0.7118, 0.7115, 0.7284, 0.7202, 0.7117, 0.8111, 0.8239, 0.7907, 1.8456, 1.8144, 2.3830, 2.6634, 2.6129, 2.8127, 2.7372, 4.9424, 4.8763, 6.8413, 7.1493, 9.4180, 10.1230, 8.9613}};
//double beta2_dlsch[6][MCS_COUNT] = { {2.3639, 0.4952, 0.5207, 1.1572, 0.8026, 0.7864, 0.7996, 0.8034, 0.8200, 0.8367, 1.8701, 1.9212, 2.2947, 2.4472, 2.4091, 2.9479, 2.8973, 5.0591, 5.5134, 6.1483, 7.2166, 7.5177, 7.5704, 7.2248}, {0.5113, 0.5600, 0.7359, 0.6860, 0.7344, 0.6902, 0.7315, 0.7660, 0.7817, 0.7315, 1.7268, 1.5912, 1.8519, 2.2115, 2.4580, 2.1879, 2.9015, 4.1543, 4.6986, 5.3193, 5.6319, 6.5640, 8.2421, 5.6393}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {0.79479, 0.52872, 0.90005, 0.77170, 0.73220, 0.72060, 0.75433, 0.75451, 0.75989, 0.67655, 1.68525, 1.31100, 1.46573, 1.99843, 1.57293, 1.62852, 2.10636, 1,1,1,1,1,1,1}, {0.4398, 0.5823, 0.7094, 0.7043, 0.7282, 0.7041, 0.6979, 0.7762, 0.7871, 0.7469, 1.7752, 1.7443, 2.2266, 2.4767, 2.4146, 2.6040, 2.5708, 4.4488, 4.4944, 5.9630, 6.3740, 8.1097, 8.4210, 7.8027}};
const double beta1_dlsch[6][MCS_COUNT] = { {1.199175, 1.085656, 0.983872, 0.843789, 0.816093, 0.853078, 0.899236, 0.919665, 0.888673, 0.924181, 0.814176, 0.794108, 0.770653, 0.826266, 0.982043, 0.979621, 0.985176, 0.901741, 0.870311, 0.911303, 0.898923, 1.003359, 0.988535, 1.030639, 1.151038, 1.116939, 1.214118, 1.219148}, {0.5146, 0.5549, 0.7405, 0.6913, 0.7349, 0.7000, 0.7539, 0.7955, 0.8074, 0.7760, 1.8150, 1.6561, 1.9280, 2.3563, 2.6699, 2.3086, 3.1601, 4.5316, 5.2870, 6.0983, 6.5635, 7.7024, 9.9592, 6.6173}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {1.79358, 1.17908, 2.02600, 1.72040, 1.58618, 1.59039, 1.68111, 1.67062, 1.64911, 1.33274, 4.87800, 3.58797, 3.72338, 5.35700, 2.81752, 1.93472, 2.23259, 1,1,1,1,1,1,1}, {0.4445, 0.5918, 0.7118, 0.7115, 0.7284, 0.7202, 0.7117, 0.8111, 0.8239, 0.7907, 1.8456, 1.8144, 2.3830, 2.6634, 2.6129, 2.8127, 2.7372, 4.9424, 4.8763, 6.8413, 7.1493, 9.4180, 10.1230, 8.9613}};
const double beta2_dlsch[6][MCS_COUNT] = { {0.534622, 0.596561, 0.500838, 0.471721, 0.548218, 0.547974, 0.924245, 0.836484, 0.776917, 0.879691, 0.875722, 0.666933, 0.666393, 0.755377, 1.074985, 1.080290, 1.010914, 0.790892, 0.793435, 0.860249, 0.901508, 0.967060, 0.951372, 1.011493, 1.106151, 1.117076, 1.209397, 1.227790}, {0.5113, 0.5600, 0.7359, 0.6860, 0.7344, 0.6902, 0.7315, 0.7660, 0.7817, 0.7315, 1.7268, 1.5912, 1.8519, 2.2115, 2.4580, 2.1879, 2.9015, 4.1543, 4.6986, 5.3193, 5.6319, 6.5640, 8.2421, 5.6393}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {0.79479, 0.52872, 0.90005, 0.77170, 0.73220, 0.72060, 0.75433, 0.75451, 0.75989, 0.67655, 1.68525, 1.31100, 1.46573, 1.99843, 1.57293, 1.62852, 2.10636, 1,1,1,1,1,1,1}, {0.4398, 0.5823, 0.7094, 0.7043, 0.7282, 0.7041, 0.6979, 0.7762, 0.7871, 0.7469, 1.7752, 1.7443, 2.2266, 2.4767, 2.4146, 2.6040, 2.5708, 4.4488, 4.4944, 5.9630, 6.3740, 8.1097, 8.4210, 7.8027}};

//real channel estimation valus
/*
double beta1_dlsch[6][MCS_COUNT] = { {2.50200, 0.84047, 0.78195, 1.37929, 1.16871, 1.11906, 1.06303, 1.07447, 1.11403, 1.09223, 2.82502, 2.87556, 3.51254, 3.62920, 3.53638, 2.35980, 3.74126, 8.66532, 7.31772, 9.86882, 10.64939, 6.75208, 9.50664, 13.63057}, {0.92257, 1, 1.80445, 1.43175, 1.42093, 1.37381, 1.45392, 1.47255, 1.47451, 1.41235, 3.9079, 3.38557, 4.13059, 4.93355, 4.97277, 6.04951, 5.88896, 8.68076, 10.23746, 12.37069, 5.50538, 17.29612, 17.95050, 13.27095}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {1.79255, 1.88213, 4.44226, 2.25150, 1.93710, 2.18504, 2.57389, 1.94322, 1.78515, 2.09265, 2.37459, 1.74442, 1.74346, 1.19705, 1.56149, 1.59604, 1.6, 1,1,1,1,1,1,1}, {0.93374, 1.40389, 1.36670, 1.38679, 1.35707, 1.26353, 1.32360, 1.40164, 1.51843, 1.34863, 3.45839, 3.13726, 3.94768, 4.21966, 4.60750, 4.97894, 5.40755, 8.12814, 10.59221, 12.96427, 13.37323, 14.27206, 16.61779, 17.19656}};
double beta2_dlsch[6][MCS_COUNT] = { {2.52163, 0.83231, 0.77472, 1.36536, 1.16829, 1.11186, 1.06287, 1.07292, 1.09946, 1.10650, 2.79174, 2.75655, 3.36651, 3.49011, 3.60903, 2.73517, 3.84009, 8.20312, 7.41739, 9.64081, 10.40911, 8.11765, 10.41923, 9.34300}, {0.67252, 0.8600, 1.28633, 1.01624, 1.03066, 0.97590, 1.02560, 1.01840, 1.00547, 0.97093, 2.72573, 2.33283, 2.86181, 3.40452, 3.47957, 4.08916, 3.97628, 6.14541, 7.11017, 8.42369, 4.04812, 11.42082, 11.57171, 9.28462}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1}, {0.85784, 0.90361, 2.09766, 1.08385, 0.96300, 1.04432, 1.22763, 0.99249, 0.95544, 1.12333, 1.37924, 1.12913, 1.30644, 1.19253, 1.75488, 2.13813, 2.10636, 1,1,1,1,1,1,1}, {0.66288, 0.96402, 0.98545, 0.99386, 0.99981, 0.92678, 0.98978, 0.99600, 1.05538, 0.97777, 2.52504, 2.29338, 2.89631, 3.10812, 3.41916, 3.58671, 3.84166, 6.05254, 7.45821, 9.15812, 9.66330, 10.17852, 11.50519, 11.16299}};

*/

const char NB_functions[7][20]={"eNodeB_3GPP","eNodeB_3GPP_BBU","NGFI_RAU_IF4p5","NGFI_RRU_IF5","NGFI_RRU_IF4p5","gNodeB_3GPP",};
const char NB_timing[2][20]={"synch_to_ext_device","synch_to_other"};
const char ru_if_types[MAX_RU_IF_TYPES][20]={"local RF","IF5 RRU","IF5 Mobipass","IF4p5 RRU","IF1pp RRU"};

/// lookup table for unscrambling in RX
int16_t unscrambling_lut[65536*16] __attribute__((aligned(32)));
/// lookup table for scrambling in TX
uint8_t scrambling_lut[65536*16] __attribute__((aligned(32)));

uint8_t max_turbo_iterations=4;
#endif /* __PHY_VARS_UE__H__ */
