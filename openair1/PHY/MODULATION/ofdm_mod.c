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

/*
* @defgroup _PHY_MODULATION_
* @ingroup _physical_layer_ref_implementation_
* @{
\section _phy_modulation_ OFDM Modulation Blocks
This section deals with basic functions for OFDM Modulation.



*/
#include <pthread.h>
#include "PHY/defs_eNB.h"
#include "PHY/defs_gNB.h"
#include "PHY/impl_defs_top.h"
#include "common/utils/LOG/log.h"
#include "common/utils/LOG/px_log.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "modulation_common.h"
#include "PHY/LTE_TRANSPORT/transport_common_proto.h"
#include <stdlib.h>
// #include <errno.h> 
#define FPGA_ENABLE
//#define DEBUG_OFDM_MOD

//------  fpga settings ------
//-----fpga include---------
static unsigned char *h2c_align_mem_tmp;
static unsigned char *c2h_align_mem_tmp;

#define MAP_SIZE (1024*1024UL)
#define MAP_MASK (MAP_SIZE - 1)
#define POINT_NUM 512
#define PC_CONTROL_ADDR 14336
#define FPGA_AXI_START_ADDR (0)
#define FPGA_AXI_CLEAR (32768)

#define FPGA_CONTROL_ADDR 57344//64= (8193-1)*4 || 4是指字节，17-1指控制位在第17个数 
#define CONTROL_DATA 6132

static pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;
static int fpga_cnt  = 0;
static int total_cnt = 0;
static int c2h_dma_fd ;
static int h2c_dma_fd ;
static int init_flag = 0;
const unsigned int len = (POINT_NUM * 2 + 1) * 4;    //2^15，len的含义是需要多少个实部/虚部
const unsigned int lendata = (POINT_NUM * 2 + 1);

void put_data_to_fpga_ddr(unsigned int fpga_ddr_addr,int *buffer,unsigned int len)
{
    lseek(h2c_dma_fd,fpga_ddr_addr,SEEK_SET);
    write(h2c_dma_fd,buffer,len);
}

void get_data_from_fpga_ddr(unsigned int fpga_ddr_addr,int  *buffer,unsigned int len)
{
    lseek(c2h_dma_fd,fpga_ddr_addr,SEEK_SET);
    read(c2h_dma_fd,buffer,len);//32个字节,256位
}

int pcie_init()
{
    c2h_dma_fd = open("/dev/xdma0_c2h_0",O_RDWR | O_NONBLOCK);
    if(c2h_dma_fd < 0){
      Log("open fail errno = %d  \n", errno);
      return c2h_dma_fd;
    }
    h2c_dma_fd = open("/dev/xdma0_h2c_0",O_RDWR );
    if(h2c_dma_fd < 0)
        return -2;
    posix_memalign((void *)&h2c_align_mem_tmp,4096,0x800000);
    posix_memalign((void *)&c2h_align_mem_tmp,4096,0x800000);

    if(NULL == h2c_align_mem_tmp || NULL == c2h_align_mem_tmp)
        return -6;
    free(h2c_align_mem_tmp);
    free(c2h_align_mem_tmp);
    return 1;
}

void pcie_deinit()
{
    close(c2h_dma_fd);
    close(h2c_dma_fd);
    //close(control_fd);
}

unsigned char write_fftdata(void *buf ,unsigned int type,char*fname){
    
    FILE *fpread=fopen(fname,"w");
    if(fpread == NULL)  {
      Log("error opening device\n");
      }

    if(type==1){
        //打印int_8
        int8_t *buf1 = (int8_t*)(buf);
        for(unsigned int i=0;i<lendata;i++) fprintf(fpread, "%hhd\n",(buf1[i]));
    }
    else if(type==2){
        //打印int_16
        int16_t *buf1 = (int16_t*)(buf);
        for(unsigned int i=0;i<lendata;i++) fprintf(fpread, "%hd\n",(buf1[i]));
    }
    else if(type==3){
        //打印int_32
        int32_t *buf1 = (int32_t*)(buf);
        for(unsigned int i=0;i<lendata;i++) fprintf(fpread, "%d\n",(buf1[i]));
    }
    else {
        printf("type wrong . check write_dedata parameters.");
    }
    fclose (fpread);
    return 0;
}
unsigned char read_fftdata_int(int *buf1,char*fname){
    FILE *fpread=fopen(fname,"r");
    if(fpread == NULL)  return 1;
    for(int i=0;i<lendata;i++) {
        fscanf(fpread, "%d", &buf1[i]);
    }
    fclose (fpread);
    return 0;
}
int calculate_flag_fft(int block_num){
    //131072*block_total + 65535 = fft_flag_pc  
    //131072*block_total + 131071 = ifft_flag_pc  
    if(block_num==0) return 0;
    else return 65535 + 131072*(block_num-1);
}
void ifft512_nr_fpga(int16_t*in_buf,int16_t*out_buf,unsigned char scale_flag,int ofdm_symbols){
    Log("----------one fft begin------------\n");
    Log("ofdm symbols = %d\n",ofdm_symbols);
    struct timeval st;
    struct timeval ed;
    double time_total;
    gettimeofday(&st,NULL);
    int in_size = 512*2*14;
    int ofdm_size = 512*2*ofdm_symbols;
    int in_size_bytes = (in_size+1)*4;
    // int *in_buf_fpga  = NULL;
    // int *out_buf_fpga = NULL;
    int done_reg;
    int i;
    char *fname_in = "oai_fpga_in.txt";
    char *fname_out = "oai_fpga_out.txt";
    // posix_memalign((void **)&in_buf_fpga,  32 , 1024*1024);
    // posix_memalign((void **)&out_buf_fpga, 32 , 1024*1024);
    int in_buf_fpga [1024*256] __attribute__((aligned(32)));
    int out_buf_fpga[1024*256] __attribute__((aligned(32)));
    memset(in_buf_fpga,0,in_size_bytes);
    memset(out_buf_fpga,0,in_size_bytes);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0);
    Log("time of [mem align]= %f\n",  time_total);
    gettimeofday(&st,NULL);
    //convert int16 to int 
    for(i=0;i<ofdm_size ;i++){
        in_buf_fpga[i] = (int)in_buf[i];
    }
    // for(i=ofdm_size;i<in_size;i++){
    //   in_buf_fpga[i] = 0;
    // }
    // printf("ofdm_symbols = %d ",ofdm_symbols);
    in_buf_fpga[PC_CONTROL_ADDR] = calculate_flag_fft(ofdm_symbols);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0);
    Log("time of [format convert]= %f\n",  time_total);
 
    //put int data into fpga
    gettimeofday(&st,NULL);
    put_data_to_fpga_ddr(FPGA_AXI_START_ADDR,in_buf_fpga,in_size_bytes);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0);
    Log("time of [put data into fpga]= %f\n",  time_total);
    //get int data in fpga
    int cnt = 0;
    gettimeofday(&st,NULL);
    while(1)
    {
		    get_data_from_fpga_ddr(FPGA_CONTROL_ADDR, &done_reg, 4);// rec EN done flag
        if(done_reg == 438 || done_reg ==876 || done_reg ==1314 || done_reg ==1752 || done_reg ==2190 || done_reg ==2628 || done_reg ==3066 || done_reg ==3504 || done_reg ==3942 || done_reg ==4380 || done_reg ==4818 || done_reg ==5256 ||done_reg ==5694||done_reg ==6132)
        {
            break;
        }
        cnt = (cnt+1);
        if(cnt%10000==9999)printf("lock in get done flag\n");
    }
    get_data_from_fpga_ddr(FPGA_AXI_START_ADDR,out_buf_fpga,in_size_bytes);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0);
    Log("time of [get data from fpga]= %f\n",  time_total);
    //convert int to int16 and shuffle Log("----------one fft begin------------\n");
    Log("ofdm symbols = %d\n",ofdm_symbols);
    gettimeofday(&st,NULL);
    for(int l=0;l<14;l++){
        for(i=0;i<8;i++){
        for(int k=0;k<8;k++){
            out_buf[l*1024+2*(i+8*k)]       =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i)]    /22) ;
            out_buf[l*1024+2*(i+8*k)+1]     =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i)+1]  /22) ;
            out_buf[l*1024+2*(i+8*k+64 )]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+1)]  /22) ;
            out_buf[l*1024+2*(i+8*k+64 )+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+1)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+128)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+2)]  /22) ;
            out_buf[l*1024+2*(i+8*k+128)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+2)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+192)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+3)]  /22) ;
            out_buf[l*1024+2*(i+8*k+192)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+3)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+256)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+4)]  /22) ;
            out_buf[l*1024+2*(i+8*k+256)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+4)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+320)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+5)]  /22) ;
            out_buf[l*1024+2*(i+8*k+320)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+5)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+384)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+6)]  /22) ;
            out_buf[l*1024+2*(i+8*k+384)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+6)+1]/22) ;
            out_buf[l*1024+2*(i+8*k+448)]   =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+7)]  /22) ;
            out_buf[l*1024+2*(i+8*k+448)+1] =  (int16_t)(out_buf_fpga[l*1024+2*(8*k+64*i+7)+1]/22) ;
        }
    }
    }
    // if(in_buf_fpga[0]!=0){
    //   write_fftdata(in_buf_fpga,2,fname_in);
    //   write_fftdata(out_buf,2,fname_out);
    // }
    //free temporary space 
    // free(in_buf_fpga);
    // free(out_buf_fpga);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0);
    Log("time of [shuffle]= %f\n",  time_total);
    Log("----------one fft end------------\n");

}

int withlock_ifft_nr_fpga(int16_t*in_buf,int16_t*out_buf,unsigned char scale_flag){
    int result = pthread_mutex_trylock(&mutex1);
    if(result==0){
      ifft512_nr_fpga(in_buf,out_buf,1,1);
      fpga_cnt  += 1;
      pthread_mutex_unlock(&mutex1);
    }
    return result;

}

void test_oai_fpga(int16_t *oai_in_buf, int16_t*oai_out_buf, int16_t *fpga_out_buf){
    char *fname_ofdm_in = "oai_ofdm_in.txt";
    char *fname_ofdm_out = "oai_ofdm_out.txt";
    char *fname_ofdm_fpga_in = "fpga_ofdm_in.txt";
    char *fname_ofdm_fpga_out = "fpga_ofdm_out.txt";
    double error_sum = 0;
    double non_zero_num = 0;
    double error[1025];
    // printf("test begin  ");
    for (int i=0;i<1024;i++){
      if(oai_out_buf[i]!=0){
        error[i] = ((float)(abs(fpga_out_buf[i]))-(float)(abs(oai_out_buf[i])))/(float)(oai_out_buf[i]);
        error_sum += error[i];
        non_zero_num += 1;
      } 
      }
      if(non_zero_num!=0){
        error_sum = error_sum/non_zero_num;
      }      if(error_sum>0.1){
            printf("error_sum = %f",error_sum);
            write_fftdata(oai_in_buf,2,fname_ofdm_fpga_in);
            write_fftdata(fpga_out_buf,2,fname_ofdm_fpga_out);
            write_fftdata(oai_out_buf,2,fname_ofdm_out);
          for(int i=0;i<1024;i++){
            printf("input_fpga[%d]=%hd     ",i,oai_in_buf[i]);
            printf("fpga_output[%d]=%hd    ",i,fpga_out_buf[i]);
            printf("temp[%d]=%hd     ",i,oai_out_buf[i]);
            printf("error[%d]=%f\n",i,error[i]);
          }
          sleep(10);
      }
      if(non_zero_num!=0)printf("test end.non_zero_num = %f,error sum = %f\n",non_zero_num,error_sum);
}
void fft512_nr_fpga(int16_t*in_buf,int16_t*out_buf,unsigned char scale_flag){
    //convert int16 to int 
    int in_size = 512*2+1;
    int in_size_bytes = in_size*4;
    int *in_buf_fpga = (int *)malloc(sizeof(int)*in_size);
    int *out_buf_fpga = (int *)malloc(sizeof(int)*in_size);
    int i;
    for(i=0;i<in_size-1 ;i++){
        in_buf_fpga[i] = (int)in_buf[i];
    }
    memset(out_buf_fpga,0,in_size_bytes);

    in_buf_fpga[in_size-1] = 131071; // ifft start flag
    //put int data into fpga
    put_data_to_fpga_ddr(FPGA_AXI_START_ADDR,in_buf_fpga,in_size_bytes);
    //get int data in fpga
    get_data_from_fpga_ddr(FPGA_AXI_START_ADDR,out_buf_fpga,in_size_bytes);

    //convert int to int16 and shuffle
    for(i=0;i<8;i++){
        for(int k=0;k<8;k++){
            out_buf[2*(i+8*k)]       =  (int16_t)(out_buf_fpga[2*(8*k+64*i)]    /22) ;
            out_buf[2*(i+8*k)+1]     =  (int16_t)(out_buf_fpga[2*(8*k+64*i)+1]  /22) ;
            out_buf[2*(i+8*k+64 )]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+1)]  /22) ;
            out_buf[2*(i+8*k+64 )+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+1)+1]/22) ;
            out_buf[2*(i+8*k+128)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+2)]  /22) ;
            out_buf[2*(i+8*k+128)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+2)+1]/22) ;
            out_buf[2*(i+8*k+192)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+3)]  /22) ;
            out_buf[2*(i+8*k+192)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+3)+1]/22) ;
            out_buf[2*(i+8*k+256)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+4)]  /22) ;
            out_buf[2*(i+8*k+256)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+4)+1]/22) ;
            out_buf[2*(i+8*k+320)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+5)]  /22) ;
            out_buf[2*(i+8*k+320)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+5)+1]/22) ;
            out_buf[2*(i+8*k+384)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+6)]  /22) ;
            out_buf[2*(i+8*k+384)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+6)+1]/22) ;
            out_buf[2*(i+8*k+448)]   =  (int16_t)(out_buf_fpga[2*(8*k+64*i+7)]  /22) ;
            out_buf[2*(i+8*k+448)+1] =  (int16_t)(out_buf_fpga[2*(8*k+64*i+7)+1]/22) ;
        }
    }
    //free temporary space 
    free(in_buf_fpga);
    free(out_buf_fpga);

}

void test_buf_int(){
    char * fname_fpga_out = "/home/g/fpga/pcie/fft_hu/fft/test_buf_int_out.txt";
    char * fname_fpga_in  = "/home/g/fpga/pcie/fft_hu/fft/test_buf_int_in.txt";
    char * fname_in_data =  "/home/g/fpga/pcie/fft_hu/fft/fft_data/ifft_oai_in.txt";
    int  *buf1 = NULL;
    int  *buf2 = NULL;
    int  *test_buf = NULL;
    int  *error    = NULL;   
    int error_sum = 0;
    size_t align = 32;//32*8=256 ram data width = 256 ,so should align to 32
    posix_memalign((void **)&buf1,     align , 1024*1024);
    posix_memalign((void **)&buf2,     align , 1024*1024);
    posix_memalign((void **)&test_buf, align , 1024*1024);
    posix_memalign((void **)&error,    align , 1024*1024);
    //time measure
    struct timeval st;
    struct timeval ed;
    double time_total;
    int reg_pcie;

    while(!((reg_pcie=pcie_init())==1))printf("test_int stuck on pcie init ,return value = %d \n",reg_pcie); 
    while(read_fftdata_int(buf1,fname_in_data));          //read one group of data
    while(read_fftdata_int(buf2,fname_fpga_out));          //read one group of data
    // write_fftdata(buf1,3,fname_fpga_in);
    gettimeofday(&st, NULL);
    put_data_to_fpga_ddr(FPGA_AXI_START_ADDR,buf1,len);
    get_data_from_fpga_ddr(FPGA_AXI_START_ADDR,test_buf,len);
    gettimeofday(&ed, NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0); 
    printf("[time_total_all]= %f\n",  time_total);
    for(int i=0;i<1024;i++){
        error[i] = test_buf[i] - buf2[i];
        error = error + error_sum;
    }
    printf("error_sum=%d\n",error_sum);
    if(error_sum!=0)assert(0);
    pcie_deinit();
    // free(buf1);
    // free(buf2);
    // free(test_buf);
    // free(error);
}




//------  fpga settings end------




void normal_prefix_mod(int32_t *txdataF,int32_t *txdata,uint8_t nsymb,LTE_DL_FRAME_PARMS *frame_parms)
{


  
  PHY_ofdm_mod(txdataF,        // input
               txdata,         // output
               frame_parms->ofdm_symbol_size,                

               1,                 // number of symbols
               frame_parms->nb_prefix_samples0,               // number of prefix samples
               CYCLIC_PREFIX);
  PHY_ofdm_mod(txdataF+frame_parms->ofdm_symbol_size,        // input
               txdata+OFDM_SYMBOL_SIZE_COMPLEX_SAMPLES0,         // output
               frame_parms->ofdm_symbol_size,                
               nsymb-1,
               frame_parms->nb_prefix_samples,               // number of prefix samples
               CYCLIC_PREFIX);
  

  
}

void nr_normal_prefix_mod(int32_t *txdataF,int32_t *txdata,uint8_t nsymb,NR_DL_FRAME_PARMS *frame_parms, uint32_t slot)
{
  // This function works only slot wise. For more generic symbol generation refer nr_feptx0()
  if (frame_parms->numerology_index != 0) { // case where numerology != 0
    if (!(slot%(frame_parms->slots_per_subframe/2))) {
      PHY_ofdm_mod(txdataF,
             txdata,
             frame_parms->ofdm_symbol_size,
             1,
             frame_parms->nb_prefix_samples0,
             CYCLIC_PREFIX);
      PHY_ofdm_mod(txdataF+frame_parms->ofdm_symbol_size,
             txdata + frame_parms->ofdm_symbol_size + frame_parms->nb_prefix_samples0,
             frame_parms->ofdm_symbol_size,
             nsymb - 1,
             frame_parms->nb_prefix_samples,
             CYCLIC_PREFIX);
    }
    else {
      PHY_ofdm_mod(txdataF,
             txdata,
             frame_parms->ofdm_symbol_size,
             nsymb,
             frame_parms->nb_prefix_samples,
             CYCLIC_PREFIX);
    }
  }
  else { // numerology = 0, longer CP for every 7th symbol
      PHY_ofdm_mod(txdataF,
             txdata,
             frame_parms->ofdm_symbol_size,
             1,
             frame_parms->nb_prefix_samples0,
             CYCLIC_PREFIX);
      PHY_ofdm_mod(txdataF+frame_parms->ofdm_symbol_size,
             txdata + frame_parms->ofdm_symbol_size + frame_parms->nb_prefix_samples0,
             frame_parms->ofdm_symbol_size,
             6,
             frame_parms->nb_prefix_samples,
             CYCLIC_PREFIX);
      PHY_ofdm_mod(txdataF + 7*frame_parms->ofdm_symbol_size,
             txdata + 6*(frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples) +
                    frame_parms->ofdm_symbol_size + frame_parms->nb_prefix_samples0,
             frame_parms->ofdm_symbol_size,
             1,
             frame_parms->nb_prefix_samples0,
             CYCLIC_PREFIX);
      PHY_ofdm_mod(txdataF + 8*frame_parms->ofdm_symbol_size,
             txdata + 6*(frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples) +
                    2*(frame_parms->ofdm_symbol_size + frame_parms->nb_prefix_samples0),
             frame_parms->ofdm_symbol_size,
             6,
             frame_parms->nb_prefix_samples,
             CYCLIC_PREFIX);
  }

}

void PHY_ofdm_mod(int *input,                       /// pointer to complex input
                  int *output,                      /// pointer to complex output
                  int fftsize,            /// FFT_SIZE
                  unsigned char nb_symbols,         /// number of OFDM symbols
                  unsigned short nb_prefix_samples,  /// cyclic prefix length
                  Extension_t etype                /// type of extension
                 )
{

  if(nb_symbols == 0) return;
  int16_t temp[2*2*6144*4] __attribute__((aligned(32)));
  int i,j;

  volatile int *output_ptr=(int*)0;

  int *temp_ptr=(int*)0;
  idft_size_idx_t idftsize;

  switch (fftsize) {
  case 128:
    idftsize = IDFT_128;
    break;

  case 256:
    idftsize = IDFT_256;
    break;

  case 512:
    idftsize = IDFT_512;
    break;

  case 1024:
    idftsize = IDFT_1024;
    break;

  case 1536:
    idftsize = IDFT_1536;
    break;

  case 2048:
    idftsize = IDFT_2048;
    break;

  case 3072:
    idftsize = IDFT_3072;
    break;

  case 4096:
    idftsize = IDFT_4096;
    break;

  case 6144:
    idftsize= IDFT_6144;
    break;

 case 12288:
    idftsize= IDFT_12288;
    break;

 case 24576:
    idftsize= IDFT_24576;
    break;

  default:
    idftsize = IDFT_512;
    break;
  }

#ifdef DEBUG_OFDM_MOD
  printf("[PHY] OFDM mod (size %d,prefix %d) Symbols %d, input %p, output %p\n",
      fftsize,nb_prefix_samples,nb_symbols,input,output);
#endif

#ifdef TIME_STATISTIC_1
    Log("[PHY] OFDM mod (size %d,prefix %d) Symbols %d, input %p, output %p\n",
      fftsize,nb_prefix_samples,nb_symbols,input,output);
    gettimeofday(&st, NULL);
#endif

#ifdef FPGA_ENABLE
    Log("FPGA enabled\n");

    if(init_flag!=1){
      printf("first time\n");
      while (pcie_init()!=1)printf("lock!\n");
      init_flag = 1;
    }
    
    pthread_mutex_lock(&mutex1);
    Log("idftsize = %d\n",idftsize);
    struct timeval st;
    struct timeval ed;
    double time_total;
    gettimeofday(&st, NULL);
    ifft512_nr_fpga((int16_t *)input,
        (int16_t *)temp,
         1,
         nb_symbols);
    gettimeofday(&ed,NULL);
    time_total = ((ed.tv_sec - st.tv_sec) + (ed.tv_usec - st.tv_usec) / 1000000.0); 
    Log("[time_total_ifft]= %f\n",  time_total);
    pthread_mutex_unlock(&mutex1);
    //----compare data -----------

    for(i=0;i<nb_symbols;i++){
      switch (etype) {
    case CYCLIC_PREFIX:
      output_ptr = &output[(i*fftsize) + ((1+i)*nb_prefix_samples)];
      temp_ptr = (int *)(temp+i*fftsize*2);


      //      msg("Doing cyclic prefix method\n");

#ifndef __AVX2__
      if (fftsize==128) 
#endif
      {
        memcpy((void*)output_ptr,(void*)temp_ptr,fftsize<<2);
      }
      // printf("prefix_samples = %d\n",nb_prefix_samples);
      //prefix samples = 36
      memcpy((void*)&output_ptr[-nb_prefix_samples],(void*)&output_ptr[fftsize-nb_prefix_samples],nb_prefix_samples<<2);
      break;

    case CYCLIC_SUFFIX:

      output_ptr = &output[(i*fftsize)+ (i*nb_prefix_samples)];

      temp_ptr = (int *)(temp+i*fftsize*2);

      memcpy((void*)output_ptr,(void*)temp_ptr,fftsize<<2);

      memcpy((void*)&output_ptr[fftsize],(void*)&output_ptr[0],nb_prefix_samples<<2);
      //      msg("Doing cyclic suffix method\n");

      // for (j=0; j<fftsize ; j++) {
      //   output_ptr[j] = temp_ptr[2*j];
      // }


      // for (j=0; j<nb_prefix_samples; j++)
      //   output_ptr[fftsize+j] = output_ptr[j];

      break;

    case ZEROS:

      break;

    case NONE:

      //      msg("NO EXTENSION!\n");
      output_ptr = &output[fftsize];

      temp_ptr = (int *)(temp+i*fftsize*2);

      for (j=0; j<fftsize ; j++) {
        output_ptr[j] = temp_ptr[2*j];


      }

      break;

    default:
      break;

    }
    }
    // pcie_deinit();
#endif

#ifndef FPGA_ENABLE
  // if(init_flag!=1){
  //     printf("first time\n");
  //     while (pcie_init()!=1)printf("lock! in init\n");
  //     init_flag = 1;
  //   }
  for (i=0; i<nb_symbols; i++) {

#ifdef TIME_STATISTIC_0
    gettimeofday(&st, NULL);
#endif

#ifdef DEBUG_OFDM_MOD
    printf("[PHY] symbol %d/%d offset %d (%p,%p -> %p)\n",i,nb_symbols,i*fftsize+(i*nb_prefix_samples),input,&input[i*fftsize],&output[(i*fftsize) + ((i)*nb_prefix_samples)]);
#endif

#ifndef __AVX2__
    // handle 128-bit alignment for 128-bit SIMD (SSE4,NEON,AltiVEC)
    
    idft(idftsize,(int16_t *)&input[i*fftsize],
         (fftsize==128) ? (int16_t *)temp : (int16_t *)&output[(i*fftsize) + ((1+i)*nb_prefix_samples)],
         1);
    // ifft512_nr_fpga((int16_t *)input,
    //     (int16_t *)fpga_output,
    //      1,
    //      nb_symbols);
#else

    // int16_t *fpga_output;
    // posix_memalign((void **)&fpga_output, 32 , 1024*1024);//256*1025
    // memset(fpga_output,0,1024*1024);

    pthread_mutex_lock(&mutex1);

      idft(idftsize,(int16_t *)&input[i*fftsize],
        (int16_t *)temp,
         1);
    // ifft512_nr_fpga((int16_t *)&input[i*fftsize],
    //     (int16_t *)temp,
    //      1,
    //      1);
    //   test_oai_fpga((int16_t *)&input[i*fftsize],(int16_t *)temp,(int16_t *)fpga_output);

      pthread_mutex_unlock(&mutex1);
      // free(fpga_output);

    // on AVX2 need 256-bit alignment

#endif
    // Copy to frame buffer with Cyclic Extension
    // Note:  will have to adjust for synchronization offset!

    switch (etype) {
    case CYCLIC_PREFIX:
      output_ptr = &output[(i*fftsize) + ((1+i)*nb_prefix_samples)];
      temp_ptr = (int *)temp;


      //      msg("Doing cyclic prefix method\n");

#ifndef __AVX2__
      if (fftsize==128) 
#endif
      {
        memcpy((void*)output_ptr,(void*)temp_ptr,fftsize<<2);
      }
      // printf("prefix_samples = %d\n",nb_prefix_samples);
      //prefix samples = 36
      memcpy((void*)&output_ptr[-nb_prefix_samples],(void*)&output_ptr[fftsize-nb_prefix_samples],nb_prefix_samples<<2);
      break;

    case CYCLIC_SUFFIX:

      output_ptr = &output[(i*fftsize)+ (i*nb_prefix_samples)];

      temp_ptr = (int *)temp;

      memcpy((void*)output_ptr,(void*)temp_ptr,fftsize<<2);

      memcpy((void*)&output_ptr[fftsize],(void*)&output_ptr[0],nb_prefix_samples<<2);
      //      msg("Doing cyclic suffix method\n");

      // for (j=0; j<fftsize ; j++) {
      //   output_ptr[j] = temp_ptr[2*j];
      // }


      // for (j=0; j<nb_prefix_samples; j++)
      //   output_ptr[fftsize+j] = output_ptr[j];

      break;

    case ZEROS:

      break;

    case NONE:

      //      msg("NO EXTENSION!\n");
      output_ptr = &output[fftsize];

      temp_ptr = (int *)temp;

      for (j=0; j<fftsize ; j++) {
        output_ptr[j] = temp_ptr[2*j];


      }

      break;

    default:
      break;

    }
    #ifdef TIME_STATISTIC_0
      gettimeofday(&ed, NULL);
      time_total = (ed.tv_sec - st.tv_sec) * 1000000 + (ed.tv_usec - st.tv_usec); //us
      Log("[%s] size %d, cost time = %lf\n", ANSI_FMT("idft_once", ANSI_FG_CYAN), fftsize, time_total);
    #endif
    
  }
  // pcie_deinit();
  
  #ifdef TIME_STATISTIC_1
    gettimeofday(&ed, NULL);
    time_total = (ed.tv_sec - st.tv_sec) * 1000000 + (ed.tv_usec - st.tv_usec); //us
    Log("[%s] cost time = %lf\n", ANSI_FMT("PHY_ofdm_mod_idft", ANSI_FG_CYAN), time_total);
  #endif

  #endif
}


void do_OFDM_mod(int32_t **txdataF, int32_t **txdata, uint32_t frame,uint16_t next_slot, LTE_DL_FRAME_PARMS *frame_parms)
{

  int aa, slot_offset, slot_offset_F;

  slot_offset_F = (next_slot)*(frame_parms->ofdm_symbol_size)*((frame_parms->Ncp==1) ? 6 : 7);
  slot_offset = (next_slot)*(frame_parms->samples_per_tti>>1);

  for (aa=0; aa<frame_parms->nb_antennas_tx; aa++) {
    if (is_pmch_subframe(frame,next_slot>>1,frame_parms)) {
      if ((next_slot%2)==0) {
        LOG_D(PHY,"Frame %d, subframe %d: Doing MBSFN modulation (slot_offset %d)\n",frame,next_slot>>1,slot_offset);
        PHY_ofdm_mod(&txdataF[aa][slot_offset_F],        // input
                     &txdata[aa][slot_offset],         // output
                     frame_parms->ofdm_symbol_size,                
                     12,                 // number of symbols
                     frame_parms->ofdm_symbol_size>>2,               // number of prefix samples
                     CYCLIC_PREFIX);

        if (frame_parms->Ncp == EXTENDED)
          PHY_ofdm_mod(&txdataF[aa][slot_offset_F],        // input
                       &txdata[aa][slot_offset],         // output
                       frame_parms->ofdm_symbol_size,                
                       2,                 // number of symbols
                       frame_parms->nb_prefix_samples,               // number of prefix samples
                       CYCLIC_PREFIX);
        else {
          LOG_D(PHY,"Frame %d, subframe %d: Doing PDCCH modulation\n",frame,next_slot>>1);
          normal_prefix_mod(&txdataF[aa][slot_offset_F],
                            &txdata[aa][slot_offset],
                            2,
                            frame_parms);
        }
      }
    } else {
      if (frame_parms->Ncp == EXTENDED)
        PHY_ofdm_mod(&txdataF[aa][slot_offset_F],        // input
                     &txdata[aa][slot_offset],         // output
                     frame_parms->ofdm_symbol_size,                
                     6,                 // number of symbols
                     frame_parms->nb_prefix_samples,               // number of prefix samples
                     CYCLIC_PREFIX);
      else {
        normal_prefix_mod(&txdataF[aa][slot_offset_F],
                          &txdata[aa][slot_offset],
                          7,
                          frame_parms);
      }
    }
  }

}

void apply_nr_rotation(NR_DL_FRAME_PARMS *fp,
                       int16_t* trxdata,
                       int slot,
                       int first_symbol,
                       int nsymb,
                       int length) {
  int symb_offset = (slot%fp->slots_per_subframe)*fp->symbols_per_slot;

  c16_t *symbol_rotation = fp->symbol_rotation[0];

  for (int sidx=0;sidx<nsymb;sidx++) {

    LOG_D(PHY,"Rotating symbol %d, slot %d, symbol_subframe_index %d, length %d (%d,%d)\n",
      first_symbol + sidx,
      slot,
      sidx + first_symbol + symb_offset,
      length,
      symbol_rotation[sidx + first_symbol + symb_offset].r,
      symbol_rotation[sidx + first_symbol + symb_offset].i);

    rotate_cpx_vector(((c16_t*) trxdata) + sidx * length,
                      symbol_rotation + sidx + first_symbol + symb_offset,
                      ((c16_t*) trxdata) + sidx * length,
                      length,
                      15);
  }
}
                       
