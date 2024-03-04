#include "nr_fpga.h"

int c2h_dma_fd;
int h2c_dma_fd;

const unsigned int len = (POINT_NUM * 2 + 8) * 4;    //2^15，len的含义是需要多少个实部/虚部
const unsigned int lendata = (POINT_NUM * 2 + 8);

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
    if(c2h_dma_fd < 0)
        return -1;
    h2c_dma_fd = open("/dev/xdma0_h2c_0",O_RDWR );
    if(h2c_dma_fd < 0)
        return -2;
    posix_memalign((void *)&h2c_align_mem_tmp,4096,0x800000);
    posix_memalign((void *)&c2h_align_mem_tmp,4096,0x800000);

    if(NULL == h2c_align_mem_tmp || NULL == c2h_align_mem_tmp)
        return -6;

    return 1;
}

void pcie_deinit()
{
    close(c2h_dma_fd);
    close(h2c_dma_fd);
    //close(control_fd);
}

unsigned char write_fftdata(void *buf ,unsigned int type){
    
    FILE *fpread=fopen("fft_out.txt","w");
    if(fpread == NULL)  assert(0);
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

void fft512_nr_fpga(int16_t*in_buf,int16_t*out_buf,unsigned char scale_flag){
    //convert int16 to int 
    int in_size = 512*2+1;
    int in_size_bytes = in_size*4;
    int *in_buf_fpga = (int *)malloc(sizeof(int)*in_size);
    int *out_buf_fpga = (int *)malloc(sizeof(int)*in_size);
    int i;
    for(i=0;i<in_size-1 ;i++){
        in_buf_fpga[i] = (int)in_buf[i]+10;
    }
    memset(out_buf_fpga,0,in_size_bytes);

    in_buf_fpga[in_size-1] = 975; //start flag
    //put int data into fpga
    put_data_to_fpga_ddr(FPGA_AXI_START_ADDR,in_buf_fpga,in_size_bytes);
    //get int data in fpga
    get_data_from_fpga_ddr(FPGA_AXI_START_ADDR,out_buf_fpga,in_size_bytes);

    //convert int to int16
    for(i=0;i<in_size;i++){
        out_buf[i] = (int16_t)out_buf_fpga[i];
        // printf("out_buf[%d]=%d\n",i,out_buf_fpga[i]);
    }
    //free temporary space 
    free(in_buf_fpga);
    free(out_buf_fpga);

}