#ifndef __NR_FPGA_H__
#define __NR_FPGA_H__

#include "/home/g/Desktop/px-witcg-ran/witxg-ran/openair1/PHY/NR_TRANSPORT/nr_fpga.h"
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <semaphore.h>
#include <stdarg.h>
#include <syslog.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <pthread.h>//添加多线程支持
#if __BYTE_ORDER == __LITTLE_ENDIAN
#  define ltohl(x)       (x)
#  define ltohs(x)       (x)
#  define htoll(x)       (x)
#  define htols(x)       (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#  define ltohl(x)     __bswap_32(x)
#  define ltohs(x)     __bswap_16(x)
#  define htoll(x)     __bswap_32(x)
#  define htols(x)     __bswap_16(x)
#endif
static unsigned char *h2c_align_mem_tmp;
static unsigned char *c2h_align_mem_tmp;

#define MAP_SIZE (1024*1024UL)
#define MAP_MASK (MAP_SIZE - 1)
#define POINT_NUM 512

#define FPGA_AXI_START_ADDR (0)
#define FPGA_AXI_CLEAR (32768)
#define POLL_MODE
#define FPGA_CONTROL_ADDR 32768 //64= (8193-1)*4 || 4是指字节，17-1指控制位在第17个数 
#define CONTROL_DATA 1
#define log_fpga(a) write_log(a)

#ifdef __cplusplus
 extern "C" {
 #endif

void fft512_nr_fpga(int16_t*in_buf,int16_t*out_buf,unsigned char scale_flag);
#ifdef __cplusplus
}
#endif
#endif


