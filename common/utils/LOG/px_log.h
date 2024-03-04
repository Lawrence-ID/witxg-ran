// -------------------px_Log--------------------
#ifndef __PX_LOG_H__
#define __PX_LOG_H__

#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>

#define SHOW_PX_LOG 0 // 1: 输出Log，0: 不输出Log

// #define TIME_STATISTIC_2 // 若不需要统计时间，注释此行即可

static struct timeval st, ed;
static double time_total;

#define ANSI_FG_BLACK   "\33[1;30m"
#define ANSI_FG_RED     "\33[1;31m"
#define ANSI_FG_GREEN   "\33[1;32m"
#define ANSI_FG_YELLOW  "\33[1;33m"
#define ANSI_FG_BLUE    "\33[1;34m"
#define ANSI_FG_MAGENTA "\33[1;35m"
#define ANSI_FG_CYAN    "\33[1;36m"
#define ANSI_FG_WHITE   "\33[1;37m"
#define ANSI_BG_BLACK   "\33[1;40m"
#define ANSI_BG_RED     "\33[1;41m"
#define ANSI_BG_GREEN   "\33[1;42m"
#define ANSI_BG_YELLOW  "\33[1;43m"
#define ANSI_BG_BLUE    "\33[1;44m"
#define ANSI_BG_MAGENTA "\33[1;35m"
#define ANSI_BG_CYAN    "\33[1;46m"
#define ANSI_BG_WHITE   "\33[1;47m"
#define ANSI_NONE       "\33[0m"

#define ANSI_FMT(str, fmt) fmt str ANSI_NONE

#define _Log(...) \
  do { \
    if(SHOW_PX_LOG == 1) { printf(__VA_ARGS__); } \
  } while (0)

#define Log(format, ...) \
    _Log(ANSI_FMT("PID:%d [%s:%d %s] " format, ANSI_FG_BLUE) "\n", \
        getpid(), __FILE__, __LINE__, __func__, ## __VA_ARGS__)

#endif

// static inline void start_time_statistic(){
//   #ifdef TIME_STATISTIC
//     gettimeofday(&st, NULL);
//   #endif
// }

// static inline void end_time_statistic(char *str){
//   #ifdef TIME_STATISTIC
//     gettimeofday(&ed, NULL);
//     time_total = (ed.tv_sec - st.tv_sec) * 1000000 + (ed.tv_usec - st.tv_usec); //us
//     Log("[%s] cost time = %lf\n", ANSI_FMT(s, ANSI_FG_YELLOW), time_total);
//   #endif
// }
// -------------------px_Log--------------------