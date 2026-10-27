/* 桩头文件：uC/OS-II 的最小声明，仅用于 PC 上的语法检查 */
#ifndef UCOS_II_STUB_H
#define UCOS_II_STUB_H

#include <stdint.h>
#include "app_cfg.h"

typedef unsigned char  INT8U;
typedef signed char    INT8S;
typedef unsigned short INT16U;
typedef signed short   INT16S;
typedef unsigned int   INT32U;
typedef signed int     INT32S;

typedef uint32_t OS_STK;
typedef struct { int dummy; } OS_EVENT;
typedef struct { int dummy; } OS_TCB;
typedef struct { int dummy; } OS_FLAG_GRP;
typedef struct { int dummy; } OS_Q;
typedef struct { int dummy; } OS_MEM;
typedef struct { int dummy; } OS_TMR;
typedef uint32_t OS_CPU_SR;

#define OS_ERR_NONE     0u
#define OS_PRIO_SELF    0xFFu
#define OS_TRUE         1u
#define OS_FALSE        0u

#define OS_ENTER_CRITICAL()  do { } while (0)
#define OS_EXIT_CRITICAL()   do { } while (0)

OS_EVENT *OSSemCreate(INT16U cnt);
INT8U     OSSemPend(OS_EVENT *pevent, INT32U timeout, INT8U *perr);
INT8U     OSSemPost(OS_EVENT *pevent);

INT8U OSTaskCreate(void (*task)(void *pd), void *p_arg, OS_STK *ptos, INT8U prio);
INT8U OSTaskNameSet(INT8U prio, char *pname, INT8U *perr);
INT8U OSTaskDel(INT8U prio);

void  OSTimeDly(INT16U ticks);
INT8U OSTimeDlyHMSM(INT8U hours, INT8U minutes, INT8U seconds, INT16U milli);

void  OSInit(void);
void  OSStart(void);
void  OS_CPU_SysTickInitFreq(uint32_t freq);

#endif
