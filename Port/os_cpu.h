/*
*********************************************************************************************************
*                                              uC/OS-II
*                                        The Real-Time Kernel
*
*                    Copyright 1992-2021 Silicon Laboratories Inc. www.silabs.com
*
*                                 SPDX-License-Identifier: APACHE-2.0
*
*               This software is subject to an open source license and is distributed by
*                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
*                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
*
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                             ARMv7-M Port
*
* Filename  : os_cpu.h
* Version   : V2.93.01
*********************************************************************************************************
* For       : ARMv7-M Cortex-M
* Mode      : Thumb-2 ISA
* Toolchain : ARM C Compiler
*********************************************************************************************************
* Note(s)   : (1) This port supports the ARM Cortex-M3, Cortex-M4 and Cortex-M7 architectures.
*             (2) It has been tested with the following Hardware Floating Point Unit.
*                 (a) Single-precision: FPv4-SP-D16-M and FPv5-SP-D16-M
*                 (b) Double-precision: FPv5-D16-M
*********************************************************************************************************
*/

//全局变量
#ifndef  OS_CPU_H
#define  OS_CPU_H

#ifdef   OS_CPU_GLOBALS
#define  OS_CPU_EXT
#else
#define  OS_CPU_EXT  extern
#endif


/*
*********************************************************************************************************
*                                     EXTERNAL C LANGUAGE LINKAGE
*
* Note(s) : (1) C++ compilers MUST 'extern'ally declare ALL C function prototypes & variable/object
*               declarations for correct C language linkage.
*********************************************************************************************************
*/

#ifdef __cplusplus
extern  "C" {                                    /* See Note #1.                                       */
#endif

/*
*********************************************************************************************************
*                                              宏定义
*
* 说明：
* (1) 需要在 'app_cfg.h' 中手动定义 CPU_CFG_KA_IPL_BOUNDARY 和 CPU_CFG_NVIC_PRIO_BITS。
*
* (2) 由于本移植使用 BASEPRI 寄存器来区分“内核感知”和“非内核感知”的中断，请务必正确设置你的外设中断优先级。
*     例如，如果 CPU_CFG_KA_IPL_BOUNDARY 设置为 4，那么外设中断优先级 4~15 将被视为“内核感知”（临界段内会被屏蔽），
*     而优先级 0~3 则被视为“非内核感知”（临界段内不会被屏蔽，仍然可以响应）。
*
* (3) 确定 NVIC 可编程优先级实际使用的位数。这个数值通常可以在单片机参考手册中找到。使用 4 位时，可以提供 16 级可编程优先级。
*
*                     NVIC_IPRx 寄存器格式
*                 7                0
*                +------------------+
*                |       PRIO       |
*                +------------------+
*
*                位[7:4]：优先级掩码位（有效位）
*                位[3:0]：保留
*
*         例如，本例中若处理器只实现了位[7:4]，则 CPU_CFG_NVIC_PRIO_BITS 应设为 4。
*********************************************************************************************************
*/

#ifndef  OS_CPU_EXCEPT_STK_SIZE
#define  OS_CPU_EXCEPT_STK_SIZE      256u        /* 默认异常栈大小为 256 个 OS_STK 条目 */
#endif

#ifndef  __TARGET_FPU_SOFTVFP
#define  OS_CPU_ARM_FP_EN              1u        /* 如果编译器没定义软浮点宏，则启用 FPU 支持 */
#else
#define  OS_CPU_ARM_FP_EN              0u
#endif

#ifndef CPU_CFG_KA_IPL_BOUNDARY
#error  "CPU_CFG_KA_IPL_BOUNDARY        未在 'app_cfg.h' 中定义"   /* 参见说明 (1) 和 (2) */
#else
#if (CPU_CFG_KA_IPL_BOUNDARY == 0u)
#error  "CPU_CFG_KA_IPL_BOUNDARY        应该 > 0 "
#endif
#endif

#ifndef CPU_CFG_NVIC_PRIO_BITS
#error  "CPU_CFG_NVIC_PRIO_BITS         未在 'app_cfg.h' 中定义"   /* 参见说明 (3) */
#else
#if (CPU_CFG_KA_IPL_BOUNDARY >= (1u << CPU_CFG_NVIC_PRIO_BITS))
#error  "CPU_CFG_KA_IPL_BOUNDARY        不能大于可编程优先级的最大值"
#endif
#endif

/*
*********************************************************************************************************
*                                              DATA TYPES
*                                         (Compiler Specific)
*********************************************************************************************************
*/

//数据类型
typedef unsigned char  BOOLEAN;
typedef unsigned char  INT8U;                    /* Unsigned  8 bit quantity                           */
typedef signed   char  INT8S;                    /* Signed    8 bit quantity                           */
typedef unsigned short INT16U;                   /* Unsigned 16 bit quantity                           */
typedef signed   short INT16S;                   /* Signed   16 bit quantity                           */
typedef unsigned int   INT32U;                   /* Unsigned 32 bit quantity                           */
typedef signed   int   INT32S;                   /* Signed   32 bit quantity                           */
typedef float          FP32;                     /* Single precision floating point                    */
typedef double         FP64;                     /* Double precision floating point                    */

typedef unsigned int   OS_STK;                   /* Each stack entry is 32-bit wide                    */
typedef unsigned int   OS_CPU_SR;                /* Define size of CPU status register (PSR = 32 bits) */


/*
*********************************************************************************************************
*                                              Cortex-M
*                                      Critical Section Management
*
* Method #1:  Disable/Enable interrupts using simple instructions.  After critical section, interrupts
*             will be enabled even if they were disabled before entering the critical section.
*             NOT IMPLEMENTED
*
* Method #2:  Disable/Enable interrupts by preserving the state of interrupts.  In other words, if
*             interrupts were disabled before entering the critical section, they will be disabled when
*             leaving the critical section.
*             NOT IMPLEMENTED
*
* Method #3:  Disable/Enable interrupts by preserving the state of interrupts.  Generally speaking you
*             would store the state of the interrupt disable flag in the local variable 'cpu_sr' and then
*             disable interrupts.  'cpu_sr' is allocated in all of uC/OS-II's functions that need to
*             disable interrupts.  You would restore the interrupt disable state by copying back 'cpu_sr'
*             into the CPU's status register.
*********************************************************************************************************
*/

//临界段
#define  OS_CRITICAL_METHOD   3u

#if OS_CRITICAL_METHOD == 3u

/* 保存当前 BASEPRI 优先级（用于异常处理），并将 BASEPRI 设置为 CPU_CFG_KA_IPL_BOUNDARY*/
#define  OS_ENTER_CRITICAL()  do { cpu_sr = OS_CPU_SR_Save(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));} while (0)

/* 恢复 CPU 的 BASEPRI 优先级 */
#define  OS_EXIT_CRITICAL()   do { OS_CPU_SR_Restore(cpu_sr);} while (0)
#endif

/*
*********************************************************************************************************
*                                        Cortex-M Miscellaneous
*********************************************************************************************************
*/

#define  OS_STK_GROWTH        1u                 	/* 栈生长方向从高地址到低地址增长         */

#define  OS_TASK_SW()         OSCtxSw()						/* 定义任务切换宏 */


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

OS_CPU_EXT  OS_STK   OS_CPU_ExceptStk[OS_CPU_EXCEPT_STK_SIZE];
OS_CPU_EXT  OS_STK  *OS_CPU_ExceptStkBase;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#if OS_CRITICAL_METHOD == 3u			/* 看 os_cpu_a.asm 文件 */
OS_CPU_SR  OS_CPU_SR_Save         (OS_CPU_SR  new_basepri);
void       OS_CPU_SR_Restore      (OS_CPU_SR  cpu_sr);
#endif

void       OSCtxSw                (void);					//用户切换函数
void       OSIntCtxSw             (void);					//中断任务切换函数
void       OSStartHighRdy         (void);					//操作系统第一次启动时的调用的任务切换

/* 看 OS_CPU_C.C 文件 */
void       OS_CPU_SysTickInit     (INT32U     cnts);
void       OS_CPU_SysTickInitFreq (INT32U     cpu_freq);

void       OS_CPU_SysTickHandler  (void);
void       OS_CPU_PendSVHandler   (void);

#if (OS_CPU_ARM_FP_EN > 0u)
void       OS_CPU_FP_Reg_Push     (OS_STK    *stkPtr);
void       OS_CPU_FP_Reg_Pop      (OS_STK    *stkPtr);
#endif

/*
*********************************************************************************************************
*                                   EXTERNAL C LANGUAGE LINKAGE END
*********************************************************************************************************
*/

#ifdef __cplusplus
}                                                /* End of 'extern'al C lang linkage.                  */
#endif


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif
