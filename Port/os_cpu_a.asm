;********************************************************************************************************
;                                              uC/OS-II
;                                        The Real-Time Kernel
;
;                    Copyright 1992-2021 Silicon Laboratories Inc. www.silabs.com
;
;                                 SPDX-License-Identifier: APACHE-2.0
;
;               This software is subject to an open source license and is distributed by
;                Silicon Laboratories Inc. pursuant to the terms of the Apache License,
;                    Version 2.0 available at www.apache.org/licenses/LICENSE-2.0.
;
;********************************************************************************************************

;********************************************************************************************************
;
;                                             ARMv7-M Port
;
; Filename  : os_cpu_a.asm
; Version   : V2.93.01
;********************************************************************************************************
; For       : ARMv7-M Cortex-M
; Mode      : Thumb-2 ISA
; Toolchain : ARM C Compiler
;********************************************************************************************************
; Note(s)   : (1) This port supports the ARM Cortex-M3, Cortex-M4 and Cortex-M7 architectures.
;             (2) It has been tested with the following Hardware Floating Point Unit.
;                 (a) Single-precision: FPv4-SP-D16-M and FPv5-SP-D16-M
;                 (b) Double-precision: FPv5-D16-M
;********************************************************************************************************

;********************************************************************************************************
;                                          PUBLIC FUNCTIONS
;********************************************************************************************************
;声明全局变量
    EXTERN  OSRunning
    EXTERN  OSPrioCur
    EXTERN  OSPrioHighRdy
    EXTERN  OSTCBCur
    EXTERN  OSTCBHighRdy
    EXTERN  OSIntExit
    EXTERN  OSTaskSwHook
    EXTERN  OS_CPU_ExceptStkBase
    EXTERN  OS_KA_BASEPRI_Boundary

;声明函数
    EXPORT  OSStartHighRdy
    EXPORT  OS_CPU_SR_Save
    EXPORT  OS_CPU_SR_Restore
    EXPORT  OSCtxSw
    EXPORT  OSIntCtxSw
    EXPORT  OS_CPU_PendSVHandler

    IF {FPU} != "SoftVFP"
    EXPORT  OS_CPU_FP_Reg_Push
    EXPORT  OS_CPU_FP_Reg_Pop
    ENDIF


;********************************************************************************************************
;                                               EQUATES
;********************************************************************************************************

NVIC_INT_CTRL   EQU     0xE000ED04      ; 中断控制状态寄存器（ICSR）地址
NVIC_SYSPRI14   EQU     0xE000ED22      ; 系统异常优先级寄存器（对应 PendSV 的优先级字节）
NVIC_PENDSV_PRI EQU           0xFF      ; PendSV 的优先级值（最低优先级，数值 255）
NVIC_PENDSVSET  EQU     0x10000000      ; 触发 PendSV 异常的控制位（ICSR 的第 28 位）


;********************************************************************************************************
;                                     CODE GENERATION DIRECTIVES
;********************************************************************************************************

    AREA |.text|, CODE, READONLY, ALIGN=2
    THUMB
    REQUIRE8
    PRESERVE8


;********************************************************************************************************
;                                   FLOATING POINT REGISTERS PUSH
;                             void  OS_CPU_FP_Reg_Push (OS_STK  *stkPtr)
;
; Note(s) : 1) This function saves S16-S31 registers of the Floating Point Unit.
;
;           2) Pseudo-code is:
;              a) Push remaining FPU regs S16-S31 on process stack;
;              b) Update OSTCBCur->OSTCBStkPtr;
;********************************************************************************************************

    IF {FPU} != "SoftVFP"

OS_CPU_FP_Reg_Push
    MRS     R1, PSP                                             ; PSP is process stack pointer
    CBZ     R1, OS_CPU_FP_nosave                                ; Skip FP register save the first time

    VSTMDB  R0!, {S16-S31}
    LDR     R1, =OSTCBCur
    LDR     R2, [R1]
    STR     R0, [R2]
OS_CPU_FP_nosave
    BX      LR

    ENDIF


;********************************************************************************************************
;                                   FLOATING POINT REGISTERS POP
;                             void  OS_CPU_FP_Reg_Pop (OS_STK  *stkPtr)
;
; Note(s) : 1) This function restores S16-S31 of the Floating Point Unit.
;
;           2) Pseudo-code is:
;              a) Restore regs S16-S31 of new process stack;
;              b) Update OSTCBHighRdy->OSTCBStkPtr pointer of new proces stack;
;********************************************************************************************************

    IF {FPU} != "SoftVFP"

OS_CPU_FP_Reg_Pop
    VLDMIA  R0!, {S16-S31}
    LDR     R1, =OSTCBHighRdy
    LDR     R2, [R1]
    STR     R0, [R2]
    BX      LR

    ENDIF

;********************************************************************************************************
;                                   临界段方法 3 的相关函数
;
; 描述 : 通过保存 BASEPRI 的状态来禁用/使能内核感知的中断。通常，BASEPRI 中断异常处理的状态
;        保存在局部变量 'cpu_sr' 中，然后禁用内核感知的中断（'cpu_sr' 分配在所有需要禁用
;        内核感知中断的函数中）。通过将 'cpu_sr' 复制回 BASEPRI 寄存器来恢复先前的 BASEPRI 中断状态。
;
; 原型   : OS_CPU_SR  OS_CPU_SR_Save   (OS_CPU_SR  new_basepri);
;          void       OS_CPU_SR_Restore(OS_CPU_SR  cpu_sr);
;
;
; 注意   : 1) 这些函数通常像下面这样使用：
;
;               void Task (void *p_arg)
;               {
;               #if OS_CRITICAL_METHOD == 3          /* 为 CPU 状态寄存器分配存储空间  */
;                   OS_CPU_SR  cpu_sr;
;               #endif
;
;                       :
;                       :
;                   OS_ENTER_CRITICAL();             /* cpu_sr = OS_CPU_SR_Save(new_basepri);     */
;                       :
;                       :
;                   OS_EXIT_CRITICAL();              /* OS_CPU_RestoreSR(cpu_sr);                 */
;                       :
;                       :
;               }
;
;          2) 通过写入 BASEPRI 来提高优先级不会立即生效。
;             (a) 含义：此勘误意味着在 MSR 提升 BASEPRI 之后的指令可能会被一个不够高优先级的
;                 异常错误地抢占。
;             (b) 解决方法：可以用以下代码序列替换提升 BASEPRI 的 MSR：
;
;                  CPSID i
;                  MSR to BASEPRI
;                  DSB
;                  ISB
;                  CPSIE i
;********************************************************************************************************
OS_CPU_SR_Save
    CPSID   I           ; (1) 关所有中断 (PRIMASK=1)
    PUSH    {R1}        ; (2) 保护 R1 到栈中
    MRS     R1, BASEPRI ; (3) 读取当前 BASEPRI 值到 R1
    MSR     BASEPRI, R0 ; (4) 将 R0 的新传入的值写入 BASEPRI 
    DSB                 ; (5) 数据同步屏障，确保写 BASEPRI 完成
    ISB                 ; (6) 指令同步屏障，刷新流水线
    MOV     R0, R1      ; (7) 将旧的 BASEPRI 值移到 R0 作为返回值
    POP     {R1}        ; (8) 恢复 R1
    CPSIE   I           ; (9) 开中断 (PRIMASK=0)
    BX      LR          ; (10) 返回，R0 中带回旧值

OS_CPU_SR_Restore
    CPSID   I           ; (1) 关中断
    MSR     BASEPRI, R0 ; (2) 将 传入的旧 BASEPRI 值(cpu_sr) 写回 BASEPRI
    DSB                 ; (3) 数据同步屏障
    ISB                 ; (4) 指令同步屏障
    CPSIE   I           ; (5) 开中断
    BX      LR          ; (6) 返回


;********************************************************************************************************
;                                        开始多任务调度
;                                      void OSStartHighRdy(void)
;
; 注意 : 1) 该函数通过触发 PendSV 异常（本质上产生一次上下文切换）来启动第一个任务。
;
;        2) 任务执行期间，使用 PSP（进程栈指针）作为栈指针。
;           当异常发生时，内核将切换到 MSP（主栈指针），直到异常返回。
;
;        3) OSStartHighRdy() 必须完成：
;           a) 设置 PendSV 异常优先级为最低；
;           b) 将初始 PSP 设为 0，告诉上下文切换器这是首次运行；
;           c) 将主栈设为 OS_CPU_ExceptStkBase；
;           d) 将 OSRunning 设为 TRUE；
;           e) 获取当前高优先级，OSPrioCur = OSPrioHighRdy；
;           f) 获取当前就绪任务的 TCB，OSTCBCur = OSTCBHighRdy；
;           g) 从 TCB 中获取新的进程栈指针，SP = OSTCBHighRdy->OSTCBStkPtr；
;           h) 从新任务的栈中恢复 R0-R11 和 LR；
;           i) 开中断（任务将以中断使能状态运行）。
;********************************************************************************************************

OSStartHighRdy
    CPSID   I                                                   ; 关中断，防止上下文切换过程中被打断
    LDR     R0, =NVIC_SYSPRI14                                  ; 设置 PendSV 异常优先级
    LDR     R1, =NVIC_PENDSV_PRI
    STRB    R1, [R0]                                            ; 将最低优先级（0xFF）写入 PendSV 优先级寄存器

    MOVS    R0, #0                                              ; 将 PSP 设为 0，用于标识第一次上下文切换
    MSR     PSP, R0

    LDR     R0, =OS_CPU_ExceptStkBase                           ; 将 MSP 初始化为异常栈基址
    LDR     R1, [R0]
    MSR     MSP, R1

    BL      OSTaskSwHook                                        ; 调用钩子函数（用于 FPU 寄存器压栈/出栈）

    LDR     R0, =OSRunning                                      ; OSRunning = TRUE
    MOVS    R1, #1
    STRB    R1, [R0]

    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    LDR     R0, =OSTCBCur                                       ; OSTCBCur = OSTCBHighRdy
    LDR     R1, =OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]

    LDR     R0, [R2]                                            ; R0 = OSTCBHighRdy->OSTCBStkPtr（新任务的栈顶）
    MSR     PSP, R0                                             ; 加载 PSP 为新任务的栈指针

    MRS     R0, CONTROL                                         ; 设置 CONTROL 寄存器，使能线程模式使用 PSP
    ORR     R0, R0, #2
    MSR     CONTROL, R0
    ISB                                                         ; 指令同步屏障，确保修改生效

    LDMFD   SP!, {R4-R11, LR}                                   ; 从新任务的栈中恢复 R4-R11 和 LR（R14）
    LDMFD   SP!, {R0-R3}                                        ; 恢复 R0-R3（注意：SP 此时指向自动压栈区域）
    LDMFD   SP!, {R12, LR}                                      ; 恢复 R12 和 LR（LR 被覆盖，但没关系）
    LDMFD   SP!, {R1, R2}                                       ; 加载 PC（到 R1）并丢弃 xPSR（到 R2）
    CPSIE   I                                                   ; 开中断
    BX      R1                                                  ; 跳转到 R1 中的 PC 地址，开始执行任务

;********************************************************************************************************
;                       执行上下文切换（来自任务级） - OSCtxSw()
;                       执行上下文切换（来自中断级） - OSIntCtxSw()
;
; 注意 : 1) 当 μC/OS-II 想要执行任务上下文切换时，会调用 OSCtxSw()。该函数触发 PendSV 异常，
;           实际工作由 PendSV 异常处理程序完成。
;
;        2) 当 OSIntExit() 判断由于中断的发生而需要进行上下文切换时，会调用 OSIntCtxSw()。
;           该函数简单地触发一个 PendSV 异常，该异常将在没有其他中断活跃且中断使能后得到处理。
;********************************************************************************************************

OSCtxSw
OSIntCtxSw
    LDR     R0, =NVIC_INT_CTRL                                  ; 触发 PendSV 异常（引起上下文切换）
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR


;********************************************************************************************************
;                                       处理 PendSV 异常
;                                   void OS_CPU_PendSVHandler(void)
;
; 注意 : 1) PendSV 用于引发上下文切换。这是在 Cortex-M 上执行上下文切换的推荐方法。
;           这是因为 Cortex-M 在任何异常上都会自动保存一半的处理器上下文，并在异常返回时恢复。
;           因此只需要保存 R4-R11 和 R14 并调整栈指针。以这种方式使用 PendSV 异常意味着
;           无论是从线程发起还是由中断或异常引起，上下文保存和恢复都是相同的。
;
;        2) 伪代码如下：
;           a) 获取进程栈指针
;           b) 在进程栈上保存剩余寄存器 r4-r11 和 r14
;           c) 将进程栈指针保存到其 TCB 中，OSTCBCur->OSTCBStkPtr = SP
;           d) 调用 OSTaskSwHook()
;           e) 获取当前高优先级，OSPrioCur = OSPrioHighRdy
;           f) 获取当前就绪任务的 TCB，OSTCBCur = OSTCBHighRdy
;           g) 从 TCB 获取新进程栈指针，SP = OSTCBHighRdy->OSTCBStkPtr
;           h) 从新进程栈中恢复 R4-R11 和 R14
;           i) 执行异常返回，将恢复其余上下文
;
;        3) 进入 PendSV 处理程序时：
;           a) 以下寄存器已由处理器自动保存在进程栈上：
;              xPSR, PC, LR, R12, R0-R3
;           b) 处理器模式切换到处理模式（从线程模式）
;           c) 栈为主栈（从进程栈切换过来）
;           d) OSTCBCur      指向要挂起的任务的 OS_TCB
;              OSTCBHighRdy  指向要恢复的任务的 OS_TCB
;
;        4) 由于 PendSV 在系统中被设置为最低优先级（由上面的 OSStartHighRdy() 设置），
;           我们知道它只会在没有其他异常或中断活跃时运行，因此可以安全地假定被切换出去的
;           上下文使用的是进程栈（PSP）。
;
;        5) 通过写入 BASEPRI 来提高优先级不会立即生效。
;           (a) 含义：此勘误意味着在 MSR 提升 BASEPRI 之后的指令可能会被一个不够高优先级的
;               异常错误地抢占。
;           (b) 解决方法：可以用以下代码序列替换提升 BASEPRI 的 MSR：
;
;                  CPSID i
;                  MSR to BASEPRI
;                  DSB
;                  ISB
;                  CPSIE i
;********************************************************************************************************

OS_CPU_PendSVHandler
    CPSID   I                                                   ; Cortex-M7 勘误说明。参见注意 #5
    MOV32   R2, OS_KA_BASEPRI_Boundary                          ; 设置异常抢占所需的 BASEPRI 优先级级别
    LDR     R1, [R2]
    MSR     BASEPRI, R1
    DSB
    ISB
    CPSIE   I

    MRS     R0, PSP                                             ; PSP 是进程栈指针
    STMFD   R0!, {R4-R11, R14}                                  ; 在进程栈上保存剩余寄存器 r4-11, R14

    LDR     R5, =OSTCBCur                                       ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R5]
    STR     R0, [R1]                                            ; R0 是被切换出的进程的 SP

                                                                ; 此时，进程的整个上下文已保存
    MOV     R4, LR                                              ; 保存 LR 异常返回值
    BL      OSTaskSwHook                                        ; 调用 OSTaskSwHook() 用于 FPU 压栈和出栈

    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    LDR     R1, =OSTCBHighRdy                                   ; OSTCBCur  = OSTCBHighRdy;
    LDR     R2, [R1]
    STR     R2, [R5]

    ORR     LR,  R4, #0x04                                      ; 确保异常返回使用进程栈
    LDR     R0,  [R2]                                           ; R0 是新进程的 SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    LDMFD   R0!, {R4-R11, R14}                                  ; 从新进程栈恢复 r4-11, R14
    MSR     PSP, R0                                             ; 用新进程 SP 加载 PSP

    MOV32   R2, #0                                              ; 将 BASEPRI 优先级级别恢复为 0
    CPSID   I
    MSR     BASEPRI, R2
    DSB
    ISB
    CPSIE   I
    BX      LR                                                  ; 异常返回将恢复其余上下文

    ALIGN

    END
		