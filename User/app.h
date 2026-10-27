#ifndef APP_H
#define APP_H

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "BSP.h"
#include "attitude.h"

//------------------ 任务优先级 ------------------
#define  APP_CFG_MOTOR_TASK_PRIO       4u
#define  APP_CFG_GY86_TASK_PRIO        5u
#define  APP_CFG_BLUETOOTH_TASK_PRIO   6u


//------------------ 任务栈大小 ------------------
#define  APP_CFG_MOTOR_TASK_STK_SIZE     256u
#define  APP_CFG_GY86_TASK_STK_SIZE      256u
#define  APP_CFG_BLUETOOTH_TASK_STK_SIZE 256u

//------------------ 任务栈定义 ------------------
static OS_STK  AppTaskStartStk[APP_CFG_STARTUP_TASK_STK_SIZE];
static OS_STK  AppTaskMotorStk[APP_CFG_MOTOR_TASK_STK_SIZE];
static OS_STK  AppTaskGY86Stk[APP_CFG_GY86_TASK_STK_SIZE];
static OS_STK  AppTaskBlueToothStk[APP_CFG_BLUETOOTH_TASK_STK_SIZE];


void AppTaskCreate(void);

/* 全局姿态（定义在 app.c，其它任务可只读访问） */
extern AttitudeState g_att;

//------------------ 任务函数声明 ------------------
static void AppTaskStart      (void *p_arg);
static void AppTaskMotor      (void *p_arg);
static void AppTaskGY86       (void *p_arg);
static void AppTaskBlueTooth   (void *p_arg);


#endif
