/**
  ******************************************************************************
  * @file    attitude.h
  * @brief   Madgwick AHRS 姿态解算模块（六轴：陀螺仪 + 加速度计）
  *
  * 算法来源：SOH Madgwick, "An efficient orientation filter for inertial and
  *          inertial/magnetic sensor arrays", 2010
  *          参考实现：MadgwickAHRS.c (2012-02-19 版)
  *
  * 与原始实现的区别（只有三处，算法本身一字未改）：
  *   1. 用 dt 参数代替硬编码的 sampleFreq
  *   2. 用结构体代替全局 volatile 变量
  *   3. 加了加速度/梯度为零时的保护，避免除零产生 NaN
  *
  * 对应的课程内容：
  *   陀螺仪积分   -> 第 4 课   q_dot = 0.5 * q (x) (0, omega)
  *   误差与梯度   -> 第 8 课   s = J^T * f ，f = v - a ，v = C(q)^T * [0,0,1]
  *   归一化与反馈 -> 第 8 课   q_dot -= beta * s/|s|
  ******************************************************************************
  */

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

/* ==========================================================================
 * 单位换算系数
 *
 *   加速度计：MPU6050_Init() 里 ACCEL_CONFIG = 0x10 -> ±8g -> 4096 LSB/g
 *             （已用 1g 实测验证：静止时 |a| ≈ 4160 LSB ≈ 1.02g ✓）
 *
 *   陀螺仪：寄存器写的是 GYRO_CONFIG = 0x08（±500°/s，本该 65.5 LSB/(°/s)），
 *           但实测发现这颗芯片【实际工作在 ±2000°/s】：
 *             把板子慢慢抬起 60°，加速度计反解出真实转过 54°，
 *             而陀螺只积出 10.7° —— 差了约 4~5 倍。
 *             之前"顺时针转 90° 只得到 22°"也是 4.09 倍，两者吻合。
 *           这是 clone 芯片的典型现象：配置寄存器写得进去、读得回来，
 *           但芯片内部量程并没有跟着变。只能靠实测标定。
 *
 *           ±2000°/s -> 16.4 LSB/(°/s)
 *
 *   ★ 判断陀螺量程对不对的方法（固件已内置）★
 *     上电平放 -> 缓慢抬头到 60° -> 停住 -> 看 [STAT] 那行的 ratio
 *       ratio ≈ 100  -> 正确
 *       ratio ≈ 400  -> 偏小 4 倍（应改 65.5f）
 *       ratio ≈ 200  -> 偏小 2 倍（应改 32.8f）
 * ========================================================================== */
#define ATT_GYRO_LSB_PER_DPS   16.4f
#define ATT_ACC_LSB_PER_G      4096.0f

/* 度 -> 弧度 */
#define ATT_DEG2RAD            0.017453292519943295f
/* LSB -> rad/s 一步到位 */
#define ATT_GYRO_LSB_TO_RAD    (ATT_DEG2RAD / ATT_GYRO_LSB_PER_DPS)

/* ==========================================================================
 * 传感器轴 -> 机体轴 的映射（实测标定出来的）
 *
 * 本工程 GY-86 是【竖着】插在转接板上的，所以传感器的 X 轴是竖直的。
 * 实测（平放 / 抬头 / 右滚三组数据反解）得到：
 *
 *     机头(前) = -s_y
 *     右翼(右) = +s_z
 *     机腹(下) = -s_x
 *
 * 对应矩阵 M = [ 0 -1  0 ]
 *              [ 0  0  1 ]      det(M) = +1，是合法的旋转
 *              [-1  0  0 ]
 *
 * 三个轴必须构成右手系（行列式 = +1）。如果以后换安装方式，重新标定：
 *   ① 平放，记 a₁  ② 抬头30°，记 a₂  ③ 右滚30°，记 a₃
 *   再解 R 使 R·u_i = a_i（u 是机体下的"向上"方向）。
 * ========================================================================== */
#define ATT_MAP_X0   0.0f      /* 机头 = 0*sx + (-1)*sy + 0*sz */
#define ATT_MAP_X1  -1.0f
#define ATT_MAP_X2   0.0f
#define ATT_MAP_Y0   0.0f      /* 右翼 = 0*sx + 0*sy + 1*sz */
#define ATT_MAP_Y1   0.0f
#define ATT_MAP_Y2   1.0f
#define ATT_MAP_Z0  -1.0f      /* 机腹 = (-1)*sx + 0*sy + 0*sz */
#define ATT_MAP_Z1   0.0f
#define ATT_MAP_Z2   0.0f

/* 默认反馈增益 beta (单位 1/s)。一般 0.05 ~ 0.5，越大跟得越快但越抖。 */
#define ATT_BETA_DEFAULT        0.2f

/* 陀螺零偏校准的"可信度"门槛（标准差, 单位 rad/s）
 * 静止时 MPU6050 的噪声典型 0.05~0.2 °/s = 0.001~0.0035 rad/s，
 * 这里取 0.02 rad/s ≈ 1.15 °/s，留了 5~10 倍余量。
 * 一旦有人碰飞机/桌子在晃，标准差会明显超过它，校准就会被判为不可信。 */
#ifndef ATT_GYRO_CALIB_STD_MAX
#define ATT_GYRO_CALIB_STD_MAX  0.02f
#endif

/* 姿态状态 */
typedef struct {
    float q[4];             /* q0, q1, q2, q3 */
    float beta;             /* 反馈增益 */
    float gyro_bias[3];     /* 陀螺零偏, 单位 rad/s（已含轴映射符号） */
} AttitudeState;

/* 陀螺零偏校准累加器 */
typedef struct {
    float    sum[3];        /* 累加 (rad/s) */
    float    sumsq[3];      /* 累加平方, 用来算标准差 */
    uint16_t n;             /* 样本数 */
} GyroCalib;

/* --------------------------------------------------------------------------
 * 初始化：用加速度计确定初始姿态（俯仰、滚转），偏航设为 0
 *   acc_g: 三轴加速度, 单位 g（不用归一化，函数内部会归一化）
 *   说明：这一步很重要。从 (1,0,0,0) 起步的话，算法要 3 秒左右才能收敛，
 *         这 3 秒里姿态是错的。
 * -------------------------------------------------------------------------- */
void ATT_Init(AttitudeState *a, const float acc_g[3]);

/* 陀螺零偏校准：静止采样 -> 平均 -> 写入 a->gyro_bias
 *   用法：ATT_GyroCalibBegin(); 循环 ATT_GyroCalibAdd(); ATT_GyroCalibEnd();
 *   返回 0 = 成功；1 = 数据不可信（采样的标准差太大，说明飞机在动） */
void ATT_GyroCalibBegin(GyroCalib *c);
void ATT_GyroCalibAdd(GyroCalib *c, const int16_t raw[3]);
int  ATT_GyroCalibEnd(GyroCalib *c, AttitudeState *a);

/* 原始值 -> 物理单位 */
void ATT_RawToGyro(const AttitudeState *a, const int16_t raw[3], float rad_s[3]);
void ATT_RawToAcc (const int16_t raw[3], float g[3]);

/* 核心：一步姿态更新
 *   gyro: 角速度, 单位 rad/s（已扣零偏）
 *   acc : 加速度, 单位 g（不用自己归一化）
 *   dt  : 本次与上次调用的时间间隔, 单位秒
 */
void ATT_Update(AttitudeState *a, const float gyro[3], const float acc[3], float dt);

/* 输出欧拉角（内旋 ZYX，单位 rad）
 *   roll  : 绕 x（机头轴）
 *   pitch : 绕 y（机翼轴）
 *   yaw   : 绕 z
 */
void ATT_GetEuler(const AttitudeState *a, float *roll, float *pitch, float *yaw);

#endif /* __ATTITUDE_H */
