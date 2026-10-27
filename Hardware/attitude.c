/**
  ******************************************************************************
  * @file    attitude.c
  * @brief   Madgwick AHRS 姿态解算（六轴）
  ******************************************************************************
  */

#include "attitude.h"
#include <math.h>
#include <string.h>

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/* 快速平方根倒数：位运算给初值 + 一次牛顿迭代
 * 对应 MadgwickAHRS.c 里的 invSqrt()，第 11 课讲过原理。
 * 注意：原码用 long 强转指针，在 64 位平台上会出错，这里改用 union + uint32_t */
static float att_inv_sqrt(float x)
{
    union { float f; uint32_t i; } u;

    u.f = x;
    u.i = 0x5f3759dfu - (u.i >> 1);          /* 魔数给初值 */
    u.f = u.f * (1.5f - 0.5f * x * u.f * u.f); /* 一次牛顿迭代 */
    return u.f;
}

/* 判断是否为有限数（用来挡住 NaN / Inf 传播） */
static int att_is_finite(float x)
{
    return (x == x) && (x < 1.0e30f) && (x > -1.0e30f);
}

/* 夹紧到 [-1, 1]，给 asinf 用 */
static float att_clamp_unit(float x)
{
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

/* ==========================================================================
 * 原始值 -> 物理单位
 * ========================================================================== */

/* 陀螺仪: LSB -> rad/s，扣除零偏（传感器系），再映射到机体轴
 * 注意：零偏是在【传感器系】里测的，所以先扣除、再映射 */
void ATT_RawToGyro(const AttitudeState *a, const int16_t raw[3], float rad_s[3])
{
    float sx = raw[0] * ATT_GYRO_LSB_TO_RAD - a->gyro_bias[0];
    float sy = raw[1] * ATT_GYRO_LSB_TO_RAD - a->gyro_bias[1];
    float sz = raw[2] * ATT_GYRO_LSB_TO_RAD - a->gyro_bias[2];

    rad_s[0] = ATT_MAP_X0 * sx + ATT_MAP_X1 * sy + ATT_MAP_X2 * sz;
    rad_s[1] = ATT_MAP_Y0 * sx + ATT_MAP_Y1 * sy + ATT_MAP_Y2 * sz;
    rad_s[2] = ATT_MAP_Z0 * sx + ATT_MAP_Z1 * sy + ATT_MAP_Z2 * sz;
}

/* 加速度计: LSB -> g，映射到机体轴，再取反
 *
 * ★ 为什么要取反 ★
 *   加速度计静止时测的是"支撑力"，方向朝【上】；
 *   而 Madgwick 算法里的 a 是"重力方向在机体系的分量"，朝【下】。
 *   所以映射完还要整体取负号。
 *
 *   验证：竖装的 GY-86 平放时 s=(+1,0,0) -> 映射后 (0,0,-1) -> 取反 (0,0,+1) ✓ */
void ATT_RawToAcc(const int16_t raw[3], float g[3])
{
    float sx = raw[0] / ATT_ACC_LSB_PER_G;
    float sy = raw[1] / ATT_ACC_LSB_PER_G;
    float sz = raw[2] / ATT_ACC_LSB_PER_G;

    float bx = ATT_MAP_X0 * sx + ATT_MAP_X1 * sy + ATT_MAP_X2 * sz;
    float by = ATT_MAP_Y0 * sx + ATT_MAP_Y1 * sy + ATT_MAP_Y2 * sz;
    float bz = ATT_MAP_Z0 * sx + ATT_MAP_Z1 * sy + ATT_MAP_Z2 * sz;

    g[0] = -bx;
    g[1] = -by;
    g[2] = -bz;
}

/* ==========================================================================
 * 初始化：用加速度计定初始姿态
 *
 * 静止时加速度计读数 a ≈ C(q)^T * [0,0,1] = C 的第三行
 * C 的第三行 = [ -sin(pitch), cos(pitch)sin(roll), cos(pitch)cos(roll) ]
 *   =>  pitch = asin(-a_x)
 *       roll  = atan2(a_y, a_z)
 * ========================================================================== */
void ATT_Init(AttitudeState *a, const float acc_g[3])
{
    float ax = acc_g[0], ay = acc_g[1], az = acc_g[2];
    float n  = sqrtf(ax * ax + ay * ay + az * az);
    float pitch, roll;

    memset(a, 0, sizeof(AttitudeState));
    a->q[0] = 1.0f;
    a->beta = ATT_BETA_DEFAULT;

    if (n < 0.1f) {
        return;                     /* 加速度数据无效，就保持水平 */
    }

    ax /= n;  ay /= n;  az /= n;

    pitch = asinf(att_clamp_unit(-ax));
    roll  = atan2f(ay, az);

    /* 内旋 ZYX: q = qz(yaw=0) * qy(pitch) * qx(roll)
     * 半角展开后就是下面四行 */
    {
        float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);

        a->q[0] = cr * cp;          /* q0 */
        a->q[1] = sr * cp;          /* q1 */
        a->q[2] = cr * sp;          /* q2 */
        a->q[3] = -sr * sp;         /* q3 */
    }
}

/* ==========================================================================
 * 陀螺零偏校准
 *
 * 原理：静止时真实角速度 = 0，所以读数的平均值就是零偏。
 * 同时统计标准差：如果太大，说明校准期间飞机在动，结果不可信。
 * ========================================================================== */
void ATT_GyroCalibBegin(GyroCalib *c)
{
    memset(c, 0, sizeof(GyroCalib));
}

void ATT_GyroCalibAdd(GyroCalib *c, const int16_t raw[3])
{
    /* 注意：这里在【传感器系】里累加，不做轴映射。
     * 因为零偏是"某个传感器轴输出多少"，挂到机体轴之后就不对应了。 */
    float v[3];

    v[0] = raw[0] * ATT_GYRO_LSB_TO_RAD;
    v[1] = raw[1] * ATT_GYRO_LSB_TO_RAD;
    v[2] = raw[2] * ATT_GYRO_LSB_TO_RAD;

    c->sum[0]   += v[0];  c->sum[1]   += v[1];  c->sum[2]   += v[2];
    c->sumsq[0] += v[0] * v[0];
    c->sumsq[1] += v[1] * v[1];
    c->sumsq[2] += v[2] * v[2];
    c->n++;
}

/* 返回 0 = 成功, 1 = 数据不可信 */
int ATT_GyroCalibEnd(GyroCalib *c, AttitudeState *a)
{
    int   i;
    int   bad = 0;
    float n   = (float)c->n;

    if (c->n < 10) {
        a->gyro_bias[0] = a->gyro_bias[1] = a->gyro_bias[2] = 0.0f;
        return 1;
    }

    for (i = 0; i < 3; i++) {
        float mean = c->sum[i] / n;
        float var  = c->sumsq[i] / n - mean * mean;
        float std;

        if (var < 0.0f) var = 0.0f;
        std = sqrtf(var);

        a->gyro_bias[i] = mean;

        /* 用标准差判断这次校准可不可信（阈值见 attitude.h） */
        if (std > ATT_GYRO_CALIB_STD_MAX) {
            bad = 1;
        }
    }

    return bad;
}

/* ==========================================================================
 * 核心：一步姿态更新
 *
 * 与 MadgwickAHRS.c 的 MadgwickAHRSupdateIMU 一一对应，只是：
 *   - 用 a->q[] 代替全局 q0..q3
 *   - 用 dt 代替 1.0f/sampleFreq
 *   - 加了几处保护
 * ========================================================================== */
void ATT_Update(AttitudeState *a, const float gyro[3], const float acc[3], float dt)
{
    float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float qDot0, qDot1, qDot2, qDot3;
    float an2, rn;

    if (dt <= 0.0f || dt > 0.1f) {
        return;                            /* dt 异常，直接丢掉这一拍 */
    }

    /* ---------- ① 陀螺仪项：qDot = 0.5 * q (x) (0, omega) ---------- */
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* ---------- ② 加速度有效才做修正 ---------- */
    an2 = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];

    if (an2 > 1.0e-4f) {                   /* |a| > 0.01 g 才认为有效 */
        float r  = att_inv_sqrt(an2);
        float ax = acc[0] * r;
        float ay = acc[1] * r;
        float az = acc[2] * r;

        /* ---------- ③ 梯度 s = J^T * f ---------- */
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
        float s0, s1, s2, s3, sn2;

        s0 = 4.0f * q0 * q2q2 + 2.0f * q2 * ax + 4.0f * q0 * q1q1 - 2.0f * q1 * ay;
        s1 = 4.0f * q1 * q3q3 - 2.0f * q3 * ax + 4.0f * q0q0 * q1 - 2.0f * q0 * ay
           - 4.0f * q1 + 8.0f * q1q1 * q1 + 8.0f * q1 * q2q2 + 4.0f * q1 * az;
        s2 = 4.0f * q0q0 * q2 + 2.0f * q0 * ax + 4.0f * q2 * q3q3 - 2.0f * q3 * ay
           - 4.0f * q2 + 8.0f * q2 * q1q1 + 8.0f * q2q2 * q2 + 4.0f * q2 * az;
        s3 = 4.0f * q1q1 * q3 - 2.0f * q1 * ax + 4.0f * q2q2 * q3 - 2.0f * q2 * ay;

        /* ---------- ④ 归一化梯度，乘 beta 从 qDot 里减掉 ---------- */
        sn2 = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;

        if (sn2 > 1.0e-12f) {
            float rb = a->beta * att_inv_sqrt(sn2);

            qDot0 -= rb * s0;
            qDot1 -= rb * s1;
            qDot2 -= rb * s2;
            qDot3 -= rb * s3;
        }
    }

    /* ---------- ⑤ 一阶积分 ---------- */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* ---------- ⑥ 归一化（必须做） ---------- */
    rn = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

    if (rn > 1.0e-12f && att_is_finite(rn)) {
        float ri = att_inv_sqrt(rn);

        q0 *= ri;  q1 *= ri;  q2 *= ri;  q3 *= ri;

        a->q[0] = q0;  a->q[1] = q1;  a->q[2] = q2;  a->q[3] = q3;
    } else {
        /* 四元数被污染了：重新用加速度计初始化，避免整机瘫痪 */
        ATT_Init(a, acc);
    }
}

/* ==========================================================================
 * 四元数 -> 欧拉角（内旋 ZYX）
 * ========================================================================== */
void ATT_GetEuler(const AttitudeState *a, float *roll, float *pitch, float *yaw)
{
    float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];

    if (pitch) {
        *pitch = asinf(att_clamp_unit(2.0f * (q0 * q2 - q1 * q3)));
    }
    if (roll) {
        *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    }
    if (yaw) {
        *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }
}
