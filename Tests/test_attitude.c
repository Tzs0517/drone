/**
 * attitude.c 的离线测试（在电脑上跑，不需要单片机）
 * 编译： gcc -std=c99 -Wall -Wextra -O2 test_attitude.c attitude.c -lm -o test_attitude.exe
 */
#include <stdio.h>
#include <math.h>
#include "attitude.h"

#define D2R 0.017453292519943295f
#define R2D 57.29577951308232f

static int g_fail = 0;

static void check(const char *name, float got, float want, float tol)
{
    int ok = fabsf(got - want) <= tol;
    printf("    %-28s 得到 %+9.3f   期望 %+9.3f   容差 %5.2f   %s\n",
           name, got, want, tol, ok ? "OK" : "**FAIL**");
    if (!ok) g_fail++;
}

/* 跑 seconds 秒，返回最终欧拉角（度） */
static void run(AttitudeState *a, float gyro[3], float acc[3], float dt, float seconds,
                float *roll, float *pitch, float *yaw)
{
    int n = (int)(seconds / dt + 0.5f);
    int i;
    for (i = 0; i < n; i++) {
        ATT_Update(a, gyro, acc, dt);
    }
    ATT_GetEuler(a, roll, pitch, yaw);
    *roll *= R2D; *pitch *= R2D; *yaw *= R2D;
}

int main(void)
{
    AttitudeState a;
    float roll, pitch, yaw;
    float acc[3], gyro[3] = {0.0f, 0.0f, 0.0f};
    const float dt = 0.005f;

    printf("================ 测试 1：水平静止 ================\n");
    acc[0] = 0.0f; acc[1] = 0.0f; acc[2] = 1.0f;
    ATT_Init(&a, acc);
    run(&a, gyro, acc, dt, 2.0f, &roll, &pitch, &yaw);
    check("水平 roll",  roll,  0.0f, 0.1f);
    check("水平 pitch", pitch, 0.0f, 0.1f);
    check("水平 yaw",   yaw,   0.0f, 0.1f);
    printf("    |q| = %.8f\n", sqrtf(a.q[0]*a.q[0]+a.q[1]*a.q[1]+a.q[2]*a.q[2]+a.q[3]*a.q[3]));

    printf("\n================ 测试 2：倾斜 30 度（俯仰） ================\n");
    /* 俯仰 30 度时，加速度计读数 = C^T*[0,0,1] = [-sin30, 0, cos30] */
    acc[0] = -sinf(30.0f * D2R);
    acc[1] = 0.0f;
    acc[2] =  cosf(30.0f * D2R);
    printf("    模拟读数: [%.4f, %.4f, %.4f]\n", acc[0], acc[1], acc[2]);
    ATT_Init(&a, acc);                       /* 用加速度计起步 */
    run(&a, gyro, acc, dt, 3.0f, &roll, &pitch, &yaw);
    check("俯仰角", pitch, 30.0f, 0.5f);
    check("滚转角", roll,   0.0f, 0.5f);

    printf("\n--- 再从 identity 起步（对比：不初始化会怎样） ---\n");
    a.q[0] = 1.0f; a.q[1] = a.q[2] = a.q[3] = 0.0f; a.beta = ATT_BETA_DEFAULT;
    a.gyro_bias[0] = a.gyro_bias[1] = a.gyro_bias[2] = 0.0f;
    run(&a, gyro, acc, dt, 3.0f, &roll, &pitch, &yaw);
    printf("    3 秒后 pitch = %.2f 度（应该已经收敛到 30）\n", pitch);
    check("从 identity 起步 3 秒后", pitch, 30.0f, 1.0f);

    printf("\n================ 测试 3：倾斜 45 度（滚转） ================\n");
    acc[0] = 0.0f;
    acc[1] =  sinf(45.0f * D2R);
    acc[2] =  cosf(45.0f * D2R);
    printf("    模拟读数: [%.4f, %.4f, %.4f]\n", acc[0], acc[1], acc[2]);
    ATT_Init(&a, acc);
    run(&a, gyro, acc, dt, 3.0f, &roll, &pitch, &yaw);
    check("滚转角", roll, 45.0f, 0.5f);
    check("俯仰角", pitch, 0.0f, 0.5f);

    printf("\n================ 测试 4：偏航积分（陀螺仪绕 z 转 90 度/秒） ================\n");
    ATT_Init(&a, (float[]){0.0f, 0.0f, 1.0f});
    for (int i = 0; i < 60; i++) {           /* 先让它稳定 */
        ATT_Update(&a, (float[]){0,0,0}, (float[]){0,0,1}, dt);
    }
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 90.0f * D2R;
    acc[0] = 0.0f; acc[1] = 0.0f; acc[2] = 1.0f;
    run(&a, gyro, acc, dt, 1.0f, &roll, &pitch, &yaw);
    check("1 秒后偏航角", yaw, 90.0f, 1.0f);

    printf("\n================ 测试 5：单位换算 + 轴映射 ================\n");
    {
        AttitudeState t;
        float g[3], w[3];
        /* GY-86 竖装：传感器 sx 朝上。4096 LSB = 1 g */
        int16_t raw_acc[3]  = { 4096, 0, 0 };
        /* 用宏算出"相当于 10 °/s 的原始值"，这样改量程系数也不会误报 */
        int16_t raw_gyro[3] = { 0, 0, (int16_t)(10.0f * ATT_GYRO_LSB_PER_DPS) };

        ATT_Init(&t, (float[]){0,0,1});
        t.gyro_bias[0] = t.gyro_bias[1] = t.gyro_bias[2] = 0.0f;

        ATT_RawToAcc(raw_acc, g);
        ATT_RawToGyro(&t, raw_gyro, w);

        printf("    raw_acc {4096,0,0} -> 机体+取反 = (%.3f, %.3f, %.3f)\n",
               g[0], g[1], g[2]);
        printf("    raw_gyro {0,0,%d} (10dps) -> 机体 = (%.6f, %.6f, %.6f)\n",
               (int)raw_gyro[2], w[0], w[1], w[2]);
        /* 映射: 机头=-sy, 右翼=+sz, 机腹=-sx; 加速度再取反
         *   s=(1,0,0) -> body=(0,0,-1) -> 取反 (0,0,+1)
         *   s=(0,0,10dps) -> body=(0, 10dps, 0) */
        check("加速度 z 分量(应 +1)", g[2],  1.0f, 1e-4f);
        check("加速度 x 分量(应  0)", g[0],  0.0f, 1e-4f);
        check("陀螺 右翼分量",        w[1], 10.0f * D2R, 1e-5f);
    }

    printf("\n================ 测试 6：陀螺零偏校准 ================\n");
    {
        GyroCalib c;
        AttitudeState t;
        int16_t raw[3];
        int i;
        /* 真实零偏 0.5 °/s，噪声 ±4 LSB */
        const int bias_lsb = (int)(0.5f * ATT_GYRO_LSB_PER_DPS + 0.5f);
        unsigned int seed = 12345u;

        ATT_Init(&t, (float[]){0,0,1});
        ATT_GyroCalibBegin(&c);
        for (i = 0; i < 1000; i++) {
            int n1, n2, n3;
            seed = seed * 1103515245u + 12345u;  n1 = (int)((seed >> 16) % 9) - 4;
            seed = seed * 1103515245u + 12345u;  n2 = (int)((seed >> 16) % 9) - 4;
            seed = seed * 1103515245u + 12345u;  n3 = (int)((seed >> 16) % 9) - 4;
            raw[0] = (int16_t)(bias_lsb + n1);
            raw[1] = (int16_t)(bias_lsb + n2);
            raw[2] = (int16_t)(bias_lsb + n3);
            ATT_GyroCalibAdd(&c, raw);
        }
        {
            int bad = ATT_GyroCalibEnd(&c, &t);
            printf("    真实零偏 = %.4f °/s\n", 0.5f);
            printf("    估出零偏 = %.4f °/s   (可信标志: %s)\n",
                   t.gyro_bias[2] * R2D, bad ? "不可信" : "可信");
            check("零偏估计", t.gyro_bias[2] * R2D, 0.5f, 0.05f);
            check("可信标志", (float)bad, 0.0f, 0.0f);
        }
    }

    printf("\n================ 测试 7：校准期间被碰（应判为不可信） ================\n");
    {
        GyroCalib c;
        AttitudeState t;
        int16_t raw[3];
        int i;
        ATT_Init(&t, (float[]){0,0,1});
        ATT_GyroCalibBegin(&c);
        for (i = 0; i < 1000; i++) {
            int extra = (i < 300) ? 300 : 0;   /* 前 300 个样本被打扰 */
            raw[0] = (int16_t)(extra);
            raw[1] = (int16_t)(extra);
            raw[2] = (int16_t)(extra);
            ATT_GyroCalibAdd(&c, raw);
        }
        {
            int bad = ATT_GyroCalibEnd(&c, &t);
            printf("    可信标志 = %d   (期望 1 = 不可信)\n", bad);
            check("能识别出被打扰", (float)bad, 1.0f, 0.0f);
        }
    }

    printf("\n==================================================\n");
    printf(g_fail ? "  有 %d 项没通过\n" : "  全部通过\n", g_fail);
    printf("==================================================\n");
    return g_fail ? 1 : 0;
}
