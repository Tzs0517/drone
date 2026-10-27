#include "stm32f4xx.h"
#include "ucos_ii.h"
#include <os_trace.h>
#include <math.h>
#include "BSP.h"
#include "app.h"
#include "attitude.h"

//------------------ 信号量 / 消息队列（示例） ----------
OS_EVENT *Sem_PPM_Frame ;    		// PPM帧完成信号量
OS_EVENT *Sem_SensorData;     // 保护传感器数据互斥量


/* -------------------- 全局传感器数据（任务间共享） -------------------- */
GY86_Data g_sensor_data;         // 存储最新传感器数据

/* -------------------- 姿态（任务间共享） -------------------- */
AttitudeState g_att;             // Madgwick 姿态状态

/* -------------------- 遥测：实测采样周期 -------------------- */
static volatile uint32_t g_dt_us   = 0;   /* 最近 100 次采样的平均周期(us) */
static volatile uint32_t g_rate_hz = 0;   /* 由 dt 推算的实际采样率 */
/* 陀螺仪"纯积分角度"（单位：度），从上电开始累加。
 * 用途：跟加速度计算出的倾角对比，就能判断陀螺量程对不对。
 *
 * ★ 注意：必须用 float 累加！
 *   之前写成 (int32_t)(gyro*dt*k) 累加，慢速转动时每步增量只有 0.5，
 *   强转成整数会截断成 0，累加器永远不增长（打印出来一直是 0）。 */
static volatile float    g_gi_deg[3] = {0.0f, 0.0f, 0.0f};

/* 开机时的初始倾角（度）。
 * 陀螺积分只能反映"从开机到现在转了多少"，如果开机时机身本来就歪着，
 * 直接拿积分和绝对角度比就会对不上。所以记住开机基准，改用【增量】比较。 */
static volatile float    g_boot_pitch_deg = 0.0f;
static volatile float    g_boot_roll_deg  = 0.0f;


/* --------------------------------------------------------------------------
 * 采样参数
 *   ATT_SAMPLE_TICK : 采样周期（tick, 1 tick = 1 ms）5 tick = 5 ms = 200 Hz
 *   ATT_CALIB_N     : 陀螺零偏校准采样点数, 600 点 * 2 ms ≈ 1.2 秒
 * -------------------------------------------------------------------------- */
#define ATT_SAMPLE_TICK      5u
#define ATT_CALIB_N          600u
#define ATT_SLOW_DIV         20u      /* 每 20 次采样做一次"慢"读取 (100 ms) */


/* 把角度(弧度)按 0.01 度为单位打印 */
static int32_t rad_to_cdeg(float rad)
{
    return (int32_t)(rad * 57.2957795f * 100.0f);
}

/* 打印一个字节的十六进制 */
static void send_hex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";
    BSP_USART1_SendChar('0');
    BSP_USART1_SendChar('x');
    BSP_USART1_SendChar((uint8_t)hx[(v >> 4) & 0x0Fu]);
    BSP_USART1_SendChar((uint8_t)hx[v & 0x0Fu]);
}


/* --------------------------------------------------------------------------
 * 传感器自检 + 初始化
 *
 * 返回 BSP_GY86_Init 的返回码：
 *   0x00      全部成功
 *   0x01~0x09 MPU6050 失败（具体是哪一步见 gy86.c 里 MPU6050_Init 的注释）
 *   0x21      Bypass 失败
 *   0x31      磁力计失败（不影响六轴姿态）
 *   0x41      气压计失败（不影响六轴姿态）
 * -------------------------------------------------------------------------- */
static uint8_t SensorInitAndDiagnose(void)
{
    uint8_t rc, id = 0, pwr = 0, aconf = 0, gconf = 0, init_rc;
    uint8_t all_zero = 1;
    int16_t ra[3] = {0, 0, 0};
    int16_t rg[3] = {0, 0, 0};
    uint8_t i;

    BSP_USART1_SendString("\r\n[DIAG] ===== GY-86 self test =====\r\n");

    /* 1) 总线上到底有没有 MPU6050
     *    ★ 加一点重试：传感器上电后需要几毫秒才应答，
     *      实测第一次读偶尔会 rc=1（误报"无应答"）。 */
    for (i = 0; i < 20u; i++) {
        rc = MPU6050_ReadReg(0x75, &id);          /* 0x75 = WHO_AM_I */
        if (rc == 0 && id == 0x68) {
            break;
        }
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
    BSP_USART1_SendString("[DIAG] WHO_AM_I   rc=");
    BSP_USART1_SendNumber(rc);
    BSP_USART1_SendString("  id=");
    send_hex8(id);
    BSP_USART1_SendString("  (tries=");
    BSP_USART1_SendNumber((int32_t)(i + 1u));
    BSP_USART1_SendString(")\r\n");
    if (rc) {
        BSP_USART1_SendString("[DIAG] ** I2C 无应答: 查供电/接线/上拉电阻 **\r\n");
    } else if (id == 0x68) {
        BSP_USART1_SendString("[DIAG] (OK)\r\n");
    } else {
        BSP_USART1_SendString("[DIAG] ** 不是 0x68, 芯片型号不对 **\r\n");
    }

    /* 2) 唤醒探针：直接看"写 0x6B 到底有没有生效" */
    BSP_USART1_SendString("[DIAG] --- wake probe ---\r\n");
    (void)MPU6050_ReadReg(0x6B, &pwr);            /* 0x6B = PWR_MGMT_1 */
    BSP_USART1_SendString("[DIAG] PWR before        = "); send_hex8(pwr);
    BSP_USART1_SendString((pwr & 0x40u) ? "  (sleep=1)\r\n" : "  (sleep=0)\r\n");

    for (i = 0; i < 2; i++) {
        (void)I2C_WriteByte(0x68, 0x6B, 0x00);     /* 写: 清 SLEEP, 用内部时钟 */
        OSTimeDlyHMSM(0, 0, 0, 30);
        (void)MPU6050_ReadReg(0x6B, &pwr);
        BSP_USART1_SendString("[DIAG] after write 0x00  = "); send_hex8(pwr);
        BSP_USART1_SendString((pwr & 0x40u) ? "  (still sleeping!)\r\n" : "  (awake)\r\n");
    }

    /* 3) 正式初始化 */
    init_rc = BSP_GY86_Init();
    BSP_USART1_SendString("[DIAG] GY86_Init = ");
    BSP_USART1_SendNumber(init_rc);
    if (init_rc == 0) {
        BSP_USART1_SendString("  (全部成功)\r\n");
    } else if (init_rc <= 0x09u) {
        BSP_USART1_SendString("  ** MPU6050 初始化失败, 对照 gy86.c 的注释看是哪一步 **\r\n");
    } else {
        BSP_USART1_SendString("  (MPU6050 正常, 磁力计/气压计有问题)\r\n");
    }

    /* 4) 电源管理寄存器: bit6 = SLEEP */
    rc = MPU6050_ReadReg(0x6B, &pwr);             /* 0x6B = PWR_MGMT_1 */
    BSP_USART1_SendString("[DIAG] PWR_MGMT_1 rc=");
    BSP_USART1_SendNumber(rc);
    BSP_USART1_SendString("  val=");
    send_hex8(pwr);
    if (pwr & 0x40u) {
        BSP_USART1_SendString("  ** SLEEP 位还是 1, 芯片在睡觉 **\r\n");
    } else {
        BSP_USART1_SendString("  (已唤醒)\r\n");
    }

    /* 4) 量程配置有没有写进去 */
    (void)MPU6050_ReadReg(0x1C, &aconf);          /* 0x1C = ACCEL_CONFIG */
    BSP_USART1_SendString("[DIAG] ACCEL_CONFIG=");
    send_hex8(aconf);
    BSP_USART1_SendString(" (期望 0x10=±8g)   ");
    (void)MPU6050_ReadReg(0x1B, &gconf);          /* 0x1B = GYRO_CONFIG */
    BSP_USART1_SendString("GYRO_CONFIG=");
    send_hex8(gconf);
    BSP_USART1_SendString(" (期望 0x08=±500dps)\r\n");

    /* 5) 磁力计识别寄存器: 0x0A/0x0B/0x0C 正常是 'H'(0x48) '4'(0x34) '3'(0x33) */
    {
        uint8_t m1 = 0, m2 = 0, m3 = 0, r1, r2, r3;

        r1 = HMC5883L_ReadReg(0x0A, &m1);
        r2 = HMC5883L_ReadReg(0x0B, &m2);
        r3 = HMC5883L_ReadReg(0x0C, &m3);

        BSP_USART1_SendString("[DIAG] HMC5883L ID: ");
        send_hex8(m1); BSP_USART1_SendString(" ");
        send_hex8(m2); BSP_USART1_SendString(" ");
        send_hex8(m3);
        if (r1 || r2 || r3) {
            BSP_USART1_SendString("  ** 读不到: Bypass 没生效 / 磁力计不在 **\r\n");
        } else if (m1 == 0x48u && m2 == 0x34u && m3 == 0x33u) {
            BSP_USART1_SendString("  (应该读到 0x48 0x34 0x33 = 'H43', OK)\r\n");
        } else {
            BSP_USART1_SendString("  ** 不是 'H43' **\r\n");
        }
    }

    /* 6) 连读三组原始数据，看有没有在变 */
    BSP_USART1_SendString("[DIAG] raw samples:\r\n");
    for (i = 0; i < 3; i++) {
        rc = MPU6050_ReadAccelGyro(ra, rg);
        if (ra[0] || ra[1] || ra[2] || rg[0] || rg[1] || rg[2]) {
            all_zero = 0;
        }
        BSP_USART1_SendString("    rc=");
        BSP_USART1_SendNumber(rc);
        BSP_USART1_SendString("  a=(");
        BSP_USART1_SendNumber(ra[0]); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(ra[1]); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(ra[2]); BSP_USART1_SendString(")  g=(");
        BSP_USART1_SendNumber(rg[0]); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(rg[1]); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(rg[2]); BSP_USART1_SendString(")\r\n");
        OSTimeDlyHMSM(0, 0, 0, 50);
    }

    if (all_zero) {
        BSP_USART1_SendString("[DIAG] ** 数据仍然全 0: 芯片没在采样 **\r\n");
        return init_rc ? init_rc : 9u;
    }

    BSP_USART1_SendString("[DIAG] ===== end =====\r\n\r\n");

    /* ★ 关键：只要数据是活的，就算就绪 ★
     * 不能用 BSP_GY86_Init 的返回码来决定——I2C 的 ACK 检查可能误报，
     * 但寄存器其实写成功了（之前配置明明是 0x10/0x08，却被判成失败）。 */
    BSP_USART1_SendString("[DIAG] data is alive -> attitude enabled\r\n\r\n");
    return 0;
}


/* --------------------------------------------------------------------------
 * 上电初始化：陀螺零偏校准 + 用加速度计定初始姿态
 *
 * 必须在电机不转、飞机静止时执行。
 * -------------------------------------------------------------------------- */
static void AttitudeStartup(void)
{
    GyroCalib gc;
    int16_t   raw_acc[3] = {0, 0, 0};
    int16_t   raw_gyro[3] = {0, 0, 0};
    float     acc_g[3] = {0.0f, 0.0f, 1.0f};
    uint16_t  i;
    int       bad;

    BSP_USART1_SendString("\r\n[ATT] keep still, calibrating gyro...\r\n");

    /* ---- 第一步：读一组加速度，定初始姿态（俯仰/滚转），偏航设 0 ---- */
    for (i = 0; i < 10u; i++) {
        if (MPU6050_ReadAccelGyro(raw_acc, raw_gyro) == 0) {
            break;
        }
        OSTimeDly(5);
    }
    ATT_RawToAcc(raw_acc, acc_g);
    ATT_Init(&g_att, acc_g);

    /* 打印初始读数，用来检查安装方向对不对 */
    BSP_USART1_SendString("[ATT] init raw acc = ");
    BSP_USART1_SendNumber(raw_acc[0]); BSP_USART1_SendString(", ");
    BSP_USART1_SendNumber(raw_acc[1]); BSP_USART1_SendString(", ");
    BSP_USART1_SendNumber(raw_acc[2]); BSP_USART1_SendString("\r\n");

    /* 顺便算出静止倾角，检查是不是真的水平（以及安装方向对不对）
     *   期望：水平放置时 pitch≈0, roll≈0，且 az 接近 +4096
     *   如果打印出来是 80 多度，说明板子是立着的，或者轴映射不对 */
    {
        float an = sqrtf(acc_g[0] * acc_g[0] + acc_g[1] * acc_g[1] + acc_g[2] * acc_g[2]);
        if (an > 0.1f) {
            float ax = acc_g[0] / an, ay = acc_g[1] / an, az = acc_g[2] / an;
            float pitch = asinf(ax >  1.0f ?  1.0f : (ax < -1.0f ? -1.0f : -ax));
            float roll  = atan2f(ay, az);

            BSP_USART1_SendString("[ATT] static tilt: pitch=");
            BSP_USART1_SendNumber((int32_t)(pitch * 57.2957795f));
            BSP_USART1_SendString(" deg, roll=");
            BSP_USART1_SendNumber((int32_t)(roll * 57.2957795f));
            BSP_USART1_SendString(" deg\r\n");

            /* 记住开机基准：后面用【增量】跟陀螺积分对比 */
            g_boot_pitch_deg = pitch * 57.2957795f;
            g_boot_roll_deg  = roll  * 57.2957795f;

            if (fabsf(pitch * 57.2957795f) > 45.0f || fabsf(roll * 57.2957795f) > 45.0f) {
                BSP_USART1_SendString("[ATT] ** 机身不水平(或轴映射不对), 请水平放置后重启 **\r\n");
            }
        }
    }

    /* ---- 第二步：陀螺零偏校准（静止取平均） ---- */
    ATT_GyroCalibBegin(&gc);
    for (i = 0; i < ATT_CALIB_N; i++) {
        if (MPU6050_ReadAccelGyro(raw_acc, raw_gyro) == 0) {
            ATT_GyroCalibAdd(&gc, raw_gyro);
        }
        OSTimeDly(2);
    }
    bad = ATT_GyroCalibEnd(&gc, &g_att);

    /* 打印零偏（单位 0.01 度/秒） */
    BSP_USART1_SendString("[ATT] gyro bias = ");
    BSP_USART1_SendNumber((int32_t)(g_att.gyro_bias[0] * 57.2957795f * 100.0f));
    BSP_USART1_SendString(", ");
    BSP_USART1_SendNumber((int32_t)(g_att.gyro_bias[1] * 57.2957795f * 100.0f));
    BSP_USART1_SendString(", ");
    BSP_USART1_SendNumber((int32_t)(g_att.gyro_bias[2] * 57.2957795f * 100.0f));
    BSP_USART1_SendString("  (0.01 deg/s)  ");

    if (bad) {
        BSP_USART1_SendString("** UNTRUSTED, please restart and keep still **\r\n");
    } else {
        BSP_USART1_SendString("OK\r\n");
    }
}


/* -------------------- 任务创建函数 -------------------- */
void AppTaskCreate(void)
{
    OSTaskCreate(AppTaskStart,
                 (void *)0,
                 &AppTaskStartStk[APP_CFG_STARTUP_TASK_STK_SIZE - 1u],
                 APP_CFG_STARTUP_TASK_PRIO);
}

/* -------------------- 启动任务 -------------------- */
static void AppTaskStart(void *p_arg)
{
    INT8U err;
    (void)p_arg;
		OS_CPU_SR cpu_sr;

		SystemCoreClockUpdate();
    OS_CPU_SysTickInitFreq(SystemCoreClock);   // 初始化 UCOS 系统时钟节拍

		BSP_InitClock();
	
		BSP_USART1_Init(9600);
		BSP_I2C_Init();
		BSP_PWM_GPIO_Config();
		BSP_PWM_TIM_Config();
		BSP_PPM_GPIO_Config();
		BSP_PPM_TIM_Config();
		OS_TRACE_INIT();

    /* ---- 传感器自检 + 初始化 ---- */
    {
        uint8_t irc = SensorInitAndDiagnose();

        if (irc != 0u) {
            /* 传感器没数据：姿态解算没有意义，直接跳过 */
            BSP_USART1_SendString("[ATT] no sensor data -> attitude disabled\r\n");
        } else {
            /* ---- 姿态模块初始化（阻塞约 1.5 秒，期间请保持静止） ---- */
            AttitudeStartup();
        }
    }
	
    // 1. 创建同步对象
    Sem_PPM_Frame = OSSemCreate(0);            // 初始为 0
    Sem_SensorData = OSSemCreate(1);

    // 2. 创建其他任务（必须进入临界区）
    OS_ENTER_CRITICAL();

    OSTaskCreate(AppTaskMotor, (void *)0,
                 &AppTaskMotorStk[APP_CFG_MOTOR_TASK_STK_SIZE - 1u],
                 APP_CFG_MOTOR_TASK_PRIO);
		OSTaskNameSet(APP_CFG_MOTOR_TASK_PRIO, "Motor", &err);


    OSTaskCreate(AppTaskGY86, (void *)0,
                 &AppTaskGY86Stk[APP_CFG_GY86_TASK_STK_SIZE - 1u],
                 APP_CFG_GY86_TASK_PRIO);
		OSTaskNameSet(APP_CFG_GY86_TASK_PRIO, "GY86", &err);

    OSTaskCreate(AppTaskBlueTooth, (void *)0,
                 &AppTaskBlueToothStk[APP_CFG_BLUETOOTH_TASK_STK_SIZE - 1u],
                 APP_CFG_BLUETOOTH_TASK_PRIO);
		OSTaskNameSet(APP_CFG_BLUETOOTH_TASK_PRIO, "Blue", &err);

    OS_EXIT_CRITICAL();
		
		TIM_ITConfig(PPM_TIM, TIM_IT_CC2, ENABLE);


    // 启动任务自我删除
    OSTaskDel(OS_PRIO_SELF);
}


/* -------------------- Motor 任务 -------------------- */
static void AppTaskMotor(void *p_arg)
{
    INT8U err;
    uint16_t throttle;
		//OS_CPU_SR cpu_sr;

    (void)p_arg;
		

    while (1) {
        // 等待 PPM 帧完成信号量
        OSSemPend(Sem_PPM_Frame, 0, &err);
				if (err != OS_ERR_NONE) {
            continue;
        }

        // 读取通道 8（索引 7）的值，该函数内部已用临界区保护
        throttle = BSP_PPM_GetChannel(7);

        // 限幅（安全范围）
        if (throttle < PWM_MIN_PULSE) throttle = PWM_MIN_PULSE;
        if (throttle > PWM_MAX_PULSE) throttle = PWM_MAX_PULSE;

        // 设置电机 PWM
        BSP_PWM_SetMotor(throttle);
    }
}

/* -------------------- GY86 读取 + 姿态解算任务 -------------------- */
static void AppTaskGY86(void *p_arg)
{
    INT8U err;
    int16_t   raw_acc[3], raw_gyro[3], raw_mag[3];
    float     acc_g[3], gyro_rad[3];
    uint32_t  last_cyc;
    uint16_t  slow_cnt = 0;
    uint32_t  dt_acc = 0;
    uint16_t  dt_cnt = 0;

    (void)p_arg;

    last_cyc = DWT->CYCCNT;

    while (1) {
        uint32_t now = DWT->CYCCNT;
        float    dt  = (float)(now - last_cyc) / (float)SystemCoreClock;

        /* ---- 快速路径：只读 MPU6050（14 字节），200 Hz ---- */
        if (MPU6050_ReadAccelGyro(raw_acc, raw_gyro) == 0) {

            last_cyc = now;                 /* dt = 两次读取之间的真实间隔 */

            if (dt <= 0.0f || dt > 0.05f) {
                dt = (float)ATT_SAMPLE_TICK * 0.001f;   /* 异常时回退到名义周期 */
            }

            /* 遥测：统计真实采样周期（用来验证 dt 和采样率） */
            dt_acc += (uint32_t)(dt * 1000000.0f);
            if (++dt_cnt >= 100u) {
                g_dt_us   = dt_acc / dt_cnt;
                g_rate_hz = (g_dt_us > 0u) ? (1000000u / g_dt_us) : 0u;
                dt_acc = 0;
                dt_cnt = 0;
            }

            /* 原始值 -> 物理单位 */
            ATT_RawToAcc(raw_acc, acc_g);
            ATT_RawToGyro(&g_att, raw_gyro, gyro_rad);

            /* 姿态解算 */
            ATT_Update(&g_att, gyro_rad, acc_g, dt);

            /* 遥测：累积"纯陀螺积分角度"（度），不经过任何修正
             * 用 float 累加，避免小增量被整数截断吃掉 */
            g_gi_deg[0] += gyro_rad[0] * dt * 57.2957795f;   /* rad -> deg */
            g_gi_deg[1] += gyro_rad[1] * dt * 57.2957795f;
            g_gi_deg[2] += gyro_rad[2] * dt * 57.2957795f;

            /* 更新共享区（给显示任务用） */
            OSSemPend(Sem_SensorData, 0, &err);
            g_sensor_data.accel_x = raw_acc[0];
            g_sensor_data.accel_y = raw_acc[1];
            g_sensor_data.accel_z = raw_acc[2];
            g_sensor_data.gyro_x  = raw_gyro[0];
            g_sensor_data.gyro_y  = raw_gyro[1];
            g_sensor_data.gyro_z  = raw_gyro[2];
            OSSemPost(Sem_SensorData);
        }

        /* ---- 慢速路径：每 100 ms 读一次磁力计（给后面九轴用；气压计暂时不读） ---- */
        if (++slow_cnt >= ATT_SLOW_DIV) {
            slow_cnt = 0;

            if (HMC5883L_ReadMag(raw_mag) == 0) {
                OSSemPend(Sem_SensorData, 0, &err);
                g_sensor_data.mag_x = raw_mag[0];
                g_sensor_data.mag_y = raw_mag[1];
                g_sensor_data.mag_z = raw_mag[2];
                OSSemPost(Sem_SensorData);
            }
        }

        OSTimeDly(ATT_SAMPLE_TICK);
    }
}

/* -------------------- 蓝牙发送任务 -------------------- */
/* 输出格式（CSV，方便直接贴进 Excel）：
 *   roll,pitch,yaw,ax,ay,az,gx,gy,gz
 *   角度单位 0.01 度；ax..gz 是原始 LSB 值
 */
static void AppTaskBlueTooth(void *p_arg)
{
    INT8U err;
    GY86_Data send_data;
    uint32_t counter = 0;
    float roll, pitch, yaw;

    (void)p_arg;

    while (1) {
        // 从共享数据区复制最新数据
        OSSemPend(Sem_SensorData, 0, &err);
        send_data = g_sensor_data;
        OSSemPost(Sem_SensorData);

        // 取姿态（弧度）
        ATT_GetEuler(&g_att, &roll, &pitch, &yaw);

        // 每 20 行打印一次遥测（用来核对 dt 和采样率）
        if (counter > 0u && (counter % 20u) == 0u) {
            BSP_USART1_SendString("[STAT] sysclk=");
            BSP_USART1_SendNumber((int32_t)(SystemCoreClock / 1000000u));
            BSP_USART1_SendString("MHz dt=");
            BSP_USART1_SendNumber((int32_t)g_dt_us);
            BSP_USART1_SendString("us rate=");
            BSP_USART1_SendNumber((int32_t)g_rate_hz);
            BSP_USART1_SendString("Hz\r\n");

            /* ---- 陀螺量程自检 ----
             * 用【加速度计直接反解的俯仰】做基准（瞬时、绝对可信），
             * 不用滤波器输出（运动中会滞后，比值会失真）。
             *
             *   accPitch : 加速度计反解的俯仰角（0.1度）
             *   gyroY    : 陀螺纯积分（0.1度）
             *   ratio    : 100 * accPitch增量 / gyroY
             *              100 左右 = 量程正确
             *              400 左右 = 偏小 4 倍
             *              200 左右 = 偏小 2 倍 */
            {
                int16_t raw_a[3];
                float   a[3];
                float   an, p_acc, dp_f;
                int32_t dp, gy_, ratio;

                raw_a[0] = send_data.accel_x;
                raw_a[1] = send_data.accel_y;
                raw_a[2] = send_data.accel_z;
                ATT_RawToAcc(raw_a, a);

                an    = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
                p_acc = 0.0f;
                if (an > 0.1f) {
                    float t = -a[0] / an;
                    if (t >  1.0f) t =  1.0f;
                    if (t < -1.0f) t = -1.0f;
                    p_acc = asinf(t) * 57.2957795f;
                }

                dp_f  = p_acc - g_boot_pitch_deg;
                dp    = (int32_t)(dp_f * 10.0f);
                gy_   = (int32_t)(g_gi_deg[1] * 10.0f);
                ratio = ((gy_ > 5) || (gy_ < -5))
                        ? (int32_t)(100.0f * (dp_f * 10.0f) / (float)gy_)
                        : 0;

                BSP_USART1_SendString("[STAT] accPitch=");
                BSP_USART1_SendNumber(dp);
                BSP_USART1_SendString(" gyroY=");
                BSP_USART1_SendNumber(gy_);
                BSP_USART1_SendString(" ratio=");
                BSP_USART1_SendNumber(ratio);
                BSP_USART1_SendString("\r\n");
            }

            /* 磁力计原始值：水平转一圈，x/y 应该画出个圆；
             * 如果某一轴基本不变或数值卡在 ±2048，说明量程饱和/接线有问题 */
            BSP_USART1_SendString("[MAG] x=");
            BSP_USART1_SendNumber(send_data.mag_x);
            BSP_USART1_SendString(" y=");
            BSP_USART1_SendNumber(send_data.mag_y);
            BSP_USART1_SendString(" z=");
            BSP_USART1_SendNumber(send_data.mag_z);
            BSP_USART1_SendString("\r\n");
        }

        // 每 50 行打印一次表头
        if (counter % 50u == 0u) {
            BSP_USART1_SendString("roll,pitch,yaw,ax,ay,az,gx,gy,gz,gix,giy,giz\r\n");
        }
        counter++;

        BSP_USART1_SendNumber(rad_to_cdeg(roll));   BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(rad_to_cdeg(pitch));  BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(rad_to_cdeg(yaw));    BSP_USART1_SendString(",");

        BSP_USART1_SendNumber(send_data.accel_x);   BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(send_data.accel_y);   BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(send_data.accel_z);   BSP_USART1_SendString(",");

        BSP_USART1_SendNumber(send_data.gyro_x);    BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(send_data.gyro_y);    BSP_USART1_SendString(",");
        BSP_USART1_SendNumber(send_data.gyro_z);    BSP_USART1_SendString(",");

        /* 陀螺纯积分角度（0.1 度）—— 用来和上面的角度对比，判断陀螺量程 */
        BSP_USART1_SendNumber((int32_t)(g_gi_deg[0] * 10.0f)); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber((int32_t)(g_gi_deg[1] * 10.0f)); BSP_USART1_SendString(",");
        BSP_USART1_SendNumber((int32_t)(g_gi_deg[2] * 10.0f));
        BSP_USART1_SendString("\r\n");

        // 50ms + 发送耗时(约60ms) -> 实际约 9 Hz
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}
