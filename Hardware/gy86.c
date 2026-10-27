//gy86.c
#include "gy86.h"
#include "i2c.h"
#include <string.h>

//MS5611的校准数据
static uint16_t ms5611_cal[6];

/* 毫秒级延时（用 DWT 周期计数器，BSP_InitClock 里已经使能） */
static void gy86_delay_ms(uint32_t ms)
{
    uint32_t clk = SystemCoreClock ? SystemCoreClock : 84000000u;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = ms * (clk / 1000u);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

/* 尝试把芯片从 sleep 里唤醒：写 0x00，然后回读验证，最多试 5 次
 * 返回 1 = 成功唤醒, 0 = 失败 */
static uint8_t mpu_wake_retry(void)
{
    uint8_t pwr = 0;
    uint8_t i;

    for(i = 0; i < 5; i++) {
        (void)I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
        gy86_delay_ms(20);

        if(I2C_ReadByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, &pwr)) continue;
        if((pwr & 0x40u) == 0u) {
            return 1;                    /* SLEEP 位清掉了 */
        }
    }
    return 0;
}

/* MPU6050初始化
 * 返回 0 表示成功；其它值表示失败在哪一步：
 *   1 = 读 WHO_AM_I 失败（I2C 无应答）
 *   2 = WHO_AM_I 不是 0x68
 *   5 = 写 ACCEL_CONFIG 失败
 *   6 = 写 GYRO_CONFIG 失败
 *   7 = 写 CONFIG(DLPF) 失败
 *   9 = 唤醒失败（写 0x6B 清不掉 SLEEP 位）
 *
 * ★ 策略：先"直接唤醒"，不依赖复位 ★
 *   原来复位后只等 100ms，不同 batch/clone 的复位时间可能更久，
 *   唤醒命令被丢掉，芯片停在 sleep：寄存器能读写、配置能写进去，
 *   但数据寄存器恒为 0（就是之前全 0 的现象）。
 *   现在改成先直接唤醒（大多数情况一次就好），失败才复位再来一次。
 */
uint8_t MPU6050_Init(void) {
    uint8_t id = 0;

    // 1) 确认芯片在总线上
    if(I2C_ReadByte(MPU6050_ADDRESS, MPU6050_WHO_AM_I, &id)) return 1;
    if(id != 0x68) return 2;

    // 2) 先直接唤醒（不复位，避免复位时序带来的不确定性）
    if(!mpu_wake_retry()) {
        // 3) 还不行：做一次软复位，等久一点再唤醒
        (void)I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x80);
        gy86_delay_ms(200);
        if(!mpu_wake_retry()) return 9;
    }

    // 4) 设置加速度计±8g，陀螺仪±500°/s，DLPF 44Hz
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10)) return 5;
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x08))  return 6;
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x03))       return 7;
    gy86_delay_ms(20);

    return 0;
}

/* 读 MPU6050 的任意一个寄存器（调试用）
 *   reg: 寄存器地址, 例如 0x75(WHO_AM_I)、0x6B(PWR_MGMT_1)
 *   返回 0 = 成功, 1 = I2C 失败 */
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *val) {
    return I2C_ReadByte(MPU6050_ADDRESS, reg, val);
}

// 使能Bypass模式，让主控MCU可以直接访问HMC5883L
uint8_t MPU6050_EnableBypass(void) {
    return I2C_WriteByte(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x02);
}

// 读取MPU6050加速度计和陀螺仪数据
uint8_t MPU6050_ReadAccelGyro(int16_t* accel, int16_t* gyro) {
    uint8_t buffer[14];
    
    if(I2C_ReadData(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, buffer, 14)) return 1;
    
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);   // Accel X
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);   // Accel Y
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);   // Accel Z
    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);    // Gyro X
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);  // Gyro Y
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]); // Gyro Z
    
    return 0;
}

// HMC5883L初始化
uint8_t HMC5883L_Init(void) {
    // 设置采样率75Hz，正常测量模式
    if(I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x78)) return 1;
    gy86_delay_ms(5);
    // 设置增益±1.3Ga
    if(I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20)) return 1;
    gy86_delay_ms(5);
    // 设置连续测量模式
    if(I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00)) return 1;
    gy86_delay_ms(10);          // 第一次测量完成需要一点时间

    return 0;
}

/* 读 HMC5883L 的任意一个寄存器（调试用）
 *   0x0A/0x0B/0x0C 是识别寄存器, 正常应该读出 'H'(0x48) '4'(0x34) '3'(0x33) */
uint8_t HMC5883L_ReadReg(uint8_t reg, uint8_t *val) {
    return I2C_ReadByte(HMC5883L_ADDRESS, reg, val);
}

// 读取HMC5883L磁力计数据
uint8_t HMC5883L_ReadMag(int16_t* mag) {
    uint8_t buffer[6];
    
    if(I2C_ReadData(HMC5883L_ADDRESS, HMC5883L_DATA_X_MSB, buffer, 6)) return 1;
    
    mag[0] = (int16_t)((buffer[0] << 8) | buffer[1]);   // Mag X
    /* ★ HMC5883L 的寄存器顺序是 X -> Z -> Y，不是 X -> Y -> Z！
     *     0x03/0x04 = X_MSB/LSB
     *     0x05/0x06 = Z_MSB/LSB   <-- 第二对是 Z
     *     0x07/0x08 = Y_MSB/LSB   <-- 第三对是 Y
     *   原来按 X/Y/Z 解析，Y 和 Z 装反了。 */
    mag[1] = (int16_t)((buffer[4] << 8) | buffer[5]);   // Mag Y
    mag[2] = (int16_t)((buffer[2] << 8) | buffer[3]);   // Mag Z
    
    return 0;
}

// MS5611初始化
uint8_t MS5611_Init(void) {
    uint8_t i;
    uint8_t buffer[2];
    
    // 复位MS5611
    if(I2C_WriteByte(MS5611_ADDRESS, MS5611_RESET, 0x00)) return 1;
    gy86_delay_ms(10);          // 复位后需要几毫秒才能读 PROM
    
    // 读取校准数据
    for(i = 0; i < 6; i++) {
        if(I2C_ReadData(MS5611_ADDRESS, 0xA2 + i * 2, buffer, 2)) return 1;
        ms5611_cal[i] = (buffer[0] << 8) | buffer[1];
    }
    
    return 0;
}

// 读取MS5611数据
uint8_t MS5611_ReadData(int32_t* temperature, int32_t* pressure) {
    uint8_t buffer[3];
    uint32_t d1 = 0, d2 = 0;
    int64_t dt, temp, off, sens, t2, off2, sens2;
    
    // 开始D1转换（压力）
    I2C_WriteByte(MS5611_ADDRESS, MS5611_CONV_D1, 0x00);
    //Delay_ms(10);
    
    // 读取D1
    I2C_WriteByte(MS5611_ADDRESS, MS5611_ADC_READ, 0x00);
    I2C_ReadData(MS5611_ADDRESS, 0x00, buffer, 3);
    d1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    
    // 开始D2转换（温度）
    I2C_WriteByte(MS5611_ADDRESS, MS5611_CONV_D2, 0x00);
    //Delay_ms(10);
    
    // 读取D2
    I2C_WriteByte(MS5611_ADDRESS, MS5611_ADC_READ, 0x00);
    I2C_ReadData(MS5611_ADDRESS, 0x00, buffer, 3);
    d2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    
    // 计算实际温度和压力值
    dt = d2 - ((int32_t)ms5611_cal[4] << 8); //实际温度与参考温度的差值
    temp = 2000 + ((dt * (int64_t)ms5611_cal[5]) >> 23); //初步计算出的温度值（单位：0.01°C）
    
	//压力偏移量，考虑了温度影响
    off = ((int64_t)ms5611_cal[1] << 16) + (((int64_t)ms5611_cal[3] * dt) >> 7);
    //压力灵敏度，考虑了温度影响
	sens = ((int64_t)ms5611_cal[0] << 15) + (((int64_t)ms5611_cal[2] * dt) >> 8);
    
    // 二阶温度补偿
    if(temp < 2000) {
        t2 = (dt * dt) >> 31;// 温度补偿项
        off2 = 5 * ((temp - 2000) * (temp - 2000)) >> 1;
        sens2 = off2 >> 1;
        
        if(temp < -1500) {
            off2 += 7 * (temp + 1500) * (temp + 1500);
            sens2 += 11 * ((temp + 1500) * (temp + 1500)) >> 1;
        }
        
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }
    
    *temperature = temp;
    *pressure = (((d1 * sens) >> 21) - off) >> 15;
    
    return 0;
}

// GY86整体初始化
uint8_t GY86_Init(void) {
    if(MPU6050_Init()) return 1;
    //Delay_ms(10);
    
    if(MPU6050_EnableBypass()) return 1;
    //Delay_ms(10);
    
    if(HMC5883L_Init()) return 1;
    //Delay_ms(10);
    
    if(MS5611_Init()) return 1;
    //Delay_ms(10);
    
    return 0;
}

// 读取所有传感器数据
uint8_t GY86_ReadAll(GY86_Data* data) {
    int16_t accel[3], gyro[3], mag[3];
    
    if(MPU6050_ReadAccelGyro(accel, gyro)) return 1;
    if(HMC5883L_ReadMag(mag)) return 1;
    if(MS5611_ReadData(&data->temperature, &data->pressure)) return 1;
    
    data->accel_x = accel[0]; data->accel_y = accel[1]; data->accel_z = accel[2];
    data->gyro_x = gyro[0]; data->gyro_y = gyro[1]; data->gyro_z = gyro[2];
    data->mag_x = mag[0]; data->mag_y = mag[1]; data->mag_z = mag[2];
    
    return 0;
}
