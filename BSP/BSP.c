#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "BSP.h"
#include "app.h"

/* ---------------------- 全局变量声明 -----------------------*/
// MS5611的校准数据
uint16_t ms5611_cal[6];

/* -------------------- 时钟初始化 -------------------- */
void BSP_InitClock(void)
{
		// 开启HSE（外部高速时钟）
    //RCC_HSEConfig(RCC_HSE_ON);
	
    // 等待HSE稳定
    //while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
    
    // 配置PLL，目标频率84MHz
    //RCC_PLLConfig(RCC_PLLSource_HSE, 8, 168, 2, 7);
    //RCC_PLLCmd(ENABLE);
    
    // 等待PLL就绪
    //while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    // 设置系统时钟源为PLL
    //RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    // 使能 PWM PPM USART I2C 时钟
    RCC_AHB1PeriphClockCmd(	PWM_GPIO_CLK | 
														PPM_GPIO_CLK | 
														USART1_GPIO_CLK |
														I2C_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(	PWM_TIM_CLK | 
														PPM_TIM_CLK , ENABLE);
		// 使能 USART1 时钟（APB2）
    RCC_APB2PeriphClockCmd(USART1_CLK, ENABLE);
		
		
		//使能 DWT 周期计数器（用于 SystemView 时间戳）
		// 使能内核调试跟踪单元（解锁 DWT）
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		// 清零计数器
		DWT->CYCCNT = 0;
		// 使能 CYCCNT 开始计数
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

























/* -------------------- PWM 底层初始化 -------------------- */
void BSP_PWM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 配置 PA6, PA7
    GPIO_InitStructure.GPIO_Pin = PWM_MOTOR1_PIN | PWM_MOTOR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PWM_MOTOR1_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(PWM_MOTOR1_PORT, GPIO_PinSource6, PWM_MOTOR1_AF);
    GPIO_PinAFConfig(PWM_MOTOR2_PORT, GPIO_PinSource7, PWM_MOTOR2_AF);

    // 配置 PB0, PB1
    GPIO_InitStructure.GPIO_Pin = PWM_MOTOR3_PIN | PWM_MOTOR4_PIN;
    GPIO_Init(PWM_MOTOR3_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(PWM_MOTOR3_PORT, GPIO_PinSource0, PWM_MOTOR3_AF);
    GPIO_PinAFConfig(PWM_MOTOR4_PORT, GPIO_PinSource1, PWM_MOTOR4_AF);
}

void BSP_PWM_TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRESCALER;
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;

    // 四个通道分别初始化
    TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
    TIM_Cmd(PWM_TIM, ENABLE);
}


/* -------------------- PWM 电机控制接口 -------------------- */
void BSP_PWM_SetMotor(uint16_t pulse_us)
{
    if (pulse_us < PWM_MIN_PULSE) pulse_us = PWM_MIN_PULSE;
    if (pulse_us > PWM_MAX_PULSE) pulse_us = PWM_MAX_PULSE;
    
    TIM_SetCompare1(PWM_TIM, pulse_us);
}





















/* -------------------- PPM 静态变量 -------------------- */
static volatile uint32_t ppm_last_capture = 0;
static volatile uint32_t ppm_channel_values[8] = {0};
static volatile uint8_t  ppm_channel_index = 0;

/* -------------------- PPM 底层初始化  -------------------- */
void BSP_PPM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
        
    // 配置PA1为输入模式（TIM2_CH2）
    GPIO_InitStructure.GPIO_Pin = PPM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(PPM_PORT, &GPIO_InitStructure);
    
    // 将PA1连接到TIM2
    GPIO_PinAFConfig(PPM_PORT, GPIO_PinSource1, PPM_AF);
}

void BSP_PPM_TIM_Config(void)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = PPM_PRESCALER;          // 84MHz/84=1MHz
    TIM_TimeBaseStructure.TIM_Period = PPM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数
    TIM_TimeBaseInit(PPM_TIM, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = PPM_CHANNEL;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(PPM_TIM, &TIM_ICInitStructure);

    //TIM_ITConfig(PPM_TIM, TIM_IT_CC2, ENABLE);

		
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(PPM_TIM, ENABLE);
}

/* -------------------- PPM 中断服务函数  -------------------- */
void TIM2_IRQHandler(void)
{
   
	OSIntEnter();
    if (TIM_GetITStatus(PPM_TIM, TIM_IT_CC2) != RESET)
    {
        uint32_t current = TIM_GetCapture2(PPM_TIM);
        uint32_t width;
        if (current >= ppm_last_capture)
            width = current - ppm_last_capture;
        else
            width = (PPM_PERIOD - ppm_last_capture) + current;

        if (width > 2000) {                     // 同步脉冲
            ppm_channel_index = 0;
            if (Sem_PPM_Frame != (OS_EVENT*)0) {
                OSSemPost(Sem_PPM_Frame);       // 通知新帧
            }
        } else if (width > 800 && width < 2200) {
            if (ppm_channel_index < 8) {
                ppm_channel_values[ppm_channel_index] = width;
                ppm_channel_index++;
            }
        }
        ppm_last_capture = current;
        TIM_ClearITPendingBit(PPM_TIM, TIM_IT_CC2);
    }
    OSIntExit();
}

uint16_t BSP_PPM_GetChannel(uint8_t ch)
{
    if (ch < 8) return (uint16_t)ppm_channel_values[ch];
    else return 0;
}




























/* -------------------- USART1 底层初始化 -------------------- */
void BSP_USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
        
    // 配置PA9(TX)和PA10(RX)
    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN | USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART1_PORT, &GPIO_InitStructure);
    
    // 引脚复用（PA9/10 -> USART1）
    GPIO_PinAFConfig(USART1_PORT, GPIO_PinSource9, USART1_AF);
    GPIO_PinAFConfig(USART1_PORT, GPIO_PinSource10, USART1_AF);
    
    // USART1 配置
    USART_InitStructure.USART_BaudRate = baudrate;											//波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;					//字长
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//停止位：1位
    USART_InitStructure.USART_Parity = USART_Parity_No;									//校验位：无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;			//使能发送和接收
    USART_Init(USART1, &USART_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}

/* -------------------- USART1 接口 -------------------- */
void BSP_USART1_SendChar(uint8_t ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}

void BSP_USART1_SendString(char *str)
{
    while (*str)
    {
        BSP_USART1_SendChar(*str++);
    }
}

void BSP_USART1_SendNumber(int32_t num)
{
    char buffer[12];
    char *ptr = buffer + 11;
    *ptr = '\0';
    
    // 处理0的情况
    if (num == 0)
    {
        BSP_USART1_SendChar('0');
        return;
    }
    
    // 处理负数
    int is_negative = 0;
    if (num < 0)
    {
        is_negative = 1;
        num = -num;
    }
    
    // 从低位到高位转换
    while (num > 0)
    {
        *--ptr = (num % 10) + '0';
        num /= 10;
    }
    
    // 添加负号
    if (is_negative)
    {
        *--ptr = '-';
    }
    
    // 发送字符串
    BSP_USART1_SendString(ptr);
}



























/* -------------------- I2C 底层初始化 -------------------- */
void BSP_I2C_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
        
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
}

/* -------------------- I2C 接口函数 -------------------- */
// I2C延时函数
static void I2C_Delay(void) {
    volatile uint32_t i;
    // 调到 500，让 SCL 降到极低频率（约几 kHz），确保万无一失
    for (i = 0; i < 500; i++); 
}
// 设置SDA为输入模式
static void SDA_IN(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);
}

// 设置SDA为输出模式
static void SDA_OUT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);
}

// 产生I2C起始信号
static void I2C_Start(void) {
    SDA_OUT();
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
}

// 产生I2C停止信号
static void I2C_Stop(void) {
    SDA_OUT();
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
}

// 等待应答信号-主机等待从机应答
static uint8_t I2C_Wait_Ack(void) {
    uint8_t timeout = 0;
    SDA_IN();
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    
    while(GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)) {
        if(timeout++ > I2C_TIMEOUT) {
            I2C_Stop();
            return 1;
        }
    }
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    return 0;
}

// 产生应答信号-主机应答从机
static void I2C_Ack(void) {
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    SDA_OUT();
    GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
}

// 不产生应答信号 - 主机不应答从机
static void I2C_NAck(void) {
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    SDA_OUT();
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
}

// I2C发送一个字节
static void I2C_Send_Byte(uint8_t data) {
    uint8_t i;
    SDA_OUT();
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    
    for(i = 0; i < 8; i++) {
        if(data & 0x80)
            GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
        else
            GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN);
        data <<= 1;
        I2C_Delay();
        GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
        I2C_Delay();
        GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
        I2C_Delay();
    }
}

// I2C读取一个字节
static uint8_t I2C_Read_Byte(uint8_t ack) {
    uint8_t i, data = 0;
    SDA_IN();
    
    for(i = 0; i < 8; i++) {
        GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
        I2C_Delay();
        data <<= 1;
        if(GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN))
            data |= 0x01;
        GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
        I2C_Delay();
    }
    
    if(ack) I2C_Ack();
    else I2C_NAck();
    
    return data;
}
// 写入数据函数 从机设备地址+寄存器地址
uint8_t I2C_WriteData(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    uint8_t i;
    
    I2C_Start();
    I2C_Send_Byte(devAddr << 1);  // 地址左移1位，最低位为0表示写操作
    if(I2C_Wait_Ack()) return 1;
    
    I2C_Send_Byte(regAddr);
    if(I2C_Wait_Ack()) return 1;
    
    for(i = 0; i < len; i++) {
        I2C_Send_Byte(data[i]);
        if(I2C_Wait_Ack()) return 1;
    }
    
    I2C_Stop();
    return 0;
}

// 读取数据函数
uint8_t I2C_ReadData(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    I2C_Start();
    I2C_Send_Byte(devAddr << 1);
    if(I2C_Wait_Ack()) return 1;
    
    I2C_Send_Byte(regAddr);
    if(I2C_Wait_Ack()) return 1;
    
    I2C_Start();
    I2C_Send_Byte((devAddr << 1) | 0x01);
    if(I2C_Wait_Ack()) return 1;
    
    while(len) {
        if(len == 1) {
            *data = I2C_Read_Byte(0);
        } else {
            *data = I2C_Read_Byte(1);
        }
        data++;
        len--;
    }
    
    I2C_Stop();
    return 0;
}

// 写入单个字节
uint8_t I2C_WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return I2C_WriteData(devAddr, regAddr, &data, 1);
}

// 读取单个字节
uint8_t I2C_ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    return I2C_ReadData(devAddr, regAddr, data, 1);
}



























/* -------------------- GY86 底层初始化 -------------------- */
// MPU6050初始化
uint8_t MPU6050_Init(void) {
    uint8_t check;
    
    // 检查设备ID 确认是MPU6050的设备ID
    if(I2C_ReadByte(MPU6050_ADDRESS, MPU6050_WHO_AM_I, &check)) return 1;
    if(check != 0x68) return 1;
    
    // 触发复位设备
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x80)) return 1;
    //Delay_ms(100);
    
    // 唤醒设备，使用PLL作为时钟源
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01)) return 1;
    // 设置加速度计±8g范围，陀螺仪±500°/s范围
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x10)) return 1;
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x08)) return 1;
    // 设置DLPF带宽44Hz，滤除高频噪声
    if(I2C_WriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x03)) return 1;
    
    return 0;
}

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
    // 设置增益±1.3Ga
    if(I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20)) return 1;
    // 设置连续测量模式
    if(I2C_WriteByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00)) return 1;
    
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
    mag[1] = (int16_t)((buffer[2] << 8) | buffer[3]);   // Mag Y  
    mag[2] = (int16_t)((buffer[4] << 8) | buffer[5]);   // Mag Z
    
    return 0;
}

// MS5611初始化
uint8_t MS5611_Init(void) {
    uint8_t i;
    uint8_t buffer[2];
    
    // 复位MS5611
    if(I2C_WriteByte(MS5611_ADDRESS, MS5611_RESET, 0x00)) return 1;

    
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
/* 返回值说明（方便定位问题）：
 *   0x00       全部成功
 *   0x01~0x09  MPU6050 失败，具体码见 gy86.c 里 MPU6050_Init 的注释
 *   0x21       使能 Bypass 失败
 *   0x31       HMC5883L（磁力计）失败  —— 不影响六轴姿态
 *   0x41       MS5611（气压计）失败    —— 不影响六轴姿态
 */
uint8_t BSP_GY86_Init(void) {
    uint8_t rc;

    rc = MPU6050_Init();          if(rc) return rc;
    rc = MPU6050_EnableBypass();  if(rc) return 0x20u + rc;
    rc = HMC5883L_Init();         if(rc) return 0x30u + rc;
    rc = MS5611_Init();           if(rc) return 0x40u + rc;

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










/* -------------------- BSP 总初始化 -------------------- */
void BSP_Init(void)
{
	BSP_InitClock();
	BSP_PWM_GPIO_Config();
	BSP_PWM_TIM_Config();
	BSP_PPM_GPIO_Config();
	BSP_PPM_TIM_Config();
	BSP_USART1_Init(9600);
	BSP_I2C_Init();
	BSP_GY86_Init();
}
