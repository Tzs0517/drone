#ifndef BSP_H
#define BSP_H

#include "stm32f4xx.h"
#include "ucos_ii.h"

/* ---------------------- 全局变量声明 -----------------------*/
// MS5611的校准数据
extern uint16_t ms5611_cal[6];

// 传感器数据结构体
typedef struct {
    int16_t accel_x, accel_y, accel_z;//3轴加速度计  
    int16_t gyro_x, gyro_y, gyro_z; //3轴陀螺仪 
    int16_t mag_x, mag_y, mag_z; //三轴磁力
    int32_t temperature; //温度传感器
    int32_t pressure; //气压
} GY86_Data;

/* 全局信号量/互斥量声明 */
extern OS_EVENT *Sem_PPM_Frame;     // PPM 帧完成信号量




/* ---------------------- 寄存器地址定义 -----------------------*/
// 传感器地址定义
#define MPU6050_ADDRESS         0x68
#define HMC5883L_ADDRESS        0x1E
#define MS5611_ADDRESS          0x77

// MPU6050寄存器定义
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_ACCEL_XOUT_H    0x3B

// HMC5883L寄存器定义
#define HMC5883L_CONFIG_A       0x00
#define HMC5883L_CONFIG_B       0x01
#define HMC5883L_MODE           0x02
#define HMC5883L_DATA_X_MSB     0x03

// MS5611寄存器定义
#define MS5611_RESET            0x1E
#define MS5611_CONV_D1          0x40
#define MS5611_CONV_D2          0x50
#define MS5611_ADC_READ         0x00








/* ======================== PWM 电机控制 (TIM3) ======================== */
// 引脚定义
#define PWM_MOTOR1_PIN         GPIO_Pin_6
#define PWM_MOTOR1_PORT        GPIOA
#define PWM_MOTOR1_AF          GPIO_AF_TIM3
#define PWM_MOTOR1_CHANNEL     1

#define PWM_MOTOR2_PIN         GPIO_Pin_7
#define PWM_MOTOR2_PORT        GPIOA
#define PWM_MOTOR2_AF          GPIO_AF_TIM3
#define PWM_MOTOR2_CHANNEL     2

#define PWM_MOTOR3_PIN         GPIO_Pin_0
#define PWM_MOTOR3_PORT        GPIOB
#define PWM_MOTOR3_AF          GPIO_AF_TIM3
#define PWM_MOTOR3_CHANNEL     3

#define PWM_MOTOR4_PIN         GPIO_Pin_1
#define PWM_MOTOR4_PORT        GPIOB
#define PWM_MOTOR4_AF          GPIO_AF_TIM3
#define PWM_MOTOR4_CHANNEL     4

#define PWM_TIM                TIM3
#define PWM_TIM_CLK            RCC_APB1Periph_TIM3
#define PWM_PRESCALER          83      // 84MHz/84 = 1MHz
#define PWM_PERIOD             19999   // 1MHz / 20000 = 50Hz
#define PWM_MIN_PULSE          1000    // us
#define PWM_MAX_PULSE          2000    // us

// 端口时钟
#define PWM_GPIO_CLK           (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)







/* ======================== PPM 接收机 (TIM2_CH2) ======================== */
//引脚定义
#define PPM_PIN                GPIO_Pin_1
#define PPM_PORT               GPIOA
#define PPM_AF                 GPIO_AF_TIM2

#define PPM_TIM                TIM2
#define PPM_TIM_CLK            RCC_APB1Periph_TIM2
#define PPM_CHANNEL            TIM_Channel_2
#define PPM_PRESCALER          83      // 84MHz/84 = 1MHz
#define PPM_PERIOD             65535


// 端口时钟
#define PPM_GPIO_CLK           RCC_AHB1Periph_GPIOA






/* ======================== USART1 蓝牙/调试 ======================== */
// 引脚定义
#define USART1_TX_PIN          GPIO_Pin_9
#define USART1_RX_PIN          GPIO_Pin_10
#define USART1_PORT            GPIOA
#define USART1_AF              GPIO_AF_USART1

#define USART1_CLK             RCC_APB2Periph_USART1
#define USART1_GPIO_CLK        RCC_AHB1Periph_GPIOA








/* ======================== I2C  (GY86) ======================== */
// 引脚定义
#define I2C_PORT                GPIOC
#define I2C_SCL_PIN             GPIO_Pin_0
#define I2C_SDA_PIN             GPIO_Pin_1
#define I2C_CLK                 RCC_AHB1Periph_GPIOC

#define I2C_TIMEOUT 100









/* -------------------- 初始化函数声明 -------------------- */
void BSP_Init(void);
void BSP_InitClock(void);
void BSP_PWM_GPIO_Config(void);
void BSP_PWM_TIM_Config(void);
void BSP_PPM_GPIO_Config(void);
void BSP_PPM_TIM_Config(void);
void BSP_USART1_Init(uint32_t baudrate);
void BSP_I2C_Init(void);
uint8_t BSP_GY86_Init(void);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_EnableBypass(void) ;
uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *val);   /* 调试用：读 MPU6050 任意寄存器 */
uint8_t HMC5883L_Init(void);
uint8_t HMC5883L_ReadReg(uint8_t reg, uint8_t *val);  /* 调试用：读磁力计寄存器 */
uint8_t MS5611_Init(void);





/* ----------------------接口函数声明 -----------------------*/
// PWM
void BSP_PWM_SetMotor(uint16_t pulse_us);

// PPM
uint16_t BSP_PPM_GetChannel(uint8_t ch);

// USART1
void BSP_USART1_SendChar(uint8_t ch);
void BSP_USART1_SendString(char *str);
void BSP_USART1_SendNumber(int32_t num);

// I2C 
void I2C_Delay(void);
void SDA_IN(void);
void SDA_OUT(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Wait_Ack(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_Send_Byte(uint8_t data);
uint8_t I2C_Read_Byte(uint8_t ack);
uint8_t I2C_WriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C_ReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
uint8_t I2C_WriteData(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) ;
uint8_t I2C_ReadData(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) ;

// GY86
uint8_t MPU6050_ReadAccelGyro(int16_t* accel, int16_t* gyro);
uint8_t HMC5883L_ReadMag(int16_t* mag);
uint8_t MS5611_ReadData(int32_t* temperature, int32_t* pressure) ;
uint8_t GY86_ReadAll(GY86_Data* data);







#endif
