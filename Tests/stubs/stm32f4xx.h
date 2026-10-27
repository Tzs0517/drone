/* 桩头文件：仅用于在 PC 上做语法检查，不参与实际编译 */
#ifndef STM32F4XX_STUB_H
#define STM32F4XX_STUB_H

#include <stdint.h>
#include <stddef.h>

typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

typedef struct { int dummy; } GPIO_TypeDef;
extern GPIO_TypeDef *GPIOA, *GPIOB, *GPIOC;

typedef struct {
    uint32_t GPIO_Pin;
    int GPIO_Mode;
    int GPIO_OType;
    int GPIO_PuPd;
    int GPIO_Speed;
    int GPIO_AF;
} GPIO_InitTypeDef;

#define GPIO_Pin_0   (1u<<0)
#define GPIO_Pin_1   (1u<<1)
#define GPIO_Pin_2   (1u<<2)
#define GPIO_Pin_3   (1u<<3)
#define GPIO_Pin_4   (1u<<4)
#define GPIO_Pin_5   (1u<<5)
#define GPIO_Pin_6   (1u<<6)
#define GPIO_Pin_7   (1u<<7)
#define GPIO_Pin_8   (1u<<8)
#define GPIO_Pin_9   (1u<<9)
#define GPIO_Pin_10  (1u<<10)
#define GPIO_Pin_11  (1u<<11)
#define GPIO_Pin_12  (1u<<12)
#define GPIO_Pin_13  (1u<<13)
#define GPIO_Pin_14  (1u<<14)
#define GPIO_Pin_15  (1u<<15)

#define GPIO_Mode_IN        0
#define GPIO_Mode_OUT       1
#define GPIO_Mode_AF        2
#define GPIO_Mode_AN        3
#define GPIO_OType_PP       0
#define GPIO_OType_OD       1
#define GPIO_PuPd_NOPULL    0
#define GPIO_PuPd_UP        1
#define GPIO_PuPd_DOWN      2
#define GPIO_Speed_2MHz     0
#define GPIO_Speed_25MHz    1
#define GPIO_Speed_50MHz    2
#define GPIO_Speed_100MHz   3

#define GPIO_AF_TIM2        1
#define GPIO_AF_TIM3        2
#define GPIO_AF_USART1      7

void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void GPIO_SetBits(GPIO_TypeDef *port, uint16_t pins);
void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t pins);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *port, uint16_t pin);

void RCC_AHB1PeriphClockCmd(uint32_t periph, FunctionalState state);
void RCC_APB1PeriphClockCmd(uint32_t periph, FunctionalState state);
void RCC_APB2PeriphClockCmd(uint32_t periph, FunctionalState state);

#define RCC_AHB1Periph_GPIOA   (1u<<0)
#define RCC_AHB1Periph_GPIOB   (1u<<1)
#define RCC_AHB1Periph_GPIOC   (1u<<2)
#define RCC_APB1Periph_TIM2    (1u<<0)
#define RCC_APB1Periph_TIM3    (1u<<1)
#define RCC_APB2Periph_USART1  (1u<<4)

/* DWT 周期计数器 */
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;
extern DWT_Type *DWT;
#define DWT_CTRL_CYCCNTENA_Msk  (1u<<0)

typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern CoreDebug_Type *CoreDebug;
#define CoreDebug_DEMCR_TRCENA_Msk (1u<<24)

extern uint32_t SystemCoreClock;
void SystemCoreClockUpdate(void);

/* 定时器 */
typedef struct { int dummy; } TIM_TypeDef;
extern TIM_TypeDef *TIM2, *TIM3;
#define TIM_IT_CC2  (1u<<2)
void TIM_ITConfig(TIM_TypeDef *tim, uint16_t it, FunctionalState state);

#endif
