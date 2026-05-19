#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ================= MOTOR PINS (L298N) ================= */

/* L298N 1 - __Bánh__ __trái__ */
#define M1_IN1_Pin         GPIO_PIN_5
#define M1_IN1_GPIO_Port   GPIOA
#define M1_IN2_Pin         GPIO_PIN_6
#define M1_IN2_GPIO_Port   GPIOA

#define M2_IN1_Pin         GPIO_PIN_7
#define M2_IN1_GPIO_Port   GPIOA
#define M2_IN2_Pin         GPIO_PIN_0
#define M2_IN2_GPIO_Port   GPIOB

/* L298N 2 - __Bánh__ __phải__ */
#define M3_IN1_Pin         GPIO_PIN_1
#define M3_IN1_GPIO_Port   GPIOB
#define M3_IN2_Pin         GPIO_PIN_13
#define M3_IN2_GPIO_Port   GPIOB

#define M4_IN1_Pin         GPIO_PIN_10
#define M4_IN1_GPIO_Port   GPIOB
#define M4_IN2_Pin         GPIO_PIN_12
#define M4_IN2_GPIO_Port   GPIOB

/* ================= PWM ENABLE ================= */
#define EN_LEFT_Pin         GPIO_PIN_8
#define EN_LEFT_GPIO_Port   GPIOA

#define EN_RIGHT_Pin        GPIO_PIN_9
#define EN_RIGHT_GPIO_Port  GPIOA

/* ================= LINE SENSOR PINS ================= */
#define SENSOR_S1_Pin         GPIO_PIN_0
#define SENSOR_S1_GPIO_Port   GPIOC

#define SENSOR_S2_Pin         GPIO_PIN_1
#define SENSOR_S2_GPIO_Port   GPIOC

#define SENSOR_S3_Pin         GPIO_PIN_2
#define SENSOR_S3_GPIO_Port   GPIOC

#define SENSOR_S4_Pin         GPIO_PIN_3
#define SENSOR_S4_GPIO_Port   GPIOC

#define SENSOR_S5_Pin         GPIO_PIN_1
#define SENSOR_S5_GPIO_Port   GPIOA

/* ================= HC-05 / USART6 ================= */
/*
 * PC6 = USART6_TX -> HC05 RXD
 * PC7 = USART6_RX <- HC05 TXD
 */
#define HC05_TX_Pin          GPIO_PIN_6
#define HC05_TX_GPIO_Port    GPIOC

#define HC05_RX_Pin          GPIO_PIN_7
#define HC05_RX_GPIO_Port    GPIOC

/* ================= HANDLE ================= */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart6;

/* ================= FUNCTION PROTOTYPES ================= */
void Error_Handler(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_USART6_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif
