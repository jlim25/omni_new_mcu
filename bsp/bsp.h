#ifndef __BSP_H
#define __BSP_H

#include "main.h"
#include "stdbool.h"
#include "usart.h"

/* PIN Definition */
#define LED_PIN GPIO_PIN_0 // GREEN
#define LED_GPIO_PORT GPIOB

/* UART Debug Channel */
#define DEBUG_UART &huart3

/* MOTOR UART */
#define SERVO_UART &huart1

#define SERVO_DIR_GPIO_Port GPIOB
#define SERVO_DIR_Pin GPIO_PIN_5

#endif /* __BSP_H */
