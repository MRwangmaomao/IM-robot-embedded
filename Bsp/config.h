#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "millisecondtimer.h"

#define PI      3.1415926
#define DEBUG   1

#define IMU_PUBLISH_RATE 50 //hz
#define BAT_PUBLISH_RATE 0.2 //hz
#define COMMAND_RATE 50 //hz
#define DEBUG_RATE 1

#define K_P    0.1 // P constant
#define K_I    0.2 // I constant
#define K_D    0.2 // D constant


/** motor param **/
#define PWM_BITS        8
#define MAX_RPM         280 //motor's maximum RPM 366 电机最大转速
#define COUNTS_PER_REV  469 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev) 轮子每转一圈的脉冲数
#define WHEEL_DIAMETER  0.065 //wheel's diameter in meters

#define LR_WHEELS_DISTANCE 0
#define FR_WHEELS_DISTANCE 0.29

#define 	USE_SERIAL1
#define 	USE_SERIAL2
#define 	USE_SERIAL3
#define  	USE_MOTOR1
#define  	USE_MOTOR2
#define 	USE_ENCODER1
#define 	USE_ENCODER2
#define 	USE_I2C
#define 	USE_SERVO1
#define 	USE_SERVO2
#define 	USE_SONAR

/** --------Serial Config-------- **/
typedef enum {
	SERIAL1 = 0,
	SERIAL2 = 1,
	SERIAL3 = 2,
	SERIAL_END = 3
}Serial_TypeDef; 

#define SERIALn							3

//PA9 TX PA10 RX
#define RIKI_SERIAL1					USART1
#define RIKI_SERIAL1_IRQ				USART1_IRQn
#define RIKI_SERIAL1_CLK             	RCC_APB2Periph_USART1
#define RIKI_SERIAL1_GPIO_CLK           RCC_APB2Periph_GPIOA
#define RIKI_SERIAL1_GPIO_PORT          GPIOA
#define RIKI_SERIAL1_TX_PIN            	GPIO_Pin_9
#define RIKI_SERIAL1_RX_PIN             GPIO_Pin_10
#define RIKI_SERIAL1_NVIC				1

//PA2 TX PA3 RX
#define RIKI_SERIAL2					USART2
#define RIKI_SERIAL2_IRQ				USART2_IRQn
#define RIKI_SERIAL2_CLK             	RCC_APB1Periph_USART2
#define RIKI_SERIAL2_GPIO_CLK        	RCC_APB2Periph_GPIOA
#define RIKI_SERIAL2_GPIO_PORT      	GPIOA
#define RIKI_SERIAL2_TX_PIN            	GPIO_Pin_2
#define RIKI_SERIAL2_RX_PIN             GPIO_Pin_3
#define RIKI_SERIAL2_NVIC				2

//PB10 TX PB11 RX
#define RIKI_SERIAL3									USART3
#define RIKI_SERIAL3_IRQ							USART3_IRQn
#define RIKI_SERIAL3_CLK             	RCC_APB1Periph_USART3
#define RIKI_SERIAL3_GPIO_CLK        	RCC_APB2Periph_GPIOB
#define RIKI_SERIAL3_GPIO_PORT      	GPIOB
#define RIKI_SERIAL3_TX_PIN            	GPIO_Pin_10
#define RIKI_SERIAL3_RX_PIN             GPIO_Pin_11
#define RIKI_SERIAL3_NVIC				3

/** Motor Config **/ 
typedef enum {
	MOTOR1 = 0,
	MOTOR2 = 1,
	MOTOR_END = 2
}Motor_TypeDef; 

 


/** Encoder config **/
typedef enum {
	ENCODER1 = 0,
	ENCODER2 = 1,
	ENCODER3 = 0,
	ENCODER4 = 1,
	ENCODER_END = 2
}Encoder_TypeDef; 
 

/** I2C Config **/

#define RIKI_SDA_PIN                GPIO_Pin_9
#define RIKI_SCL_PIN                GPIO_Pin_8
#define RIKI_I2C_GPIO_PORT          GPIOB
#define RIKI_I2C_GPIO_CLK           RCC_APB2Periph_GPIOB

 

/** LED config **/
#define RIKI_LED_PIN								GPIO_Pin_15
#define RIKI_LED_GPIO_PORT					GPIOG
#define RIKI_LED_GPIO_CLK						RCC_APB2Periph_GPIOG
 
/** volt adc config **/
#define Voltage_Threshold 47.3000
#define ADC1_DR_ADDRESS         ((u32)0x4001244C)
#define RIKI_BATTERY_PIN        GPIO_Pin_0
#define RIKI_BATTERY_GPIO_PORT      GPIOC
#define RIKI_BATTERY_GPIO_CLK       RCC_APB2Periph_GPIOC
#define RIKI_BATTERY_ADC_CLK        RCC_APB2Periph_ADC1
#define RIKI_BATTERY_DMA_CLK        RCC_AHBPeriph_DMA1

/** Bumper config**/
#define RIKI_BUMPER_FRONT_PIN        GPIO_Pin_2 
 
#define RIKI_BUMPER_BACK_PIN        GPIO_Pin_3 
 
#define RIKI_BUMPER_GPIO_PORT      GPIOE
#define RIKI_BUMPER_GPIO_CLK       RCC_APB2Periph_GPIOE

#endif // _CONFIG_H_
