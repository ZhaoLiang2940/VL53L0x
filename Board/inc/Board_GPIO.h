/*******************************************************************************
 * Board_GPIO.h
 *
 *  Created on: 2020Äê11ÔÂ1ÈÕ
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#ifndef __BOARD_GPIO_H__
#define	__BOARD_GPIO_H__

#include "main.h"


#define				VL53L0x_SDA_PORT			GPIOB
#define				VL53L0x_SDA_PIN				7

#define				VL53L0x_SCL_PORT			GPIOB
#define				VL53L0x_SCL_PIN				6

#define				VL53L0x_XSHUT_PORT			GPIOB
#define				VL53L0x_XSHUT_PIN			4

#define				VL53L0x_INT_PORT			GPIOB
#define				VL53L0x_INT_PIN				5

#define				CC2640_WKUP_PORT			GPIOB
#define				CC2640_WKUP_PIN				0

#define				CC2640_RESET_PORT		    GPIOB
#define				CC2640_RESET_PIN			1

#define				LED_PORT					GPIOA
#define				LED_PIN						15

#define				KEY_PORT					GPIOB
#define				KEY_PIN						3


typedef enum
{
	INPUT_MODE,
	OUTPUT_MODE,
	AF_MODE,
	AN_MODE
}GPIO_Modex;

typedef enum
{
	OUT_PP,
	OUT_OD
}GPIO_OTypex;

typedef enum
{
	SPEED_Low,
	SPEED_MD,
	SPEED_Fast,
	SPEED_Hight
}GPIO_Speedx;

typedef enum
{
	PUPD_NO,
	PUPD_UP,
	PUPD_DOWN,
	PUPD_XX
}GPIO_Pupdx;


typedef enum{
	GPIO_Low,
	GPIO_Hight
}GPIO_x;

extern		void 			GPIO_Init(void);
extern		uint8_t			GPIO_SET_BIT(GPIO_TypeDef* gpiox, uint8_t Bitx, bool Value);
extern		uint8_t			GPIO_GET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx);

#endif

