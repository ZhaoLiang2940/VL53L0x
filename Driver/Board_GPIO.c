/*******************************************************************************
 * Board_GPIO.c
 *
 *  Created on: 2020年11月1日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "Board_GPIO.h"


/* GPIO Mode寄存器 */
uint8_t			GPIOA_MODE_REG[] = 	{
										INPUT_MODE,								/* GPIOA_PIN0  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN1  不用输入下拉 				*/
										AF_MODE,								/* GPIOA_PIN2  串口输出 					*/
										AF_MODE,								/* GPIOA_PIN3  串口输入 					*/
										INPUT_MODE,								/* GPIOA_PIN4  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN5  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN6  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN7  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN8  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN9  不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN10 不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN11 不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN12 不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN13 不用输入下拉 				*/
										INPUT_MODE,								/* GPIOA_PIN14 不用输入下拉 				*/
										OUTPUT_MODE,								/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};

uint8_t			GPIOB_MODE_REG[] = 	{
										OUTPUT_MODE,							/* GPIOB_PIN0  CC640唤醒 					*/
										OUTPUT_MODE,							/* GPIOB_PIN1  CC2640发送数据 				*/
										INPUT_MODE,								/* GPIOB_PIN2  不用输入下拉					*/
										INPUT_MODE,								/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										OUTPUT_MODE,							/* GPIOB_PIN4  LED控制输出 					*/
										OUTPUT_MODE,							/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										AF_MODE,								/* GPIOB_PIN6  I2C SCL 						*/
										AF_MODE,								/* GPIOB_PIN7  I2C SDA 						*/
									};

/* GPIO OTYPER寄存器 */
uint8_t			GPIOA_OTYPER_REG[] = 	{
										OUT_PP,									/* GPIOA_PIN0  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN1  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN2  串口输出 					*/
										OUT_PP,									/* GPIOA_PIN3  串口输入 					*/
										OUT_PP,									/* GPIOA_PIN4  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN5  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN6  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN7  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN8  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN9  不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN10 不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN11 不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN12 不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN13 不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN14 不用输入下拉 				*/
										OUT_PP,									/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};	
	
uint8_t			GPIOB_OTYPER_REG[] = 	{	
										OUT_PP,									/* GPIOB_PIN0  CC640唤醒 					*/
										OUT_PP,									/* GPIOB_PIN1  CC2640发送数据 				*/
										OUT_PP,									/* GPIOB_PIN2  不用输入下拉					*/
										OUT_PP,									/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										OUT_PP,									/* GPIOB_PIN4  LED控制输出 					*/
										OUT_PP,									/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										OUT_OD,									/* GPIOB_PIN6  I2C SCL 						*/
										OUT_OD,									/* GPIOB_PIN7  I2C SDA 						*/
									};

/* GPIO OSPEDD寄存器 */
uint8_t			GPIOA_OSPEED_REG[] = 	{
										SPEED_Low,								/* GPIOA_PIN0  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN1  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN2  串口输出 					*/
										SPEED_Low,								/* GPIOA_PIN3  串口输入 					*/
										SPEED_Low,								/* GPIOA_PIN4  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN5  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN6  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN7  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN8  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN9  不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN10 不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN11 不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN12 不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN13 不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN14 不用输入下拉 				*/
										SPEED_Low,								/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};

uint8_t			GPIOB_OSPEED_REG[] = 	{
										SPEED_Low,								/* GPIOB_PIN0  CC640唤醒 					*/
										SPEED_Low,								/* GPIOB_PIN1  CC2640发送数据 				*/
										SPEED_Low,								/* GPIOB_PIN2  不用输入下拉					*/
										SPEED_Low,								/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										SPEED_Low,								/* GPIOB_PIN4  LED控制输出 					*/
										SPEED_Low,								/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										SPEED_Low,								/* GPIOB_PIN6  I2C SCL 						*/
										SPEED_Low,								/* GPIOB_PIN7  I2C SDA 						*/
									};
									
/* GPIO PUPD寄存器 */
uint8_t			GPIOA_PUPD_REG[] = 	{
										PUPD_DOWN,								/* GPIOA_PIN0  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN1  不用输入下拉 				*/
										PUPD_NO,								/* GPIOA_PIN2  串口输出 					*/
										PUPD_NO,								/* GPIOA_PIN3  串口输入 					*/
										PUPD_DOWN,								/* GPIOA_PIN4  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN5  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN6  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN7  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN8  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN9  不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN10 不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN11 不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN12 不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN13 不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN14 不用输入下拉 				*/
										PUPD_DOWN,								/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};

uint8_t			GPIOB_PUPD_REG[] = 	{
										PUPD_NO,								/* GPIOB_PIN0  CC640唤醒 					*/
										PUPD_NO,								/* GPIOB_PIN1  CC2640发送数据 				*/
										PUPD_DOWN,								/* GPIOB_PIN2  不用输入下拉					*/
										PUPD_DOWN,								/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										PUPD_NO,								/* GPIOB_PIN4  LED控制输出 					*/
										PUPD_NO,								/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										PUPD_NO,								/* GPIOB_PIN6  I2C SCL 						*/
										PUPD_NO,								/* GPIOB_PIN7  I2C SDA 						*/
									};		

/* GPIO OUT Put Data寄存器 */
uint8_t			GPIOA_OD_REG[] = 	{
										GPIO_Low,								/* GPIOA_PIN0  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN1  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN2  串口输出 					*/
										GPIO_Low,								/* GPIOA_PIN3  串口输入 					*/
										GPIO_Low,								/* GPIOA_PIN4  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN5  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN6  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN7  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN8  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN9  不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN10 不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN11 不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN12 不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN13 不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN14 不用输入下拉 				*/
										GPIO_Low,								/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};

uint8_t			GPIOB_OD_REG[] = 	{
										GPIO_Low,								/* GPIOB_PIN0  CC640唤醒 					*/
										GPIO_Low,								/* GPIOB_PIN1  CC2640发送数据 				*/
										GPIO_Low,								/* GPIOB_PIN2  不用输入下拉					*/
										GPIO_Low,								/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										GPIO_Low,								/* GPIOB_PIN4  LED控制输出 					*/
										GPIO_Low,								/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										GPIO_Low,								/* GPIOB_PIN6  I2C SCL 						*/
										GPIO_Low,								/* GPIOB_PIN7  I2C SDA 						*/
									};				

/* GPIO OUT Put Data寄存器 */
uint8_t			GPIOA_AF_REG[] = 	{
										GPIO_AF(1),								/* GPIOA_PIN0  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN1  不用输入下拉 				*/
										GPIO_AF(4),								/* GPIOA_PIN2  串口输出 					*/
										GPIO_AF(4),								/* GPIOA_PIN3  串口输入 					*/
										GPIO_AF(1),								/* GPIOA_PIN4  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN5  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN6  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN7  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN8  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN9  不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN10 不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN11 不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN12 不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN13 不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN14 不用输入下拉 				*/
										GPIO_AF(1),								/* GPIOA_PIN15 VL53L0x中断，设置成中断形式 	*/
									};

uint8_t			GPIOB_AF_REG[] = 	{
										GPIO_AF(1),								/* GPIOB_PIN0  CC640唤醒 					*/
										GPIO_AF(1),								/* GPIOB_PIN1  CC2640发送数据 				*/
										GPIO_AF(1),								/* GPIOB_PIN2  不用输入下拉					*/
										GPIO_AF(1),								/* GPIOB_PIN3  按键输入，设置成中断形式 	*/
										GPIO_AF(1),								/* GPIOB_PIN4  LED控制输出 					*/
										GPIO_AF(1),								/* GPIOB_PIN5  VL53L0X睡眠控制 				*/
										GPIO_AF(1),								/* GPIOB_PIN6  I2C SCL 						*/
										GPIO_AF(1),								/* GPIOB_PIN7  I2C SDA 						*/
									};	
/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_Init()
*
*函数作用：	板载相关硬件初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void GPIO_Init(void)
{
	RCC->IOPENR |= 0X03;
	GPIOA->MODER &= ~(0X3 << 30);
	GPIOA->MODER |=  (0X1 << 30);
	GPIOA->ODR |= (0X1 << 15);
	
	GPIOB->MODER &= ~(0XF <<  0);
	GPIOB->MODER |=  (0X5 <<  0);
	
	GPIOB->OTYPER  &= ~(0X3 << 0);
	GPIOB->OSPEEDR &= ~(0X3 << 0);
	
	GPIOB->PUPDR &= ~(0XF <<  0);
	GPIOB->PUPDR |=  (0X9 <<  0);
	
	GPIOB->BSRR |=  (0X3 <<  0);
	
#if 0	
 	uint32_t regVal = 0;
	RCC->IOPENR |= 0X03;														/* Enable PORTA, PORTB Clock */
	
	regVal = 0X28000000;
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓GPIOA↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	/* Moder寄存器设置IO工作模式 */
	for(uint8_t i = 0; i < sizeof(GPIOA_MODE_REG); i++)
	{	
		if((i == 13) || (i == 14)) continue;
		regVal |= GPIOA_MODE_REG[i] << i*2;
	}
	GPIOA->MODER = regVal;
	regVal = 0;
	
	/* OTYPE寄存器设置输出方式 */
	for(uint8_t i = 0; i < sizeof(GPIOA_OTYPER_REG); i++)
	{
		regVal |= GPIOA_OTYPER_REG[i] << i;
	}
	GPIOA->OTYPER = regVal;
	regVal = 0x0C000000;
	
	/* Ospeed寄存器设置IO口工作速率 */
	for(uint8_t i = 0; i < sizeof(GPIOA_OSPEED_REG); i++)
	{
		if(i == 13) continue;
		regVal |= GPIOA_OSPEED_REG[i] << i*2;
	}
	GPIOA->OSPEEDR = regVal;
	regVal = 0;
	
	/* PUPD寄存器设置IO口上下拉电阻选择 */
	for(uint8_t i = 0; i < sizeof(GPIOA_PUPD_REG); i++)
	{
		regVal |= GPIOA_PUPD_REG[i] << i*2;
	}
	GPIOA->PUPDR = regVal;
	regVal = 0;
	
	/* OD寄存器设置每个IO输出状态 */
	for(uint8_t i = 0; i < sizeof(GPIOA_OD_REG); i++)
	{
		regVal |= GPIOA_OD_REG[i] << i;
	}
	GPIOA->ODR = regVal;
	regVal = 0;
	
	
	/* OD寄存器设置每个IO输出状态 */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOA_AF_REG[i] << i*4;
	}
	GPIOA->AFR[0] = regVal;
	regVal = 0;
	
	/* OD寄存器设置每个IO输出状态 */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOA_AF_REG[8 + i] << i*4;
	}
	GPIOA->AFR[1] = regVal;
	regVal = 0;
	
	
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑GPIOA↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓GPIOB↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	/* Moder寄存器设置IO工作模式 */
	for(uint8_t i = 0; i < sizeof(GPIOB_MODE_REG); i++)
	{
		regVal |= GPIOB_MODE_REG[i] << i*2;
	}
	GPIOB->MODER = regVal;
	regVal = 0;
	
	/* OTYPE寄存器设置输出方式 */
	for(uint8_t i = 0; i < sizeof(GPIOB_OTYPER_REG); i++)
	{
		regVal |= GPIOB_OTYPER_REG[i] << i;
	}
	GPIOB->OTYPER = regVal;
	regVal = 0;
	
	/* Ospeed寄存器设置IO口工作速率 */
	for(uint8_t i = 0; i < sizeof(GPIOB_OSPEED_REG); i++)
	{
		regVal |= GPIOB_OSPEED_REG[i] << i*2;
	}
	GPIOB->OSPEEDR = regVal;
	regVal = 0;
	
	/* PUPD寄存器设置IO口上下拉电阻选择 */
	for(uint8_t i = 0; i < sizeof(GPIOB_PUPD_REG); i++)
	{
		regVal |= GPIOB_PUPD_REG[i] << i*2;
	}
	GPIOB->PUPDR = regVal;
	regVal = 0;
	
	/* OD寄存器设置每个IO输出状态 */
	for(uint8_t i = 0; i < sizeof(GPIOB_OD_REG); i++)
	{
		regVal |= GPIOB_OD_REG[i] << i;
	}
	GPIOB->ODR = regVal;
	regVal = 0;
	
	/* OD寄存器设置每个IO输出状态 */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOB_AF_REG[i] << i*4;
	}
	GPIOB->AFR[0] = regVal;
	regVal = 0;
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑GPIOB↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/	
#endif
}


/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_SET_BIT()
*
*函数作用：	设置引脚输出状态
*
*参数说明：	gpiox：引脚端口号
*			Bitx：位号
*			Value：设置的值
*
*函数返回：	2：错误；
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t GPIO_SET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx, bool Value)
{
	if((gpiox != GPIOA) && (gpiox != GPIOB)) return 2;
	if(Value)
		gpiox->BSRR |= (0X1 << Bitx);
	else
		gpiox->BSRR |= (0X1 << (Bitx + 16));
	return Value;
}

/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_GET_BIT()
*
*函数作用：	读取引脚状态
*
*参数说明：	gpiox：引脚端口号
*			Bitx：位号
*
*函数返回：	2：错误
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t	GPIO_GET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx)
{
	uint32_t regVal = 0;
	if((gpiox != GPIOA) && (gpiox != GPIOB)) return 2;
	regVal = gpiox->IDR;
	
	if(regVal & (1 << Bitx)) return 1;
	else	return 0;
}



