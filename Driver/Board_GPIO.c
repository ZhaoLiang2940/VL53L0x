/*******************************************************************************
 * Board_GPIO.c
 *
 *  Created on: 2020��11��1��
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "Board_GPIO.h"


/* GPIO Mode�Ĵ��� */
uint8_t			GPIOA_MODE_REG[] = 	{
										INPUT_MODE,								/* GPIOA_PIN0  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN1  ������������ 				*/
										AF_MODE,								/* GPIOA_PIN2  ������� 					*/
										AF_MODE,								/* GPIOA_PIN3  �������� 					*/
										INPUT_MODE,								/* GPIOA_PIN4  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN5  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN6  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN7  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN8  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN9  ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN10 ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN11 ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN12 ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN13 ������������ 				*/
										INPUT_MODE,								/* GPIOA_PIN14 ������������ 				*/
										OUTPUT_MODE,								/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};

uint8_t			GPIOB_MODE_REG[] = 	{
										OUTPUT_MODE,							/* GPIOB_PIN0  CC640���� 					*/
										OUTPUT_MODE,							/* GPIOB_PIN1  CC2640�������� 				*/
										INPUT_MODE,								/* GPIOB_PIN2  ������������					*/
										INPUT_MODE,								/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										OUTPUT_MODE,							/* GPIOB_PIN4  LED������� 					*/
										OUTPUT_MODE,							/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										AF_MODE,								/* GPIOB_PIN6  I2C SCL 						*/
										AF_MODE,								/* GPIOB_PIN7  I2C SDA 						*/
									};

/* GPIO OTYPER�Ĵ��� */
uint8_t			GPIOA_OTYPER_REG[] = 	{
										OUT_PP,									/* GPIOA_PIN0  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN1  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN2  ������� 					*/
										OUT_PP,									/* GPIOA_PIN3  �������� 					*/
										OUT_PP,									/* GPIOA_PIN4  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN5  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN6  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN7  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN8  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN9  ������������ 				*/
										OUT_PP,									/* GPIOA_PIN10 ������������ 				*/
										OUT_PP,									/* GPIOA_PIN11 ������������ 				*/
										OUT_PP,									/* GPIOA_PIN12 ������������ 				*/
										OUT_PP,									/* GPIOA_PIN13 ������������ 				*/
										OUT_PP,									/* GPIOA_PIN14 ������������ 				*/
										OUT_PP,									/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};	
	
uint8_t			GPIOB_OTYPER_REG[] = 	{	
										OUT_PP,									/* GPIOB_PIN0  CC640���� 					*/
										OUT_PP,									/* GPIOB_PIN1  CC2640�������� 				*/
										OUT_PP,									/* GPIOB_PIN2  ������������					*/
										OUT_PP,									/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										OUT_PP,									/* GPIOB_PIN4  LED������� 					*/
										OUT_PP,									/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										OUT_OD,									/* GPIOB_PIN6  I2C SCL 						*/
										OUT_OD,									/* GPIOB_PIN7  I2C SDA 						*/
									};

/* GPIO OSPEDD�Ĵ��� */
uint8_t			GPIOA_OSPEED_REG[] = 	{
										SPEED_Low,								/* GPIOA_PIN0  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN1  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN2  ������� 					*/
										SPEED_Low,								/* GPIOA_PIN3  �������� 					*/
										SPEED_Low,								/* GPIOA_PIN4  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN5  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN6  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN7  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN8  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN9  ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN10 ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN11 ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN12 ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN13 ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN14 ������������ 				*/
										SPEED_Low,								/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};

uint8_t			GPIOB_OSPEED_REG[] = 	{
										SPEED_Low,								/* GPIOB_PIN0  CC640���� 					*/
										SPEED_Low,								/* GPIOB_PIN1  CC2640�������� 				*/
										SPEED_Low,								/* GPIOB_PIN2  ������������					*/
										SPEED_Low,								/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										SPEED_Low,								/* GPIOB_PIN4  LED������� 					*/
										SPEED_Low,								/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										SPEED_Low,								/* GPIOB_PIN6  I2C SCL 						*/
										SPEED_Low,								/* GPIOB_PIN7  I2C SDA 						*/
									};
									
/* GPIO PUPD�Ĵ��� */
uint8_t			GPIOA_PUPD_REG[] = 	{
										PUPD_DOWN,								/* GPIOA_PIN0  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN1  ������������ 				*/
										PUPD_NO,								/* GPIOA_PIN2  ������� 					*/
										PUPD_NO,								/* GPIOA_PIN3  �������� 					*/
										PUPD_DOWN,								/* GPIOA_PIN4  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN5  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN6  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN7  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN8  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN9  ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN10 ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN11 ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN12 ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN13 ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN14 ������������ 				*/
										PUPD_DOWN,								/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};

uint8_t			GPIOB_PUPD_REG[] = 	{
										PUPD_NO,								/* GPIOB_PIN0  CC640���� 					*/
										PUPD_NO,								/* GPIOB_PIN1  CC2640�������� 				*/
										PUPD_DOWN,								/* GPIOB_PIN2  ������������					*/
										PUPD_DOWN,								/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										PUPD_NO,								/* GPIOB_PIN4  LED������� 					*/
										PUPD_NO,								/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										PUPD_NO,								/* GPIOB_PIN6  I2C SCL 						*/
										PUPD_NO,								/* GPIOB_PIN7  I2C SDA 						*/
									};		

/* GPIO OUT Put Data�Ĵ��� */
uint8_t			GPIOA_OD_REG[] = 	{
										GPIO_Low,								/* GPIOA_PIN0  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN1  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN2  ������� 					*/
										GPIO_Low,								/* GPIOA_PIN3  �������� 					*/
										GPIO_Low,								/* GPIOA_PIN4  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN5  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN6  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN7  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN8  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN9  ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN10 ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN11 ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN12 ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN13 ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN14 ������������ 				*/
										GPIO_Low,								/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};

uint8_t			GPIOB_OD_REG[] = 	{
										GPIO_Low,								/* GPIOB_PIN0  CC640���� 					*/
										GPIO_Low,								/* GPIOB_PIN1  CC2640�������� 				*/
										GPIO_Low,								/* GPIOB_PIN2  ������������					*/
										GPIO_Low,								/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										GPIO_Low,								/* GPIOB_PIN4  LED������� 					*/
										GPIO_Low,								/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										GPIO_Low,								/* GPIOB_PIN6  I2C SCL 						*/
										GPIO_Low,								/* GPIOB_PIN7  I2C SDA 						*/
									};				

/* GPIO OUT Put Data�Ĵ��� */
uint8_t			GPIOA_AF_REG[] = 	{
										GPIO_AF(1),								/* GPIOA_PIN0  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN1  ������������ 				*/
										GPIO_AF(4),								/* GPIOA_PIN2  ������� 					*/
										GPIO_AF(4),								/* GPIOA_PIN3  �������� 					*/
										GPIO_AF(1),								/* GPIOA_PIN4  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN5  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN6  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN7  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN8  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN9  ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN10 ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN11 ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN12 ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN13 ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN14 ������������ 				*/
										GPIO_AF(1),								/* GPIOA_PIN15 VL53L0x�жϣ����ó��ж���ʽ 	*/
									};

uint8_t			GPIOB_AF_REG[] = 	{
										GPIO_AF(1),								/* GPIOB_PIN0  CC640���� 					*/
										GPIO_AF(1),								/* GPIOB_PIN1  CC2640�������� 				*/
										GPIO_AF(1),								/* GPIOB_PIN2  ������������					*/
										GPIO_AF(1),								/* GPIOB_PIN3  �������룬���ó��ж���ʽ 	*/
										GPIO_AF(1),								/* GPIOB_PIN4  LED������� 					*/
										GPIO_AF(1),								/* GPIOB_PIN5  VL53L0X˯�߿��� 				*/
										GPIO_AF(1),								/* GPIOB_PIN6  I2C SCL 						*/
										GPIO_AF(1),								/* GPIOB_PIN7  I2C SDA 						*/
									};	
/********************************************************************************
*               Board_GPIO.c
*�������ƣ�	GPIO_Init()
*
*�������ã�	�������Ӳ����ʼ��
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
void GPIO_Init(void)
{
	RCC->IOPENR  |=   0X3;                                                      // Enable GPIOA��GPIOB port clock
	GPIOA->MODER &= ~(0X3 << 30);                                               // 
	GPIOA->MODER |=  (0X1 << 30);                                               // GPIOA_PIN15 OutPut
	GPIOA->ODR   |=  (0X1 << 15);                                               // Output Hight 
	
#if 0	
 	uint32_t regVal = 0;
	RCC->IOPENR |= 0X03;														/* Enable PORTA, PORTB Clock */
	
	regVal = 0X28000000;
	/*������������������������������������������������������GPIOA������������������������������������������������������*/
	/* Moder�Ĵ�������IO����ģʽ */
	for(uint8_t i = 0; i < sizeof(GPIOA_MODE_REG); i++)
	{	
		if((i == 13) || (i == 14)) continue;
		regVal |= GPIOA_MODE_REG[i] << i*2;
	}
	GPIOA->MODER = regVal;
	regVal = 0;
	
	/* OTYPE�Ĵ������������ʽ */
	for(uint8_t i = 0; i < sizeof(GPIOA_OTYPER_REG); i++)
	{
		regVal |= GPIOA_OTYPER_REG[i] << i;
	}
	GPIOA->OTYPER = regVal;
	regVal = 0x0C000000;
	
	/* Ospeed�Ĵ�������IO�ڹ������� */
	for(uint8_t i = 0; i < sizeof(GPIOA_OSPEED_REG); i++)
	{
		if(i == 13) continue;
		regVal |= GPIOA_OSPEED_REG[i] << i*2;
	}
	GPIOA->OSPEEDR = regVal;
	regVal = 0;
	
	/* PUPD�Ĵ�������IO������������ѡ�� */
	for(uint8_t i = 0; i < sizeof(GPIOA_PUPD_REG); i++)
	{
		regVal |= GPIOA_PUPD_REG[i] << i*2;
	}
	GPIOA->PUPDR = regVal;
	regVal = 0;
	
	/* OD�Ĵ�������ÿ��IO���״̬ */
	for(uint8_t i = 0; i < sizeof(GPIOA_OD_REG); i++)
	{
		regVal |= GPIOA_OD_REG[i] << i;
	}
	GPIOA->ODR = regVal;
	regVal = 0;
	
	
	/* OD�Ĵ�������ÿ��IO���״̬ */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOA_AF_REG[i] << i*4;
	}
	GPIOA->AFR[0] = regVal;
	regVal = 0;
	
	/* OD�Ĵ�������ÿ��IO���״̬ */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOA_AF_REG[8 + i] << i*4;
	}
	GPIOA->AFR[1] = regVal;
	regVal = 0;
	
	
	/*������������������������������������������������������GPIOA������������������������������������������������������*/
	
	/*������������������������������������������������������GPIOB������������������������������������������������������*/
	/* Moder�Ĵ�������IO����ģʽ */
	for(uint8_t i = 0; i < sizeof(GPIOB_MODE_REG); i++)
	{
		regVal |= GPIOB_MODE_REG[i] << i*2;
	}
	GPIOB->MODER = regVal;
	regVal = 0;
	
	/* OTYPE�Ĵ������������ʽ */
	for(uint8_t i = 0; i < sizeof(GPIOB_OTYPER_REG); i++)
	{
		regVal |= GPIOB_OTYPER_REG[i] << i;
	}
	GPIOB->OTYPER = regVal;
	regVal = 0;
	
	/* Ospeed�Ĵ�������IO�ڹ������� */
	for(uint8_t i = 0; i < sizeof(GPIOB_OSPEED_REG); i++)
	{
		regVal |= GPIOB_OSPEED_REG[i] << i*2;
	}
	GPIOB->OSPEEDR = regVal;
	regVal = 0;
	
	/* PUPD�Ĵ�������IO������������ѡ�� */
	for(uint8_t i = 0; i < sizeof(GPIOB_PUPD_REG); i++)
	{
		regVal |= GPIOB_PUPD_REG[i] << i*2;
	}
	GPIOB->PUPDR = regVal;
	regVal = 0;
	
	/* OD�Ĵ�������ÿ��IO���״̬ */
	for(uint8_t i = 0; i < sizeof(GPIOB_OD_REG); i++)
	{
		regVal |= GPIOB_OD_REG[i] << i;
	}
	GPIOB->ODR = regVal;
	regVal = 0;
	
	/* OD�Ĵ�������ÿ��IO���״̬ */
	for(uint8_t i = 0; i < 8; i++)
	{
		regVal |= GPIOB_AF_REG[i] << i*4;
	}
	GPIOB->AFR[0] = regVal;
	regVal = 0;
	/*������������������������������������������������������GPIOB������������������������������������������������������*/	
#endif
}


/********************************************************************************
*               Board_GPIO.c
*�������ƣ�	GPIO_SET_BIT()
*
*�������ã�	�����������״̬
*
*����˵����	gpiox�����Ŷ˿ں�
*			Bitx��λ��
*			Value�����õ�ֵ
*
*�������أ�	2������
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	GPIO_GET_BIT()
*
*�������ã�	��ȡ����״̬
*
*����˵����	gpiox�����Ŷ˿ں�
*			Bitx��λ��
*
*�������أ�	2������
*
*�������ߣ�	ZhaoSir
********************************************************************************/
uint8_t	GPIO_GET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx)
{
	uint32_t regVal = 0;
	if((gpiox != GPIOA) && (gpiox != GPIOB)) return 2;
	regVal = gpiox->IDR;
	
	if(regVal & (1 << Bitx)) return 1;
	else	return 0;
}



