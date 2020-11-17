/*******************************************************************************
 * MyCode.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "stm32l0xx.h"
#include "stdbool.h"
#include "Board_Init.h" 
#include "Board_GPIO.h" 

uint8_t TXBuff[7] = {0X5A, 0X00, 0X00, 0X00, 0XFE, 0XFE, 0XA5};
bool 	bitVal = false;
/********************************************************************************
*               main.c
*函数名称：	main()
*
*函数作用：	工程主函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
int main(void)
{
	uint8_t Index = 0;
	uint32_t SysTickCount = 0UL;
	Board_Init();
	SysTickCount = SYS_GetTick();
	while(1)
	{		
		/*if((SYS_GetTick() - SysTickCount) > 3000)
		{
			SysTickCount = SYS_GetTick();
			TXBuff[5] = TXBuff[4] = Index;
			Index ++;
			USART2_TX((const char*)&TXBuff, sizeof(TXBuff));
			bitVal = 1 - bitVal;
			GPIO_SET_BIT(GPIOB, 0, bitVal);
		}	*/
		//__WFI;
	}
}





/* 
 RTC 设置1S每次；
	启动测量模式下，
		有三个档位，第一个档位1S测量一次；第二个档位500MS测量一次；第三个档位100MS测量一次； 长按5S按键进行开机和关机，开机之后，短按1S按键在三种模式下切换；决定采样周期；
	开机状态下长按按键5S进入关机；
	关机状态下，长按电源按键5S进入开机；开机和关机快速闪烁5次LED；每次切换采样周期的时候闪烁一次LED；

	蓝牙扫描周期为5S，每次广播数据1S，如果1S内没有连接，关闭蓝牙；5S之后再次广播；
	蓝牙连接之后就广播距离数据；

	蓝牙的广播与否由STM32决定，STM32通过WKUP引脚决定他的休眠和广播；同时蓝牙的连接状态也会通过引脚反馈给STM32；从而决定他的休眠
*/


