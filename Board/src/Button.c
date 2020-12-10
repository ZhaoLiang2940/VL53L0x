/*******************************************************************************
 * Button.c
 *
 * Created on: 2020年10月26日
 *
 * Author: ZhaoSir
 *******************************************************************************/
#include "Button.h"
#include "Board_Init.h"
 
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
void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, PIN3_INT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}


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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == WAKEUP_PIN)
    {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
        //Exist_LowPowerMode();
    }
}

 
 
