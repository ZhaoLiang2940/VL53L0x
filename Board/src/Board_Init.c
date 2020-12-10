/*******************************************************************************
 * Board_Init.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "main.h"
#include "USART.h"
#include "Button.h"
#include "Board_Init.h"
#include "Board_GPIO.h"
#define         RTC_ASYNCH_PREDIV           0x7C
#define         RTC_SYNCH_PREDIV            0x0127


    
static          void 	            RTC_AWU_Init(void);

RTC_HandleTypeDef                   RTCHandle;

/********************************************************************************
*               Board_Init.c
*函数名称：	SystemClock_Init()
*
*函数作用：	设置系统时钟；使用MSI作为HCLK和PCLK，使用4.194Mhz
*			LSE驱动RTC唤醒系统；
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
*
*The system Clock is configured as follow : 
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 32000000
*            HCLK(Hz)                       = 32000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            APB2 Prescaler                 = 1
*            HSI Frequency(Hz)              = 16000000
*            PLL_MUL                        = 4
*            PLL_DIV                        = 2
*            Flash Latency(WS)              = 1
*            Main regulator output voltage  = Scale1 mode
********************************************************************************/
uint16_t SystemClock_Init(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /*  The voltage scaling allows optimizing the power consumption when the device is 
        clocked below the maximum system frequency, to update the voltage scaling value 
        regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        //Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
    clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        //Error_Handler();
    }   
    return 0;
}


/********************************************************************************
*               Board_Init.c
*函数名称：	Board_Init()
*
*函数作用：	板载相关硬件初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void Board_Init(void)
{
	SystemClock_Init();
	GPIO_Init();
	USART2_Init();	
	RTC_AWU_Init();
    HAL_Init();
}

/********************************************************************************
*               Board_Init.c
*函数名称：	Enter_LowpowerMode()
*
*函数作用：	进入低功耗模式执行的处理操作
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void Enter_LowPowerMode(uint8_t sleepTiem)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    if(sleepTiem > 28) sleepTiem = 28;
    sleepTiem =  sleepTiem * 10;
    GPIO_InitStructure.Pin = GPIO_PIN_All;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); 
    Button_Init();
    __HAL_RCC_GPIOA_CLK_DISABLE();
    

    HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);
    SwitchBLEToSleep()
    __HAL_RCC_GPIOB_CLK_DISABLE();
    
    HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle,SleepTime(sleepTiem) , RTC_WAKEUPCLOCK_RTCCLK_DIV16);//SleepTime(sleepTiem)
    // Enter Stop Mode 
    
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/********************************************************************************
*               Board_Init.c
*函数名称：	Exist_LowPowerMode()
*
*函数作用：	退出睡眠模式的初始化工作
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
uint32_t Exist_LowPowerMode(void)
{
    uint32_t SampleTime = 0;
    SystemClockConfig_STOP();
    SampleTime = HAL_GetTick() ;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    SwitchBLEToActive();
    USART2_Init();	
 //   if(SendToBLETimes > 10) SendToBLETimes = 0;
    
    return SampleTime;
}


/********************************************************************************
*               Board_Init.c
*函数名称：	SystemClockConfig_STOP()
*
*函数作用：	Configures system clock after wake-up from STOP: enable HSI, PLL
*           and select PLL as system clock source
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void SystemClockConfig_STOP(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /*  The voltage scaling allows optimizing the power consumption when the device is 
        clocked below the maximum system frequency, to update the voltage scaling value 
        regarding system frequency refer to product datasheet.  
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Get the Oscillators configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        //Error_Handler();
    }

    /*  Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
        clocks dividers 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        //  Error_Handler();
    }
}

/********************************************************************************
*               Board_Init.c
*函数名称：	HAL_RTCEx_WakeUpTimerEventCallback()
*
*函数作用：	RTC Wake Up callback
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}




/********************************************************************************
*               Board_Init.c
*函数名称：	RTC_AWU_Init()
*
*函数作用：	AWU自动唤醒初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void RTC_AWU_Init(void)
{

    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();
  
    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();

    __HAL_RCC_RTC_ENABLE();
    /* Select HSI as system clock source after Wake Up from Stop mode */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    
    /* Configure RTC */
    RTCHandle.Instance = RTC;
    /* Configure RTC prescaler and RTC data registers as follow:
    - Hour Format = Format 24
    - Asynch Prediv = Value according to source clock
    - Synch Prediv = Value according to source clock
    - OutPut = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType = Open Drain */
    RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    RTCHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
    {
        /* Initialization Error */
       // Error_Handler(); 
    }
}







/********************************************************************************
*               Board_Init.c
*函数名称：	SysTick_Handler()
*
*函数作用：	Systick中断服务函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void HAL_SYSTICK_Callback(void)
{
    HAL_IncTick();
}

