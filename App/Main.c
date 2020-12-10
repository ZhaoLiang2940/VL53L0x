/*******************************************************************************
 * main.c
 *
 *  Created on: 2020?10?26?
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "main.h"
#include "USART.h"
#include "VL53L0x.h"
#include "runStatus.h"
#include "Board_Init.h" 
#include "Board_GPIO.h"
#include "DeviceAction.h"


#define             SamplePeriod                    1000
volatile            uint16_t                        SendToBLETimes = 0;


/*
 * 工作模式规划：
 * 1. 一上电检测一次报警距离，采样周期，睡眠时间参数是否设置合理，不合理的话就要强制设置为默认值
 * 2. 读取一次VL53L01传感器检测距离和报警值的距离之间的关系
 *    2.1：大于报警距离，采样10次每次采集结束后闪烁LED3次，设置睡眠时间
 *    2.2：小于报警距离，设置系统为睡眠状态，睡眠时间为常规睡眠时间和采样周期
 * 3. 检测系统状态是否有串口数据要处理
 * 3. 根据系统状态是否需要发送，进行数据发送
 * 4. 根据系统状态检测是否需要进行睡眠
 *
 *
 * 睡眠状态下，短按下按键，唤醒设备进行数据采集；采集20次（可以设定）
 * 
 * 唤醒状态下，按下按键1~2S不放，关闭蓝牙，但是继续采集数据；
 * 唤醒状态下，按下按键3~5S不放，设备整体休眠
 * 唤醒状态下，按下按键超过5S，蓝牙清除绑定信息，进入配对模式
 */
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
    uint32_t        SampleTime = 0;
    uint16_t        Distance_data = 0;
    
    initSystemStatus();
    Board_Init();   
    SampleTime = HAL_GetTick();
    
    while(1)
    {
        if(SystemRunSatus != SleepStatus)
        {
            if((HAL_GetTick() - SampleTime) > SamplePeriod)
            {
                SampleTime = HAL_GetTick() ;
                SendDistanceToBLE(SendToBLETimes);
                SendToBLETimes ++;
            }
            if(USART2_RX_IsStart() && USART2_RX_IsComplete())
            {
                CompleteUSART2_RX();
            }
            if(SendToBLETimes >= 5)
            {
                SendToBLETimes = 0;
                readVL53L0X_Data(&Distance_data);
                Enter_LowPowerMode(10);
                SampleTime = Exist_LowPowerMode();     
            }
        }
    }
}



