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
 * ����ģʽ�滮��
 * 1. һ�ϵ���һ�α������룬�������ڣ�˯��ʱ������Ƿ����ú���������Ļ���Ҫǿ������ΪĬ��ֵ
 * 2. ��ȡһ��VL53L01������������ͱ���ֵ�ľ���֮��Ĺ�ϵ
 *    2.1�����ڱ������룬����10��ÿ�βɼ���������˸LED3�Σ�����˯��ʱ��
 *    2.2��С�ڱ������룬����ϵͳΪ˯��״̬��˯��ʱ��Ϊ����˯��ʱ��Ͳ�������
 * 3. ���ϵͳ״̬�Ƿ��д�������Ҫ����
 * 3. ����ϵͳ״̬�Ƿ���Ҫ���ͣ��������ݷ���
 * 4. ����ϵͳ״̬����Ƿ���Ҫ����˯��
 *
 *
 * ˯��״̬�£��̰��°����������豸�������ݲɼ����ɼ�20�Σ������趨��
 * 
 * ����״̬�£����°���1~2S���ţ��ر����������Ǽ����ɼ����ݣ�
 * ����״̬�£����°���3~5S���ţ��豸��������
 * ����״̬�£����°�������5S�������������Ϣ���������ģʽ
 */
/********************************************************************************
*               main.c
*�������ƣ�	main()
*
*�������ã�	����������
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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



