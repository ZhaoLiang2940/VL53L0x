/*******************************************************************************
 * DeviceAction.c
 *
 *  Created on: 2020��12��03��
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "DeviceAction.h"

    
static 		void 		            EnterCalibrationMode(void);
static 		void 		            WorkModeSetting(uint8_t mode);
static 		void 		            ShowBluetoothStatus(uint8_t status);
static 		void 		            AlarmDistanceSetting(uint8_t distance);
static 		void 		            SamplePeriodSetting(uint8_t period);
static 		void 		            ErrorCMDHandle(void);
static      uint8_t                 SamplePeriodRead(void);
            
volatile	uint8_t		            BluetoothStatus = 0;									// CC2640��״̬
volatile	uint8_t		            AlarmDistance   = 0;									// ���������õľ��뱨����Ϣ
volatile	uint8_t		            SamplePeriod    = 0;									// ��������������
volatile	uint8_t 	            VL53L0x_WorkMode = 0;
volatile    DeviceInfomation        SystemConfigFactory;



VL53L0X_Dev_t 	                    vl53l0x_dev;										    // �豸I2C���ݲ���
VL53L0X_RangingMeasurementData_t    VL53L0x_Data;

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	Device_Init()
*
*�������ã�	�豸��ʼ��
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
bool readVL53L0X_Data(uint16_t*  readdata)
{
    VL53L0X_Init(&vl53l0x_dev);
	POWERDOWN_VL53L0x();
    readVL5Ll0x_PollingtMode(&vl53l0x_dev, &VL53L0x_Data, SamplePeriodRead(), &readData);
    return *readdata > AlarmDistanceRead();
}
/********************************************************************************
*               DeviceAction.c
*�������ƣ�	UserCommandHandle()
*
*�������ã�	����CC2640�·��Ĵ���ָ��
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
int UserCommandHandle(const uint8_t* CMD)
{
	const char *CMD_PTR = (const char*)CMD;
	if(strcmp("OK", CMD_PTR) == 0)
	{
		return 0;
	}
	else
	{
		uint8_t check = 0;
		uint8_t parameter = 0;
		check = CMD[0] + CMD[1] + CMD[2];
		check = 0XFF - check;
		if((CMD_PTR[0] != 'C') || (CMD[3] != check))
			return 1;
		
		parameter = CMD[2];
		switch(CMD_PTR[1])
		{
			case 0X01:
				EnterCalibrationMode();
				break;
			case 0X02:
				WorkModeSetting(parameter);
				break;
			case 0X03:
				ShowBluetoothStatus(parameter);
				break;
			case 0X04:
				AlarmDistanceSetting(parameter);
				break;
			case 0X05:
				SamplePeriodSetting(parameter);
				break;
            case 0X06:
				readVL53L0X_Data();
				break;
			default:
				ErrorCMDHandle();
				break;
		}
	}
	return 0;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	EnterCalibrationMode()
*
*�������ã�	����������У׼ģʽ
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void EnterCalibrationMode(void)
{
    uint8_t k = 0;
    VL53L0X_Error result = VL53L0X_ERROR_NONE;
	result = VL53L0x_Adjust(&vl53l0x_dev);
    if(result == VL53L0X_ERROR_NONE)
    {
        for(k = 0; k < 5; k ++)												// �ȼ��������˸5�α�ʶУ׼ʧ��
        {
            OPEN_LED();
            DelayMS(400);
            CLOSE_LED();
            DelayMS(400);		
        }
    }
    else
    {
        for(k = 0; k < 5; k ++)												// �ȼ��������˸5�α�ʶУ׼ʧ��
        {
            OPEN_LED();
            DelayMS(100);
            CLOSE_LED();
            DelayMS(400);		
        }
    }
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	WorkModeSetting()
*
*�������ã�	����������ģʽ����
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void WorkModeSetting(uint8_t mode)
{
	VL53L0x_WorkMode = mode;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	ShowBluetoothStatus()
*
*�������ã�	չʾBluetooth���豸״̬
*
*����˵����	status��״̬��Ϣ
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void ShowBluetoothStatus(uint8_t status)
{
	BluetoothStatus = status;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	AlarmDistanceSetting()
*
*�������ã�	���ô�������������
*
*����˵����	distance�������������λcm
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void AlarmDistanceSetting(uint8_t distance)
{
	AlarmDistance = distance;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	AlarmDistanceSetting()
*
*�������ã�	���ô�������������
*
*����˵����	distance�������������λcm
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
uint8_t AlarmDistanceRead(void)
{
	return AlarmDistance;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	SamplePeriodSetting()
*
*�������ã�	������������
*
*����˵����	period���������ڣ���λs
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void SamplePeriodSetting(uint8_t period)
{
	SamplePeriod = period;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	SamplePeriodSetting()
*
*�������ã�	������������
*
*����˵����	period���������ڣ���λs
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static uint8_t SamplePeriodRead(void)
{
	return SamplePeriod;
}

/********************************************************************************
*               DeviceAction.c
*�������ƣ�	ErrorCMDHandle()
*
*�������ã�	�������������
*
*����˵����	period���������ڣ���λs
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void ErrorCMDHandle(void)
{
	
}


/*
 * CMD[0]:	������ʼ��
 *			'C'
 * CMD[1]��	������
 *         	01������У׼ģʽ
 * 			02�����ô���������ģʽ
 *			03��֪ͨ��Ƭ�������豸״̬
 *			04:	���ô�������������
 *			05�����ô�������������
 *          06: ����һ�β���
 * CMD[2]:	�������
 *			CMD[1] = 01������Я������ 
 *			CMD[1] = 02: ����ָʾ����ģʽ
 *			CMD[1] = 03��������״̬����
 *			CMD[1] = 04������������������ֵ����λCM
 *			CMD[1] = 05����������������
 * CMD[3]   У��� = 0XFF - (CMD[0] + CMD[1] + CMD[2])			
 *
 */



