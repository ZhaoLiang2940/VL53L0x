/*******************************************************************************
 * DeviceAction.c
 *
 *  Created on: 2020年12月03日
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
            
volatile	uint8_t		            BluetoothStatus = 0;									// CC2640的状态
volatile	uint8_t		            AlarmDistance   = 0;									// 传感器设置的距离报警信息
volatile	uint8_t		            SamplePeriod    = 0;									// 传感器采样周期
volatile	uint8_t 	            VL53L0x_WorkMode = 0;
volatile    DeviceInfomation        SystemConfigFactory;



VL53L0X_Dev_t 	                    vl53l0x_dev;										    // 设备I2C数据参数
VL53L0X_RangingMeasurementData_t    VL53L0x_Data;

/********************************************************************************
*               DeviceAction.c
*函数名称：	Device_Init()
*
*函数作用：	设备初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
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
*函数名称：	UserCommandHandle()
*
*函数作用：	处理CC2640下发的串口指令
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
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
*函数名称：	EnterCalibrationMode()
*
*函数作用：	传感器进入校准模式
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void EnterCalibrationMode(void)
{
    uint8_t k = 0;
    VL53L0X_Error result = VL53L0X_ERROR_NONE;
	result = VL53L0x_Adjust(&vl53l0x_dev);
    if(result == VL53L0X_ERROR_NONE)
    {
        for(k = 0; k < 5; k ++)												// 等间隔慢速闪烁5次标识校准失败
        {
            OPEN_LED();
            DelayMS(400);
            CLOSE_LED();
            DelayMS(400);		
        }
    }
    else
    {
        for(k = 0; k < 5; k ++)												// 等间隔慢速闪烁5次标识校准失败
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
*函数名称：	WorkModeSetting()
*
*函数作用：	传感器工作模式设置
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void WorkModeSetting(uint8_t mode)
{
	VL53L0x_WorkMode = mode;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	ShowBluetoothStatus()
*
*函数作用：	展示Bluetooth的设备状态
*
*参数说明：	status：状态信息
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void ShowBluetoothStatus(uint8_t status)
{
	BluetoothStatus = status;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	AlarmDistanceSetting()
*
*函数作用：	设置传感器报警距离
*
*参数说明：	distance：距离参数，单位cm
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void AlarmDistanceSetting(uint8_t distance)
{
	AlarmDistance = distance;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	AlarmDistanceSetting()
*
*函数作用：	设置传感器报警距离
*
*参数说明：	distance：距离参数，单位cm
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t AlarmDistanceRead(void)
{
	return AlarmDistance;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	SamplePeriodSetting()
*
*函数作用：	采样周期设置
*
*参数说明：	period：采样周期，单位s
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void SamplePeriodSetting(uint8_t period)
{
	SamplePeriod = period;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	SamplePeriodSetting()
*
*函数作用：	采样周期设置
*
*参数说明：	period：采样周期，单位s
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static uint8_t SamplePeriodRead(void)
{
	return SamplePeriod;
}

/********************************************************************************
*               DeviceAction.c
*函数名称：	ErrorCMDHandle()
*
*函数作用：	错误参数处理函数
*
*参数说明：	period：采样周期，单位s
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void ErrorCMDHandle(void)
{
	
}


/*
 * CMD[0]:	命令起始符
 *			'C'
 * CMD[1]：	功能码
 *         	01：进入校准模式
 * 			02：设置传感器工作模式
 *			03：通知单片机蓝牙设备状态
 *			04:	设置传感器报警距离
 *			05：设置传感器采样周期
 *          06: 发起一次测量
 * CMD[2]:	命令参数
 *			CMD[1] = 01：不用携带参数 
 *			CMD[1] = 02: 参数指示工作模式
 *			CMD[1] = 03：蓝牙的状态参数
 *			CMD[1] = 04：传感器报警距离阈值，单位CM
 *			CMD[1] = 05：传感器采样周期
 * CMD[3]   校验和 = 0XFF - (CMD[0] + CMD[1] + CMD[2])			
 *
 */



