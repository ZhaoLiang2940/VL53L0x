#include "vl53l0x_gen.h"


VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体
volatile uint16_t Distance_data=0;//保存测距数据


/********************************************************************************
*               vl53l0x_gen.c
*函数名称：VL53L0x_SetMode()
*
*函数作用：设置VL53L0X数据采集模式
*
*参数说明：dev: 设备I2C参数结构体
*       mode: 0:默认;1:高精度;2:长距离
*
*函数返回：无
*
*函数作者：ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0x_SetMode(VL53L0X_Dev_t *dev,uint8_t mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 VL53L0x_RESET(dev);                                                        // 复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit(dev);

     if(AjustOK  != 0)                                                          // 已校准好了,写入校准值
     {
	    status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount, Vl53l0x_data.isApertureSpads);    // 设定Spads校准值
        if(status != VL53L0X_ERROR_NONE)    goto error;
        DelayMS(2);
	    status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings, Vl53l0x_data.PhaseCal);            //设定Ref校准值
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
	    status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, Vl53l0x_data.OffsetMicroMeter);            // 设定偏移校准值
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		status = VL53L0X_SetXTalkCompensationRateMegaCps(dev, Vl53l0x_data.XTalkCompensationRateMegaCps);   // 设定串扰校准值
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		 
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);                               // Ref参考校准
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);                    // 执行参考SPAD管理
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
	 }
	 status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);                                //使能单次测量模式
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);                   //使能SIGMA范围检查
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);             //使能信号速率范围检查
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);       //设定SIGMA范围
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);//设定信号速率范围范围
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget);                        //设定完整测距最长时间
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);    //设定VCSEL脉冲周期
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		return status;
	 }
	 return status;
	
}	

/********************************************************************************
*               vl53l0x_gen.c
*函数名称：VL53L0x_StartOneTimeSample()
*
*函数作用：启动一次VL53L0X数据采集
*
*参数说明：dev: 设备I2C参数结构体
*       pdata: 保存测量距离的结构体
*       buf：存放传感器状态数组
*
*函数返回：无
*
*函数作者：ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0x_StartOneTimesSample(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata, char *buf)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);               // 执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE)     return status;
   
	RangeStatus = pdata->RangeStatus;                                           // 获取当前测量状态
    memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus, buf);                             // 根据测量状态读取状态字符串
	
	Distance_data = pdata->RangeMilliMeter;                                     // 保存最近一次测距测量数据

    return status;
}



/********************************************************************************
*               vl53l0x_gen.c
*函数名称：readVL5Ll0x_PollingtMode()
*
*函数作用：读取VL53L0X的数据，采用轮询方式
*
*参数说明：dev：设备I2C参数结构体
*       mode：采样模式； 0:默认;1:高精度;2:长距离
*
*函数返回：无
*
*函数作者：ZhaoSir
********************************************************************************/
void readVL5Ll0x_PollingtMode(VL53L0X_Dev_t *dev, uint8_t mode)
{
    static char buf[VL53L0X_MAX_STRING_LENGTH];                                 // 测试模式字符串字符缓冲区
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;                                  // 工作状态
	uint8_t i=0;
	
	while(VL53L0x_SetMode(dev, mode))                                          // 配置测量模式
	{
	    DelayMS(500);
		i++;  if(i == 2)  return;
	}

	while(1)
	{
        if(Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0x_StartOneTimesSample(dev, &vl53l0x_data, buf);    // 执行一次测量
        }
	}	
}

