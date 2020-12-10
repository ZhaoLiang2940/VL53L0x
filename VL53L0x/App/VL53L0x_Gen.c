#include "VL53L0x_gen.h"
#include "Board_Init.h" 



/********************************************************************************
*               vl53l0x_gen.c
*�������ƣ�VL53L0x_SetMode()
*
*�������ã�����VL53L0X���ݲɼ�ģʽ
*
*����˵����dev: �豸I2C�����ṹ��
*       mode: 0:Ĭ��;1:�߾���;2:������
*
*�������أ���
*
*�������ߣ�ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0x_SetMode(VL53L0X_Dev_t *dev,uint8_t mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 VL53L0x_RESET(dev);                                                        // ��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	 status = VL53L0X_StaticInit(dev);

     if(AjustOK  != 0)                                                          // ��У׼����,д��У׼ֵ
     {
	    status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount, Vl53l0x_data.isApertureSpads);    // �趨SpadsУ׼ֵ
        if(status != VL53L0X_ERROR_NONE)    goto error;
        DelayMS(2);
	    status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings, Vl53l0x_data.PhaseCal);            //�趨RefУ׼ֵ
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
	    status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, Vl53l0x_data.OffsetMicroMeter);            // �趨ƫ��У׼ֵ
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		status = VL53L0X_SetXTalkCompensationRateMegaCps(dev, Vl53l0x_data.XTalkCompensationRateMegaCps);   // �趨����У׼ֵ
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		 
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);                               // Ref�ο�У׼
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);                    // ִ�вο�SPAD����
		if(status != VL53L0X_ERROR_NONE)    goto error;
		DelayMS(2);
	 }
	 status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);                                //ʹ�ܵ��β���ģʽ
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);                   //ʹ��SIGMA��Χ���
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);             //ʹ���ź����ʷ�Χ���
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);       //�趨SIGMA��Χ
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget);                        //�趨��������ʱ��
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);    //�趨VCSEL��������
	 if(status != VL53L0X_ERROR_NONE)       goto error;
	 DelayMS(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//�趨VCSEL�������ڷ�Χ
	 
	 error://������Ϣ
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		return status;
	 }
	 return status;
	
}	

/********************************************************************************
*               vl53l0x_gen.c
*�������ƣ�VL53L0x_StartOneTimeSample()
*
*�������ã�����һ��VL53L0X���ݲɼ�
*
*����˵����dev: �豸I2C�����ṹ��
*       pdata: �����������Ľṹ��
*       buf����Ŵ�����״̬����
*
*�������أ���
*
*�������ߣ�ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0x_StartOneTimesSample(VL53L0X_Dev_t *dev, VL53L0X_RangingMeasurementData_t *pdata, char *buf, uint16_t* Distance_data)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);               // ִ�е��β�ಢ��ȡ����������
	if(status !=VL53L0X_ERROR_NONE)     return status;
   
	RangeStatus = pdata->RangeStatus;                                           // ��ȡ��ǰ����״̬
    memset(buf, 0x00, VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus, buf);                             // ���ݲ���״̬��ȡ״̬�ַ���
	
	*Distance_data = pdata->RangeMilliMeter;                                     // �������һ�β���������

    return status;
}



/********************************************************************************
*               vl53l0x_gen.c
*�������ƣ�readVL5Ll0x_PollingtMode()
*
*�������ã���ȡVL53L0X�����ݣ�������ѯ��ʽ
*
*����˵����dev���豸I2C�����ṹ��
*       mode������ģʽ�� 0:Ĭ��;1:�߾���;2:������
*
*�������أ���
*
*�������ߣ�ZhaoSir
********************************************************************************/
uint8_t readVL5Ll0x_PollingtMode(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t* vl53l0x_data, uint8_t mode, uint16_t* Distance_data)
{
    static char buf[VL53L0X_MAX_STRING_LENGTH];                                 // ����ģʽ�ַ����ַ�������
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;                                  // ����״̬
	uint8_t i=0;
	uint16_t disDataBuff[5] = {0};
	uint16_t* disDataPoint = disDataBuff;
	while(VL53L0x_SetMode(dev, mode))                                          // ���ò���ģʽ
	{
	    DelayMS(500);
		i++;  if(i == 5)  return 1;
	}
	i=0;
	while(1)
	{
        if(Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0x_StartOneTimesSample(dev, vl53l0x_data, buf, disDataPoint++);    // ִ��һ�β���
			i++;  
			if(i == 5)  
			{
				*Distance_data = (disDataBuff[0] + disDataBuff[1] + disDataBuff[2] + disDataBuff[3] + disDataBuff[4]) / 5;
				return 0;	
			}
        }
		else
			return 2;
	}	
}

