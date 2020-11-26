/*******************************************************************************
 * 	VL53L0x.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "VL53L0x.h"
#include "Board_GPIO.h" 
#include "Board_Init.h" 


VL53L0X_DeviceInfo_t vl53l0x_dev_info;											// 设备ID版本信息
uint8_t AjustOK = 0;															// 校准标志位

 
/*
 *	VL53L0X各测量模式参数
 *	0：默认;1:高精度;2:长距离;3:高速
 */
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},																		// 默认
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},																		// 高精度
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},																		// 长距离
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},																		// 高速
		
};




/********************************************************************************
*               vl53l0x.c
*函数名称：	vl53l0x_Addr_set()
*
*函数作用：	设置VL53L0X地址
*
*参数说明：	dev：设备I2C参数结构体
*       	newaddr：新的设备地址
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0X_SetAddress(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t sta = 0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress == dev->I2cDevAddr)                                         // 新设备I2C地址与旧地址一致,直接退出
		return VL53L0X_ERROR_NONE;
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev, 0x88, 0x00);
	if(Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x01;                                                             // 设置I2C标准模式出错
		goto set_error;
	}
	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status != VL53L0X_ERROR_NONE)
	{
		sta = 0x02;                                                             // 读取寄存器出错
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev, FinalAddress);
		if(Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x03;                                                         // 设置I2C地址出错
			goto set_error;
		}
		//修改参数结构体的I2C地址
		dev->I2cDevAddr = FinalAddress;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status != VL53L0X_ERROR_NONE)
		{
			sta = 0x04;                                                         // 新I2C地址读写出错
			goto set_error;
		}	
	}
	set_error:
	if(Status != VL53L0X_ERROR_NONE)
	{
		Status = 1;
	}

	return Status;
}

/********************************************************************************
*               vl53l0x.c
*函数名称：	VL53L0x_RESET()
*	
*函数作用：	初始化VL53L0X硬件
*	
*参数说明：	dev：设备I2C参数结构体
*	
*函数返回：	无
*	
*函数作者：	ZhaoSir
********************************************************************************/
void VL53L0x_RESET(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;                                                     // 保存设备原I2C地址
    VL53L0X_Xshut(0);                                                           // 失能VL53L0X
    DelayMS(30);
	VL53L0X_Xshut(1);                                                           // 使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	DelayMS(30);
	dev->I2cDevAddr = 0x52;
	VL53L0X_SetAddress(dev,addr);                                                // 设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit(dev);
}


/********************************************************************************
*               vl53l0x.c
*函数名称：	VL53L0X_Init()
*	
*函数作用：	初始化VL53L0X硬件
*	
*参数说明：	dev：设备I2C参数结构体
*	
*函数返回：	无
*	
*函数作者：	ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0X_Init(VL53L0X_Dev_t *dev)
{
	/*
	 *	XSHUT 	----->  PB4
	 *	INT		----->  PB5
	 *	I2C_SCL	----->  PB6
	 *	I2C_SDA	----->  PB7
	 */
	VL53L0X_Error Status        = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice    = dev;

	RCC->IOPENR |= 0X02;
	GPIOB->MODER   &= ~(0X3 <<  8);
	GPIOB->MODER   |=  (0X1 <<  8);
	
	GPIOB->OTYPER  &= ~(0X1 <<  4);
	GPIOB->PUPDR   &= ~(0X3 <<  8);
	GPIOB->PUPDR   |=  (0X1 <<  8);
	GPIOB->OSPEEDR &= ~(0X3 <<  8);
	GPIOB->OSPEEDR |=  (0X1 <<  8);	
	
	pMyDevice->I2cDevAddr = VL53L0X_Addr;                                   // I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;                                              // I2C通信模式
	pMyDevice->comms_speed_khz = 400;                                       // I2C通信速率
	
	/*  初始化IIC和相关的引脚  */
	VL53L0X_i2c_init();
	VL53L0X_Xshut(0);                                                       // 失能VL53L0X
	DelayMS(30);
	VL53L0X_Xshut(1);                                                       // 使能VL53L0X,让传感器处于工作
	DelayMS(30);
	
	Status = VL53L0X_SetAddress(pMyDevice, 0x54);                           // 设置VL53L0X传感器I2C地址
    if(Status != VL53L0X_ERROR_NONE)    goto error;
	Status = VL53L0X_DataInit(pMyDevice);                                   //设备初始化
	if(Status != VL53L0X_ERROR_NONE)    goto error;
	DelayMS(2);

	Status = VL53L0X_GetDeviceInfo(pMyDevice, &vl53l0x_dev_info);           // 获取设备ID信息
    if(Status!=VL53L0X_ERROR_NONE) goto error;
/********************************************************************/
//	AT24CXX_Read(0,(uint8_t*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//读取24c02保存的校准数据,若已校准 Vl53l0x_data.adjustok==0xAA
/********************************************************************/

	if(Vl53l0x_data.adjustok == 0xAA)                               		// 已校准
	  AjustOK=1;	
	else                                                            		// 没校准
	  AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		return Status;
	}
	return Status;
}





