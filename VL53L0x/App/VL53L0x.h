#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_gen.h"
#include "vl53l0x_cali.h"


//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52(���������λ)
#define VL53L0X_Addr 0x52


#define		VL53L0X_Xshut(x) 	GPIO_SET_BIT(GPIOB, 4, x)
#define		DelayMS(x) 			Systick_DelayMs(x)

//ʹ��2.8V IO��ƽģʽ
#define USE_I2C_2V8  1

//����ģʽ
#define Default_Mode   0// Ĭ��
#define HIGH_ACCURACY  1//�߾���
#define LONG_RANGE     2//������
#define HIGH_SPEED     3//����

//vl53l0xģʽ���ò�����
typedef  struct
{
	FixPoint1616_t signalLimit;    //Signal������ֵ 
	FixPoint1616_t sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod ;  //VCSEL��������
	uint8_t finalRangeVcselPeriod ;//VCSEL�������ڷ�Χ
	
}mode_data;


extern 		mode_data 		Mode_data[];
extern 		uint8_t  		AjustOK;

extern		void            VL53L0x_RESET(VL53L0X_Dev_t *dev);//vl53l0x��λ
extern		uint8_t         VL53L0X_Read(uint8_t mode);//��ȡһ�β�����������
extern		VL53L0X_Error 	VL53L0X_Init(VL53L0X_Dev_t *dev);

#endif


