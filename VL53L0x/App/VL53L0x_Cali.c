#include "vl53l0x_cali.h"



_vl53l0x_adjust Vl53l0x_adjust; //鏍″噯鏁版嵁24c02鍐欑紦瀛樺尯(鐢ㄤ簬鍦ㄦ牎鍑嗘ā寮忔牎鍑嗘暟鎹啓鍏�24c02)
_vl53l0x_adjust Vl53l0x_data;   //鏍″噯鏁版嵁24c02璇荤紦瀛樺尯锛堢敤浜庣郴缁熷垵濮嬪寲鏃跺悜24C02璇诲彇鏁版嵁锛�

#define adjust_num 5//鏍″噯閿欒娆℃暟

//VL53L0X鏍″噯鍑芥暟
//dev:璁惧I2C鍙傛暟缁撴瀯浣�
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev)
{
	
	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100<<16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	uint8_t i=0;

	VL53L0X_StaticInit(dev);//鏁板�兼仮澶嶉粯璁�,浼犳劅鍣ㄥ浜庣┖闂茬姸鎬�

	spads:
//	delay_ms(10);
	//printf("The SPADS Calibration Start...\r\n");
	Status = VL53L0X_PerformRefSpadManagement(dev,&refSpadCount,&isApertureSpads);//鎵ц鍙傝�僑pad绠＄悊
	if(Status == VL53L0X_ERROR_NONE)
	{
	    //printf("refSpadCount = %d\r\n",refSpadCount);
	    Vl53l0x_adjust.refSpadCount = refSpadCount;
	    //printf("isApertureSpads = %d\r\n",isApertureSpads);	
	    Vl53l0x_adjust.isApertureSpads = isApertureSpads;
        //printf("The SPADS Calibration Finish...\r\n\r\n");		
	    i=0;
	}
	else
	{
	    i++;
	    if(i==adjust_num) return Status;
	    //printf("SPADS Calibration Error,Restart this step\r\n");
	    goto spads;
	}
	//璁惧鍙傝�冩牎鍑�---------------------------------------------------
	ref:
//	delay_ms(10);
	//printf("The Ref Calibration Start...\r\n");
	Status = VL53L0X_PerformRefCalibration(dev,&VhvSettings,&PhaseCal);//Ref鍙傝�冩牎鍑�
	if(Status == VL53L0X_ERROR_NONE)
	{
		//printf("VhvSettings = %d\r\n",VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		//printf("PhaseCal = %d\r\n",PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		//printf("The Ref Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		//printf("Ref Calibration Error,Restart this step\r\n");
		goto ref;
	}
	//鍋忕Щ鏍″噯------------------------------------------------
	offset:
//	delay_ms(10);
	//printf("Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
	//printf("The Offset Calibration Start...\r\n");
	
	Status = VL53L0X_PerformOffsetCalibration(dev,CalDistanceMilliMeter,&OffsetMicroMeter);//鍋忕Щ鏍″噯
	if(Status == VL53L0X_ERROR_NONE)
	{
		//printf("CalDistanceMilliMeter = %d mm\r\n",CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		//printf("OffsetMicroMeter = %d mm\r\n",OffsetMicroMeter);	
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		//printf("The Offset Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		//printf("Offset Calibration Error,Restart this step\r\n");
		goto offset;
	}
	//涓叉壈鏍″噯-----------------------------------------------------
	xtalk:
//	delay_ms(20);
	//printf("Cross Talk Calibration:need a grey target\r\n");
	//printf("The Cross Talk Calibration Start...\r\n");	
	Status = VL53L0X_PerformXTalkCalibration(dev,XTalkCalDistance,&XTalkCompensationRateMegaCps);//涓叉壈鏍″噯
	if(Status == VL53L0X_ERROR_NONE)
	{
		//printf("XTalkCalDistance = %d mm\r\n",XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		//printf("XTalkCompensationRateMegaCps = %d\r\n",XTalkCompensationRateMegaCps);	
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		//printf("The Cross Talk Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		//printf("Cross Talk Calibration Error,Restart this step\r\n");
		goto xtalk;
	}
	//printf("All the Calibration has Finished!\r\n");
	//printf("Calibration is successful!!\r\n");

	Vl53l0x_adjust.adjustok = 0xAA;//鏍″噯鎴愬姛
//	AT24CXX_Write(0,(uint8_t*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//灏嗘牎鍑嗘暟鎹繚瀛樺埌24c02
	memcpy(&Vl53l0x_data,&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//灏嗘牎鍑嗘暟鎹鍒跺埌Vl53l0x_data缁撴瀯浣�
	return Status;
}

//vl53l0x鏍″噯娴嬭瘯
//dev:璁惧I2C鍙傛暟缁撴瀯浣�
void vl53l0x_calibration_test(VL53L0X_Dev_t *dev)
{  
/*	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t key=0;
	uint8_t i=0;
	
	LCD_Fill(30,170,320,300,WHITE);
	POINT_COLOR=RED;//璁剧疆瀛椾綋涓虹孩鑹�
	LCD_ShowString(30,170,300,16,16,"need a white target,and ");
	LCD_ShowString(30,190,250,16,16,"the distance keep 100mm.");
	POINT_COLOR=BLUE;//璁剧疆瀛椾綋涓鸿摑鑹�
	LCD_ShowString(30,220,200,16,16,"KEY_UP: Return menu");
	LCD_ShowString(30,240,200,16,16,"KEY1:   Calibration");
	while(1)
	{
		key = KEY_Scan(0);
		if(key==KEY1_PRES)
		{
			POINT_COLOR=RED;//璁剧疆瀛椾綋涓虹孩鑹�
			LCD_ShowString(30,260,200,16,16,"Start calibration...");
			status = vl53l0x_adjust(dev);//杩涘叆鏍″噯
			if(status!=VL53L0X_ERROR_NONE)//鏍″噯澶辫触
			{ 
				 //printf("Calibration is error!!\r\n");
				 i=3;
				 while(i--)
				 {
					  delay_ms(500);
					  LCD_ShowString(30,260,200,16,16,"                    ");
					  delay_ms(500);
					  LCD_ShowString(30,260,200,16,16,"Calibration is error");
				 }
			}
			else
				 LCD_ShowString(30,260,200,16,16,"Calibration is complete!");
			delay_ms(500);

			break;
				
		 }
		 else if(key==WKUP_PRES)
		 {
			 LED1(1);
			 break;//杩斿洖涓婁竴鑿滃崟
		 }		 
		 delay_ms(200);
		 LED0_Toggle;
		
	}
*/
}
