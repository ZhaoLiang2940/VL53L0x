#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H
#include "stdint.h"
#include "stm32l0xx.h"

#define			VL_READ_SDA				((GPIOB->IDR & (0X1 << 7)) ? 1 : 0)
#define			VL_SDA_IN()				(GPIOB->MODER &= ~(0X3 << 12))
#define			VL_SDA_OUT() 			(GPIOB->MODER |=  (0X1 << 12))
#define			VL_IIC_SDA_H			(GPIOB->ODR |=  (0X1 << 7))
#define			VL_IIC_SDA_L			(GPIOB->ODR &= ~(0X1 << 7))
#define			VL_IIC_SCL_H			(GPIOB->ODR |=  (0X1 << 6))
#define			VL_IIC_SCL_L			(GPIOB->ODR &= ~(0X1 << 6))
#define 		VL53L0x_DelayUS(x)


//状态
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

//IIC操作函数
extern  void VL53L0X_i2c_init(void);//初始化IIC的IO口
extern  uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data);              //IIC写一个8位数据
extern  uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data);             //IIC写一个16位数据
extern  uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data);            //IIC写一个32位数据
extern  uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count);//IIC连续写

extern  uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata);             //IIC读一个8位数据
extern  uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata);            //IIC读一个16位数据
extern  uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata);           //IIC读一个32位数据
extern  uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count);  //IIC连续读



#endif 


