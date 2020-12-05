#include "VL53L0x_I2c.h"
#include "Board_GPIO.h" 
#include "Board_Init.h"


//VL53L0X I2C初始化
void VL53L0X_i2c_init(void)
{
	
	GPIOB->MODER &= ~(0XF << 12);
	GPIOB->MODER |=  (0X5 << 12);
	
	GPIOB->OTYPER  &= ~(0X3 << 6);
	//GPIOB->OTYPER  |=  (0X3 << 6);
	
	GPIOB->PUPDR   &= ~(0XF << 12);
	GPIOB->PUPDR   |=  (0X5 << 12);
	GPIOB->OSPEEDR &= ~(0XF << 12);
	GPIOB->OSPEEDR |=  (0XF << 12);	
	
	GPIOB->ODR |=  (0X3 << 6);
}
/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	I2C_Delay()
*
*函数作用：	I2C时序里面用到的一个小延时，并不准确；
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void I2C_Delay(int us)
{
	for(int i = 0; i < us; us--)
	{
		__NOP();
	}
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Start()
*
*函数作用：	产生I2C的一个起始信号
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void VL_IIC_Start(void)
{
	VL_SDA_OUT();																// sda线输出
	VL_IIC_SDA_H;	  	  
	VL_IIC_SCL_H;
	I2C_Delay(12);
 	VL_IIC_SDA_L;																// START:when CLK is high,DATA change form high to low 
	I2C_Delay(12);
	VL_IIC_SCL_L;																// 钳住I2C总线，准备发送或接收数据 
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Stop()
*
*函数作用：	产生I2C的一个停止信号
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void VL_IIC_Stop(void)
{
	VL_SDA_OUT();																// sda线输出
	VL_IIC_SCL_L;
	VL_IIC_SDA_L;																// STOP:when CLK is high DATA change form low to high
 	I2C_Delay(12);
	VL_IIC_SCL_H; 
	VL_IIC_SDA_H;																// 发送I2C总线结束信号
	I2C_Delay(12);						   	
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Wait_Ack()
*
*函数作用：	等待应答信号到来
*
*参数说明：	无
*
*函数返回：	1，接收应答失败
*			0，接收应答成功
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t VL_IIC_Wait_Ack(void)
{
	uint16_t ucErrTime = 0;
	VL_SDA_IN();  						// SDA设置为输入  
	
	VL_IIC_SDA_H;I2C_Delay(2);	
	VL_IIC_SCL_H;I2C_Delay(2);
	
	while(VL_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 2500)
		{
			VL_IIC_Stop();
			return 1;
		}
	}
	VL_IIC_SCL_L;																// 时钟输出0 	   
	return 0;  
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Ack()
*
*函数作用：	产生ACK应答
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void VL_IIC_Ack(void)
{
	VL_IIC_SCL_L;
	VL_SDA_OUT();
	VL_IIC_SDA_L;
	I2C_Delay(10);
	VL_IIC_SCL_H;
	I2C_Delay(10);
	VL_IIC_SCL_L;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_NAck()
*
*函数作用：	不产生ACK应答
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/   
void VL_IIC_NAck(void)
{
	VL_IIC_SCL_L;
	VL_SDA_OUT();
	VL_IIC_SDA_H;
	I2C_Delay(10);
	VL_IIC_SCL_H;
	I2C_Delay(10);
	VL_IIC_SCL_L;
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Send_Byte()
*
*函数作用：	IIC发送一个字节
*
*参数说明：	txd：需要发送的数据
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/ 
void VL_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	VL_SDA_OUT(); 	    
    VL_IIC_SCL_L;																// 拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			VL_IIC_SDA_H;
		else
			VL_IIC_SDA_L;
		txd<<=1; 	  
		I2C_Delay(10); 
		VL_IIC_SCL_H;
		I2C_Delay(10);
		VL_IIC_SCL_L;	
		I2C_Delay(10);
    }	 
} 

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Read_Byte()
*
*函数作用：	读取一个字节
*
*参数说明：	ack=1时，发送ACK，ack=0，发送nACK
*
*函数返回：	读取到的内容
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i, receive = 0;
	VL_SDA_IN();																// SDA设置为输入
	for(i = 0; i < 8; i++)
	{
		VL_IIC_SCL_L; 
		I2C_Delay(10);
		VL_IIC_SCL_H;
		receive <<= 1;
		if(VL_READ_SDA)	receive++;   
		I2C_Delay(10);												// 1
	}					 
	if (!ack)
		VL_IIC_NAck();															// 发送nACK
	else
		VL_IIC_Ack(); 															// 发送ACK   
	return receive;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Write_1Byte()
*
*函数作用：	写一个字节
*
*参数说明：	SlaveAddress：写入设备的地址
*			REG_Address：写入寄存器的地址‘
*			REG_data：写入的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Write_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();															// 释放总线
		return 1;																// 没应答则退出

	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();	
	VL_IIC_Send_Byte(REG_data);
	VL_IIC_Wait_Ack();	
	VL_IIC_Stop();
	return 0;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Read_1Byte()
*
*函数作用：	读取一个字节
*
*参数说明：	SlaveAddress：读取设备的地址
*			REG_Address：读取寄存器的地址‘
*			REG_data：返回的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t *REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// 发写命令
	__nop();__nop();__nop();__nop();
	if(VL_IIC_Wait_Ack())
	{
		 VL_IIC_Stop();															// 释放总线
		 return 1;																// 没应答则退出
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	VL_IIC_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);										// 发读命令
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte(0);
	VL_IIC_Stop();
	return 0;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Write_nByte()
*
*函数作用：	写入多个字节
*
*参数说明：	SlaveAddress：读取设备的地址
*			REG_Address：读取寄存器的地址‘
*			len：读取数据长度
*			REG_data：返回的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// 发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();															// 释放总线
		return 1;																// 没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);												// 发送buff的数据
		VL_IIC_Wait_Ack();	
	}
	VL_IIC_Stop();																// 释放总线

	return 0;	
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL_IIC_Read_nByte()
*
*函数作用：	读取多个字节
*
*参数说明：	SlaveAddress：读取设备的地址
*			REG_Address：读取寄存器的地址‘
*			len：读取数据长度
*			REG_data：返回的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address,uint16_t len,uint8_t *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// 发写命令
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();															// 释放总线
		return 1;																// 没应答则退出
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress|0x01);										// 发读命令
	VL_IIC_Wait_Ack();
	while(len)
	{
		if(len==1)
		{
			*buf = VL_IIC_Read_Byte(0);
		}
		else
		{
			*buf = VL_IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	VL_IIC_Stop();																// 释放总线
	return 0;
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_write_multi()
*
*函数作用：	对VL53L0x进行连续写入多个数据
*
*参数说明：	SlaveAddress：读取设备的地址
*			REG_Address：读取寄存器的地址‘
*			len：读取数据长度
*			REG_data：返回的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count)
{
	uint8_t status = STATUS_OK;

	if(VL_IIC_Write_nByte(address, index, count, pdata))
	{
	   status  = STATUS_FAIL;
	}
	return status;
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_read_multi()
*
*函数作用：	对VL53L0x进行连续读取多个数据
*
*参数说明：	SlaveAddress：读取设备的地址
*			REG_Address：读取寄存器的地址‘
*			len：读取数据长度
*			REG_data：返回的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count)
{
	uint8_t status = STATUS_OK;
	if(VL_IIC_Read_nByte(address,index,count,pdata))
	{
	  status  = STATUS_FAIL;
	}
	return status;
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_write_byte()
*
*函数作用：	写1个数据
*
*参数说明：	address：写入地址
*			index：寄存器索引
*			data：写入的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data)
{
	uint8_t status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_write_word()
*
*函数作用：	写1字个数据
*
*参数说明：	address：写入地址
*			index：寄存器索引
*			data：写入的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data)
{
	uint8_t status = STATUS_OK;
	
	uint8_t buffer[2];
	
	//将16位数据拆分成8位
	buffer[0] = (uint8_t)(data>>8);												// 高八位
	buffer[1] = (uint8_t)(data&0xff);											// 低八位	
	if(index%2==1)
	{  
		//串行通信不能处理对非2字节对齐寄存器的字节
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multi(address,index,buffer,2);
	}
	
	return status;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_write_dword()
*
*函数作用：	写1字个数据
*
*参数说明：	address：写入地址
*			index：寄存器索引
*			data：写入的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data)
{
	
    uint8_t status = STATUS_OK;

    uint8_t buffer[4];	
	
	//将32位数据拆分成8位
	buffer[0] = (uint8_t)(data>>24);
	buffer[1] = (uint8_t)((data&0xff0000)>>16);
	buffer[2] = (uint8_t)((data&0xff00)>>8);
	buffer[3] = (uint8_t)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
}


/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_read_byte()
*
*函数作用：	读取一个字节
*
*参数说明：	address：读取地址
*			index：寄存器索引
*			data：读取的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata)
{
	uint8_t status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;	 
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_read_word()
*
*函数作用：	读取一个字
*
*参数说明：	address：读取地址
*			index：寄存器索引
*			data：读取的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata)
{
	uint8_t status = STATUS_OK;
	
	uint8_t buffer[2];
	
	status = VL53L0X_read_multi(address,index,buffer,2);
	
	*pdata = ((uint16_t)buffer[0]<<8)+(uint16_t)buffer[1];
	
	return status;
}

/********************************************************************************
*               VL53L0x_I2c.c
*函数名称：	VL53L0X_read_dword()
*
*函数作用：	读取2个字
*
*参数说明：	address：读取地址
*			index：寄存器索引
*			data：读取的数据
*
*函数返回：	应答：0；没有应答：1
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata)
{
	uint8_t status = STATUS_OK;
	
	uint8_t buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((uint32_t)buffer[0]<<24)+((uint32_t)buffer[1]<<16)+((uint32_t)buffer[2]<<8)+((uint32_t)buffer[3]);
	
	return status;
}



