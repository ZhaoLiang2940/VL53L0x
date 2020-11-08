#include "VL53L0x_I2c.h"


//VL53L0X I2C��ʼ��
void VL53L0X_i2c_init(void)
{


}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Start()
*
*�������ã�	����I2C��һ����ʼ�ź�
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
void VL_IIC_Start(void)
{
	VL_SDA_OUT();																// sda�����
	VL_IIC_SDA_H;	  	  
	VL_IIC_SCL_H;
	VL53L0x_DelayUS(4);
 	VL_IIC_SDA_L;																// START:when CLK is high,DATA change form high to low 
	VL53L0x_DelayUS(4);
	VL_IIC_SCL_L;																// ǯסI2C���ߣ�׼�����ͻ�������� 
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Stop()
*
*�������ã�	����I2C��һ��ֹͣ�ź�
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
void VL_IIC_Stop(void)
{
	VL_SDA_OUT();																// sda�����
	VL_IIC_SCL_L;
	VL_IIC_SDA_L;																// STOP:when CLK is high DATA change form low to high
 	VL53L0x_DelayUS(4);
	VL_IIC_SCL_H; 
	VL_IIC_SDA_H;																// ����I2C���߽����ź�
	VL53L0x_DelayUS(4);							   	
}


/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Wait_Ack()
*
*�������ã�	�ȴ�Ӧ���źŵ���
*
*����˵����	��
*
*�������أ�	1������Ӧ��ʧ��
*			0������Ӧ��ɹ�
*
*�������ߣ�	ZhaoSir
********************************************************************************/
uint8_t VL_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	VL_SDA_IN();  																// SDA����Ϊ����  
	VL_IIC_SDA_H; VL53L0x_DelayUS(1);	   
	VL_IIC_SCL_H; VL53L0x_DelayUS(1);	 
	while(VL_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{
			VL_IIC_Stop();
			return 1;
		}
	}
	VL_IIC_SCL_L;																// ʱ�����0 	   
	return 0;  
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Ack()
*
*�������ã�	����ACKӦ��
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
void VL_IIC_Ack(void)
{
	VL_IIC_SCL_L;
	VL_SDA_OUT();
	VL_IIC_SDA_L;
	VL53L0x_DelayUS(2);
	VL_IIC_SCL_H;
	VL53L0x_DelayUS(2);
	VL_IIC_SCL_L;
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_NAck()
*
*�������ã�	������ACKӦ��
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/   
void VL_IIC_NAck(void)
{
	VL_IIC_SCL_L;
	VL_SDA_OUT();
	VL_IIC_SDA_H;
	VL53L0x_DelayUS(2);
	VL_IIC_SCL_H;
	VL53L0x_DelayUS(2);
	VL_IIC_SCL_L;
}


/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Send_Byte()
*
*�������ã�	IIC����һ���ֽ�
*
*����˵����	txd����Ҫ���͵�����
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
void VL_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	VL_SDA_OUT(); 	    
    VL_IIC_SCL_L;																// ����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			VL_IIC_SDA_H;
		else
			VL_IIC_SDA_L;
		txd<<=1; 	  
		VL53L0x_DelayUS(2);  
		VL_IIC_SCL_H;
		VL53L0x_DelayUS(2); 
		VL_IIC_SCL_L;	
		VL53L0x_DelayUS(2);
    }	 
} 

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Read_Byte()
*
*�������ã�	��ȡһ���ֽ�
*
*����˵����	ack=1ʱ������ACK��ack=0������nACK
*
*�������أ�	��ȡ��������
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i, receive = 0;
	VL_SDA_IN();																// SDA����Ϊ����
	for(i = 0; i < 8; i++)
	{
		VL_IIC_SCL_L; 
		VL53L0x_DelayUS(4);
		VL_IIC_SCL_H;
		receive <<= 1;
		if(VL_READ_SDA)	receive++;   
		VL53L0x_DelayUS(4); 													// 1
	}					 
	if (!ack)
		VL_IIC_NAck();															// ����nACK
	else
		VL_IIC_Ack(); 															// ����ACK   
	return receive;
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Write_1Byte()
*
*�������ã�	дһ���ֽ�
*
*����˵����	SlaveAddress��д���豸�ĵ�ַ
*			REG_Address��д��Ĵ����ĵ�ַ��
*			REG_data��д�������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Write_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);
	if(VL_IIC_Wait_Ack())
	{
		VL_IIC_Stop();															// �ͷ�����
		return 1;																// ûӦ�����˳�

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
*�������ƣ�	VL_IIC_Read_1Byte()
*
*�������ã�	��ȡһ���ֽ�
*
*����˵����	SlaveAddress����ȡ�豸�ĵ�ַ
*			REG_Address����ȡ�Ĵ����ĵ�ַ��
*			REG_data�����ص�����
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t *REG_data)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// ��д����
	if(VL_IIC_Wait_Ack())
	{
		 VL_IIC_Stop();															// �ͷ�����
		 return 1;																// ûӦ�����˳�
	}		
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	VL_IIC_Start(); 
	VL_IIC_Send_Byte(SlaveAddress|0x01);										// ��������
	VL_IIC_Wait_Ack();
	*REG_data = VL_IIC_Read_Byte(0);
	VL_IIC_Stop();
	return 0;
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Write_nByte()
*
*�������ã�	д�����ֽ�
*
*����˵����	SlaveAddress����ȡ�豸�ĵ�ַ
*			REG_Address����ȡ�Ĵ����ĵ�ַ��
*			len����ȡ���ݳ���
*			REG_data�����ص�����
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// ��д����
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();															// �ͷ�����
		return 1;																// ûӦ�����˳�
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();
	while(len--)
	{
		VL_IIC_Send_Byte(*buf++);												// ����buff������
		VL_IIC_Wait_Ack();	
	}
	VL_IIC_Stop();																// �ͷ�����

	return 0;	
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL_IIC_Read_nByte()
*
*�������ã�	��ȡ����ֽ�
*
*����˵����	SlaveAddress����ȡ�豸�ĵ�ַ
*			REG_Address����ȡ�Ĵ����ĵ�ַ��
*			len����ȡ���ݳ���
*			REG_data�����ص�����
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address,uint16_t len,uint8_t *buf)
{
	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress);												// ��д����
	if(VL_IIC_Wait_Ack()) 
	{
		VL_IIC_Stop();															// �ͷ�����
		return 1;																// ûӦ�����˳�
	}
	VL_IIC_Send_Byte(REG_Address);
	VL_IIC_Wait_Ack();

	VL_IIC_Start();
	VL_IIC_Send_Byte(SlaveAddress|0x01);										// ��������
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
	VL_IIC_Stop();																// �ͷ�����
	return 0;
}


/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL53L0X_write_multi()
*
*�������ã�	��VL53L0x��������д��������
*
*����˵����	SlaveAddress����ȡ�豸�ĵ�ַ
*			REG_Address����ȡ�Ĵ����ĵ�ַ��
*			len����ȡ���ݳ���
*			REG_data�����ص�����
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	VL53L0X_read_multi()
*
*�������ã�	��VL53L0x����������ȡ�������
*
*����˵����	SlaveAddress����ȡ�豸�ĵ�ַ
*			REG_Address����ȡ�Ĵ����ĵ�ַ��
*			len����ȡ���ݳ���
*			REG_data�����ص�����
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	VL53L0X_write_byte()
*
*�������ã�	д1������
*
*����˵����	address��д���ַ
*			index���Ĵ�������
*			data��д�������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data)
{
	uint8_t status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL53L0X_write_word()
*
*�������ã�	д1�ָ�����
*
*����˵����	address��д���ַ
*			index���Ĵ�������
*			data��д�������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data)
{
	uint8_t status = STATUS_OK;
	
	uint8_t buffer[2];
	
	//��16λ���ݲ�ֳ�8λ
	buffer[0] = (uint8_t)(data>>8);												// �߰�λ
	buffer[1] = (uint8_t)(data&0xff);											// �Ͱ�λ	
	if(index%2==1)
	{  
		//����ͨ�Ų��ܴ����Է�2�ֽڶ���Ĵ������ֽ�
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
*�������ƣ�	VL53L0X_write_dword()
*
*�������ã�	д1�ָ�����
*
*����˵����	address��д���ַ
*			index���Ĵ�������
*			data��д�������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data)
{
	
    uint8_t status = STATUS_OK;

    uint8_t buffer[4];	
	
	//��32λ���ݲ�ֳ�8λ
	buffer[0] = (uint8_t)(data>>24);
	buffer[1] = (uint8_t)((data&0xff0000)>>16);
	buffer[2] = (uint8_t)((data&0xff00)>>8);
	buffer[3] = (uint8_t)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
}


/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL53L0X_read_byte()
*
*�������ã�	��ȡһ���ֽ�
*
*����˵����	address����ȡ��ַ
*			index���Ĵ�������
*			data����ȡ������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/ 
uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata)
{
	uint8_t status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;	 
}

/********************************************************************************
*               VL53L0x_I2c.c
*�������ƣ�	VL53L0X_read_word()
*
*�������ã�	��ȡһ����
*
*����˵����	address����ȡ��ַ
*			index���Ĵ�������
*			data����ȡ������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	VL53L0X_read_dword()
*
*�������ã�	��ȡ2����
*
*����˵����	address����ȡ��ַ
*			index���Ĵ�������
*			data����ȡ������
*
*�������أ�	Ӧ��0��û��Ӧ��1
*
*�������ߣ�	ZhaoSir
********************************************************************************/
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata)
{
	uint8_t status = STATUS_OK;
	
	uint8_t buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((uint32_t)buffer[0]<<24)+((uint32_t)buffer[1]<<16)+((uint32_t)buffer[2]<<8)+((uint32_t)buffer[3]);
	
	return status;
}


