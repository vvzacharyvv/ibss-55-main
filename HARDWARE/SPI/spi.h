#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "stm32h7xx_hal_spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//SPI��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern SPI_HandleTypeDef SPI3_Handler;  //SPI���

void SPI3_Init(void);
void SPI3_SetSpeed(u32 SPI_BaudRatePrescaler);
u8 SPI3_ReadWriteByte(u8 TxData);
#endif
