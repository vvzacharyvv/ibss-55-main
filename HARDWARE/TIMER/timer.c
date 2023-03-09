#include "timer.h"
#include "uart.h"
#include "adc.h"
#include "SV.h"
#include "gait.h"
#include "matrix.h"
#include "VOFA.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//��ʱ���ж���������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved													  
////////////////////////////////////////////////////////////////////////////////// 	

TIM_HandleTypeDef TIM5_Handler;    
TIM_HandleTypeDef TIM2_Handler;      
TIM_HandleTypeDef TIM7_Handler;     
TIM_OC_InitTypeDef TIM2_CH2Handler;

int TIM5_20msFlag, TIM2_100msFlag;
//u8 mesg[4]={0X03,0X02,0XD0,0X12};	
//u8 canbuf1[8]={0};
//u8 canbuf2[8]={0};
//u8 canbuf3[4]={0};


void TIM7_Init(u16 arr,u16 psc)
{  
    TIM7_Handler.Instance=TIM7;                          
    TIM7_Handler.Init.Prescaler=psc;                     //��Ƶ
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM7_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
   
		HAL_TIM_Base_Init(&TIM7_Handler);    
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //ʹ�ܶ�ʱ���Ͷ�ʱ�������жϣ�TIM_IT_UPDATE   
}

void TIM5_Init(u16 arr,u16 psc)
{  	
    TIM5_Handler.Instance=TIM5;                          
    TIM5_Handler.Init.Prescaler=psc;                    
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;  
    TIM5_Handler.Init.Period=arr;                     
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
   
		HAL_TIM_Base_Init(&TIM5_Handler);    
    HAL_TIM_Base_Start_IT(&TIM5_Handler); 
}

void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                        
    TIM2_Handler.Init.Prescaler=psc;                    
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    
    TIM2_Handler.Init.Period=arr;                      
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
   
		HAL_TIM_Base_Init(&TIM2_Handler);    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); 
  
		HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM5)
	{
		__HAL_RCC_TIM5_CLK_ENABLE();           

		HAL_NVIC_SetPriority(TIM5_IRQn,0,1);    //�����ж����ȼ�����ռ���ȼ��������ȼ�
		HAL_NVIC_EnableIRQ(TIM5_IRQn);          //�����ж�   
	}  
	   if(htim->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();           

		HAL_NVIC_SetPriority(TIM2_IRQn,1,2);   
		HAL_NVIC_EnableIRQ(TIM2_IRQn);         
	} 
		if(htim->Instance==TIM7)
	{
		__HAL_RCC_TIM7_CLK_ENABLE();         

		HAL_NVIC_SetPriority(TIM7_IRQn,1,3);   
		HAL_NVIC_EnableIRQ(TIM7_IRQn);          
	} 	
}

//��ʱ���жϷ�����
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM7_Handler);
		
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM5_Handler);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}


extern struct CreepMotionControl mc;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t NRF24L01buf[4]={0};
	
	if(htim==(&TIM5_Handler))
	{
		TIM5_20msFlag++;
		
//		float t_tCV[]={ 3.0,		//x
//										0.0,		//y
//										0.0};		//z
//		CLASSMC_setCoMVel(&mc, t_tCV);		

//		air_control_amble(&mc);
//		// air_control_trot(&mc);

//		CLASSMC_nextStep(&mc);
//		CLASSMC_inverseKinematics(&mc);
//		CLASSMC_setJointPosition(&mc);

	}
	if(htim==(&TIM2_Handler))
	{			
		TIM2_100msFlag++;
		
		
//		Read_All_Ad();			
//		battery_indicatior();
		
//		if(NRF24L01_RxPacket(NRF24L01buf)==0)
//		{
//			for(int k=1;k<=NRF24L01buf[0];k++)
//				example_uart_callback( NRF24L01buf[k]);
//			vofa_send_lines();
//			NRF24L01_RX_Mode();	
//			HAL_Delay(5);
//		}
		
	}
	if(htim==(&TIM7_Handler))
	{	

	}
}