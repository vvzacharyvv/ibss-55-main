/*
 * VOFA.c
 *
 *  Created on: 2020��8��14��
 *      Author: WHY
 */
#include "VOFA.h"
#include "gait.h"

float my_data_buffer[20] = {0};//�����͸������ݻ�����
uint8_t Send_channels,ii;
uint8_t text_success1[32]={'1','c','h','a','n','g','e','1'};
uint8_t text_success2[32]={'1','c','h','a','n','g','e','2'};
uint8_t text_success3[32]={'1','c','h','a','n','g','e','3'};
uint8_t text_success4[32]={'1','c','h','a','n','g','e','4'};
uint8_t text_success5[32]={'1','c','h','a','n','g','e','5'};
uint8_t float_to_uint8(float num, uint8_t bit)
{
	uint8_t* temp;

	temp = (uint8_t *) &num;
	return  *(temp + bit);
}
float uint8_to_float(uint8_t* p)
{
	float temp;
	*((uint8_t*)&temp) = *p;
	*((uint8_t*)&temp+1) = *(p+1);
	*((uint8_t*)&temp+2) = *(p+2);
	*((uint8_t*)&temp+3) = *(p+3);
	return temp;
}
//����1�жϺ���Ҫִ�еĺ���
void example_uart_callback(uint8_t userData)
{
    static uint8_t state = 0,arg_num = 0,recv_char[4];
    float recv_float;

	//volt+����״̬��
	switch(state)
	{
		case 0:
			if(userData == 0x23)//֡ͷ��һ�ֽ�
				state = 1;
			else
				state = 0;
			break;
		case 1:
			if(userData == 0x02)//֡ͷ�ڶ��ֽ�
				state = 2;
			else
				state = 0;
			break;
		case 2:
			arg_num = userData;//�������
			state = 3;
			break;
		case 3:
			recv_char[0] = userData;//���ݵ�һ�ֽ�
			state = 4;
			break;
		case 4:
			recv_char[1] = userData;//���ݵڶ��ֽ�
			state = 5;
			break;
		case 5:
			recv_char[2] = userData;//���ݵ����ֽ�
			state = 6;
			break;
		case 6:
			recv_char[3] = userData;//���ݵ����ֽ�
			state = 7;
			break;
		case 7:
			//flag_isrun=userData;
		if(userData=='1'||userData=='0')  flag_isrun=userData;
		else flag_isrun=flag_isrun;
		
			state = 8;
			break;
		case 8:
			if(userData == 0xaa)//֡β
			{
				recv_float = uint8_to_float(recv_char);//���ֽ�ת��Ϊ������

				switch(arg_num)
				{
					case 1:
						my_data_buffer[1] = recv_float;
						NRF24L01_TX_Mode();
					NRF24L01_TxPacket(text_success1);
					NRF24L01_RX_Mode();
						//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
						//mc.targetCoMVelocity.element[0][0] = recv_float;
//						pid2.kp = recv_float;
//						if(flash_check(0,0))	eeprom_erase_sector(0);
//						write_buf=float_conversion_uint32(pid2.kp);
//						eeprom_page_program(0, 0, &write_buf);
//						//write_pid_to_flash();
						break;
					case 2:		//	targetCoMVelocity
						mc.targetCoMVelocity.element[ recv_char[0] ][0] = recv_float;
						NRF24L01_TX_Mode();
					NRF24L01_TxPacket(text_success2);
					NRF24L01_RX_Mode();
						break;
					case 3:	//	gaitMode
						updateStatus = 1;
						gaitMode = recv_char[3];
						NRF24L01_TX_Mode();
					NRF24L01_TxPacket(text_success3);
					NRF24L01_RX_Mode();
						break;
					case 4:	//	timeGait
						updateStatus = 1;
						timeGaitUpdate = recv_float;
						NRF24L01_TX_Mode();
					NRF24L01_TxPacket(text_success4);
					NRF24L01_RX_Mode();
						break;
					case 5:	//	the pressure height in the end of swing phase
						updateStatus = 1;
						for(int i=0; i<4; i++)	//	LF RF LH RH
							pressHightBuffer[i]= (float)recv_char[i];
					NRF24L01_TX_Mode();
					NRF24L01_TxPacket(text_success5);
					NRF24L01_RX_Mode();
						break;
						
					case 6:

						break;
				}
			}
			state = 0; //״̬��λ���ȴ���һ�ν���
			break;
	}
}

//���´����͸������ݻ�����
void store_my_data_buffer(void)
{
	my_data_buffer[0] = 123;//mc.targetCoMVelocity.element[0][0];
	// my_data_buffer[1] = 666.6;
//	my_data_buffer[2] = realSpeed30;
//	my_data_buffer[3] = realSpeed40;
//	my_data_buffer[4] = pid2.kp;
//	my_data_buffer[5] = pid2.ki;
//	my_data_buffer[6] = XY.y;
//	my_data_buffer[7] = distance00;
//	my_data_buffer[8] = distance11;

	Send_channels=4;	// max 6
}

//volt+��λ�����ͺ���
//buffer_addr  	  ����float���ݵ�ַ
//channels     	  ����float����ͨ����
//my_data_buffer �����͸������ݻ�����
void vofa_send_lines(void)
{
	uint32_t j,k = 0;//i = 1,
	uint8_t buffer_to_send[100];

	ii=1;

	store_my_data_buffer();//���´����͸������ݻ�����
	while(k < Send_channels)
	{
		for(j = 0;j < 4;j++)
		{
			buffer_to_send[ii] = float_to_uint8(* (my_data_buffer + k),j);
			ii++;
		}
		k++;
	}

    //���ֽ�֡β
	buffer_to_send[ii++] = 0x00;
	buffer_to_send[ii++] = 0x00;
	buffer_to_send[ii++] = 0x80;
	buffer_to_send[ii++] = 0x7f;
	
	//NRF24L01_TX_Mode();
	//HAL_UART_Transmit(&huart7,buffer_to_send,i,0xffff);//sizeof(buffer_to_send)
	NRF24L01_TxPacket(buffer_to_send);
}


////u8 isprogramrun(u8 *tmp_buf)
////{
////	if(*(tmp_buf+1)=='=')
////	{
////		if(*(tmp_buf+2)=='2')
////		{
////			if(*(tmp_buf+4)=='4')
////			{
////				if(*(tmp_buf+3)=='3')
////				{
////					return 1;
////				}
////				if(*(tmp_buf+3)=='5')
////				{
////					return 0;
////				}

////			}
////		}


////	}

////}
