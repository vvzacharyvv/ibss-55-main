from dairus for ibss_11
air control and servo 
PCB_1.15

#PCB_3.1
NRF24L01ģ��(SPI3(PC10/PC11/PC12)/IRQ(PA10)/CS(PA15)/CE(PA9))
stm32��Դ����PC12��ΪPC13	SV.c SV_Init
PWM_pump (PB8) ��Ϊ ��PWM_pump1 (PB1)
��SV_all0 (PC8) ��Ϊ ��PWM_pump2 (PB2)
��SV_all1 (PC9) ��Ϊ ��PWM_pump3 (PB3)
��� ��PWM_pump4 (PB4)
ɾ�� UART7��MCU

��																		��
P_xx_2	������												P_xx_2	������
P_xx_1	������						h						P_xx_1	������	

P_PWM_xx_3 P_PWM_xx_2 P_PWM_xx_1 		P_PWM_xx_1 P_PWM_xx_2 P_PWM_xx_3 
	0			1			2				5			4			3
	
	1	4
2	fornt	5
	0	3
	
	6	9
8	hind	11
	7	10
	
	