#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 
//HALLIB add 
//stm32h7xx_hal_spi.c
//stm32h7xx_hal_spi_ex.c
////////////////////////////////////////////////////////////////////////////////// 	

SPI_HandleTypeDef SPI3_Handler;  //SPI3���

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI3�ĳ�ʼ��
void SPI3_Init(void)
{
    SPI3_Handler.Instance=SPI3;                      //SP3
    SPI3_Handler.Init.Mode=SPI_MODE_MASTER;          //����SPI����ģʽ������Ϊ��ģʽ
    SPI3_Handler.Init.Direction=SPI_DIRECTION_2LINES;//����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI3_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI3_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH; //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI3_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;      //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI3_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI3_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS�ź�����ʧ��
    SPI3_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI��ģʽIO״̬����ʹ��
    SPI3_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI3_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI3_Handler.Init.TIMode=SPI_TIMODE_DISABLE;     //�ر�TIģʽ
    SPI3_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI3_Handler.Init.CRCPolynomial=7;               //CRCֵ����Ķ���ʽ
    HAL_SPI_Init(&SPI3_Handler);


//	HAL_NVIC_SetPriority(SPI3_IRQn, 3, 0);
//	HAL_NVIC_EnableIRQ(SPI3_IRQn);		
    __HAL_SPI_ENABLE(&SPI3_Handler);                 //ʹ��SPI3
    SPI3_ReadWriteByte(0Xff);                        //��������
}

//SPI3�ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_SPI_Init()����
//hspi:SPI���
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    RCC_PeriphCLKInitTypeDef SPI3ClkInit;
	
    __HAL_RCC_GPIOC_CLK_ENABLE();                   //ʹ��GPIOCʱ��
    __HAL_RCC_SPI3_CLK_ENABLE();                    //ʹ��SPI3ʱ��
    
	//����SPI3��ʱ��Դ 
	SPI3ClkInit.PeriphClockSelection=RCC_PERIPHCLK_SPI3;	    //����SPI3ʱ��Դ
	SPI3ClkInit.Spi123ClockSelection=RCC_SPI123CLKSOURCE_PLL;	//SPI3ʱ��Դʹ��PLL1Q
	HAL_RCCEx_PeriphCLKConfig(&SPI3ClkInit);
	
    //PC10,11,12
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //����    
    GPIO_Initure.Alternate=GPIO_AF6_SPI3;           //����ΪSPI3
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);             //��ʼ��
}

//SPI�ٶ����ú���
//SPI�ٶ�=PLL1Q/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_256
//PLL1Qʱ��һ��Ϊ200Mhz��
void SPI3_SetSpeed(u32 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    __HAL_SPI_DISABLE(&SPI3_Handler);            //�ر�SPI
    SPI3_Handler.Instance->CFG1&=~(0X7<<28);     //λ30-28���㣬�������ò�����
    SPI3_Handler.Instance->CFG1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
    __HAL_SPI_ENABLE(&SPI3_Handler);             //ʹ��SPI
    
}

//SPI3 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI3_ReadWriteByte(u8 TxData)
{
    u8 Rxdata;
    HAL_SPI_TransmitReceive(&SPI3_Handler,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //�����յ�������		
}

///////////////IRQ
//u8 SPI3_IRQHandler(u8 TxData)
//{
//	u8 Rxdata;
//	HAL_SPI_IRQHandler(&SPI3_Handler);
//	//HAL_SPI_TransmitReceive_IT(&SPI3_Handler,&TxData,&Rxdata,1);
//	HAL_SPI_Receive_IT(&SPI3_Handler,&Rxdata,1);
//	return Rxdata;  
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    volatile uint8_t y = 5;
//}