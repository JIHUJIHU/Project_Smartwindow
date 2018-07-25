/**
  ******************************************************************************
  * @file    USART/Interrupt/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "stm32f10x_rcc.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void Timer2_Init(void);

/* Private functions ---------------------------------------------------------*/
void delay_us(uint32_t nCount);
void delay_ms(uint32_t nCount);
void USART1_SEND(uint8_t i);
void putstr(char *str);

#include "nrf24.h"


// Define what part of demo will be compiled:
//   0 : disable
//   1 : enable
#define DEMO_RX_SINGLE      0 // Single address receiver (1 pipe)
#define DEMO_RX_MULTI       0 // Multiple address receiver (3 pipes)
#define DEMO_RX_SOLAR       0 // Solar temperature sensor receiver
#define DEMO_TX_SINGLE      0 // Single address transmitter (1 pipe)
#define DEMO_TX_MULTI       0 // Multiple address transmitter (3 pipes)
#define DEMO_RX_SINGLE_ESB  1 // Single address receiver with Enhanced ShockBurst (1 pipe)
#define DEMO_TX_SINGLE_ESB  0 // Single address transmitter with Enhanced ShockBurst (1 pipe)

#define stp GPIO_Pin_12
#define dir GPIO_Pin_13


// Kinda foolproof :)
#if ((DEMO_RX_SINGLE + DEMO_RX_MULTI + DEMO_RX_SOLAR + DEMO_TX_SINGLE + DEMO_TX_MULTI + DEMO_RX_SINGLE_ESB + DEMO_TX_SINGLE_ESB) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif

// Buffer to store a payload of maximum width
uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;

// Length of received payload
uint8_t payload_length;

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I) 
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Clear pending IRQ flags
        nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}


void delay_us(uint32_t nCount)
{
     int i;
     for(; nCount !=0; nCount--){
          for(i=0; i<10; i++)asm("NOP");
     }
}

void delay_ms(uint32_t nCount)
{
     for(; nCount !=0; nCount--)delay_us(1000);
}

#define DEMO_RX_SINGLE 1
#define DEMO_TX_SINGLE 0
/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
  
    /* �ý��� Ŭ��(Clock) ���� */
    RCC_Configuration();
    
    /* ���ͷ�Ʈ ��Ʈ�ѷ�(NVIC) ����*/
    NVIC_Configuration();

    GPIO_InitTypeDef PORT;
    SPI_InitTypeDef SPI;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |  RCC_APB2Periph_GPIOC |
                             RCC_APB2Periph_GPIOE, ENABLE);
    
    // Enable SPI2 peripheral
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

    // Enable SPI2 GPIO peripheral (PORTB)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
 
    GPIO_Configuration();
    /* USART ���� */
    USART1_Configuration();

    // nRF24 IRQ pin ����
    PORT.GPIO_Mode  = GPIO_Mode_Out_PP;
    PORT.GPIO_Speed = GPIO_Speed_2MHz;
    PORT.GPIO_Pin   = nRF24_IRQ_PIN;
    GPIO_Init(nRF24_IRQ_PORT, &PORT);

    // SPI pins (SPI2) Pin ����
    PORT.GPIO_Mode  = GPIO_Mode_AF_PP;
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    PORT.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &PORT);


    // SPI2 �ʱ�ȭ ����
    SPI.SPI_Mode = SPI_Mode_Master;
    SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI.SPI_CPOL = SPI_CPOL_Low;
    SPI.SPI_CPHA = SPI_CPHA_1Edge;
    SPI.SPI_CRCPolynomial = 7;
    SPI.SPI_DataSize = SPI_DataSize_8b;
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(nRF24_SPI_PORT, &SPI);
    SPI_NSSInternalSoftwareConfig(nRF24_SPI_PORT, SPI_NSSInternalSoft_Set);
    SPI_Cmd(nRF24_SPI_PORT, ENABLE);


    // nRF24L01 GPIO pins(CSN, CE) ����
    nRF24_GPIO_Init();

    // RX/TX ��� ����(��������)
    nRF24_CE_L();


    // nRF24 �ʱ�ȭ
    nRF24_Init();

    

// ���� ���� ��
#if (DEMO_RX_SINGLE)

    // ShockBurst  ���� : ��� ������ ����(0xFF)
    nRF24_DisableAA(0xFF);

    // ä�� ���� : 90 -> 2400 + 90 = 2490 MHz
    nRF24_SetRFChannel(90);

    // ������ ���ۼӵ�(Data Rate) : 1Mbps
    nRF24_SetDataRate(nRF24_DR_1Mbps);

    // CRC ����Ʈ ��(��������) : 1byte
    nRF24_SetCRCScheme(nRF24_CRC_1byte);

    // Address ������ : 5
    nRF24_SetAddrWidth(5);

    // RX PIPE#1 ����
    static const uint8_t nRF24_ADDR[] = { 'n', 'R', 'F', '6', '6' };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

    // ���� ��� ���� : Rx ���
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // transceiver ȯ��(Wake-up)
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // ���� ��� -> ���� ��� ����   
     nRF24_CE_H();
    // The main loop
    int nothing2 = 0,  nothing3 = 0, nothing4 = 0;
    int bflag1 = 0, bflag2 = 0;
    int x;
    while (1) {
        // Rx FiFo ������ ���¸� ����(Polling)�Ͽ� ������ ������ Ȯ����
      if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
        
              // Rx ������ �����͸� �޾ƿ� nRF24_payload ���ۿ� ����, Payload_length�� ����� ������ ���� ��ȯ
              // �Լ��� ������ pipe�� ���° ���� ���������� ��ȯ��
              pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
              
             if(nRF24_payload[0] == 66){
                //â�� ����
               if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)){                
                  while(1){
                    GPIO_SetBits(GPIOD,dir);
                    for(x=1;x<200;x++){
                      GPIO_SetBits(GPIOD,stp);
                      delay_ms(1);
                      GPIO_ResetBits(GPIOD,stp);
                      delay_ms(1);
                    }
                    if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)){
                      break;
                    }
                    delay_us(1500);
                  }
                }
             }
             
             else if(nRF24_payload[0] == 88){
                //â�� �ݱ�
                //����Ʈ ����ġ �� �޾ƿ���
              if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)){
               while(1){
                  GPIO_ResetBits(GPIOD,dir);
                  for(x=1;x<400;x++){
                    GPIO_SetBits(GPIOD,stp);
                    delay_ms(1);
                    GPIO_ResetBits(GPIOD,stp);
                    delay_ms(1);
                  }
                  if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)){
                      break;
                  }
                  delay_us(2000);
                 }
               }
             }

              // IRQ flags �ʱ�ȭ
              nRF24_ClearIRQFlags();

              // ���� �����͸� ���.
              putstr("RCV PIPE#");
              putstr(" PAYLOAD:>");
              USART1_SEND((nRF24_payload[0]%10)+'0');
              USART1_SEND((nRF24_payload[1]%10)+'0');
              USART1_SEND((nRF24_payload[2]%10)+'0');
              USART1_SEND((nRF24_payload[3]%10)+'0');
              USART1_SEND((nRF24_payload[4]%10)+'0');
        
              putstr("<\r\n");
    	}
    }

#endif // DEMO_RX_SINGLE
 
 
    

 // ��ư ���� ��
 #if (DEMO_TX_SINGLE)

    // ShockBurst ���� : ��� ������ ����(0xFF)
    nRF24_DisableAA(0xFF);

    // ä�� ���� : 90 -> 2400 + 90 = 2490 MHz
    nRF24_SetRFChannel(90);

    // ������ ���ۼӵ�(Data Rate) : 1Mbps
    nRF24_SetDataRate(nRF24_DR_1Mbps);

    // CRC ����Ʈ ��(��������) : 1byte
    nRF24_SetCRCScheme(nRF24_CRC_1byte);

    // Address ������ : 5
    nRF24_SetAddrWidth(5);

    // TX PIPE ����
    static const uint8_t nRF24_ADDR[] = { 'n', 'R', 'F', '6', '6' };
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // ���� ��� ���� : TX ���
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

   // ���� ��� -> ���� ��� ����
    nRF24_SetPowerMode(nRF24_PWR_UP);
    
    // The main loop
    int j = 66;
    int k = 88;
    int i = 0;
    int nothing = 0;
    payload_length = 5;
    while (1) {
      
      if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)){
        // ���� Packet ����
    	for (i = 0; i < payload_length; i++) {
    		nRF24_payload[i] = j;
    	}

    	// ��Ŷ ����
    	tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
        nRF24_CE_H();

    	switch (tx_res) {
                        //���� ����
			case nRF24_TX_SUCCESS:
				putstr("OK");
                                nothing++;
				break;
                        //���� Timeout
			case nRF24_TX_TIMEOUT:
				putstr("TIMEOUT");
				break;
                        //���� Retry �ִ�
			case nRF24_TX_MAXRT:
				putstr("MAX RETRANSMIT");
				break;
                        //���� �߻�
			default:
				putstr("ERROR");
				break;
		}
    	
      }
      
      if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)){
           // ���� Packet ����
          for (i = 0; i < payload_length; i++) {
                  nRF24_payload[i] = k;
          }

          // ��Ŷ ����
          tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
          nRF24_CE_H();

          switch (tx_res) {
                          //���� ����
                          case nRF24_TX_SUCCESS:
                                  putstr("OK");
                                  nothing++;
                                  break;
                          //���� Timeout
                          case nRF24_TX_TIMEOUT:
                                  putstr("TIMEOUT");
                                  break;
                          //���� Retry �ִ�
                          case nRF24_TX_MAXRT:
                                  putstr("MAX RETRANSMIT");
                                  break;
                          //���� �߻�
                          default:
                                  putstr("ERROR");
                                  break;
                  }
      }

    	// Wait ~0.5s
        putstr("\r\n");
    	delay_ms(300);
    }
#endif // DEMO_TX_SINGLE
}

/* Timer2 ���� */
void Timer2_Init(void)
{
  /* Timer ������ Structure ���� ����*/
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

   /* Timer2 Clock ���� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   
  
  /* USART1 ���ͷ�Ʈ Ȱ��ȭ(Enable) */
  //Reload = 65535, ī��Ʈ�� 0-65535 ���� �����ϰ� ���µ�
  TIM_TimeBaseStructure.TIM_Period = 65535;
  //Prescaler = 72 -> 72MHz -> 1MHz -> 1usec ���� 1�� ī��Ʈ
  TIM_TimeBaseStructure.TIM_Prescaler = 24;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

}

void USART1_SEND(uint8_t i)
{
    /* USART1 Send : 1Byte �����͸� �۽�*/
    USART_SendData(USART1, i);
    /* USART1 �����Ͱ� �۽� �Ϸ�� ������ ����� */
    /* USART_FLAG_TXE : Transmit data register empty flag */
    /* Tx ������ �������Ͱ� ������� ��� �ش� Flag�� SET(1) ���°� ��. ������ ���̶�� RESET ���� �̹Ƿ�*/
    /* �Ϸᰡ �ɶ����� USART_FLAG_TXE�� RESET(0)�� ��� While���� ���� ���*/
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);      
}

void putstr(char *str) 
{ 
    char ch;     
    while((ch=*str)!= '\0') { 
	USART_SendData(USART1, *str);     
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    
	str++; 
     } 
 }

void USART1_Configuration(void)
{
  /* USART ��� ���� ����ü �ۼ�*/
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* USART1 ����(����ü ����) */
  USART_Init(USART1, &USART_InitStructure);
  
  /* USART1 ���ͷ�Ʈ ����(RXNE : Receive Data register not empty interrupt) */
  /* �����͸� ���Ź޾� Receive Data register�� ���� �����Ͱ� ���� ��� ���ͷ�Ʈ �߻� */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* USART1 Ȱ��ȭ */
  USART_Cmd(USART1, ENABLE);

  /*�ۼ��� ��*/
  /* USART ��� ���� ����ü �ۼ�*/
  
  /* USART2 ����(����ü ����) */
  
  /* USART2 ���ͷ�Ʈ ����(RXNE : Receive Data register not empty interrupt) */
  /* �����͸� ���Ź޾� Receive Data register�� ���� �����Ͱ� ���� ��� ���ͷ�Ʈ �߻� */
  
  /* USART2 Ȱ��ȭ */

}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{   

    /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div1);


  /* GPIO Ŭ�� Ȱ��ȭ(Enable) */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
  
  /* USART1 Ŭ�� Ȱ��ȭ(Enable) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
  
  /* USART2 Ŭ�� Ȱ��ȭ(Enable) */
  /* �ۼ��� ��. */

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* USART1 Rx ���� Input Floating���� ����(Configuration) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
  
  /* USART1 Tx ���� Output Alternate Function Push-Pull���� ����(Configuration) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);

  /* USART1 Rx ���� Input Floating���� ����(Configuration) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* USART1 Tx ���� Output Alternate Function Push-Pull���� ����(Configuration) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  //����
  GPIO_InitStructure.GPIO_Pin = stp | dir;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD,&GPIO_InitStructure);

}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* USART1 ���ͷ�Ʈ Ȱ��ȭ(Enable) */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* USART2 ���ͷ�Ʈ Ȱ��ȭ(Enable) */
  /* �ۼ��� �� */
    
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/