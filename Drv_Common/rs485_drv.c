#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"

typedef struct
{
	USART_TypeDef * uartNo;
	uint32_t rcuUart;
	uint32_t rcuGpio;
	GPIO_TypeDef* gpio;
	uint32_t txPin;
	uint32_t rxPin;
	uint8_t irq;
} UartHwInfo_t;

static UartHwInfo_t g_uartHwInfo = {USART1, RCC_APB2Periph_USART1, RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_9, GPIO_Pin_10, USART1_IRQn};


static void GpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	

  RCC_APB2PeriphClockCmd(g_uartHwInfo.rcuGpio , ENABLE);
  /*Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = g_uartHwInfo.txPin  ;				
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	  
  GPIO_Init(g_uartHwInfo.gpio  ,&GPIO_InitStructure);
  /*Configure USART Rx as input floating*/
  GPIO_InitStructure.GPIO_Pin = g_uartHwInfo.rxPin   ;				
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	  
  GPIO_Init(g_uartHwInfo.gpio  ,&GPIO_InitStructure);
}

static void UartInit(uint32_t baudRate)
{
	USART_InitTypeDef USART_InitStructure;
  
  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  RCC_APB2PeriphClockCmd(g_uartHwInfo.rcuUart ,ENABLE );
    /* 复位UART */
  USART_DeInit(g_uartHwInfo.uartNo );
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* USART configuration */
  USART_Init(g_uartHwInfo.uartNo , &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(g_uartHwInfo.uartNo, ENABLE);
}

#define SWITCH_RS485_TO_RX()           GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define SWITCH_RS485_TO_TX()           GPIO_SetBits(GPIOC, GPIO_Pin_10)

static void SwitchInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;				
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	  
  GPIO_Init(GPIOC ,&GPIO_InitStructure);
                                
}

void RS485DrvInit(void)
{
  GpioInit();
  UartInit(9600);
  SwitchInit();	
}

static bool ReceiveByte(uint8_t *key)
{
  if ( USART_GetFlagStatus(g_uartHwInfo.uartNo, USART_FLAG_RXNE) != RESET)
  {
    *key = (uint8_t)g_uartHwInfo.uartNo->DR;
    return true ;
  }
  else
  {
    return false ;
  }
}

int32_t ReceiveByteTimeout(uint8_t *c, uint32_t timeout)
{
	while (timeout-- > 0)
	{
		if (ReceiveByte(c))
		{
			return 0;
		}
	}
	return -1;
}

/**
  * @brief  Test to see if a key has been pressed on the HyperTerminal
  * @param  key: The key pressed
  * @retval 1: Correct
  *         0: Error
  */
bool GetKeyPressed(uint8_t *key)
{
	return ReceiveByte(key);
}


/**
  * @brief  Print a character on the HyperTerminal
  * @param  c: The character to be printed
  * @retval None
  */
static void SerialPutChar(uint8_t c)
{
  SWITCH_RS485_TO_TX();
  USART_SendData(g_uartHwInfo.uartNo, (uint8_t)c);
  while(RESET == USART_GetFlagStatus(g_uartHwInfo.uartNo, USART_FLAG_TC));
  SWITCH_RS485_TO_RX();
}

void SendByte(uint8_t c)
{
	SerialPutChar(c);
}

/**
***********************************************************
* @brief printf函数默认打印输出到显示器，如果要输出到串口，
		 必须重新实现fputc函数，将输出指向串口，称为重定向
* @param
* @return 
***********************************************************
*/
int fputc(int ch, FILE *f)
{
	SWITCH_RS485_TO_TX();
  USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_SendData(g_uartHwInfo.uartNo, (uint8_t)ch);
	while (RESET == USART_GetFlagStatus(g_uartHwInfo.uartNo, USART_FLAG_TC));
	SWITCH_RS485_TO_RX();
	return ch;
}
