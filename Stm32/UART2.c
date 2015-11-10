/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
UART1.c file
???:??  (Camel)
??E-mail:375836945@qq.com
????:MDK-Lite  Version: 4.23
????: 2014-01-28
??:
1.??1???
2.??????????,??4.0?????????????PC?????
3.????????printf()?????,????printf??????
------------------------------------
*/
#include "UART2.h"
#include "stdio.h"
#include "extern_variable.h"
#include "ReceiveData.h"
#include "Control.h"
#include "stm32f10x_it.h"
#include "math.h"
#include "CommApp.h"
#include "CommPC.h"
#include "Led.h"



//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40






/**************************????********************************************
*????:		void U1NVIC_Configuration(void)
*?  ?:		??1????
????:?
????:??	
*******************************************************************************/
void UART2NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}



/**************************????********************************************
*????:		void Initial_UART1(u32 baudrate)
*?  ?:		???UART1
????:u32 baudrate   ??RS232??????
????:??	
*******************************************************************************/
void UART2_init(u32 pclk2,u32 bound)
{  	 

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	RCC->APB2ENR|=1<<2;   //??PORTA???  
	RCC->APB2ENR|=1<<14;  //?????? 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
	
	USART_Cmd(USART2,ENABLE);
	
  USART2->CR1|=1<<8;    //PE????
	USART2->CR1|=1<<5;    //???????????	    	
 
  UART2NVIC_Configuration();//????
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  
  
  
  printf("MCU clock frequency:%dMHz \r\n",pclk2);
  printf("UART 1 baud frequncy:%d \r\n",bound);
 
  
}


volatile uint8_t Udatatmp;


//------------------------------------------------------
void USART2_IRQHandler(void)
{
  uint8_t i;
	
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
		/*
    USART_SendData(USART2, UartBuf_RD(&Uart2Txbuf)); 
    if(UartBuf_Cnt(&Uart2Txbuf)==0)  
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		*/
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }
  
  else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		
    Udatatmp = (uint8_t) USART_ReceiveData(USART2);          
		//LedB_on;
           
    
  	CommApp(Udatatmp);

		
	}
	
  
}


