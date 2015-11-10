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
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.串口1初始化
2.供参数打印回串口助手，安卓4.0以上版本蓝牙透传接口以及和PC上位机接口
3.提供标准输入输出printf()的底层驱动，也就是说printf可以直接调用
------------------------------------
*/
#include "UART1.h"
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

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 




/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功　　能:		串口1中断配置
输入参数：无
输出参数：没有	
*******************************************************************************/
void UART1NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure; 
        /* Enable the USART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}



/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART1
输入参数：u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void UART1_init(u32 pclk2,u32 bound)
{  	 

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	
  USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
 
  UART1NVIC_Configuration();//中断配置
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                  
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  
  
  UartTxbuf.Wd_Indx = 0;
  UartTxbuf.Rd_Indx = 0;
  UartTxbuf.Mask = TX_BUFFER_SIZE - 1;
  UartTxbuf.pbuf = &tx_buffer[0];
  
  UartRxbuf.Wd_Indx = 0;
  UartRxbuf.Rd_Indx = 0;
  UartRxbuf.Mask = RX_BUFFER_SIZE - 1;
  UartRxbuf.pbuf = &rx_buffer[0];
  
  
  printf("MCU clock frequency:%dMHz \r\n",pclk2);
  printf("UART 1 baud frequncy:%d \r\n",bound);
 
  
}


/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
  
  UartBuf_WD(&UartTxbuf,DataToSend);//将待发送数据放在环形缓冲数组中
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据
}


uint8_t Uart1_Put_Char(unsigned char DataToSend)
{
  UartBuf_WD(&UartTxbuf,DataToSend);//将待发送数据放在环形缓冲数组中
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据
	return DataToSend;
}

//环形 数组结构体实例化两个变量
UartBuf UartTxbuf;//环形发送结构体
UartBuf UartRxbuf;//环形接收结构体

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char tx_buffer[TX_BUFFER_SIZE];

//读取环形数据中的一个字节
uint8_t UartBuf_RD(UartBuf *Ringbuf)
{
  uint8_t temp;
  temp = Ringbuf->pbuf[Ringbuf->Rd_Indx & Ringbuf->Mask];//数据长度掩码很重要，这是决定数据环形的关键
  Ringbuf->Rd_Indx++;//读取完成一次，读指针加1，为下一次 读取做 准备
  return temp;
}
//将一个字节写入一个环形结构体中
void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn)
{
  
  Ringbuf->pbuf[Ringbuf->Wd_Indx & Ringbuf->Mask] = DataIn;//数据长度掩码很重要，这是决定数据环形的关键
  Ringbuf->Wd_Indx++;//写完一次，写指针加1，为下一次写入做准备

}
//环形数据区的可用字节长度，当写指针写完一圈，追上了读指针
//那么证明数据写满了，此时应该增加缓冲区长度，或者缩短外围数据处理时间
uint16_t UartBuf_Cnt(UartBuf *Ringbuf)
{
  return (Ringbuf->Wd_Indx - Ringbuf->Rd_Indx) & Ringbuf->Mask;//数据长度掩码很重要，这是决定数据环形的关键
}

void UartBufClear(UartBuf *Ringbuf)
{
	Ringbuf->Rd_Indx=Ringbuf->Wd_Indx;
}

void UartSendBuffer(uint8_t *dat, uint8_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		UartBuf_WD(&UartTxbuf,*dat);
		dat++;
	}
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据
}



volatile uint8_t Udatatmp;//串口接收临时数据字节


//------------------------------------------------------
void USART1_IRQHandler(void)
{
  uint8_t i;
	
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    USART_SendData(USART1, UartBuf_RD(&UartTxbuf)); //环形数据缓存发送
    if(UartBuf_Cnt(&UartTxbuf)==0)  
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//假如缓冲空了，就关闭串口发送中断
  }
  
  else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除接收中断标志
		
    //此种环形缓冲数组串口接收方式，适用于解包各种数据，很方便。对数据的要求是:
    //发送方必须要求有数据包头，以便解决串口数据无地址的问题
    Udatatmp = (uint8_t) USART_ReceiveData(USART1);          //临时数据赋值
		//LedB_on;


		UartBuf_WD(&UartRxbuf,Udatatmp);               //写串口接收缓冲数组
    
  	CommApp(Udatatmp);

		
	}
	
  
}
//////////////
////////////
/////////////
///////////////
//USART2

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
	
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
  RCC->APB1ENR|=1<<17;  //使能串口时钟
	
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
           
    USART_SendData(USART2, Udatatmp);
  	CommApp(Udatatmp);

		
	}
	
  
}


