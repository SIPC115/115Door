#ifndef _Led_H_
#define _Led_H_
#include "stm32f10x.h"


//
#define LED_NUM 4
#define LA	    0x01
#define LB      0x02
#define LC      0x04
#define LD      0x08
//
#define E_READY 		   0
#define E_CALI			   1
#define E_BAT_LOW		   2
#define E_CALI_FAIL	   3
#define E_LOST_RC 	   4
#define E_AUTO_LANDED  5
#define E_BatChg       6


typedef union{
	uint8_t byte;
	struct 
	{
			uint8_t A	:1;
		  uint8_t B	:1;
			uint8_t C	:1;
		  uint8_t D	:1;
			uint8_t reserved	:4;
	}bits;
}LEDBuf_t;

typedef struct Led_tt
{
uint8_t event;
uint8_t state;
uint16_t cnt;
}LED_t;

#define LedA_off    GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define LedA_on     GPIO_ResetBits(GPIOC, GPIO_Pin_13)


#define LedB_off    GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define LedB_on     GPIO_ResetBits(GPIOC, GPIO_Pin_14)


#define LedC_off    GPIO_SetBits(GPIOC, GPIO_Pin_15)
#define LedC_on     GPIO_ResetBits(GPIOC, GPIO_Pin_15)


#define LedD_off    GPIO_SetBits(GPIOC, GPIO_Pin_15)
#define LedD_on     GPIO_ResetBits(GPIOC, GPIO_Pin_15)

#define Door_Open    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define Door_Close     GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define LEDA_troggle GPIO_WriteBit(GPIOC,GPIO_Pin_13, !GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13))
#define LEDB_troggle GPIO_WriteBit(GPIOC,GPIO_Pin_14, !GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14))
#define LEDC_troggle GPIO_WriteBit(GPIOC,GPIO_Pin_15, !GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15))


extern LED_t LEDCtrl;

void LedInit(void);   //Led初始化函数外部声明
void LEDFSM(void);



#endif

