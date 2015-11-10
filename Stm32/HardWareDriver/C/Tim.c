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
Tim.c file
编写者：小马  (Camel)、Nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.初始化定时器3和定时器4
2.定时器3-->串口打印各种参数（生产调试使用，默认情况下串口输出上位机数据）
3.定时器4-->飞控主循环基准定时器，属于关键中断，将定时器4的主优先级以及从优先级设为最高很有必要
------------------------------------
*/
#include "tim.h"
#include "config.h"
#include "imu.h"

#define TASK_TICK_FREQ				1000			//Hz 主任务频率

uint16_t cntBaro=0;
uint16_t cntBatChk=0;

int LedCounter;//LED闪烁计数值
float Compass_HMC[3];

uint8_t accUpdated=0;
volatile uint16_t anyCnt=0,anyCnt2=0;
uint8_t  loop500HzFlag,loop200HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop20HzFlag,loop10HzFlag;
volatile uint16_t loop500Hzcnt,loop200HzCnt,loop50HzCnt , loop600HzCnt,loop100HzCnt, loop20HzCnt , loop10HzCnt=0;


//控制入口
void TIM2_IRQHandler(void)		//1ms中断一次
{
    if( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
    {     
					anyCnt++;
					loop200HzCnt++;
					loop100HzCnt++;

					if(++loop50HzCnt * 50 >= (1000))
					{
							loop50HzCnt=0;
							loop50HzFlag=1;
					}
					if(++loop20HzCnt * 20 >=1000 )
					{
							loop20HzCnt=0;
							loop20HzFlag=1;
					}
					if(++loop10HzCnt * 10 >=1000 )
					{
							loop10HzCnt=0;
							loop10HzFlag=1;
					}
          
          TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);   //清除中断标志   
    }
}



int DebugCounter;             //打印信息输出时间间隔计数值
extern u8 RX_ADDRESS[5];




//定时器4初始化：用来中断处理PID
void TIM4_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM2);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2,ENABLE);
    printf("Timer 2 init success...\r\n");
    
}	



void TimerNVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//飞控主循环基准定时器，优先级高于串口打印
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 

