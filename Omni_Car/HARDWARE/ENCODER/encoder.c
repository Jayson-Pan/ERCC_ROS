#include "encoder.h"

/*******************************************************
Function:Initialize TIM2 to encoder interface mode
Input   ;none
Output  :none
函数    ：把TIM2初始化为编码器接口模式
入口参数：无
返回值  ：无
********************************************************/
void Encoder_Init_Tim2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器4的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PB端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          //根据设定参数初始化GPIOB

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;    //滤波10
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}


/**************************************************************************
函数功能：读取TIM2编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int Read_Encoder_TIM2(void)
{
    int Encoder_TIM;
    Encoder_TIM = TIM2->CNT; //读取计数
    if (Encoder_TIM > 0xefff)Encoder_TIM = Encoder_TIM - 0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
    //TIM4->CNT范围为0-0xffff，初值为0。
    TIM2->CNT = 0; //读取完后计数清零
    return Encoder_TIM; //返回值
}


/**************************************************************************
Function: TIM2 interrupt service function
Input   : none
Output  : none
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 0X0001) //溢出中断
    {
    }
    TIM2->SR &= ~(1 << 0); //清除中断标志位
}

/**************************************************************************
Function: Initialize TIM4 to encoder interface mode
Input   : none
Output  : none
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_Tim4(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);//使能PD端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


    TIM_DeInit(TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;  //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);                          

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}


/**************************************************************************
函数功能：读取TIM4编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int Read_Encoder_TIM4(void)
{
    int Encoder_TIM;
    Encoder_TIM = TIM4->CNT; //读取计数
    if (Encoder_TIM > 0xefff)Encoder_TIM = Encoder_TIM - 0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
    //TIM4->CNT范围为0-0xffff，初值为0。
    TIM4->CNT = 0; //读取完后计数清零
    return Encoder_TIM; //返回值
}

/**************************************************************************
Function: TIM4 interrupt service function
Input   : none
Output  : none
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{
    if (TIM4->SR & 0X0001) //溢出中断
    {
    }
    TIM4->SR &= ~(1 << 0); //清除中断标志位
}

/**************************************************************************
Function: Initialize TIM1 to encoder interface mode
Input   : none
Output  : none
函数功能：把TIM1初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_Tim1(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能定时器1的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);//使能PE端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    TIM_DeInit(TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;  //PE9、PE11端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOE, &GPIO_InitStructure);                          

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //TIM1完全重映射

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    
    // 高级定时器需要使能主输出，即使在编码器模式下也需要
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM的更新标志位
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**************************************************************************
函数功能：读取TIM1编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int Read_Encoder_TIM1(void)
{
    int Encoder_TIM;
    Encoder_TIM = TIM1->CNT; //读取计数
    if (Encoder_TIM > 0xefff)Encoder_TIM = Encoder_TIM - 0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
    //TIM1->CNT范围为0-0xffff，初值为0。
    TIM1->CNT = 0; //读取完后计数清零
    return Encoder_TIM; //返回值
}

/**************************************************************************
Function: TIM1 interrupt service function
Input   : none
Output  : none
函数功能：TIM1中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM1_UP_IRQHandler(void)
{
    if (TIM1->SR & 0X0001) //溢出中断
    {
    }
    TIM1->SR &= ~(1 << 0); //清除中断标志位
}

