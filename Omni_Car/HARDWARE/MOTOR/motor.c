#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
函数功能：电机方向引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;            //定义结构体GPIO_InitStructure

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); // 使能PE端口时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽，增大电流输出能力
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //IO口速度
    GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE初始化
}
/**************************************************************************
Function: The motor PWM initialization
Input   : psc: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：电机PWM引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数
返回  值：无
**************************************************************************/
void PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;                //定义结构体GPIO_InitStructure
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;      //定义结构体TIM_TimeBaseStructure
    TIM_OCInitTypeDef TIM_OCInitStructure;              //定义结构体TIM_OCInitStructure

    // ===== 初始化TIM3的PWM输出，用于MOTOR_A和MOTOR_B =====
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PC端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //使能AFIO时钟

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用模式输出
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PC7 、PC8、PC9 (TIM3完全重映射后的CH2、CH3、CH4)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //IO口速度
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  //GPIO初始化
    
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //TIM3完全重映射

    TIM_TimeBaseStructure.TIM_Period = arr;                //设置下一个更新活动的自动重装载寄存器的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;             //预分配值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;           //时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //PWM脉冲宽度调制1
    TIM_OCInitStructure.TIM_Pulse = 0;                           //设置待装入捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    //设置TIM输出极性为高
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能预装载寄存器

    TIM_ARRPreloadConfig(TIM3, ENABLE);             //使能自动装载允许位
    TIM_Cmd(TIM3, ENABLE); //启动定时器3
}



