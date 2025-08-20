#include "usartx.h"

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;

/**************************************************************************
Function: Usartx1 send data task
Input   : none
Output  : none
函数功能：串口1发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while (1)
    {
        //The task is run at 20hz
        //此任务以20Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
            
        //Assign the data to be sent
        //对要进行发送的数据进行赋值
        data_transition();

        USART1_SEND();
    }
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
    Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
    Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾

    //Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
    //运动学正解，从各车轮当前速度求出三轴当前速度（旋转180度后）
    // New model: A wheel at bottom, B wheel at upper-right, C wheel at upper-left
    // X-axis positive direction: from A wheel towards BC wheels
    // 新模型：A轮在下方，B轮右上方，C轮左上方，X轴正方向从A轮指向BC轮
    Send_Data.Sensor_Str.X_speed = ((MOTOR_B.Encoder-MOTOR_C.Encoder)/2/X_PARAMETER)*1000;
	Send_Data.Sensor_Str.Y_speed = ((-MOTOR_A.Encoder*2+MOTOR_B.Encoder+MOTOR_C.Encoder)/3)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder)/3/Omni_radiaus)*1000;

    //The acceleration of the triaxial acceleration //加速度计三轴加速度
    Send_Data.Sensor_Str.Accelerometer.X_data = accel[1]; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
    Send_Data.Sensor_Str.Accelerometer.Y_data = -accel[0]; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
    Send_Data.Sensor_Str.Accelerometer.Z_data = accel[2]; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴

    //The Angle velocity of the triaxial velocity //角速度计三轴角速度
    Send_Data.Sensor_Str.Gyroscope.X_data = gyro[1]; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
    Send_Data.Sensor_Str.Gyroscope.Y_data = -gyro[0]; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
    if (Flag_Stop == 0)
        //If the motor control bit makes energy state, the z-axis velocity is sent normall
        //如果电机控制位使能状态，那么正常发送Z轴角速度
        Send_Data.Sensor_Str.Gyroscope.Z_data = gyro[2];
    else
        //If the robot is static (motor control dislocation), the z-axis is 0
        //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
        Send_Data.Sensor_Str.Gyroscope.Z_data = 0;

    //Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
    //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
    Send_Data.Sensor_Str.Power_Voltage = Voltage * 1000;

    Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
    Send_Data.buffer[1] = Flag_Stop; //Car software loss marker //小车软件失能标志位

    //The three-axis speed of / / car is split into two eight digit Numbers
    //小车三轴速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8;
    Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed ;
    Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8;
    Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;
    Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
    Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed ;

    //The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
    //IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8;
    Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;
    Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
    Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
    Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
    Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

    //The axis of the triaxial velocity of the / /imu is divided into two eight digits
    //IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8;
    Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;
    Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
    Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
    Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
    Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;

    //Battery voltage, split into two 8 digit Numbers
    //电池电压,拆分为两个8位数据发送
    Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8;
    Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;

    //Data check digit calculation, Pattern 1 is a data check
    //数据校验位计算，模式1是发送数据校验
    Send_Data.buffer[22] = Check_Sum(22, 1);

    Send_Data.buffer[23] = Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
    unsigned char i = 0;

    for (i = 0; i < 24; i++)
    {
        usart1_send(Send_Data.buffer[i]);
    }
}


/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    //Enable the gpio clock //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

    //USART_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //Reuse push-pull output //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //Float input //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;
    //Subpriority //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Sending and receiving mode //收发模式
    USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}



/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{
    static u8 Count = 0;
    u8 Usart_Receive;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        Usart_Receive = USART_ReceiveData(USART1);//Read the data //读取数据
        if (Time_count < 2500)
            // Data is not processed until 25 seconds after startup
            //开机25秒前不处理数据
            return 0;   //前期不进入中断

        //Fill the array with serial data
        //串口数据填入数组
        Receive_Data.buffer[Count] = Usart_Receive;

        //Ensure that the first data in the array is FRAME_HEADER
        //确保数组第一个数据为FRAME_HEADER
        if (Usart_Receive == FRAME_HEADER || Count > 0)
            Count++;
        else
            Count = 0;

        if (Count == 11) //Verify the length of the packet //验证数据包的长度
        {
            Count = 0; //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
            if (Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
            {
                //Data exclusionary or bit check calculation, mode 0 is sent data check
                //数据异或位校验计算，模式0是发送数据校验
                if (Receive_Data.buffer[9] == Check_Sum(9, 0))
                {

                    //Calculate the target speed of three axis from serial data, unit m/s
                    //从串口数据求三轴目标速度， 单位m/s
                    Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
                    Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
                    Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
                }
            }
        }
    }
    return 0;
}



/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low)
{
    //Data conversion intermediate variable
    //数据转换的中间变量
    short transition;

    //将高8位和低8位整合成一个16位的short型数据
    //The high 8 and low 8 bits are integrated into a 16-bit short data
    transition = ((High << 8) + Low);
    return
        transition / 1000 + (transition % 1000) * 0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
    USART1->DR = data;
    while ((USART1->SR & 0x40) == 0);
}


/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode)
{
    unsigned char check_sum = 0, k;

    //Validate the data to be sent
    //对要发送的数据进行校验
    if (Mode == 1)
        for (k = 0; k < Count_Number; k++)
        {
            check_sum = check_sum ^ Send_Data.buffer[k];
        }

    //Verify the data received
    //对接收到的数据进行校验
    if (Mode == 0)
        for (k = 0; k < Count_Number; k++)
        {
            check_sum = check_sum ^ Receive_Data.buffer[k];
        }
    return check_sum;
}

