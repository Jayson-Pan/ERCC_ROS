#include "control.h"

// Time variable
int Time_count = 0;

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X Y Z axis direction of the target movement speed
Output  : none
**************************************************************************/
void Drive_Motor(float Vx, float Vy, float Vz)
{
    static float amplitude = 3.5; // Wheel target speed limit

    // Motor A: rear left, Motor B: front right, Motor C: front left, X-axis direction: Motor A points to BC
    MOTOR_A.Target = -Vy + Omni_radiaus * Vz;
    MOTOR_B.Target = +X_PARAMETER * Vx + Y_PARAMETER * Vy + Omni_radiaus * Vz;
    MOTOR_C.Target = -X_PARAMETER * Vx + Y_PARAMETER * Vy + Omni_radiaus * Vz;

    // Wheel (motor) target speed limit
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
    MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
}
/**************************************************************************
Function: FreeRTOS task, core motion control task
Input   : none
Output  : none
**************************************************************************/
void Control_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while (1) {
        // This task runs at a frequency of 100Hz (10ms control once)
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        // Time count is no longer needed after 30 seconds
        if (Time_count < 3000) Time_count++;

        // Get the encoder data, that is, the real time wheel speed,
        // and convert to transposition international units
        Get_Velocity_Form_Encoder();

        // Do not enter the control before the end of self-check to prevent the PID control from starting integration
        if (Time_count > CONTROL_DELAY + 150) {
            Drive_Motor(Move_X,Move_Y,Move_Z); 
        }

        // Speed closed-loop control to calculate the PWM value of each motor,
        // PWM represents the actual wheel speed
				MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder,0.2);
				MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder,0.2);
				MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder,0.2);
        SetPWM(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm);

        // Get battery voltage value
        //   Voltage = Get_battery_volt();
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
**************************************************************************/
void SetPWM(int PWMA, int PWMB, int PWMC)
{
    // Left TIM4
    if (PWMA >= 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_2);
        GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    } else if (PWMA < 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_3);
        GPIO_ResetBits(GPIOE, GPIO_Pin_2);
    }

    TIM_SetCompare4(TIM3, myabs(PWMA));

    // Right TIM2
    if (PWMB >= 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_4);
        GPIO_ResetBits(GPIOE, GPIO_Pin_5);
    } else if (PWMB < 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_5);
        GPIO_ResetBits(GPIOE, GPIO_Pin_4);
    }

    TIM_SetCompare3(TIM3, myabs(PWMB));

    // PWMC
    if (PWMC >= 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
        GPIO_ResetBits(GPIOE, GPIO_Pin_10);
    } else if (PWMC < 0) {
        GPIO_SetBits(GPIOE, GPIO_Pin_10);
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);
    }

    TIM_SetCompare2(TIM3, myabs(PWMC));
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
**************************************************************************/
float float_abs(float insert)
{
    if (insert >= 0)
        return insert;
    else
        return -insert;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 7200) Pwm = 7200;
    if (Pwm < -7200) Pwm = -7200;
    Last_bias = Bias; // Save the last deviation 
    return Pwm;
}
int Incremental_PI_B(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 7200) Pwm = 7200;
    if (Pwm < -7200) Pwm = -7200;
    Last_bias = Bias; // Save the last deviation 
    return Pwm;
}

int Incremental_PI_C(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 7200) Pwm = 7200;
    if (Pwm < -7200) Pwm = -7200;
    Last_bias = Bias; // Save the last deviation 
    return Pwm;
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
    // Retrieves the original data of the encoder
    float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr;
    Encoder_A_pr = Read_Encoder_TIM4();
    Encoder_B_pr = Read_Encoder_TIM2();
    Encoder_C_pr = Read_Encoder_TIM1();

    // The encoder converts the raw data to wheel speed in m/s
    MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY / Encoder_precision * Wheel_perimeter;
    MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY / Encoder_precision * Wheel_perimeter;
    MOTOR_C.Encoder = Encoder_C_pr * CONTROL_FREQUENCY / Encoder_precision * Wheel_perimeter;
}
