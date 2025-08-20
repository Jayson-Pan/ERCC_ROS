#include "robot_init.h"

// 初始化机器人参数结构体
Robot_Parament_InitTypeDef Robot_Parament;

void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter)
{
    // Rotation radius of omnidirectional trolley
    // 全向轮小车旋转半径
    Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus;
    // motor_gear_ratio
    // 电机减速比
    Robot_Parament.GearRatio = gearratio;
    // Number_of_encoder_lines
    // 编码器精度(编码器线数)
    Robot_Parament.EncoderAccuracy = Accuracy;
    // Diameter of driving wheel
    // 主动轮直径
    Robot_Parament.WheelDiameter = tyre_diameter;

    // Encoder value corresponding to 1 turn of motor (wheel)
    // 电机(车轮)转1圈对应的编码器数值
    Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio;
    // Driving wheel circumference
    // 主动轮周长
    Wheel_perimeter = Robot_Parament.WheelDiameter * PI;
    // Rotation radius of omnidirectional trolley
    // 全向轮小车旋转半径
    Omni_radiaus = Robot_Parament.OmniTurnRadiaus;
}
