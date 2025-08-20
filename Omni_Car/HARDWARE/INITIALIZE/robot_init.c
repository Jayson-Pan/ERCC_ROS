#include "robot_init.h"

// ��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef Robot_Parament;

void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter)
{
    // Rotation radius of omnidirectional trolley
    // ȫ����С����ת�뾶
    Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus;
    // motor_gear_ratio
    // ������ٱ�
    Robot_Parament.GearRatio = gearratio;
    // Number_of_encoder_lines
    // ����������(����������)
    Robot_Parament.EncoderAccuracy = Accuracy;
    // Diameter of driving wheel
    // ������ֱ��
    Robot_Parament.WheelDiameter = tyre_diameter;

    // Encoder value corresponding to 1 turn of motor (wheel)
    // ���(����)ת1Ȧ��Ӧ�ı�������ֵ
    Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio;
    // Driving wheel circumference
    // �������ܳ�
    Wheel_perimeter = Robot_Parament.WheelDiameter * PI;
    // Rotation radius of omnidirectional trolley
    // ȫ����С����ת�뾶
    Omni_radiaus = Robot_Parament.OmniTurnRadiaus;
}
