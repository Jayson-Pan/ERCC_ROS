#include <Arduino.h>
#line 1 "C:\\Users\\kogm\\Desktop\\ROS项目\\Code\\arm_handler\\arm_handler.ino"
#include <SoftwareSerial.h>
#include <Servo.h>

// 软串口引脚定义
const int SOFT_RX = 12;  // Arduino RX -> STM32 TX (PB10)
const int SOFT_TX = 11;  // Arduino TX -> STM32 RX (PB11)

// 舵机引脚和角度定义
const int SERVO_PIN = 9;
const int ANGLE_INIT = 5;    // 初始角度
const int ANGLE_OPEN = 75;   // 打开角度
const int ANGLE_CLOSE = 0;   // 关闭角度

// 线性插值参数
const int MOVE_STEP = 1;     // 每步移动角度
const int MOVE_DELAY = 20;   // 每步延时(ms)，控制运动速度

// 创建软串口和舵机对象
SoftwareSerial stm32Serial(SOFT_RX, SOFT_TX);
Servo armServo;

// 全局变量
int currentAngle = ANGLE_INIT;  // 当前舵机角度
int targetAngle = ANGLE_INIT;   // 目标角度
bool isMoving = false;          // 是否正在移动

void setup() {
  // 初始化硬件串口用于调试
  Serial.begin(9600);
  Serial.println("Arduino ARM Handler Started");
  Serial.println("Wiring:");
  Serial.println("Arduino 11(TX) -> STM32 PB11(RX)");
  Serial.println("Arduino 12(RX) -> STM32 PB10(TX)");
  Serial.println("Arduino GND -> STM32 GND");
  Serial.println("Servo Signal -> Arduino Pin 9");
  
  // 初始化软串口通信
  stm32Serial.begin(115200);
  Serial.println("Software Serial initialized at 115200 baud");
  
  // 初始化舵机
  armServo.attach(SERVO_PIN);
  armServo.write(currentAngle);
  Serial.print("Servo initialized at angle: ");
  Serial.println(currentAngle);
  
  delay(1000); // 等待舵机到位
  Serial.println("Ready to receive commands from STM32");
}

void loop() {
  // 检查是否有来自STM32的数据
  if (stm32Serial.available()) {
    char command = stm32Serial.read();
    processCommand(command);
  }
  
  // 执行舵机平滑移动
  smoothServoMove();
  
  delay(10); // 主循环小延时
}

// 处理来自STM32的命令
void processCommand(char command) {
  Serial.print("Received command: ");
  Serial.println(command);
  
  switch(command) {
    case '1':
      // 收到1，舵机打开到75度
      setTargetAngle(ANGLE_OPEN);
      Serial.println("Command: OPEN servo to 75 degrees");
      break;
      
    case '0':
      // 收到0，舵机关闭到0度
      setTargetAngle(ANGLE_CLOSE);
      Serial.println("Command: CLOSE servo to 0 degrees");
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      break;
  }
}

// 设置目标角度并开始移动
void setTargetAngle(int angle) {
  targetAngle = angle;
  isMoving = true;
  Serial.print("Target angle set to: ");
  Serial.println(targetAngle);
}

// 线性插值实现舵机平滑移动
void smoothServoMove() {
  static unsigned long lastMoveTime = 0;
  
  // 如果不需要移动或者还未到移动时间，直接返回
  if (!isMoving || millis() - lastMoveTime < MOVE_DELAY) {
    return;
  }
  
  // 计算移动方向
  if (currentAngle < targetAngle) {
    // 需要增加角度
    currentAngle = min(currentAngle + MOVE_STEP, targetAngle);
  } else if (currentAngle > targetAngle) {
    // 需要减少角度
    currentAngle = max(currentAngle - MOVE_STEP, targetAngle);
  }
  
  // 移动舵机
  armServo.write(currentAngle);
  lastMoveTime = millis();
  
  // 检查是否到达目标位置
  if (currentAngle == targetAngle) {
    isMoving = false;
    Serial.print("Servo movement completed. Current angle: ");
    Serial.println(currentAngle);
  }
}
