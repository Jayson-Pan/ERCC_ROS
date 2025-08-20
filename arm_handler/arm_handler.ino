#include <Servo.h>

// 舵机引脚和角度定义
const int SERVO_PIN = 5;     // 舵机引脚
const int ANGLE_INIT = 0;    // 初始角度
const int ANGLE_OPEN = 0;    // 张开角度
const int ANGLE_CLOSE = 70;  // 抓取角度

// 线性插值参数
const int MOVE_STEP = 1;     // 每步移动角度
const int MOVE_DELAY = 25;   // 每步延时(ms)，控制运动速度

// 数据包格式定义
const uint8_t FRAME_HEADER = 0x6B;  // 帧头
const uint8_t FRAME_TAIL = 0x6D;    // 帧尾
const int PACKET_SIZE = 4;          // 数据包大小

// 创建舵机对象
Servo armServo;

// 全局变量
int currentAngle = ANGLE_INIT;  // 当前舵机角度
int targetAngle = ANGLE_INIT;   // 目标角度
bool isMoving = false;          // 是否正在移动

// 数据包解析相关变量
uint8_t dataBuffer[PACKET_SIZE];
int bufferIndex = 0;
bool packetReceived = false;

void setup() {
  // 初始化硬件串口用于ROS通信
  Serial.begin(115200);
  
  delay(500);

  // 初始化舵机
  armServo.attach(SERVO_PIN);
  
  armServo.write(currentAngle);

  delay(500); // 等待舵机稳定
}

void loop() {
  // 检查是否有来自ROS的数据
  if (Serial.available()) {
    uint8_t receivedByte = Serial.read();
    parseDataPacket(receivedByte);
  }
  
  // 处理接收到的数据包
  if (packetReceived) {
    processDataPacket();
    packetReceived = false;
  }
  
  // 执行舵机平滑移动
  smoothServoMove();
  
  delay(10); // 主循环小延时
}

// 解析接收到的数据包
void parseDataPacket(uint8_t receivedByte) {
  // 如果缓冲区已满，重置索引
  if (bufferIndex >= PACKET_SIZE) {
    bufferIndex = 0;
  }
  
  // 检查帧头
  if (bufferIndex == 0) {
    if (receivedByte == FRAME_HEADER) {
      dataBuffer[bufferIndex++] = receivedByte;
    }
    // 如果不是帧头，忽略该字节
    return;
  }
  
  // 存储接收到的字节
  dataBuffer[bufferIndex++] = receivedByte;
  
  // 如果接收到完整的数据包
  if (bufferIndex == PACKET_SIZE) {
    // 检查帧尾
    if (dataBuffer[3] == FRAME_TAIL) {
      // 验证校验位
      if (verifyChecksum()) {
        packetReceived = true;
      }
    }
    bufferIndex = 0; // 重置缓冲区索引
  }
}

// 验证BBC校验位（异或校验）
bool verifyChecksum() {
  uint8_t calculatedChecksum = dataBuffer[0] ^ dataBuffer[1]; // 帧头 XOR 数据位
  return calculatedChecksum == dataBuffer[2];
}

// 处理解析完成的数据包
void processDataPacket() {
  uint8_t commandByte = dataBuffer[1];
  
  switch(commandByte) {
    case 0x01:
      // 收到0x01，舵机抓取（收缩到75度）
      setTargetAngle(ANGLE_CLOSE);
      break;
      
    case 0x00:
      // 收到0x00，舵机张开（打开到0度）
      setTargetAngle(ANGLE_OPEN);
      break;
  }
}

// 设置目标角度并开始移动
void setTargetAngle(int angle) {
  targetAngle = angle;
  isMoving = true;
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
  }
}