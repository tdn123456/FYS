#include <Arduino.h>

#include <Encoder.h>
#include <Arduino_FreeRTOS.h>
#include "pid.h"
ooooo

struct pid_calibration PID_data = {
  .kp = 0.02, // Proportional gain
  .ki = 0.01, // Integral gain
  .kd = 0.01 // Derivative gain
};

struct pid_state PID_status = {
  .actual = 0.01, // 测量的实际值
  .target = 0.01, // 期望的值
  .time_delta = 0.01, // 更新状态时应设置自上次采样/计算以来的时间
  .previous_error = 0.01, //先前计算的实际与目标之间的误差(初始为零)
  .integral = 0.01, // 积分误差随时间的总和
  .output = 0.01 // 修正后的输出值由算法计算，对误差进行补偿
};


// 定义电机控制引脚
const int L1_IN1 = 2;const int L1_IN2 = 3; // 左上电机
const int L1_encoderPinA = 22;const int L1_encoderPinB = 23;Encoder L1_encoder(L1_encoderPinA,L1_encoderPinB);
const int R1_IN1 = 4;const int R1_IN2 = 5; // 右上电机
const int R1_encoderPinA = 24;const int R1_encoderPinB = 25;Encoder R1_encoder(R1_encoderPinA,R1_encoderPinB);
const int L2_IN1 = 6;const int L2_IN2 = 7; // 左下电机
const int L2_encoderPinA = 26;const int L2_encoderPinB = 27;Encoder L2_encoder(L2_encoderPinA,L2_encoderPinB);
const int R2_IN1 = 8;const int R2_IN2 = 9; // 右下电机
const int R2_encoderPinA = 28;const int R2_encoderPinB = 29;Encoder R2_encoder(R2_encoderPinA,R2_encoderPinB);

char Serial_data = ' ';

//各电机的占空比
int Car_ps = 100;
int L1_ps = Car_ps;
int R1_ps = Car_ps;
int L2_ps = Car_ps;
int R2_ps = Car_ps;

// 编码器位置
volatile long encoderPos = 0; 
volatile long L1_encoderPos = 0; 
volatile long R1_encoderPos = 0; 
volatile long L2_encoderPos = 0; 
volatile long R2_encoderPos = 0; 

void L1_forward(int sp) // 左前轮前进
{
  analogWrite(L1_IN1, sp);
  digitalWrite(L1_IN2, LOW);
}
void R1_forward(int sp) // 右前轮前进
{
  analogWrite(R1_IN1, sp);
  digitalWrite(R1_IN2, LOW);

}
void L2_forward(int sp) // 左后轮前进
{
  analogWrite(L2_IN1, sp);
  digitalWrite(L2_IN2, LOW);
}
void R2_forward(int sp) // 右后轮前进
{
  analogWrite(R2_IN1, sp);
  digitalWrite(R2_IN2, LOW);
}
void allstop()
{
  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, LOW);
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, LOW);
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, LOW);
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, LOW);
}
void L1_backward(int sp) // 左前轮后退
{
  digitalWrite(L1_IN1, LOW);
  analogWrite(L1_IN2, sp);
}
void R1_backward(int sp) // 右前轮后退
{
  digitalWrite(R1_IN1, LOW);
  analogWrite(R1_IN2, sp);
}
void L2_backward(int sp) // 左后轮后退
{
  digitalWrite(L2_IN1, LOW);
  analogWrite(L2_IN2, sp);
}
void R2_backward(int sp) // 右后轮后退
{
  digitalWrite(R2_IN1, LOW);
  analogWrite(R2_IN2, sp);
}

long UP_encoder(Encoder& encoder_name)
{
  encoderPos = encoder_name.read();
  // noInterrupts();
  return encoderPos;
}

void moveTask(void *pvParameters) {
  while (1) {
    //  if(Serial.available()){
    //     Serial_data = Serial.read();
    //  }

    Serial.println("前进");
    /*前进*/
    L1_forward(L1_ps);
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("后退");
    /*后退*/
    L1_backward(L1_ps);
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("顺时针原地旋转");
    /*顺时针原地旋转*/
    L1_forward(L1_ps);
    R1_backward(R1_ps);
    L2_forward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("逆时针原地旋转");
    /*逆时针原地旋转*/
    L1_backward(L1_ps);
    R1_forward(R1_ps);
    L2_backward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("左边平移");
    /*左边平移*/
    L1_backward(L1_ps);
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("右边平移");
    /*右边平移*/
    L1_forward(L1_ps);
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向左上方");
    /*斜向左上方*/
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向右上方");
    /*斜向右上方*/
    L1_forward(L1_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向左下方");
    /*斜向左下方*/
    L1_backward(L1_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向右下方");
    /*斜向右下方*/
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    delay(5000);
    allstop();
    delay(1500);
  }
}

void testTask(void *pvParameters) {
    while (1) {
        L1_encoderPos = UP_encoder(L1_encoder);
        R1_encoderPos = UP_encoder(R1_encoder);
        L2_encoderPos = UP_encoder(L2_encoder);
        R2_encoderPos = UP_encoder(R2_encoder);
        


        // long middle_encoderPos = (L1_encoderPos + R1_encoderPos + L2_encoderPos + R2_encoderPos)/4;
        // if(L1_encoderPos > middle_encoderPos){
        //   L1_ps--;
        // }else if(L1_encoderPos < middle_encoderPos){
        //   L1_ps++;
        // }
        // if(R1_encoderPos > middle_encoderPos){
        //   R1_ps--;
        // }else if(R1_encoderPos < middle_encoderPos){
        //   R1_ps++;
        // }
        // if(L2_encoderPos > middle_encoderPos){
        //   L2_ps--;
        // }else if(L2_encoderPos < middle_encoderPos){
        //   L2_ps++;
        // }
        // if(R2_encoderPos > middle_encoderPos){
        //   R2_ps--;
        // }else if(R2_encoderPos < middle_encoderPos){
        //   R2_ps++;
        // }

        // Serial.println("......速度更新完成......");
        Serial.println("速度：");
        Serial.print(L1_ps);Serial.print(" ");Serial.print(R1_ps);Serial.print(" ");Serial.print(L2_ps);Serial.print(" ");Serial.println(R2_ps);
        delay(1000);
    }
}



void setup()
{
  xTaskCreate(moveTask, "电机动作", 1000, NULL, 1, NULL); // 创建任务
  xTaskCreate(testTask, "通过编码器检测速度", 1000, NULL, 2, NULL); // 创建任务

  Serial.begin(115200);

  pinMode(L1_encoderPinA, INPUT);
  pinMode(L1_encoderPinB, INPUT);
  pinMode(R1_encoderPinA, INPUT);
  pinMode(R1_encoderPinB, INPUT);
  pinMode(L2_encoderPinA, INPUT);
  pinMode(L2_encoderPinB, INPUT);
  pinMode(R2_encoderPinA, INPUT);
  pinMode(R2_encoderPinB, INPUT);

  pinMode(L1_IN1, OUTPUT);
  pinMode(L1_IN2, OUTPUT);
  pinMode(R1_IN1, OUTPUT);
  pinMode(R1_IN2, OUTPUT);
  pinMode(L2_IN1, OUTPUT);
  pinMode(L2_IN2, OUTPUT);
  pinMode(R2_IN1, OUTPUT);
  pinMode(R2_IN2, OUTPUT);

  Serial.println("初始化完毕....");
}




void loop()
{

}