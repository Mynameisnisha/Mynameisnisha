#include "Wire.h"
#include <MPU6050_light.h>
#define enA 6  
#define in1 A2
#define in2 A3
#include <Encoder.h>

#define enA 7
#define in1 A2
#define in2 A3

Encoder DC_Encoder(2, 3);

long old_ticks = -999;

MPU6050 mpu(Wire);
unsigned long timer = 0;

float kp_balance = 40.0;
float ki_balance = 0.5;
float kd_balance = 10.0;

// Initialize variables
float prev_error_balance = 0.0;
float integral_balance = 0.0;

void dc_motor_init(){
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
}

void encoder(){
  long ticks = DC_Encoder.read();
  if (ticks != old_ticks) {
    old_ticks = ticks;
  }
  Serial.println(ticks);
}

void motor_control(int pwm) {
  if (pwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwm = -pwm;
  } 
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}

void setup() {
  dc_motor_init();
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  encoder();
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
  Serial.print("X : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tZ : ");
  Serial.println(mpu.getAngleZ());
  timer = millis();  
  }

  float roll = mpu.getAngleY();  // Replace with actual gyro reading

  // PID control for balance
  float error_balance = 0.0 - roll;
  integral_balance += error_balance;
  float derivative_balance = error_balance - prev_error_balance;
  float balance_control_signal = kp_balance * error_balance + ki_balance * integral_balance + kd_balance * derivative_balance;
  motor_control(balance_control_signal);
}
