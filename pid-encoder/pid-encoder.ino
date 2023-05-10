#include "PIDController.h"

#define ENCODER_A   4
#define ENCODER_B   0
#define MOTOR_CW    2
#define CW_Channel  0
#define MOTOR_CCW   15
#define CCW_Channel 1

// Constants derived from rough trial and error
#define __Kp 260
#define __Ki 2.7
#define __Kd 2000

#define MinLevel -255
#define MaxLevel 255

volatile long int encoder_count = 0;
unsigned int inputValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 255; 
PIDController pidcontroller(__Kp, __Ki, __Kd, 0);

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  // pinMode(MOTOR_CW, OUTPUT);
  ledcSetup(CW_Channel, 500, 8);
  ledcSetup(CCW_Channel, 500, 8);
  ledcAttachPin(MOTOR_CW, CW_Channel);
  ledcAttachPin(MOTOR_CCW, CCW_Channel);

  // pinMode(MOTOR_CCW, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  pidcontroller.Limit(MinLevel, MaxLevel);
}

void loop() {
  while (Serial.available() > 0) {
    inputValue = Serial.parseInt();
    incomingByte = Serial.read(); // stores the /n character
    if (incomingByte == '\n'){
        if(MinLevel <= inputValue && inputValue <= MaxLevel){
          pidcontroller.SetPoint(inputValue);
        }
        Serial.print("Setpoint: ");
        Serial.println(pidcontroller.GetSetPoint());
    }

    continue;
  }

  motor_pwm_value = pidcontroller.Compute(encoder_count, millis());
  // Serial.print(motor_pwm_value);
  // Serial.print("   ");
  CommandMotor(motor_pwm_value);
  // Serial.println(encoder_count);
}

// Encoder A Interrupt handler
void encoder() {
  if (digitalRead(ENCODER_B))
    encoder_count++;
  else
    encoder_count--;
}

void CommandMotor(int power){
  if(-100 < power && power < 100){
    // power too low, set both pins low    
    ledcWrite(CW_Channel, 0);
    ledcWrite(CCW_Channel, 0);
  } else if(power >= 100){
    // rotate clockwise
    ledcWrite(CW_Channel, power);
    ledcWrite(CCW_Channel, 0);
  } else if(power <= -100){
    // rotate counterclockwise
    ledcWrite(CCW_Channel, power);
    ledcWrite(CW_Channel, 0);

  }
}
