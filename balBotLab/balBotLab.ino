#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

#define SAMPLE_PERIOD       5       // ms

MPU6050 mpu;
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float input, output;

void setPow(int left, int right) {
  if (left > 0) {
    OCR1A = 1600 + left;
  } else {
    OCR1A = 1400 + left;
  }

  if (right > 0) {
    OCR1B = 1400 - right;
  } else {
    OCR1B = 1600 - right;
  }
}

float integral, lastError;
unsigned long lastRan;

// Task 2: Tune the 5 parameters below so that the robot would balance
float kp = 0;
float ki = 0;
float kd = 0;
float integralMax = 0;
float outputMax = 0;

float PID(float input, float target) {

  float error = input - target;
  integral = integral + error;
  integral = constrain(integral, - integralMax, integralMax);
  float deltaT = SAMPLE_PERIOD / 1000.0;
  float output =  ki * integral * deltaT +
                  kp * error +
                  kd * (error - lastError) / deltaT;
  lastError = error;
  return constrain(output, -outputMax, outputMax);

}



void setup() {
  // put your setup code here, to run once:

  pinMode(9, OUTPUT);   
  pinMode(10, OUTPUT);
  TCCR1A = 0b10100010;  /* mode 10: PWM, phase correct, 16-bit */
  TCCR1B = 0b00010010;  /* prescale TIMER 1 to FclkIO / 8 */
  ICR1 = 2500;          /* TOP set to 2500 in order to get 400Hz */
  setPow(0, 0);

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  
}


void loop() {

  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  output = PID(, );                         // Task 1: complete the function call
  setPow(output, output);

}
