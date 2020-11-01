//LIBRARIES
#include<Wire.h>

//VARIABLES AND CONSTANTS
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float motorSpeedLeft;
float motorSpeedRight;
unsigned long lastLoopUsefulTime;
unsigned long loopStartTime;
unsigned long STD_LOOP_TIME = 10;
unsigned long lastLoopTime;
float tilt = 0;
float loopCount = 0;
float gyroCal = 0;
float gyroOffset;
long unsigned int loopTime = 0;
float accelCal = 0;
float accelOffset;
float shutOff = 0;
float calAcY = 0;
float calGyX = 0;
bool motorButtonState = 1;
bool resetButtonState = 0;
float setpoint;
float error;
float kP = 1;
float kI = 0.0009;
float kD = 3;
float P;
float I;
float D;
float IntThresh = 50000;
float ScaleFactor = .003;
float Integral = 0;
float LastTilt = 0;
float potentiometerValue = 511.5;
float steerValue = 0;
float centerValue = 0;
float motorDirectionLeft = 0;
float motorDirectionRight = 0;


void setup(){
  //SART COMMUNICATION, DEFINE INPUTS AND OUTPUTS
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  pinMode (9, OUTPUT);  //pwm left
  pinMode (10, OUTPUT);  //pwm right
  pinMode (6, OUTPUT);  //direction lfet
  pinMode (12, OUTPUT);  //direction right
  pinMode (5, INPUT);  //reset button
  pinMode (13, INPUT); //switch
  digitalWrite (9, LOW); //enable motor
  digitalWrite (10, LOW); //enable motor
  delay (3000);

  Serial.begin(115200);
}
void loop(){
  ///////////////////////////////////////////////////////////////
  //READ SENSOR VALUES
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  
  /////////////////////////////////////////////////////////////////////////////////////
  //DETERMINE SENSOR READING WHEN LEVEL, CALCULATION PERFORMED ONCE
  loopTime = millis();
  if (loopTime < 6000){
    gyroCal = gyroCal + GyX;
    accelCal = accelCal + AcY;
    loopCount = loopCount + 1;
    //delay(10);
  }
      
  else if (loopTime < 6100 && loopTime > 6000){
      gyroOffset = gyroCal / loopCount;
      accelOffset = accelCal / loopCount;
    }
  else if (loopTime > 6100){
  
  /////////////////////////////////////////////////////////////////////////////////////
  //ADJUST ACCELEROMETER AND GYROSCOPE BASED ON OFFSETS
  calGyX = GyX - gyroOffset;
  calAcY = AcY - accelOffset;
  tilt = tilt + calGyX;

  //ACCOUNT FOR GYROSCOPE DRIFT
  //gyroOffset = gyroOffset - (calAcY * .000005);

  /////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////
  //CALCULATE MOTOR SPEED USING A PID CONTROLLER
  setpoint = 0;
  if (abs(tilt) < IntThresh){ // prevent integral 'windup'
  Integral = Integral + tilt; // accumulate the error integral
  }
  else {
  Integral = Integral; // zero it if out of bounds
  }

  P = tilt * kP;
  I = Integral * kI;
  D = (tilt - LastTilt) * kD;
  motorSpeedLeft = P + I + D; // Total motorSpeed = P + I + D
  motorSpeedRight = P + I + D; // Total motorSpeed = P + I + D
  motorSpeedLeft = motorSpeedLeft * ScaleFactor; // scale Drive to be in the range 0-255
  motorSpeedRight = motorSpeedRight * ScaleFactor; // scale Drive to be in the range 0-255
  LastTilt = tilt;

  ////////////////////////////////////////////////////////////////////////////////////
  //DETERMINE MOTOR SPEED CAP AND WHETHER OR NOT IT IS ON

  steerValue = 0;
  potentiometerValue = analogRead(A2);
  steerValue = 558 - potentiometerValue; 
  steerValue = steerValue * .5;

  motorSpeedLeft = motorSpeedLeft + steerValue;
  motorSpeedRight = motorSpeedRight - steerValue;


  if (motorSpeedLeft > 0){
    digitalWrite (6, 0);       //direction left
  }
  if (motorSpeedLeft < 0){
    digitalWrite (6, 1);      //direction left
  }

  
      if (motorSpeedRight > 0){
    digitalWrite (12, 0);       //direction left
  }
  if (motorSpeedRight < 0){
    digitalWrite (12, 1);      //direction left
  }
  motorSpeedLeft = abs(motorSpeedLeft);
  motorSpeedRight = abs(motorSpeedRight);
  
  if (motorSpeedLeft > 250) {
    motorSpeedLeft = 250;
  }
  if (motorSpeedRight > 250) {
    motorSpeedRight = 250;
  }
  //OUTPUT SPEED TO THE MOTOR
  analogWrite (9, motorSpeedLeft); 
  analogWrite (10, motorSpeedRight);   
//////////////////////////////////////////////////////////////////////////////////////
//DEBUGGING
  //Serial.print (" value = "); Serial.print(steerValue);
  Serial.print (" value = "); Serial.println(potentiometerValue);
  //Serial.print (" \t |calAcY=  "); Serial.print(calAcY);
  //Serial.print (" \t |calGyX=  "); Serial.print(calGyX);
  //Serial.print (" \t |accelOffset = "); Serial.print(accelOffset);
  //Serial.print (" \t |gyroOffset = "); Serial.print(gyroOffset);
  //Serial.print (" \t |loopCount = ");Serial.print(loopCount);
  //Serial.print(" \t | P = "); Serial.print(P);
  //Serial.print(" \t | I = "); Serial.print(I);
  //Serial.print(" \t | D = "); Serial.print(D);
  //Serial.print(" \t | lastLoopUsefulTime = "); Serial.println(lastLoopUsefulTime); 
  //Serial.print(" \t | gyroOffset = "); Serial.print(gyroOffset); 
  //Serial.print(" \t | accelOffset = "); Serial.print(accelOffset); 
  //Serial.print (" \t |buttonState = "); Serial.print(buttonState);
  //Serial.print(" \t | AcY = "); Serial.print(AcY);
  //Serial.print(" \t | GyX = "); Serial.print(GyX);
  //Serial.print(" \t | motorSpeed = "); Serial.print(motorSpeed);
  //Serial.print(" \t | tilt = "); Serial.println(tilt);
  //Serial.print(" \t | time = "); Serial.print(millis());
//////////////////////////////////////////////////////////////////////////////////////
//CALCULATE LOOP TIME, DELAY SO ALL LOOPS TAKE THE SAME AMOUNT OF TIME
    lastLoopUsefulTime = millis() - loopStartTime;        
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }
    
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();  
  }
}





