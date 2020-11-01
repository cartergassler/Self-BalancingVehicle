#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float motorSpeed;
float lastLoopUsefulTime;
float loopStartTime;
int STD_LOOP_TIME = 13;
float lastLoopTime;
float tilt = 0;
float loopCount = 0;
float gyroCal = 0;
float gyroOffset;
long unsigned int loopTime = 0;
float accelCal = 0;
float accelOffset;

float setpoint;
float error;
float kP = 1;
float kI = 0.001;
float kD = 3;
float P;
float I;
float D;
float IntThresh = 50000;
float ScaleFactor = .002;
float Integral = 0;
float LastTilt = 0;


void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  pinMode (9, OUTPUT);  //enable motor pin
  pinMode (6, OUTPUT);  //direction pin6
  pinMode (5, OUTPUT);  //direction pin5
  digitalWrite (9, HIGH); //enable motor
  delay (3000);

  Serial.begin(115200);
}
void loop(){
  ///////////////////////////////////////////////////////////////
  //read sensor values
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
  //Serial.print("\t AcX = "); Serial.print(AcX);
  Serial.print(" \t | AcY = "); Serial.print(AcY);
  //Serial.print(" \t | AcZ = "); Serial.print(AcZ);
  //Serial.print(" \t | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" \t | GyX = "); Serial.print(GyX);
  //Serial.print(" \t | GyY = "); Serial.print(GyY);
  Serial.print(" \t | motorSpeed = "); Serial.println(motorSpeed);
  //Serial.print(" | GyZ = "); Serial.println(GyZ);
  /////////////////////////////////////////////////////////////////////////////////////
  //determine sensor reading when level, calculation performed once during the program
  loopTime = millis();
  if (loopTime < 6000){
    gyroCal = gyroCal + GyX;
    accelCal = accelCal + AcY;
    loopCount = loopCount + 1;
  }
    
    
  else if (loopTime < 6100 && loopTime > 6000){
      gyroOffset = gyroCal / loopCount;
      accelOffset = accelCal / loopCount;
    }
  else if (loopTime > 6100){
  /////////////////////////////////////////////////////////////////////////////////////
  //Adjust accelerometer and gyro readings for offsets
  GyX = GyX - gyroOffset;
  AcY = AcY - accelOffset;
  tilt = tilt + GyX;
  Serial.print(" \t | tilt = "); Serial.print(tilt);
  Serial.print(" \t | lastLoopUsefulTime = "); Serial.print(lastLoopUsefulTime); 
  //Serial.print(" \t | gyroOffset = "); Serial.print(gyroOffset); 
  //Serial.print(" \t | accelOffset = "); Serial.print(accelOffset); 
  //account for gyroscope drift
  gyroOffset = gyroOffset - (AcY * .003);
  /////////////////////////////////////////////////////////////////////////////////////
  //Calculate motor speed using a PID controller
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
  motorSpeed = P + I + D; // Total motorSpeed = P + I + D
  motorSpeed = motorSpeed * ScaleFactor; // scale Drive to be in the range 0-255
  LastTilt = tilt;
  Serial.print(" \t | P = "); Serial.print(P);
  Serial.print(" \t | I = "); Serial.print(I);
  Serial.print(" \t | D = "); Serial.print(D);
  ////////////////////////////////////////////////////////////////////////////////////
  //determine motor direction and output speed and direction
  
  if (tilt < 0){
    digitalWrite (6, 1);       //direction pin6
    digitalWrite (5, 0);        //direction pin5
  }
  else {
    digitalWrite (6, 0);        //direction pin7
    digitalWrite (5, 1);       //direction pin4
  }
  motorSpeed = abs(motorSpeed);
  
  if (motorSpeed >250) {
    motorSpeed = 250;
  }
  analogWrite (9, motorSpeed);  //set motor speed
//////////////////////////////////////////////////////////////////////////////////////
//calculate loop time, delay so all loops take the same amount of time
    lastLoopUsefulTime = millis() - loopStartTime;        
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }
    
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();  
  }
}


