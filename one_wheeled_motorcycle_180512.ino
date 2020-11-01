  
  //LIBRARIES
#include<Wire.h>

//VARIABLES AND CONSTANTS
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
const int MPC4725 = 0x62;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float motorSpeed;
unsigned int scaledMotorSpeed;
byte buffer[3];

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
float kD = 4;
float P;
float I;
float D;
float IntThresh = 50000;
float ScaleFactor = .045333;
float Integral = 0;
float LastTilt = 0;
bool motorDirection = 0;
float deadZone = 20;

void setup() {
  //SART COMMUNICATION, DEFINE INPUTS AND OUTPUTS
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  pinMode (13, INPUT); //switch
  pinMode (5, OUTPUT); //direction
  pinMode (6, OUTPUT); //direction
  pinMode (7, OUTPUT); //direction
  digitalWrite (5, HIGH);
  digitalWrite (6, HIGH);
  digitalWrite (7, HIGH);
  delay (3000);

  Serial.begin(115200);
}
void loop() {
  ///////////////////////////////////////////////////////////////
  //READ SENSOR VALUES
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 8, true); // request a total of 14 registers
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  /////////////////////////////////////////////////////////////////////////////////////
  //DETERMINE SENSOR READING WHEN LEVEL, CALCULATION PERFORMED ONCE
  loopTime = millis();
  if (loopTime < 6000) {
    gyroCal = gyroCal + GyX;
    accelCal = accelCal + AcY;
    loopCount = loopCount + 1;
  }

  else if (loopTime < 6100 && loopTime > 6000) {
    gyroOffset = gyroCal / loopCount;
    accelOffset = accelCal / loopCount;
  }
  else if (loopTime > 6100) {

    /////////////////////////////////////////////////////////////////////////////////////
    //ADJUST ACCELEROMETER AND GYROSCOPE BASED ON OFFSETS
    calGyX = GyX - gyroOffset;
    calAcY = AcY - accelOffset;
    tilt = tilt + calGyX;

    //////////////////////////////////////////////////////////////////////////////////
    //CALCULATE MOTOR SPEED USING A PID CONTROLLER
    setpoint = 0;
    if (abs(tilt) < IntThresh) { // prevent integral 'windup'
      Integral = Integral + tilt; // accumulate the error integral
    }
    else {
      Integral = Integral; // zero it if out of bounds
    }

    P = tilt * kP;
    I = Integral * kI;
    D = (tilt - LastTilt) * kD;
    motorSpeed = P + I + D; // Total motorSpeed = P + I + D
    scaledMotorSpeed = abs(motorSpeed) * ScaleFactor;
    LastTilt = tilt;

    ////////////////////////////////////////////////////////////////////////////////////
    //DETERMINE MOTOR SPEED CAP AND WHETHER OR NOT IT IS ON
    if (motorSpeed < 0) {
      motorDirection = 0;
    }
    else {
      motorDirection = 1;
    }

    if (motorDirection == 0) {
      digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);
    }
    else {
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
      digitalWrite(7, HIGH);
    }
    if (scaledMotorSpeed > 4100) {
      scaledMotorSpeed = 4100;
    }
    motorSpeed = motorSpeed * ScaleFactor;
   // Serial.print(scaledMotorSpeed);

    if (motorSpeed <= deadZone && motorSpeed >= (deadZone * -1)) {
      scaledMotorSpeed = 0;
     // Serial.print(" \t | ON = ");
      //Serial.println(scaledMotorSpeed);
    }
    else {
      scaledMotorSpeed = scaledMotorSpeed - deadZone;
   //   Serial.print(" \t | OFF = ");
      //Serial.println(scaledMotorSpeed);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //Communicate with the DAC
    buffer[0] = 0b01000000;
    buffer[1] = scaledMotorSpeed >> 4;
    buffer[2] = scaledMotorSpeed << 4;

    Wire.beginTransmission(MPC4725);
    Wire.write(buffer[0]);
    Wire.write(buffer[1]);
    Wire.write(buffer[2]);
    Wire.endTransmission();

    //////////////////////////////////////////////////////////////////////////////////////
    //DEBUGGING
    //Serial.print (" \t |calAcY=  "); Serial.print(calAcY);
    //Serial.print (" \t |calGyX=  "); Serial.print(calGyX);
    Serial.print (" \t |motorDirection =  "); Serial.println(motorDirection);
    //Serial.print (" \t |accelOffset = "); Serial.print(accelOffset);
    //Serial.print (" \t |gyroOffset = "); Serial.print(gyroOffset);
    //Serial.print (" \t |loopCount = ");Serial.print(loopCount);
    Serial.print(" \t | P = "); Serial.print(P);
    Serial.print(" \t | I = "); Serial.print(I);
    Serial.print(" \t | D = "); Serial.print(D);
    //Serial.print(" \t | lastLoopUsefulTime = "); Serial.print(lastLoopUsefulTime);
    //Serial.print(" \t | gyroOffset = "); Serial.print(gyroOffset);
    Serial.print(" \t | tilt = "); Serial.print(tilt);
    //Serial.print(" \t |buttonState = "); Serial.print(buttonState);
    //Serial.print(" \t | AcY = "); Serial.print(AcY);
    //Serial.print(" \t | GyX = "); Serial.print(GyX);
    //Serial.print(" \t | motorSpeed = "); Serial.print(motorSpeed);
   Serial.print(" \t | sms = "); Serial.print(scaledMotorSpeed);
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





