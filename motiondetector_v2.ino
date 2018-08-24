#include "Wire.h"

const int MPU_addr = 0x68;
short accXRaw, accYRaw, accZRaw, tmpRaw, gyroXRaw, gyroYRaw, gyroZRaw;
unsigned long lastReadTime;
float lastXAngle, lastYAngle;
float xOffset, yOffset;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  SetupMpu6050();
  CalibrateMpu6050Offset();
  WriteOffsetToSerial();
}

void loop()
{
  ReadRawMpuValues();
  ComputeFilteredValues();
  WriteFilteredValuesWithOffsetToSerial();

  delay(50);
}

void SetupMpu6050()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void CalibrateMpu6050Offset()
{
  for (int i = 0; i < 500; i++)
  {
    ReadRawMpuValues();
    ComputeFilteredValues();
    delay(50);
  }

  xOffset = lastXAngle;
  yOffset = lastYAngle;
}

void ReadRawMpuValues()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers

  accXRaw  = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accYRaw  = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZRaw  = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmpRaw   = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gyroXRaw = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyroYRaw = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyroZRaw = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
}

void WriteFilteredValuesWithOffsetToSerial()
{
  Serial.print("X = ");    Serial.print(lastXAngle - xOffset);
  Serial.print(" | Y = "); Serial.println(lastYAngle - yOffset);
}

void WriteOffsetToSerial()
{
  Serial.print("Offset ");
  Serial.print("X = ");    Serial.print(xOffset);
  Serial.print(" | Y = "); Serial.println(yOffset);
}

void ComputeFilteredValues()
{
 
  // Get the time of reading for rotation computations
  unsigned long t_now = millis();

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;
  float gyro_x = gyroXRaw / FS_SEL;
  float gyro_y = gyroYRaw / FS_SEL;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  float accel_angle_y = atan(-1 * accXRaw / sqrt(pow(accYRaw, 2) + pow(accZRaw, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accYRaw / sqrt(pow(accXRaw, 2) + pow(accZRaw, 2))) * RADIANS_TO_DEGREES;

  // Compute the (filtered) gyro angles
  float dt = (t_now - lastReadTime) / 1000.0;
  float gyro_angle_x = gyro_x * dt + lastXAngle;
  float gyro_angle_y = gyro_y * dt + lastYAngle;

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
  float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;

  // Update the saved data with the latest values only if they are OK
  if(!isnan(angle_x) && !isnan(angle_y))
  {
    lastReadTime = t_now;
    lastXAngle = angle_x;
    lastYAngle = angle_y;
  }  
}