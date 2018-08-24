#include "Wire.h"       // i2c bus
#include "I2Cdev.h"     // i2c bus
#include "DS3231.h"     // clock
#include "SPI.h"        // sd card
#include "SD.h"         // sd card
#include "U8glib.h"     // display

#define MPU_ADDR 0x68 // i2c address of MPU
#define BUTTON_PIN 7    // use pin 7 on Arduino Uno for pause button
#define BUZZER_PIN 9    // use pin 9 on Arduino Uno for buzzer
#define CS_PIN 4        // use pin 12 on Arduino UNO as CS pin for SDCard

short accXRaw, accYRaw, accZRaw, tmpRaw, gyroXRaw, gyroYRaw, gyroZRaw;
unsigned long lastReadTime;
float lastXAngle, lastYAngle;
float xOffset, yOffset;

int pauseTimeSeconds = 5; // stop loop for N seconds if button is pressed
int maxDeviation = 10;    // sound alarm if deviation is bigger then X 
bool isAlarmOn = false;

DS3231 rtc(SDA, SCL);                             // clock
U8GLIB_SSD1306_128X64 screen(U8G_I2C_OPT_NO_ACK); // display
File logFile;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  SetupRtc();
  SetupScreen();
  SetupSDCard();
  SetupMpu6050();
  SetupBuzzer();
  SetupButton();

  WriteCalibrationToScreen();
  CalibrateMpu6050Offset();
  WriteOffsetToSerial();
}

void loop()
{
  PauseIfButtonIsPressed();

  ReadRawMpuValues();
  ComputeFilteredValues();
  WriteFilteredValuesWithOffsetToSerial();
  WriteDataToLogFile();

  CheckIfPositionIsOk();

  delay(50);
}

#pragma region Setup

void SetupMpu6050()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void SetupRtc()
{
  rtc.begin();
}

void SetupScreen()
{
    screen.setFont(u8g_font_unifont);
    screen.setColorIndex(1); // Instructs the display to draw with a pixel on.
}

void SetupSDCard()
{
  if (SD.begin(CS_PIN))
  {
    Serial.println(F("SD card is ready"));
  }
  else
  {
    Serial.println(F("Unable to connect SD card"));
  }
}

void SetupBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
}

void SetupButton()
{
  pinMode(BUTTON_PIN, INPUT);
}

void CalibrateMpu6050Offset()
{
  for (int i = 0; i < 50; i++)
  {
    ReadRawMpuValues();
    ComputeFilteredValues();
    delay(50);
  }

  xOffset = lastXAngle;
  yOffset = lastYAngle;
}

#pragma endregion

#pragma region Write to Serial or Screen

void WriteFilteredValuesWithOffsetToSerial()
{
  Serial.print(F("X = "));    Serial.print(lastXAngle - xOffset);
  Serial.print(F(" | Y = ")); Serial.println(lastYAngle - yOffset);
}

void WriteOffsetToSerial()
{
  Serial.print(F("Offset "));
  Serial.print(F("X = "));    Serial.print(xOffset);
  Serial.print(F(" | Y = ")); Serial.println(yOffset);
}

void WriteCalibrationToScreen()
{
  screen.firstPage();
  do
  {
    screen.drawStr(10, 30, F("Calibration"));
  } while (screen.nextPage());
}

void WriteErrorToScreen()
{
  screen.firstPage();
  do
  {
    screen.drawStr(0, 10, rtc.getTimeStr());
    screen.drawStr(10, 30, F("ERROR"));
  } while (screen.nextPage());
}

void WritePositionOkToScreen()
{
  screen.firstPage();
  do
  {
    screen.drawStr(0, 10, rtc.getTimeStr());
    screen.drawStr(10, 30, F("Position OK"));
  } while (screen.nextPage());
}

#pragma endregion

void ReadRawMpuValues()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // request a total of 14 registers

  accXRaw  = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accYRaw  = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZRaw  = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmpRaw   = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gyroXRaw = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyroYRaw = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyroZRaw = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
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

void WriteDataToLogFile()
{
  logFile = SD.open("log.txt", FILE_WRITE);
  if (logFile)
  {
    logFile.print(rtc.getDateStr()); logFile.print(F(" "));
    logFile.print(rtc.getTimeStr()); logFile.print(F(","));
    logFile.print(lastXAngle); logFile.print(F(","));
    logFile.println(lastXAngle);
  
    logFile.close();
  }
}

void WritePauseToLogFile()
{
  logFile = SD.open("log.txt", FILE_WRITE);
  if(logFile)
  {
    logFile.println(F("pause"));
    logFile.close();
  }  
}

void PauseIfButtonIsPressed()
{
  if (digitalRead(BUTTON_PIN) == HIGH)
  {
    //Serial.println("pause");
    bool wasAlarmOn = isAlarmOn;
    if (isAlarmOn) { ToggleAlarm(false); }

    WritePauseToLogFile();

    delay(pauseTimeSeconds * 1000);
    if (wasAlarmOn) { ToggleAlarm(true); }
  }
}

void ToggleAlarm(bool raiseAlarm)
{
    isAlarmOn = raiseAlarm;
    if (raiseAlarm)
    {
        tone(BUZZER_PIN, 2500);
    }
    else
    {
        noTone(BUZZER_PIN);
    }
}

void CheckIfPositionIsOk()
{
  if ((abs(lastXAngle - xOffset) > maxDeviation || abs(lastYAngle - yOffset) > maxDeviation))
    {
        WriteErrorToScreen();
        if (!isAlarmOn) { ToggleAlarm(true); }
    }
    else
    {
      WritePositionOkToScreen();
      if (isAlarmOn) { ToggleAlarm(false); }
    }
}
