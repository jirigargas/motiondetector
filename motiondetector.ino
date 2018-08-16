#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DS3231.h"
#include "SPI.h"
#include "SD.h"
#include "U8glib.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define BUTTON_PIN 7
#define BUZZER_PIN 9 // use pin 9 on Arduino Uno for buzzer
#define CS_PIN 4     // use pin 12 as CS pin for SDCard

int pauseTimeSeconds = 5;
int maxDeviation = 10; // when deviation bigger then X sound alarm
bool isAlarmOn = false;

MPU6050 mpu(0x69); // AD0 high = 0x69, AD0 low = 0x68
DS3231 rtc(SDA, SCL);
File logFile;
U8GLIB_SSD1306_128X64 screen(U8G_I2C_OPT_NO_ACK);

//Change this 3 variables if you want to fine tune MPU precision
int buffersize = 1000; //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8; //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1; //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float last_x_angle; // These are the filtered angles
float last_y_angle;
float last_z_angle;
float last_gyro_x_angle; // Store the gyro angles to compare drift
float last_gyro_y_angle;
float last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
    if (isnan(x) || isnan(y) || isnan(z) || isnan(x_gyro) || isnan(y_gyro) || isnan(z_gyro))
        return;

    last_read_time = time;
    last_x_angle = x;
    last_y_angle = y;
    last_z_angle = z;
    last_gyro_x_angle = x_gyro;
    last_gyro_y_angle = y_gyro;
    last_gyro_z_angle = z_gyro;
}

void setup()
{

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
     Serial.begin(115200);

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    rtc.begin();
    // wait for Leonardo enumeration, others continue immediately
    while (!Serial)
    {
    }

    setupScreen();
    setupSDCard();
    setupMPU6050();

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
}

void setupScreen()
{
    screen.setFont(u8g_font_unifont);
    screen.setColorIndex(1); // Instructs the display to draw with a pixel on.
}

void setupSDCard()
{
    if (SD.begin(CS_PIN))
    {
        PrintToScreen(F("SD card is ready"));
    }
    else
    {
        PrintToScreen(F("Unable to connect SD card"));
    }
}

void PrintToScreen(const __FlashStringHelper *s)
{
    screen.firstPage();
    do
    {
        screen.drawStr(0, 40, s);
    } while (screen.nextPage());
    Serial.println(s);
}

void writeToLog(String datePart, String timePart, float x, float y, float z)
{
    logFile = SD.open("log.txt", FILE_WRITE);
    if (logFile)
    {
        logFile.print(datePart);
        logFile.print(" ");
        logFile.print(timePart);
        logFile.print(",");
        logFile.print(x);
        logFile.print(",");
        logFile.print(y);
        logFile.print(",");
        logFile.println(z);
        logFile.close();
    }
    // if the file didn't open, print an error:
    else
    {
        Serial.println("Error opening log.txt");
    }
}

/**
 * Gyroscope setup
 * */
void setupMPU6050()
{
    // initialize device
    PrintToScreen(F("Initializing I2C"));
    mpu.initialize();

    PrintToScreen(F("Testing connections"));
    mpu.testConnection()
        ? PrintToScreen(F("MPU connected"))
        : PrintToScreen(F("MPU6050 connection failed"));

    setGyroscopeOffset();
}

void playSound(int period)
{
    tone(BUZZER_PIN, 2500); // Send 1KHz sound signal...
    delay(period);          // ...for period in ms
    noTone(BUZZER_PIN);     // Stop sound...
}

void alarm(bool raise)
{
    isAlarmOn = raise;
    if (raise)
    {
        tone(BUZZER_PIN, 2500);
    }
    else
    {
        noTone(BUZZER_PIN);
    }
}

void setGyroscopeOffset()
{
    playSound(250);
    // reset offsets
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    PrintToScreen(F("Reading sensors"));
    meansensors();

    PrintToScreen(F("Calculating offsets..."));
    calibration();
    meansensors();

    PrintToScreen(F("Offsets calculated"));
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    playSound(500);
}

void meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { //First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(3); //Needed so we don't get repeated measures
    }
}

/**
 * Calibrates gyroscope offset
 * */
void calibration()
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    int ready = 0;
    bool meanAxReady = false;
    bool meanAyReady = false;
    bool meanAzReady = false;
    bool meanGxReady = false;
    bool meanGyReady = false;
    bool meanGzReady = false;

    while (ready < 6)
    {
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);

        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        meansensors();

        if (abs(mean_ax) <= acel_deadzone)
        {
            if (!meanAxReady)
            {
                PrintToScreen(F("mean ax ready"));
                meanAxReady = true;
                ready++;
            }
        }
        else
        {
            ax_offset = ax_offset - mean_ax / acel_deadzone;
        }

        if (abs(mean_ay) <= acel_deadzone)
        {
            if (!meanAyReady)
            {
                PrintToScreen(F("mean ay ready"));
                meanAyReady = true;
                ready++;
            }
        }
        else
        {
            ay_offset = ay_offset - mean_ay / acel_deadzone;
        }

        if (abs(16384 - mean_az) <= acel_deadzone)
        {
            if (!meanAzReady)
            {
                PrintToScreen(F("mean az ready"));
                meanAzReady = true;
                ready++;
            }
        }
        else
        {
            az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
        }

        if (abs(mean_gx) <= giro_deadzone)
        {
            if (!meanGxReady)
            {
                PrintToScreen(F("mean gx ready"));
                meanGxReady = true;
                ready++;
            }
        }
        else
        {
            gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
        }

        if (abs(mean_gy) <= giro_deadzone)
        {
            if (!meanGyReady)
            {
                PrintToScreen(F("mean gy ready"));
                meanGyReady = true;
                ready++;
            }
        }
        else
        {
            gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
        }

        if (abs(mean_gz) <= giro_deadzone)
        {
            if (!meanGzReady)
            {
                PrintToScreen(F("mean gz ready"));
                meanGzReady = true;
                ready++;
            }
        }
        else
        {
            gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
        }
    }
}

void pauseIfButtonIsPressed()
{
    if (digitalRead(BUTTON_PIN) == HIGH)
    {
        bool wasAlarmOn = isAlarmOn;
        if (isAlarmOn)
        {
            alarm(false);
        }

        logFile = SD.open("log.txt", FILE_WRITE);
        logFile.println(F("pause"));
        PrintToScreen(F("PAUSED"));
        delay(pauseTimeSeconds * 1000);
        if (wasAlarmOn)
        {
            alarm(true);
        }
    }
}

//char str_x[20];

void loop()
{
    pauseIfButtonIsPressed();

    double dT;

    // Read the raw values.
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Get the time of reading for rotation computations
    unsigned long t_now = millis();

    // Convert gyro values to degrees/sec
    float FS_SEL = 131;
    float gyro_x = gx / FS_SEL;
    float gyro_y = gy / FS_SEL;
    float gyro_z = gz / FS_SEL;

    // Get angle values from accelerometer
    float RADIANS_TO_DEGREES = 180 / 3.14159;
    float accel_angle_y = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_z = 0;

    // Compute the (filtered) gyro angles
    float dt = (t_now - last_read_time) / 1000.0;
    float gyro_angle_x = gyro_x * dt + last_x_angle;
    float gyro_angle_y = gyro_y * dt + last_y_angle;
    float gyro_angle_z = gyro_z * dt + last_z_angle;

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + last_gyro_x_angle;
    float unfiltered_gyro_angle_y = gyro_y * dt + last_gyro_y_angle;
    float unfiltered_gyro_angle_z = gyro_z * dt + last_gyro_z_angle;

    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    float alpha = 0.96;
    float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    float angle_z = gyro_angle_z; //Accelerometer doesn't give z-angle

    // Update the saved data with the latest values
    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    writeToLog(rtc.getDateStr(), rtc.getTimeStr(), angle_x, angle_y, angle_z);

    if ((abs(angle_x) > maxDeviation || abs(angle_y) > maxDeviation))
    {
        screen.firstPage();
        do
        {
            screen.drawStr(0, 10, rtc.getTimeStr());
            screen.drawStr(10, 30, F("ERROR"));
        } while (screen.nextPage());
        if (!isAlarmOn)
        {
            alarm(true);
        }
    }
    else
    {
        screen.firstPage();
        do
        {
            screen.drawStr(0, 10, rtc.getTimeStr());
            screen.drawStr(10, 30, F("Position OK"));
        } while (screen.nextPage());

        if (isAlarmOn)
        {
            alarm(false);
        }
    }

    // Delay so we don't swamp the serial port
    delay(20);
}
