#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DS3231.h"
#include "SPI.h"
#include "SD.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

int maxDeviation = 10; // when deviation bigger then X sound alarm
bool isAlarmOn = false;

#pragma region variables

// class default I2C address is 0x68 specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high

DS3231 rtc(SDA, SCL);

File logFile;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUZZER_PIN 9    // use pin 9 on Arduino Uno for buzzer
#define CS_PIN 4        // use pin 12 as CS pin for SDCard

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

//Change this 3 variables if you want to fine tune the skecth to your needs.
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
    last_read_time = time;
    last_x_angle = x;
    last_y_angle = y;
    last_z_angle = z;
    last_gyro_x_angle = x_gyro;
    last_gyro_y_angle = y_gyro;
    last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() { return last_read_time; }
inline float get_last_x_angle() { return last_x_angle; }
inline float get_last_y_angle() { return last_y_angle; }
inline float get_last_z_angle() { return last_z_angle; }
inline float get_last_gyro_x_angle() { return last_gyro_x_angle; }
inline float get_last_gyro_y_angle() { return last_gyro_y_angle; }
inline float get_last_gyro_z_angle() { return last_gyro_z_angle; }

#pragma endregion

/**
 * INTERRUPT DETECTION ROUTINE
 * */
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    Serial.begin(115200);
    rtc.begin();
    // wait for Leonardo enumeration, others continue immediately
    while (!Serial)
    {
    }

    setupMPU6050();
    setupSDCard();
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
}

void setupSDCard()
{
    if (SD.begin(CS_PIN))
    {
        Serial.println("SD card is ready");
    }
    else
    {
        Serial.println("Unable to connect SD card");
    }
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
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

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

    Serial.println("Reading sensors for first time...");
    meansensors();

    Serial.println("Calculating offsets...");
    calibration();

    Serial.println("Offsets calculated");

    meansensors();
    Serial.print("Sensor readings with offsets:\t");
    Serial.print(mean_ax);
    Serial.print("\t");
    Serial.print(mean_ay);
    Serial.print("\t");
    Serial.print(mean_az);
    Serial.print("\t");
    Serial.print(mean_gx);
    Serial.print("\t");
    Serial.print(mean_gy);
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset);
    mpu.setXAccelOffset(ax_offset);
    Serial.print("\t");
    Serial.print(ay_offset);
    mpu.setYAccelOffset(ay_offset);
    Serial.print("\t");
    Serial.print(az_offset);
    mpu.setZAccelOffset(az_offset);
    Serial.print("\t");
    Serial.print(gx_offset);
    mpu.setXGyroOffset(gx_offset);
    Serial.print("\t");
    Serial.print(gy_offset);
    mpu.setYGyroOffset(gy_offset);
    Serial.print("\t");
    Serial.println(gz_offset);
    mpu.setZGyroOffset(gz_offset);
    Serial.println("Data is printed as: acelX acelY acelZ giroX giroY giroZ");

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
        delay(2); //Needed so we don't get repeated measures
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
            ready++;
        else
            ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    }
}

void loop()
{
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

    // Get raw acceleration values
    //float G_CONVERT = 16384;
    float accel_x = ax;
    float accel_y = ay;
    float accel_z = az;

    // Get angle values from accelerometer
    float RADIANS_TO_DEGREES = 180 / 3.14159;
    // float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
    float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;

    float accel_angle_z = 0;

    // Compute the (filtered) gyro angles
    float dt = (t_now - get_last_time()) / 1000.0;
    float gyro_angle_x = gyro_x * dt + get_last_x_angle();
    float gyro_angle_y = gyro_y * dt + get_last_y_angle();
    float gyro_angle_z = gyro_z * dt + get_last_z_angle();

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    float alpha = 0.96;
    float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    float angle_z = gyro_angle_z; //Accelerometer doesn't give z-angle

    // Update the saved data with the latest values
    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    // Send the data to the serial port
    Serial.print(F("DEL:")); //Delta T
    Serial.print(dt, DEC);
    Serial.print(F("#ACC:")); //Accelerometer angle
    Serial.print(accel_angle_x, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_y, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_z, 2);
    Serial.print(F("#GYR:"));
    Serial.print(unfiltered_gyro_angle_x, 2); //Gyroscope angle
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_y, 2);
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_z, 2);
    Serial.print(F("#FIL:")); //Filtered angle
    Serial.print(angle_x, 2);
    Serial.print(F(","));
    Serial.print(angle_y, 2);
    Serial.print(F(","));
    Serial.print(angle_z, 2);
    Serial.println(F(""));

    writeToLog(rtc.getDateStr(), rtc.getTimeStr(), angle_x, angle_y, angle_z);

    if ((abs(angle_x) > maxDeviation || abs(angle_y) > maxDeviation))
    {
        if (!isAlarmOn)
        {
            alarm(true);
        }
    }
    else
    {
        if (isAlarmOn)
        {
            alarm(false);
        }
    }

    // Delay so we don't swamp the serial port
    delay(5);
}
