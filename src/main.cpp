#include <Arduino.h>
#include "Wire.h"
#include <SparkFunLSM9DS1.h>


#define TCAADDR 0x70 //  https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

LSM9DS1 imu0, imu1, imu2, imu3; // https://www.sparkfun.com/products/13284

uint16_t status;

unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;

const unsigned int ACC_SAMPLE_RATE = 16;
 // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz

const unsigned int ACC_SCALE = 2;

// Set accel range to +/-16g
  //imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  //imu.settings.mag.scale = 8; // Set mag range to +/-8Gs  


const unsigned int ACC_BANDWIDTH = 0;
  // imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution 
  // mode for the acclerometer.
const boolean ACC_HIGH_RESOLUTION = true;
  // imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400

  // imu.settings.accel.highResBandwidth = 0;    


// Global variables to print to serial monitor at a steady rate
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 1;


uint16_t initLSM9DS1(int sensorNumber)
{
  switch (sensorNumber)
  {
  case 0:  return imu0.begin();
  case 1:  return imu1.begin();
  case 2:  return imu2.begin();
  case 3:  return imu3.begin(); 
  default:  break;
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  // put your setup code here, to run once:

  imu0.settings.device.commInterface = IMU_MODE_I2C;
  imu0.settings.device.mAddress = LSM9DS1_M;
  imu0.settings.device.agAddress = LSM9DS1_AG;
  imu0.settings.accel.scale = ACC_SCALE;
  imu0.settings.gyro.enabled = false;
  imu0.settings.mag.enabled = false; 
  imu0.settings.temp.enabled = false;
  imu0.settings.accel.sampleRate = ACC_SAMPLE_RATE;
  imu0.settings.accel.highResEnable = ACC_HIGH_RESOLUTION;

  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu1.settings.device.mAddress = LSM9DS1_M;
  imu1.settings.device.agAddress = LSM9DS1_AG;
  imu1.settings.accel.scale = ACC_SCALE;
  imu1.settings.gyro.enabled = false;
  imu1.settings.mag.enabled = false; 
  imu1.settings.temp.enabled = false;
  imu1.settings.accel.sampleRate = ACC_SAMPLE_RATE;

  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.mAddress = LSM9DS1_M;
  imu2.settings.device.agAddress = LSM9DS1_AG;
  imu2.settings.accel.scale = ACC_SCALE;
  imu2.settings.gyro.enabled = false;
  imu2.settings.mag.enabled = false; 
  imu2.settings.temp.enabled = false;
  imu2.settings.accel.sampleRate = ACC_SAMPLE_RATE;

  imu3.settings.device.commInterface = IMU_MODE_I2C;
  imu3.settings.device.mAddress = LSM9DS1_M;
  imu3.settings.device.agAddress = LSM9DS1_AG;
  imu3.settings.accel.scale = ACC_SCALE;
  imu3.settings.gyro.enabled = false;
  imu3.settings.mag.enabled = false; 
  imu3.settings.temp.enabled = false;
  imu3.settings.accel.sampleRate = ACC_SAMPLE_RATE;


  Wire.begin();
  Serial.begin(115200);
  
  
//---------------I2C multiplixer ports------------------------------------
  for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");

    tcaselect(0); // sensor #0
    Serial.println("Initializing the LSM9DS1 #0");
    status = initLSM9DS1(0);
    Serial.print("LSM9DS1 #0 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x683D");
    Serial.println();

    tcaselect(1); // sensor #1
    Serial.println("Initializing the LSM9DS1 #1");
    status = initLSM9DS1(1);
    Serial.print("LSM9DS1 #1 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x683D");
    Serial.println();

    tcaselect(2); // sensor #2
    Serial.println("Initializing the LSM9DS1 #2");
    status = initLSM9DS1(2);
    Serial.print("LSM9DS1 #2 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x683D");
    Serial.println();

    tcaselect(3); // sensor #3
    Serial.println("Initializing the LSM9DS1 #3");
    status = initLSM9DS1(3);
    Serial.print("LSM9DS1 #3 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x683D");
    Serial.println();
  
    startTime = millis();

}

void printSensorReadings(int sensorNumber)
{
  float runTime = (float)(millis() - startTime) / 1000.0;
  float accelRate = (float)accelReadCounter / runTime;
  //Serial.print("Sensor #");
  Serial.print(runTime, 4);
  Serial.print(" ");
  //Serial.print(" runTime: ");
  //Serial.print(" ");
  Serial.print(sensorNumber);
  
  //Serial.print(" A: ");
  Serial.print(" ");
  switch (sensorNumber)
  {
    case 0: 
      Serial.print(imu0.calcAccel(imu0.ax), 3);
      Serial.print(" ");
      Serial.print(imu0.calcAccel(imu0.ay), 3);
      Serial.print(" ");
      Serial.print(imu0.calcAccel(imu0.az), 3);
      Serial.print(" ");
      break;
    case 1: 
      Serial.print(imu1.calcAccel(imu1.ax));
      Serial.print(" ");
      Serial.print(imu1.calcAccel(imu1.ay));
      Serial.print(" ");
      Serial.print(imu1.calcAccel(imu1.az));
      Serial.print(" ");
      break;
    case 2: 
      Serial.print(imu2.calcAccel(imu2.ax));
      Serial.print(" ");
      Serial.print(imu2.calcAccel(imu2.ay));
      Serial.print(" ");
      Serial.print(imu2.calcAccel(imu2.az));
      Serial.print(" ");
      break;
    case 3: 
      Serial.print(imu3.calcAccel(imu3.ax));
      Serial.print(" ");
      Serial.print(imu3.calcAccel(imu3.ay));
      Serial.print(" ");
      Serial.print(imu3.calcAccel(imu3.az));
      Serial.print(" ");
      break;
    //default: Serial.print(65535);
  }
  //Serial.print(accelRate);
  Serial.println();
  
}

void loop() {
  // Every PRINT_RATE milliseconds, print sensor data:
  if ((lastPrint + PRINT_RATE) < millis())
  {
    tcaselect(0);
    imu0.readAccel();
    printSensorReadings(0);

    tcaselect(1);
    imu1.readAccel();
    printSensorReadings(1);

    tcaselect(2);
    imu2.readAccel();
    printSensorReadings(2);

    tcaselect(3);
    imu3.readAccel();
    printSensorReadings(3);

    lastPrint = millis();
  }
}