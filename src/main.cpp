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

const unsigned int GYRO_SCALE = 245;
// [scale] sets the full-scale range of the gyroscope.
// scale can be set to either 245, 500, or 2000
// Set scale to +/-245dps

const unsigned int GYRO_SAMPLE_RATE = 6;
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952

const unsigned int GYRO_BANDWIDTH = 0;
// [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)


const boolean GYRO_LOW_POWER = false; // LP mode off
// [lowPowerEnable] turns low-power mode on or off.
  
  
const boolean GYRO_HPF_ENABLE = false;
  // [HPFEnable] enables or disables the high-pass filter


const int GYRO_LOW_HPF_CUTOFF = 1; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)


const unsigned int MAG_SCALE = 4; // Gs
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16

const unsigned int MAG_SAMPLE_RATE = 7;
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz

const unsigned int GYRO_SAMPLE_BANDWIDTH = 0;
// [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)


const boolean MAG_TEMP_COMPENSATION = true; 
// temperature compensation of the magnetometer.
  
const unsigned int MAG_XYPerformance = 3;
// 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance  

const unsigned int MAG_ZPerformance = 3;
// 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance   
  
const boolean MAG_LOW_POWER = false;

const int MAG_OPERATING_MODE = 0; // Continuous mode





// Global variables to print to serial monitor at a steady rate
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 1;


uint16_t initLSM9DS1(int sensorNumber)
{
  switch (sensorNumber)
  {
  case 0:  return imu0.begin(); break;
  case 1:  return imu1.begin(); break;
  case 2:  return imu2.begin(); break;
  case 3:  return imu3.begin(); break;
  default: return 0;  break;
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
  imu1.settings.accel.highResEnable = ACC_HIGH_RESOLUTION;

  // imu2.settings.device.commInterface = IMU_MODE_I2C;
  // imu2.settings.device.mAddress = LSM9DS1_M;
  // imu2.settings.device.agAddress = LSM9DS1_AG;
  // imu2.settings.accel.scale = ACC_SCALE;
  // imu2.settings.gyro.enabled = false;
  // imu2.settings.mag.enabled = false; 
  // imu2.settings.temp.enabled = false;
  // imu2.settings.accel.sampleRate = ACC_SAMPLE_RATE;

  // imu3.settings.device.commInterface = IMU_MODE_I2C;
  // imu3.settings.device.mAddress = LSM9DS1_M;
  // imu3.settings.device.agAddress = LSM9DS1_AG;
  // imu3.settings.accel.scale = ACC_SCALE;
  // imu3.settings.gyro.enabled = false;
  // imu3.settings.mag.enabled = false; 
  // imu3.settings.temp.enabled = false;
  // imu3.settings.accel.sampleRate = ACC_SAMPLE_RATE;



  
  imu0.settings.gyro.enabled = true;
  imu0.settings.gyro.scale = GYRO_SCALE; 
  imu0.settings.gyro.sampleRate = GYRO_SAMPLE_RATE; 
  imu0.settings.gyro.bandwidth = GYRO_BANDWIDTH;
  imu0.settings.gyro.lowPowerEnable = GYRO_LOW_POWER; 
  imu0.settings.gyro.HPFEnable = GYRO_HPF_ENABLE; 
  imu0.settings.gyro.HPFCutoff = GYRO_LOW_HPF_CUTOFF; 
  imu0.settings.gyro.flipX = false; // Don't flip X
  imu0.settings.gyro.flipY = false; // Don't flip Y
  imu0.settings.gyro.flipZ = false; // Don't flip Z

  imu1.settings.gyro.enabled = true;
  imu1.settings.gyro.scale = GYRO_SCALE; 
  imu1.settings.gyro.sampleRate = GYRO_SAMPLE_RATE; 
  imu1.settings.gyro.bandwidth = GYRO_BANDWIDTH;
  imu1.settings.gyro.lowPowerEnable = GYRO_LOW_POWER; 
  imu1.settings.gyro.HPFEnable = GYRO_HPF_ENABLE; 
  imu1.settings.gyro.HPFCutoff = GYRO_LOW_HPF_CUTOFF; 
  imu1.settings.gyro.flipX = false; // Don't flip X
  imu1.settings.gyro.flipY = false; // Don't flip Y
  imu1.settings.gyro.flipZ = false; // Don't flip Z


  
  imu0.settings.mag.enabled = true; 
  imu0.settings.mag.scale = MAG_SCALE; 
  imu0.settings.mag.sampleRate = MAG_SAMPLE_RATE; 
  imu0.settings.mag.tempCompensationEnable = MAG_TEMP_COMPENSATION;
  imu0.settings.mag.XYPerformance = MAG_XYPerformance; 
  imu0.settings.mag.ZPerformance = MAG_ZPerformance; 
  imu0.settings.mag.lowPowerEnable = MAG_LOW_POWER;
  imu0.settings.mag.operatingMode = 0; // Continuous mode

  imu1.settings.mag.enabled = true; 
  imu1.settings.mag.scale = MAG_SCALE; 
  imu1.settings.mag.sampleRate = MAG_SAMPLE_RATE; 
  imu1.settings.mag.tempCompensationEnable = MAG_TEMP_COMPENSATION;
  imu1.settings.mag.XYPerformance = MAG_XYPerformance; 
  imu1.settings.mag.ZPerformance = MAG_ZPerformance; 
  imu1.settings.mag.lowPowerEnable = MAG_LOW_POWER;
  imu1.settings.mag.operatingMode = 0; // Continuous mode

  imu0.settings.temp.enabled = true;
  imu1.settings.temp.enabled = true;


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

    // tcaselect(2); // sensor #2
    // Serial.println("Initializing the LSM9DS1 #2");
    // status = initLSM9DS1(2);
    // Serial.print("LSM9DS1 #2 WHO_AM_I's returned: 0x");
    // Serial.println(status, HEX);
    // Serial.println("Should be 0x683D");
    // Serial.println();

    // tcaselect(3); // sensor #3
    // Serial.println("Initializing the LSM9DS1 #3");
    // status = initLSM9DS1(3);
    // Serial.print("LSM9DS1 #3 WHO_AM_I's returned: 0x");
    // Serial.println(status, HEX);
    // Serial.println("Should be 0x683D");
    // Serial.println();
  
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

      Serial.print(imu0.calcGyro(imu0.ax), 3);
      Serial.print(" ");
      Serial.print(imu0.calcGyro(imu0.ay), 3);
      Serial.print(" ");
      Serial.print(imu0.calcGyro(imu0.az), 3);
      Serial.print(" ");
      
      Serial.print(imu0.calcMag(imu0.ax), 3);
      Serial.print(" ");
      Serial.print(imu0.calcMag(imu0.ay), 3);
      Serial.print(" ");
      Serial.print(imu0.calcMag(imu0.az), 3);
      Serial.print(" ");
      break;

    case 1: 
      Serial.print(imu1.calcAccel(imu1.ax), 3);
      Serial.print(" ");
      Serial.print(imu1.calcAccel(imu1.ay), 3);
      Serial.print(" ");
      Serial.print(imu1.calcAccel(imu1.az), 3);
      Serial.print(" ");

      Serial.print(imu1.calcGyro(imu1.ax), 3);
      Serial.print(" ");
      Serial.print(imu1.calcGyro(imu1.ay), 3);
      Serial.print(" ");
      Serial.print(imu1.calcGyro(imu1.az), 3);
      Serial.print(" ");

      Serial.print(imu1.calcMag(imu1.ax), 3);
      Serial.print(" ");
      Serial.print(imu1.calcMag(imu1.ay), 3);
      Serial.print(" ");
      Serial.print(imu1.calcMag(imu1.az), 3);
      Serial.print(" ");
      break;
      
    case 2: 
      Serial.print(imu2.calcAccel(imu2.ax), 3);
      Serial.print(" ");
      Serial.print(imu2.calcAccel(imu2.ay), 3);
      Serial.print(" ");
      Serial.print(imu2.calcAccel(imu2.az), 3);
      Serial.print(" ");
      break;
    case 3: 
      Serial.print(imu3.calcAccel(imu3.ax), 3);
      Serial.print(" ");
      Serial.print(imu3.calcAccel(imu3.ay), 3);
      Serial.print(" ");
      Serial.print(imu3.calcAccel(imu3.az), 3);
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
    imu0.readGyro();
    imu0.readMag();
    printSensorReadings(0);

    tcaselect(1);
    imu1.readAccel();
    imu1.readGyro();
    imu1.readMag();
    printSensorReadings(1);

   /*   tcaselect(2);
    imu2.readAccel();
    //printSensorReadings(2);

    tcaselect(3);
    imu3.readAccel();
    //printSensorReadings(3); */

    lastPrint = millis();
  }
}