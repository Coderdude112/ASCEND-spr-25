// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>
#include <STM32SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <ISM330DLCSensor.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>

// ---- //
/* Vars */
// ---- //

File dataFile; //Instance of the File class
Adafruit_BNO055 bno; //Instance of the Adafruit_BNO055 class
//GPS Code
Adafruit_GPS GPS(&Wire);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
uint32_t timer = millis();
char c;

SensirionI2cScd4x scd41;
int16_t scd41data;
uint16_t co2Concentration = 0; 
int16_t temperature = 0.0;
int16_t relativeHumidity = 0.0;

uint8_t systemCal, gyro, accel, mag; // Calibration status for each sensor
int8_t temp; // Temperature (Celsius)
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
float x, y, z; // Variables to store the x, y, z values of the sensor data
Adafruit_AS7341 as7341;

char buffer[50];
int floatBuffer[2];

// --- //
/* Functions */
// -- //
void printEvent(File &file, sensors_event_t* event); // Function to print the sensor data to the .csv file
void splitFloat(int buffer[], float in);
void toCharArray(String in);
void toCharArray(char* in);
void toCharArray(const char* in);
void toCharArray(int in);
void toCharArray(float in);
void toCharArray(bool in);
void toCharArray(uint8_t in);
void toCharArray(uint16_t in);
void toCharArray(int8_t in);

// ---- //
/* Main */
// ---- //

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(0);
    Wire.begin();
    scd41.begin(Wire, SCD41_I2C_ADDR_62);
    int scd41good = scd41.wakeUp();
    if (scd41good == 0) {
        Serial.println("Error waking up the SCD41 sensor.");
    }
    scd41.startPeriodicMeasurement();
  
    // ISM330DLC_ACC_GYRO
    // An instance can be created and enabled when the I2C bus is used following the procedure below:
    ISM330DLCSensor AccGyr(&Wire);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();
  
  // Read accelerometer and gyroscope.
  int32_t accelerometer[3];
  int32_t gyroscope[3];

  //GPS Code
    //while (!Serial);

    // also spit it out
    Serial.println("Adafruit I2C GPS library basic test!");
    
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(0x10);  // The I2C address to use is 0x10
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);
    
    // Ask for firmware version
    GPS.println(PMTK_Q_RELEASE);

     /* Initialize the BNO055 */
    if(!bno.begin()){
        Serial.print("There was a problem detecting the BNO055 ... check your wiring or I2C ADDR!");
        while(1);
    }

    //AS7341
    if (!as7341.begin()){
        Serial.println("Could not find AS7341");
        while (1) { delay(10); }
      }
      as7341.setATIME(100);
      as7341.setASTEP(999);
      as7341.setGain(AS7341_GAIN_256X);

    /* Logging onto SD */ //https://docs.arduino.cc/learn/programming/sd-guide/
    // Initialize the SD card //
    Serial.print("Initializing SD card...");
    if(!SD.begin(D32)){
        Serial.println("Initialization failed!");
        delay(10);
        while(1);
    }
    // Create a new file with a different name if the file already exists //
    int fileIndex = 1;
    char dataFileName[25] = "flightdata.csv";

    while (SD.exists(dataFileName)){
        sprintf(dataFileName, "flightdata%d.csv", fileIndex);
        fileIndex++;
    }
    // Open the file //
    Serial.println("Initialization successful.");
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if(dataFile){
        Serial.print("Writing to ");
        Serial.println(dataFileName);
        dataFile.println("X, Y, Z, qW, qX, qY, qZ, Temp, Gyro X, Gyro Y, Gyro Z, Accel X, Accel Y, Accel Z"); // Printing out the headers
        Serial.println("File setup complete.");
    }
    else{
        Serial.println("Error opening file");
    }
}

void loop() {
    // Get data from SCD 41 Sensor: //
    scd41data = scd41.readMeasurementRaw(&co2Concentration, &temperature, &relativeHumidity);
    // Get data from BNO055 //
    temp = bno.getTemp(); // Temperature (Celsius)
    // Get Sensor Events
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    // Print the data to the .csv file //
    // dataFile is already open in setup, no need to reopen it
    if(dataFile){
        
        toCharArray(temp); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(orientationData.orientation.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(orientationData.orientation.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(orientationData.orientation.z); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(angVelocityData.gyro.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(angVelocityData.gyro.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(angVelocityData.gyro.z); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(linearAccelData.acceleration.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(linearAccelData.acceleration.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(linearAccelData.acceleration.z); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(magnetometerData.magnetic.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(magnetometerData.magnetic.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(magnetometerData.magnetic.z); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(accelerometerData.acceleration.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(accelerometerData.acceleration.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(accelerometerData.acceleration.z); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gravityData.acceleration.x); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gravityData.acceleration.y); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gravityData.acceleration.z); dataFile.print(buffer); dataFile.print(", ");

        // SCD41
        scd41data = scd41.readMeasurement(&co2Concentration, &temperature, &relativeHumidity);
        toCharArray(co2Concentration); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(temperature); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(relativeHumidity); dataFile.print(buffer); dataFile.print(", ");

        dataFile.flush();
    }
    else{
        Serial.println("Error writing to file");
    }
}

// ---- //
/* Functions */
// ---- //

void splitFloat(int buffer[], float in) {
  buffer[0] = in;
  buffer[1] = abs(trunc((in - buffer[0]) * 1000));
}

void toCharArray(String in) {
  in.toCharArray(buffer, sizeof(buffer));
}
void toCharArray(char* in) {
  sprintf(buffer, in);
}
void toCharArray(const char* in) {
  sprintf(buffer, in);
}
void toCharArray(int in) {
  sprintf(buffer, "%i", in);
}
void toCharArray(float in) {
  splitFloat(floatBuffer, in);
  sprintf(buffer, "%i.%i", floatBuffer[0], floatBuffer[1]);
}
void toCharArray(bool in) {
  int convertedIn = in;
  sprintf(buffer, (in ? "true" : "false"));
}
void toCharArray(uint8_t in) {
  sprintf(buffer, "%i", in);
}
void toCharArray(uint16_t in) {
  sprintf(buffer, "%i", in);
}
void toCharArray(int8_t in) {
  sprintf(buffer, "%i", in);
}