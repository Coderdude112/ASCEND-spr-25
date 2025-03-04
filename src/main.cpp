// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>
#include <STM32SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <ISM330DLC_ACC_GYRO_Driver.h>
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
uint16_t scd41data;
uint16_t co2Concentration = 0; 
float temperature = 0.0;
float relativeHumidity = 0.0;


imu::Vector<3> euler; // Euler angles (degrees)
imu::Quaternion quat; // Quaternion (w, x, y, z)
uint8_t systemCal, gyro, accel, mag; // Calibration status for each sensor
int8_t temp; // Temperature (Celsius)
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
float x, y, z; // Variables to store the x, y, z values of the sensor data
void printEvent(File &file, sensors_event_t* event); // Function to print the sensor data to the .csv file
Adafruit_AS7341 as7341;

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
  
    // ISM330DLC_ACC_GYRO
  // This sensor uses I2C or SPI to communicate. For I2C it is then required to create a 
  // TwoWire interface before accessing to the sensors:
  TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
  dev_i2c.begin();
  
  // For SPI it is then required to create a SPI interface before accessing to the sensors:
  SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
  dev_spi.begin();

  // An instance can be created and enabled when the I2C bus is used following the procedure below:
  ISM330DLCSensor AccGyr(&dev_i2c);
  AccGyr.begin();
  AccGyr.Enable_X();  
  AccGyr.Enable_G();
  
  // An instance can be created and enabled when the SPI bus is used following the procedure below:
  ISM330DLCSensor AccGyr(&dev_spi, CS_PIN);
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
    // Get data from BNO055 //
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // Euler angles (degrees)
    quat = bno.getQuat(); // Quaternion (w, x, y, z)
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

    // Get calibration status for each sensor //
    bno.getCalibration(&systemCal, &gyro, &accel, &mag);

    // Print the data to the .csv file //
    // dataFile is already open in setup, no need to reopen it
    if(dataFile){
        // Absolute Orientation (Euler Vector, 100Hz) //
        dataFile.print("Euler: ");
        dataFile.print(euler.x()); dataFile.print(", ");
        dataFile.print(euler.y()); dataFile.print(", ");
        dataFile.print(euler.z()); dataFile.print(", ");
        // Absolute Orientation (Quaterion, 100Hz) //
        dataFile.print("Quaterion: ");
        dataFile.print(quat.w(), 4); dataFile.print(", ");
        dataFile.print(quat.x(), 4); dataFile.print(", ");
        dataFile.print(quat.y(), 4); dataFile.print(", ");
        dataFile.print(quat.z(), 4); dataFile.print(", ");
        
        dataFile.print("Temp: "); dataFile.print(temp); dataFile.print(", ");

        printEvent(dataFile, &orientationData);
        printEvent(dataFile, &angVelocityData);
        printEvent(dataFile, &linearAccelData);
        printEvent(dataFile, &magnetometerData);
        printEvent(dataFile, &accelerometerData);
        printEvent(dataFile, &gravityData);

        dataFile.flush();
    }
    else{
        Serial.println("Error writing to file");
    }
    
    /* Testing Code for BNO055 */
    // Absolute Orientation (Euler Vector, 100Hz) //
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.println("");
   
    // Absolute Orientation (Quaterion, 100Hz) //
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.println("");

    // Display calibration status for each sensor. //
    Serial.print("CALIBRATION: Sys: "); Serial.print((int)systemCal, DEC);
    Serial.print(" Gyro: "); Serial.print((int)gyro, DEC);
    Serial.print(" Accel: "); Serial.print((int)accel, DEC);
    Serial.print(" Mag: "); Serial.println((int)mag, DEC);

    // Temperature (1Hz) //
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    delay(100);
}

// ---- //
/* Functions */
// ---- //

void printEvent(File &file, sensors_event_t* event){
    if(event->type == SENSOR_TYPE_ACCELEROMETER){
        file.print("Accleration (): ");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if(event->type == SENSOR_TYPE_ORIENTATION){
        file.print("Orientation (): ");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if(event->type == SENSOR_TYPE_MAGNETIC_FIELD){
        file.print("Magnetometer (uT): ");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if(event->type == SENSOR_TYPE_GYROSCOPE){
        file.print("Gyroscope (rad/s): ");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if(event->type == SENSOR_TYPE_ROTATION_VECTOR){
        file.print("Rotation Vector (): ");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if(event->type == SENSOR_TYPE_LINEAR_ACCELERATION){
        file.print("Linear Acceleration (m/s^2): ");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if(event->type == SENSOR_TYPE_GRAVITY){
        file.print("Gravity (m/s^2):");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else{
        file.print("Unk:");
    }
    
      file.print("\tx = ");
      file.print(x);
      file.print(" |\ty = ");
      file.print(y);
      file.print(" |\tz = ");
      file.println(z);
}