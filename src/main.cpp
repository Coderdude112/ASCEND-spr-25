// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>

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
    

    /* Logging onto SD */
    // Initialize the SD card //
    Serial.print("Initializing SD card...");
    if(!SD.begin()){
        Serial.println("Initialization failed!");
        while(1);
    }
    // Create a new file with a different name if the file already exists //
    int fileIndex = 1;
    String dataFileName = "flightdata.csv";
    while (SD.exists(dataFileName)){
        dataFileName = "flightdata" + String(fileIndex) + ".csv";
        fileIndex++;
    }
    // Open the file //
    Serial.println("Initialization successful.");
    dataFile = SD.open("flightdata.csv", FILE_WRITE);
    if(dataFile){
        Serial.print("Writing to flightdata.csv...");
        dataFile.println("X, Y, Z, qW, qX, qY, qZ, Temp"); //Printing out the headers
      }
      else{
        Serial.println("Error opening flightdata.csv");
      }

    /* Initialize the BNO055 */
    if(!bno.begin()){
        Serial.print("There was a problem detecting the BNO055 ... check your wiring or I2C ADDR!");
        while(1);
    }
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
}

void loop() {
    
    // Get the Euler angles, quaternion, and temperature //
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    int8_t temp = bno.getTemp();

    // Print the data to the .csv file //
    dataFile = SD.open("flightdata.csv", FILE_WRITE);
    if(dataFile){
        dataFile.print(euler.x()); dataFile.print(", ");
        dataFile.print(euler.y());
        dataFile.print(", ");
        dataFile.print(euler.z());
        dataFile.print(", ");
        dataFile.print(quat.w());
        dataFile.print(", ");
        dataFile.print(quat.x());
        dataFile.print(", ");
        dataFile.print(quat.y());
        dataFile.print(", ");
        dataFile.print(quat.z());
        dataFile.print(", ");
        dataFile.print(temp);
        dataFile.println();
        dataFile.close();
    }
    else{
        Serial.println("Error opening flightdata.csv");
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

    // Temperature (1Hz) //
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    delay(100);
}
