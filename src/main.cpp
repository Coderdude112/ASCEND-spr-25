// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>

// ---- //
/* Vars */
// ---- //

File dataFile; //Instance of the File class
Adafruit_BNO055 bno; //Instance of the Adafruit_BNO055 class

// ---- //
/* Main */
// ---- //

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(0);

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