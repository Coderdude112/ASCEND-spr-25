// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>

// ---- //
/* Vars */
// ---- //

File dataFile; //Instance of the File class

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
}

void loop() {
    
}
