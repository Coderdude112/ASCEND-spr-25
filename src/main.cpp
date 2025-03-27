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
#include <Adafruit_BMP3XX.h>

// ---- //
/* Vars */
// ---- //

// Logging Vars
File dataFile; //Instance of the File class
char buffer[50];
int floatBuffer[2];

// GPS Vars
Adafruit_GPS GPS(&Wire);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
uint32_t timer = millis();
char c;

// BNO055 Vars
Adafruit_BNO055 bno;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
float x, y, z; // Variables to store the x, y, z values of the sensor data

// ISM 330DLC Vars
int32_t accelerometer[3];
int32_t gyroscope[3];

SensirionI2cScd4x scd41;
int16_t scd41data;
uint16_t co2Concentration = 0; 
float temperature = 0.0;
float relativeHumidity = 0.0;

// AS7341 Vars
Adafruit_AS7341 as7341;

// BPM390 Vars
Adafruit_BMP3XX baro;
const float seaLevelPressure = 1013.25;


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
void toCharArray(int32_t in);

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

    // GPS
    GPS.begin(0x10);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet

    delay(1000);
    
    GPS.println(PMTK_Q_RELEASE); // Ask for firmware version

    // BNO055
    if(!bno.begin()){
        Serial.print("There was a problem detecting the BNO055 ... check your wiring or I2C ADDR!");
        while(1);
    }

    // AS7341
    if(!as7341.begin()){
        Serial.println("Could not find AS7341");
        while (1) { delay(10); }
      }
      as7341.setATIME(100);
      as7341.setASTEP(999);
      as7341.setGain(AS7341_GAIN_256X);

    // BMP390
    baro.begin_I2C(0x76);
    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    baro.setOutputDataRate(BMP3_ODR_100_HZ);

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

    while(SD.exists(dataFileName)){
        sprintf(dataFileName, "flightdata%d.csv", fileIndex);
        fileIndex++;
    }
    // Open the file //
    Serial.println("Initialization successful.");
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if(dataFile){
        Serial.print("Writing to ");
        Serial.println(dataFileName);
        dataFile.print("Time, Fix, Quality, Latitude Degrees. Longitute Degrees, Speed, Angle, Altitude, Sats, Magnetic Variation, HDOP, VDOP, PDOP, "); //GPS Header
        dataFile.print("Temperature, X, Y, Z, Gyro X, Gyro Y, Gyro Z, Linear Accel X, Linear Accel Y, Linear Accel Z, Mag X, Mag Y, Mag Z, Accelerometer Accel X, Accelerometer Accel Y, Accelerometer Accel Z, Gravity X, Gravity Y, Gravity Z, "); // BNO055 Header
        dataFile.print("Accelerometer Accel X, Accelerometer Accel Y, Accelerometer Accel Z, Gyro X, Gyro Y, Gyro Z, "); // ISM 330DLC Header
        dataFile.print("CO2 Concentration, Temperature, Relative Humidity, "); // SCD41 Header
        dataFile.print("415nm, 445nm, 480nm, 515nm, 555nm, 590nm, 630nm, 680nm, Clear, NIR, "); // AS7341 Header
        dataFile.println("Altitude, Pressure, Temperature"); // BMP390 Header
        Serial.println("File setup complete.");
    }
    else{
        Serial.println("Error opening file");
    }
}

void loop() {
    // Print the data to the .csv file //
    // dataFile is already open in setup, no need to reopen it
    if(dataFile){

        //GPS (refer to https://adafruit.github.io/Adafruit_GPS/html/class_adafruit___g_p_s.html#a2d3b65036628a65d1e119d3d9a69678c)
        if(GPS.available()){
            GPS.read();
        }
        if(GPS.newNMEAreceived()){
          if(!GPS.parse(GPS.lastNMEA())){ // this also sets the newNMEAreceived() flag to false
            return;
          }
        }
        if(millis() - timer > 2000){
          timer = millis(); // reset the timer
          toCharArray("20"); toCharArray(GPS.year); dataFile.print(buffer); dataFile.print("-");
          toCharArray(GPS.month); dataFile.print(buffer); dataFile.print("-");
          toCharArray(GPS.day); dataFile.print(buffer); dataFile.print(" ");
          if(GPS.hour < 10){
            toCharArray('0');
          }
          toCharArray(GPS.hour); dataFile.print(buffer); toCharArray(':');
          if(GPS.minute < 10){
            toCharArray('0');
          }
          toCharArray(GPS.minute); dataFile.print(buffer); toCharArray(':');
          if(GPS.seconds < 10){
            toCharArray('0');
          }
          toCharArray(GPS.seconds); dataFile.print(buffer); toCharArray('.');
          if(GPS.milliseconds < 10){
            dataFile.print("00");
          }
          else if(GPS.milliseconds > 9 && GPS.milliseconds < 100){
            dataFile.print("0");
          }
          toCharArray(GPS.milliseconds); dataFile.print(buffer); dataFile.print(", ");
          toCharArray((int)GPS.fix); dataFile.print(buffer); dataFile.print(", ");
          toCharArray((int)GPS.fixquality); dataFile.print(buffer); dataFile.print(", ");
          if(GPS.fix){
            toCharArray(GPS.latitudeDegrees); dataFile.print(buffer); toCharArray(GPS.lat); dataFile.print(", ");
            toCharArray(GPS.longitudeDegrees); dataFile.print(buffer); toCharArray(GPS.lon); dataFile.print(", ");
            toCharArray(GPS.speed); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.angle); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.altitude); dataFile.print(buffer); dataFile.print(", ");
            toCharArray((int)GPS.satellites); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.magvariation); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.HDOP); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.VDOP); dataFile.print(buffer); dataFile.print(", ");
            toCharArray(GPS.PDOP); dataFile.print(buffer); dataFile.print(", ");
          }
        }

        // BNO055
        // Get Sensor Events
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);            // - VECTOR_EULER         - degrees
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);        // - VECTOR_GYROSCOPE     - rad/s
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);      // - VECTOR_LINEARACCEL   - m/s^2
        bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);    // - VECTOR_MAGNETOMETER  - uT
        bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);  // - VECTOR_ACCELEROMETER - m/s^2
        bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);              // - VECTOR_GRAVITY       - m/s^2
        toCharArray(bno.getTemp()); dataFile.print(buffer); dataFile.print(", ");
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

        // ISM 330DLC
        ISM330DLCSensor AccGyr(&Wire);
        AccGyr.Get_X_Axes(accelerometer);
        AccGyr.Get_G_Axes(gyroscope);
        toCharArray(accelerometer[0]); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(accelerometer[1]); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(accelerometer[2]); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gyroscope[0]); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gyroscope[1]); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(gyroscope[2]); dataFile.print(buffer); dataFile.print(", ");

        // SCD41
        scd41data = scd41.readMeasurement(co2Concentration, temperature, relativeHumidity);
        toCharArray(co2Concentration); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(temperature); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(relativeHumidity); dataFile.print(buffer); dataFile.print(", ");

        // AS7341
        toCharArray(as7341.getChannel(AS7341_CHANNEL_415nm_F1)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_445nm_F2)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_480nm_F3)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_515nm_F4)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_555nm_F5)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_590nm_F6)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_630nm_F7)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_680nm_F8)); dataFile.print(buffer); dataFile.print(", ");
        
        toCharArray(as7341.getChannel(AS7341_CHANNEL_CLEAR)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(as7341.getChannel(AS7341_CHANNEL_NIR)); dataFile.print(buffer); dataFile.print(", ");

        // BMP390
        toCharArray(baro.readAltitude(seaLevelPressure)); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(baro.readPressure()); dataFile.print(buffer); dataFile.print(", ");
        toCharArray(baro.readTemperature()); dataFile.print(buffer);

        dataFile.flush();
    }
    else{
        Serial.println("Error writing to file");
    }

    // Print data live to telemetry //
    Serial.print("Hour, Minute, Second, Millisecond, Day, Month, Year, Fix, Quality, Latitude Degrees. Longitute Degrees, Speed, Angle, Altitude, Sats, Magnetic Variation, "); //GPS Header
    Serial.print("Linear Accel X, Linear Accel Y, Linear Accel Z, "); // BNO055 Header
    Serial.print("Accelerometer Accel X, Accelerometer Accel Y, Accelerometer Accel Z, "); // ISM 330DLC Header
    Serial.print("CO2 Concentration, Temperature, "); // SCD41 Header
    Serial.println("Altitude, Pressure, Temperature"); // BMP390 Header

    if(millis() - timer > 2000){
      timer = millis(); // reset the timer
      Serial.print("20"); Serial.print(GPS.year); Serial.print("-");
      Serial.print(GPS.month); Serial.print("-");
      Serial.print(GPS.day); Serial.print(" ");
      if(GPS.hour < 10){
        Serial.print('0');
      }
      Serial.print(GPS.hour); Serial.print(':');
      if(GPS.minute < 10){
        Serial.print('0');
      }
      Serial.print(GPS.minute); Serial.print(':');
      if(GPS.seconds < 10){
        Serial.print('0');
      }
      Serial.print(GPS.seconds); Serial.print('.');
      if(GPS.milliseconds < 10){
        Serial.print("00");
      }
      else if(GPS.milliseconds > 9 && GPS.milliseconds < 100){
        Serial.print("0");
      }
      Serial.print(GPS.milliseconds); dataFile.print(", ");
      Serial.print((int)GPS.fix); dataFile.print(", ");
      Serial.print((int)GPS.fixquality); dataFile.print(", ");
      if(GPS.fix){
        Serial.print(GPS.latitudeDegrees); Serial.print(GPS.lat); Serial.print(", ");
        Serial.print(GPS.longitudeDegrees); Serial.print(GPS.lon); Serial.print(", ");
        Serial.print(GPS.speed); Serial.print(", ");
        Serial.print(GPS.angle); Serial.print(", ");
        Serial.print(GPS.altitude); Serial.print(", ");
        Serial.print((int)GPS.satellites); Serial.print(", ");
        Serial.print(GPS.magvariation); Serial.print(", ");
      }
    }

    // BNO055
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.print(linearAccelData.acceleration.x); Serial.print(", ");
    Serial.print(linearAccelData.acceleration.y); Serial.print(", ");
    Serial.print(linearAccelData.acceleration.z); Serial.print(", ");

    // ISM 330DLC
    ISM330DLCSensor AccGyr(&Wire);
    AccGyr.Get_X_Axes(accelerometer);
    Serial.print(accelerometer[0]); dataFile.print(", ");
    Serial.print(accelerometer[1]); dataFile.print(", ");
    Serial.print(accelerometer[2]); dataFile.print(", ");

    // SCD41
    scd41data = scd41.readMeasurement(co2Concentration, temperature, relativeHumidity);
    Serial.print(co2Concentration); Serial.print(", ");
    Serial.print(temperature); Serial.print(", ");

    // BMP390
    Serial.print(baro.readAltitude(seaLevelPressure)); Serial.print(", ");
    Serial.print(baro.readPressure()); Serial.print(", ");
    Serial.print(baro.readTemperature());


    delay(1000);
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
void toCharArray(int32_t in) {
  sprintf(buffer, "%i", in);
}