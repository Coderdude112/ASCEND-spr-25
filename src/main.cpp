// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>

// ---- //
/* Vars */
// ---- //

SensirionI2cScd4x scd41;

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
    uint16_t scd41data;
    uint16_t co2Concentration = 0;
    float temperature = 0.0;
    float relativeHumidity = 0.0;

}

void loop() {
    
}
