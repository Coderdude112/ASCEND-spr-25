// ------- //
/* Imports */
// ------- //

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cScd4x.h>

// ---- //
/* Vars */
// ---- //

// ---- //
/* Main */
// ---- //

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(0);
    Wire.begin();

}

void loop() {

}
