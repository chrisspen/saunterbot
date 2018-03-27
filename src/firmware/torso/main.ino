/*
 * Copyright 2017 Chris Spencer
 */

#define STATUS_LED_PIN 13

void setup() {

    //Serial.begin(57600);
    //Serial.begin(115200);
    
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, true);
    
}

void loop() {

    //Serial.println("Hello, world?");
    delay(500);
    
    // Toggle status LED to indicate we're doing something.
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));

}
