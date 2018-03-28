/*
 * Copyright 2017 Chris Spencer
 */
 
#include <ros.h>

#include "ArduinoPinout.h"
#include "AccelGyroSensor.h"

#define MAX_OUT_CHARS 255

char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

ros::NodeHandle nh;

AccelGyroSensor ag_sensor = AccelGyroSensor();

unsigned long count = 0;
unsigned long last_count = 0;

void setup() {
    
    // Needed for the MPU6050.
    Wire.begin();

    //Serial.begin(57600);
    //Serial.begin(115200);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, true);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    //ag_sensor.initialize();
    //if(!ag_sensor.is_ready()){
        //nh.loginfo("MPU6050 initialization failed.");
    //}

}

void loop() {

    //Serial.println("Hello, world?");
    delay(500);
    
    // Toggle status LED to indicate we're doing something.
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    
    // Output debugging count.
    if(millis() - last_count > 5000){
        count += 1;
        last_count = millis();
        snprintf(buffer, MAX_OUT_CHARS, "Count:%d", count);
        nh.loginfo(buffer);
    }
    
    nh.spinOnce();

}
