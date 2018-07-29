#include <Servo.h>

#define SERVO_WRITE writeMicroseconds
#define LED_PIN 13
#define SERVO_PIN 9

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created

unsigned long last_blink = 0;

void setup() {

    pinMode(LED_PIN, OUTPUT); 
    myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
    // On standard servos a parameter value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle. 
    myservo.SERVO_WRITE(1500);
}


void loop() {
    if(millis() - last_blink >= 1000){
        digitalWrite(13, !digitalRead(13)); // Toggle LED.
        last_blink = millis();
    }
}
