#include <Servo.h>

#define LED_PIN 13
#define SERVO_PIN 9
#define START_PIN 7

#define SERVO_DELAY 15
#define SERVO_STEP 10
#define MIN_POSITION 1000
#define MAX_POSITION 2000
//#define MIN_POSITION 1000-500
#define CENTER_POSITION 1500
//#define MAX_POSITION 2000+500

// Amount subtracted from the min/max end points.
//#define SUB_POSITION 300 // bad, clicks
//#define SUB_POSITION 500 // good, but limited range
//#define SUB_POSITION 400

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created

int pos = 0;    // variable to store the servo position

unsigned long last_blink = 0;

void setup() {

    pinMode(START_PIN, INPUT_PULLUP); // default HIGH, when button on pulled LOW
    pinMode(LED_PIN, OUTPUT); 
    myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
    // On standard servos a parameter value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle. 
}


void loop() {
    
    if(millis() - last_blink >= 1000){
        digitalWrite(13, !digitalRead(13)); // Toggle LED.
        last_blink = millis();
    }
    
    if(digitalRead(START_PIN)){
        for(pos = MIN_POSITION; pos < MAX_POSITION; pos += SERVO_STEP)  // goes from 0 degrees to 180 degrees
        {                                  // in steps of 1 degree
            myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
            delay(SERVO_DELAY);                       // waits 15ms for the servo to reach the position
        }
        for(pos = MAX_POSITION; pos >= MIN_POSITION; pos -= SERVO_STEP)     // goes from 180 degrees to 0 degrees
        {
            myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
            delay(SERVO_DELAY);                       // waits 15ms for the servo to reach the position
        }        
    }else{
        myservo.writeMicroseconds(CENTER_POSITION);
    }

}
