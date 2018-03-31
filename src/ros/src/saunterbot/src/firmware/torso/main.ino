/*
 * Copyright 2017 Chris Spencer
 */
 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>

#include <Servo.h>

#include "ArduinoPinout.h"
#include "AccelGyroSensor.h"
#include "PID.h"

#define MAX_OUT_CHARS 255

// Absolute min of Alturn servos, not taking into account physical leg limitations.
#define SERVO_LOWER_ABS 1000

// Absolute center of Alturn servos.
#define SERVO_CENTER_ABS 1500

// Absolute max of Alturn servos, not taking into account physical leg limitations.
#define SERVO_UPPER_ABS 2000

// Absolute min of Alturn servos, taking into account physical limits of legs.
// TODO:add more leg clearance? this limitation is mostly due to the right-knee running into the torso
#define SERVO_LOWER_PHY_RIGHT 1275 // THIS IS A HARD LIMIT FOR THE RIGHT. Passing this will block the servo and possibly damage it!!!
#define SERVO_LOWER_PHY_LEFT 1275 // practical limit, going beyond this won't damage the leg, but the leg won't respond

// The Alturn servo position where the body is relatively balanced.
#define SERVO_CENTER_PHY_RIGHT 1400 //TODO?

// Absolute max of Alturn servos, taking into account physical limits of legs.
#define SERVO_UPPER_PHY_RIGHT 1525
#define SERVO_UPPER_PHY_LEFT 1725 // THIS IS A HARD LIMIT FOR THE LEFT. Passing this will block the servo and possibly damage it!!!

// Amount of offset in degrees so the right servo matches the left.
#define SERVO_HIP_LEFT_OFFSET 20

// The pitch angle above which the servos shutoff for safety.
#define SERVO_SHUTOFF_BACKWARDS_ANGLE 15
#define SERVO_SHUTOFF_FORWARDS_ANGLE -15

// The upper (leg's back) most practical range for balancing in the context of the right leg.
#define SERVO_BALANCING_UPPER 1460

// The position where the leg's are roughly underneath the center-of-mass in the context of the right leg.
#define SERVO_BALANCING_CENTER 1375

// The lower (leg's forward) most practical range for balancing in the context of the right leg.
#define SERVO_BALANCING_LOWER 1290

// The ratio used to convert degrees to a relative servo position.
#define SERVO_DEGREE_TO_POSITION 170.0/43.0

// On left:
//1500->1750 upper max => CW
//1500->1100 lower min => CCW
// On right:
//1500->1750 upper max => CW
//1500->1275 lower min => CCW
// Both have a physical range of 225 on either side of 1500.

char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

ros::NodeHandle nh;

AccelGyroSensor ag_sensor = AccelGyroSensor();

//PIDController hip_pid_controller = PIDController(SERVO_LOWER_PHY_RIGHT, SERVO_UPPER_PHY_LEFT);
PIDController hip_pid_controller = PIDController(SERVO_BALANCING_LOWER, SERVO_BALANCING_UPPER, SERVO_DEGREE_TO_POSITION);

unsigned long count = 0;
unsigned long last_count = 0;

unsigned long last_ag_read_time = 0;
unsigned long mpu_read_count = 0;
unsigned long last_blink_time = 0;

char x_str[10];
char y_str[10];
char z_str[10];
char x2_str[10];
char y2_str[10];
char z2_str[10];

Servo servo_hip_right;
Servo servo_hip_left;
int servo_hip_right_pos = SERVO_CENTER_PHY_RIGHT;
int servo_hip_left_pos = SERVO_CENTER_PHY_RIGHT;
int center = 90;
int range = 30;
int offset = 0;
int upper_endstop = center + range + offset;
int lower_endstop = center - range + offset;
int servo_update_speed_ms = 30; // ms between step
int servo_direction = +1;

int current_hip_angle = 0;
int last_hip_angle = 0;

unsigned long last_servo_update_time = 0;
unsigned long servo_last_set_time = 0;

unsigned long balance_last_output_time = 0;

bool last_pushbutton_state = true;

bool angle_safety_shutoff = false;

bool balancing_enabled = false;

bool disabling_balancing = false;

// If true, servos are attached being sent a signal.
// Otherwise, servos are detached and not being sent a signal.
bool servo_hips_active = false;

float pitch_degrees = 0;

long ftol(double v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
    return static_cast<long>(v*1000);
}

void set_hip_position(int pos, bool verbose=false){
    // Since the left and right legs are mirror images of each other, the effective position range is a combination of the hard limits of each.
    // The lower bound is the max hard limit of lower bounds.
    // The upper bound is the min hard limit of the upper bounds.
    if(pos >= SERVO_LOWER_PHY_RIGHT && pos <= SERVO_UPPER_PHY_LEFT){

        servo_hip_right_pos = constrain(pos, SERVO_LOWER_PHY_RIGHT, SERVO_UPPER_PHY_RIGHT);
        servo_hip_left_pos = constrain(2 * SERVO_CENTER_ABS - pos + SERVO_HIP_LEFT_OFFSET, SERVO_LOWER_PHY_LEFT, SERVO_UPPER_PHY_LEFT);
        
        servo_hip_right.writeMicroseconds(servo_hip_right_pos);
        servo_hip_left.writeMicroseconds(servo_hip_left_pos);

        servo_hip_right.attach(SERVO_HIP_RIGHT);
        servo_hip_left.attach(SERVO_HIP_LEFT);

        servo_hips_active = true;
        servo_last_set_time = millis();
        if(verbose) nh.loginfo("Servo position set.");
    }else{
        if(verbose) nh.loginfo("Invalid servo position. Must be between SERVO_LOWER_PHY_RIGHT and SERVO_UPPER_PHY_LEFT.");
    }
}

void reset_hip_position(){
    //set_hip_position(SERVO_CENTER_PHY_RIGHT); // reset hip angle
    //set_hip_position(SERVO_CENTER_ABS); // reset hip angle
    set_hip_position(SERVO_BALANCING_CENTER); // reset hip angle
    
}

// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1275
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1725

// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1290
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1375
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1460
void on_servo_hip_set(const std_msgs::Int16& msg) {
    set_hip_position(msg.data, true); // verbosely
}
ros::Subscriber<std_msgs::Int16> on_servo_hip_set_sub("servo/hip/set", &on_servo_hip_set);

// rostopic pub --once /torso_arduino/servo/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0, 0, 0]}"
// rostopic pub --once /torso_arduino/servo/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0.95, 0.15, 0.1]}"
// rostopic pub --once /torso_arduino/servo/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[1, 0, 0]}"
void on_hip_pid_set(const std_msgs::Float32MultiArray& msg) {
    hip_pid_controller.Kp = msg.data[0];
    hip_pid_controller.Ki = msg.data[1];
    hip_pid_controller.Kd = msg.data[2];
    hip_pid_controller.reset();
    nh.loginfo("Hip PID parameters changed.");
    
    dtostrf(msg.data[0], 5, 3, x_str);
    dtostrf(msg.data[1], 5, 3, y_str);
    dtostrf(msg.data[2], 5, 3, z_str);
    snprintf(buffer, MAX_OUT_CHARS, "k = %s i = %s d = %s", x_str, y_str, z_str);
    nh.loginfo(buffer);
            
}
ros::Subscriber<std_msgs::Float32MultiArray> on_hip_pid_set_sub("servo/hip/pid/set", &on_hip_pid_set);

void setup() {
    
    // Needed for the MPU6050.
    Wire.begin();

    // Enable the stardard debugging LED.
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, true);

    // Enable our push button input.
    pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP); 

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(on_servo_hip_set_sub);
    nh.subscribe(on_hip_pid_set_sub);

    ag_sensor.initialize();
    if(!ag_sensor.is_ready()){
        nh.loginfo("MPU6050 initialization failed.");
    }
    
    reset_hip_position();
    hip_pid_controller.reset();

}

void loop() {

    // Toggle status LED to indicate we're doing something.
    if(millis() - last_blink_time >= 500){
        last_blink_time = millis();
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
    
    // Update push button state. High=unpressed, Low=pressed.
    if(digitalRead(PUSH_BUTTON_PIN) != last_pushbutton_state){
        last_pushbutton_state = digitalRead(PUSH_BUTTON_PIN);
        if(last_pushbutton_state){
            nh.loginfo("Pushbutton released.");
        }else{
            nh.loginfo("Pushbutton pressed.");
            
            // On depress, toggle balancing flag.
            if(balancing_enabled){
                nh.loginfo("Balancing disabled.");
                reset_hip_position();
                hip_pid_controller.reset();
            }else{
                nh.loginfo("Balancing enabled.");
                hip_pid_controller.reset();
            }
            balancing_enabled = !balancing_enabled;
        }
    }
    
    // Update MPU6050 data.
    ag_sensor.update();
    
    // If we're off by more than this, than it means we've fallen over, so stop updating the servos as a safety mechanism.
    pitch_degrees = ag_sensor.ypr[1] * 180/M_PI;
    angle_safety_shutoff = pitch_degrees > SERVO_SHUTOFF_BACKWARDS_ANGLE || pitch_degrees < SERVO_SHUTOFF_FORWARDS_ANGLE;
    
    // Update hip position to balance pitch angle.
    // Our goal is to keep the pitch degrees as close to 0 as possible by adjusting the hip position.
    // If we're tilting backwards (pitch of positive degrees), the PID should return a lower value (e.g. 1290) to lean forward as a counterbalance.
    // If we're tilting forwards (pitch of negative degrees), the PID should return a higher value (e.g. 1450) to lean backwards as a counterbalance.
    if(balancing_enabled && hip_pid_controller.is_ready()){
        // Balance as long as we have a pitch and roll of less than 45 degrees.
        if(!angle_safety_shutoff){
            // Get hip angle and set if it's changed since last iteration.
            current_hip_angle = hip_pid_controller.get_value(pitch_degrees);
            if(current_hip_angle != last_hip_angle){
                set_hip_position(current_hip_angle);
                
                //dtostrf(hip_pid_controller.Kp, 5, 3, x_str);
                //dtostrf(hip_pid_controller.Ki, 5, 3, y_str);
                //dtostrf(hip_pid_controller.Kd, 5, 3, z_str);
                //snprintf(buffer, MAX_OUT_CHARS, "Kp=%s Ki=%s Kd=%s", x_str, y_str, z_str);
                //nh.loginfo(buffer);             

                //dtostrf(hip_pid_controller.pTerm, 5, 3, x_str);
                //dtostrf(hip_pid_controller.iTerm, 5, 3, y_str);
                //dtostrf(hip_pid_controller.dTerm, 5, 3, z_str);
                //snprintf(buffer, MAX_OUT_CHARS, "pTerm=%s iTerm=%s dTerm=%s", x_str, y_str, z_str);
                //nh.loginfo(buffer);
                
                //snprintf(buffer, MAX_OUT_CHARS, "current_hip_angle=%i", current_hip_angle);
                //nh.loginfo(buffer);
            }
        }
    }
        
    // Turn off servos if unused for more than a few seconds.
    if(servo_hips_active && millis() - servo_last_set_time > 2000){
        servo_hips_active = false;
        servo_hip_right.detach();
        servo_hip_left.detach();
        nh.loginfo("Hip servos disabled due to inactivity.");
    }
    
    // DEBUG output.
    if(millis() - last_ag_read_time >= 1000){
        last_ag_read_time = millis();
    
        dtostrf(ag_sensor.ypr[0] * 180/M_PI, 5, 3, x_str);
        dtostrf(ag_sensor.ypr[1] * 180/M_PI, 5, 3, y_str);
        dtostrf(ag_sensor.ypr[2] * 180/M_PI, 5, 3, z_str);

        dtostrf(hip_pid_controller.pTerm, 5, 3, x2_str);
        dtostrf(hip_pid_controller.iTerm, 5, 3, y2_str);
        dtostrf(hip_pid_controller.dTerm, 5, 3, z2_str);     

        snprintf(buffer, MAX_OUT_CHARS,
            "balancing_enabled=%i, angle_safety_shutoff=%i, current_hip_angle=%i, pTerm=%s iTerm=%s dTerm=%s, yaw=%s pitch=%s roll=%s",
            balancing_enabled, angle_safety_shutoff, current_hip_angle, x2_str, y2_str, z2_str, x_str, y_str, z_str);
        nh.loginfo(buffer);
         
    }
        
    nh.spinOnce();

    last_hip_angle = current_hip_angle;
    disabling_balancing = false;
}
