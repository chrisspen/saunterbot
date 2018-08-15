/*
 * Copyright 2017 Chris Spencer
 */
 
#include <ros.h>
// http://wiki.ros.org/std_msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <Servo.h>

#include "ArduinoPinout.h"
#include "AccelGyroSensor.h"
#include "PIDController.h"
#include "ServoController.h"
#include "BalanceController.h"
#include "EEPROMAnything.h"
#include "VelocityTracker.h"

//#define BAUDRATE 115200 // causes occassional "Lost sync with device, restarting" errors?
//#define BAUDRATE 57600
#define BAUDRATE 76800

#define MAX_OUT_CHARS 255

// Absolute min of Alturn servos, not taking into account physical leg limitations.
#define SERVO_LOWER_ABS 1000

// Absolute center of Alturn servos.
#define SERVO_CENTER_ABS 1500

// Absolute max of Alturn servos, not taking into account physical leg limitations.
#define SERVO_UPPER_ABS 2000

// Absolute min of Alturn servos, taking into account physical limits of legs.
// TODO:add more leg clearance? this limitation is mostly due to the right-knee running into the torso
#define SERVO_LOWER_PHY_RIGHT 1000//1275 // THIS IS A HARD LIMIT FOR THE RIGHT. Passing this will block the servo and possibly damage it!!!
#define SERVO_LOWER_PHY_LEFT 1000//1275 // practical limit, going beyond this won't damage the leg, but the leg won't respond

// Absolute max of Alturn servos, taking into account physical limits of legs.
#define SERVO_UPPER_PHY_RIGHT 2000//1525
#define SERVO_UPPER_PHY_LEFT 2000//1725 // THIS IS A HARD LIMIT FOR THE LEFT. Passing this will block the servo and possibly damage it!!!

// Amount of offset in degrees so the right servo matches the left.
#define SERVO_HIP_LEFT_OFFSET 0 //20

// The pitch angle above which the servos shutoff for safety.
#define SERVO_SHUTOFF_BACKWARDS_ANGLE 15

// The upper (leg's back) most practical range for balancing in the context of the right leg.
//#define SERVO_BALANCING_UPPER 1460

// The position where the leg's are roughly underneath the center-of-mass in the context of the right leg.
//#define SERVO_BALANCING_CENTER 1375

// The lower (leg's forward) most practical range for balancing in the context of the right leg.
//#define SERVO_BALANCING_LOWER 1290

// The ratio used to convert degrees to a relative servo position.
//#define SERVO_DEGREE_TO_POSITION 170.0/43.0

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

VelocityTracker foot_tracker = VelocityTracker();

PIDController hip_pid_controller = PIDController(SERVO_LOWER_PHY_RIGHT, SERVO_UPPER_PHY_LEFT);

unsigned long last_ag_read_time = 0;
unsigned long mpu_read_count = 0;
unsigned long last_blink_time = 0;
unsigned long last_heartbeat_time = 0;
unsigned long heartbeat_count = 0;
unsigned long foot_log_time = 0;
unsigned long last_aaread_time = 0;

int diff_acc_z = 0;
int last_acc_z = 0;

bool angle_safety_shutoff_enabled = false;

char x_str[20];
char y_str[20];
char z_str[20];

char yaw_str[20];
char pitch_str[20];
char roll_str[20];
char d_str[20];
char e_str[20];
//char x2_str[10];
//char y2_str[10];
//char z2_str[10];

std_msgs::Bool bool_msg;
std_msgs::Int32 int32_msg;
std_msgs::Float32MultiArray float32ma_msg;
//float state_data[] = {0, 0, 0, 0};

// Persistent variables.
struct config_t
{
    int servo_hip_right_lower_pos_feedback;
    int servo_hip_right_upper_pos_feedback;
    int servo_hip_left_lower_pos_feedback;
    int servo_hip_left_upper_pos_feedback;
    
    // NN controller.
    float w_yeta;
    float w_yeta_dot;
    float w_theta;
    float w_theta_dot;
    float w_zeta;

    // PID controller.
    float Kp = 0;                   // (P)roportional Tuning Parameter
    float Ki = 0;                   // (I)ntegral Tuning Parameter        
    float Kd = 0;                  // (D)erivative Tuning Parameter       
    
} configuration;

// Function declarations.
void reset_hip_position();

//Servo servo_hip_right;
//Servo servo_hip_left;
//ServoController servo_weight_shifter = ServoController(SERVO_WEIGHT_SHIFTER_OUTPUT_PIN, 1000-100, 2000+100, 1500, SERVO_WEIGHT_SHIFTER_INPUT_PIN);
ServoController servo_hip_right_controller = ServoController(SERVO_HIP_RIGHT_PIN, 1000, 2000, 1500, FEEDBACK_HIP_RIGHT_PIN);
ServoController servo_hip_left_controller = ServoController(SERVO_HIP_LEFT_PIN, 1000, 2000, 1500, FEEDBACK_HIP_LEFT_PIN);

ServoController servo_knee_right_controller = ServoController(SERVO_KNEE_RIGHT_PIN, 1000+50, 2000-50, 1500, FEEDBACK_KNEE_RIGHT_PIN);
ServoController servo_knee_left_controller = ServoController(SERVO_KNEE_LEFT_PIN, 1000+50, 2000-50, 1500, FEEDBACK_KNEE_LEFT_PIN);

BalanceController balance_controller = BalanceController();
float balance_action;

int servo_hip_right_pos = SERVO_CENTER_ABS;
int servo_hip_left_pos = SERVO_CENTER_ABS;
int center = 90;
int range = 30;
int offset = 0;
int upper_endstop = center + range + offset;
int lower_endstop = center - range + offset;
int servo_update_speed_ms = 30; // ms between step
int servo_direction = +1;

bool last_angle_safety_shutoff;

int current_hip_angle = 0;
int last_hip_angle = 0;

unsigned long last_servo_update_time = 0;
unsigned long servo_last_set_time = 0;
unsigned long balance_last_output_time = 0;
unsigned long balancing_started_time = 0;
unsigned long balancing_stopped_time = 0;
unsigned long last_state_publish_time = 0;

bool last_pushbutton_state = true;

// If true, pressing the button starts auto balancing. If false, pressing the button only publishes button state, but otherwise doesn't do anything.
bool balance_button_enabled = true;

bool angle_safety_shutoff = false;

bool balancing_enabled = false;

bool disabling_balancing = false;

// If true, servos are attached being sent a signal.
// Otherwise, servos are detached and not being sent a signal.
bool servo_hips_active = false;

float pitch_degrees = 0;

long ftol(float v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
    return static_cast<long>(v*1000);
}

int calculate_foot_degrees(int hip_degrees, int body_degrees) {
    return (body_degrees - hip_degrees);
}

void enable_hip_servos() {
    servo_hips_active = true;
    servo_last_set_time = millis();
    servo_hip_right_controller.power_on();
    servo_hip_left_controller.power_on();
}

void disable_hip_servos() {
    servo_hips_active = false;
    servo_hip_right_controller.power_off();
    servo_hip_left_controller.power_off();
}

void save_configuration(){
    
    configuration.servo_hip_left_lower_pos_feedback = servo_hip_left_controller.get_lower_feedback_position();
    configuration.servo_hip_left_upper_pos_feedback = servo_hip_left_controller.get_upper_feedback_position();
    configuration.servo_hip_right_lower_pos_feedback = servo_hip_right_controller.get_lower_feedback_position();
    configuration.servo_hip_right_upper_pos_feedback = servo_hip_right_controller.get_upper_feedback_position();

    configuration.w_yeta = balance_controller.w_yeta;
    configuration.w_yeta_dot = balance_controller.w_yeta_dot;
    configuration.w_theta = balance_controller.w_theta;
    configuration.w_theta_dot = balance_controller.w_theta_dot;
    configuration.w_zeta = balance_controller.w_zeta;

    configuration.Kp = hip_pid_controller.Kp;
    configuration.Ki = hip_pid_controller.Ki;
    configuration.Kd = hip_pid_controller.Kd;

    servo_hip_left_controller.saved = true;
    servo_hip_right_controller.saved = true;

    EEPROM_writeAnything(0, configuration);
}

// Whether or not the platform is actively trying to balance.
// rostopic echo /torso_arduino/balancing/get
ros::Publisher balancing_get_publisher = ros::Publisher("balancing/get", &bool_msg);

// Whether or not the platform is considered upright.
// rostopic echo /torso_arduino/upright
ros::Publisher upright_publisher = ros::Publisher("upright/get", &bool_msg);

// rostopic pub --once /torso_arduino/upright/set std_msgs/Empty
void on_upright_set(const std_msgs::Empty& msg) {
    bool_msg.data = !angle_safety_shutoff;
    upright_publisher.publish(&bool_msg);
    nh.spinOnce();
}
ros::Subscriber<std_msgs::Empty> on_upright_set_sub("upright/set", &on_upright_set);

// rostopic echo /torso_arduino/button
ros::Publisher button_publisher = ros::Publisher("button", &bool_msg);

// rostopic echo /torso_arduino/foot/degrees
ros::Publisher foot_degrees_publisher = ros::Publisher("foot/degrees", &int32_msg);

// rostopic echo /torso_arduino/weight/param/get
ros::Publisher params_publisher = ros::Publisher("weight/param/get", &float32ma_msg);

// rostopic echo /torso_arduino/hip/pid/get
ros::Publisher hip_pid_publisher = ros::Publisher("hip/pid/get", &float32ma_msg);

void enable_balancing(){
    if(!balancing_enabled){
        nh.loginfo("Balancing enabled.");
        balancing_started_time = millis();
        balancing_enabled = true;    
        bool_msg.data = balancing_enabled;
        balancing_get_publisher.publish(&bool_msg);
    }
}

void disable_balancing(bool reset=true){
    if(balancing_enabled){
        nh.loginfo("Balancing disabled.");
        if(reset){
            reset_hip_position();
        }
        balancing_stopped_time = millis();
        balancing_enabled = false;    
        bool_msg.data = balancing_enabled;
        balancing_get_publisher.publish(&bool_msg);
    }
}

void set_hip_position(int pos, bool verbose=false){
    // Since the left and right legs are mirror images of each other, the effective position range is a combination of the hard limits of each.
    // The lower bound is the max hard limit of lower bounds.
    // The upper bound is the min hard limit of the upper bounds.

    pos = constrain(pos, SERVO_LOWER_ABS, SERVO_UPPER_ABS); // 1000-2000

    servo_hip_right_pos = pos;
    servo_hip_left_pos = constrain(2 * SERVO_CENTER_ABS - pos + SERVO_HIP_LEFT_OFFSET, SERVO_LOWER_ABS, SERVO_UPPER_ABS);
    
    servo_hip_right_controller.set_position(servo_hip_right_pos);
    servo_hip_left_controller.set_position(servo_hip_left_pos);

    enable_hip_servos();
    if(verbose){
        nh.loginfo("Servo position set.");
    }

    //else{
        //if(verbose){
            //snprintf(buffer, MAX_OUT_CHARS, "Invalid servo position. Must be between %i and %i.", SERVO_LOWER_ABS, SERVO_UPPER_ABS);
            //nh.loginfo(buffer);
        //}
    //}
}

void increment_hip_position(int amount){
    set_hip_position(servo_hip_right_controller.get_target_position() + amount);
}

void reset_hip_position(){
    set_hip_position(SERVO_CENTER_ABS); // reset hip angle
    
}

void log_balance_params(){
    nh.loginfo("NN controller parameters:");
    dtostrf(balance_controller.w_yeta, 5, 3, yaw_str);
    dtostrf(balance_controller.w_yeta_dot, 5, 3, pitch_str);
    dtostrf(balance_controller.w_theta, 5, 3, roll_str);
    dtostrf(balance_controller.w_theta_dot, 5, 3, d_str);
    dtostrf(balance_controller.w_zeta, 5, 3, e_str);
    snprintf(buffer, MAX_OUT_CHARS, "a = %s b = %s c = %s d = %s e = %s", yaw_str, pitch_str, roll_str, d_str, e_str);
    nh.loginfo(buffer);
    
    nh.loginfo("PID controller parameters:");
    dtostrf(hip_pid_controller.Kp, 5, 3, x_str);
    dtostrf(hip_pid_controller.Ki, 5, 3, y_str);
    dtostrf(hip_pid_controller.Kd, 5, 3, z_str);
    snprintf(buffer, MAX_OUT_CHARS, "p = %s i = %s d = %s", x_str, y_str, z_str);
    nh.loginfo(buffer);
}

// max forward
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1275
// max backward
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1500
// extreme left backward, don't use this
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1725

// safe max foward
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1290
// safe center
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1375
// safe max backward
// rostopic pub --once /torso_arduino/servo/hip/set std_msgs/Int16 1460
void on_servo_hip_set(const std_msgs::Int16& msg) {

    // If we set position remotely, that means we're not balancing, so disable auto-balancing.
    disable_balancing(false);

    set_hip_position(msg.data, true); // verbosely
}
ros::Subscriber<std_msgs::Int16> on_servo_hip_set_sub("servo/hip/set", &on_servo_hip_set);

// rostopic pub --once /torso_arduino/servo/knee/left/set std_msgs/Int16 1460
void on_servo_knee_left_set(const std_msgs::Int16& msg) {
    nh.loginfo("Setting left knee position.");
    servo_knee_left_controller.set_position(msg.data);
    servo_knee_left_controller.power_on();
}
ros::Subscriber<std_msgs::Int16> on_servo_knee_left_set_sub("servo/knee/left/set", &on_servo_knee_left_set);

// rostopic pub --once /torso_arduino/servo/knee/right/set std_msgs/Int16 1460
void on_servo_knee_right_set(const std_msgs::Int16& msg) {
    nh.loginfo("Setting right knee position.");
    servo_knee_right_controller.set_position(msg.data);
    servo_knee_right_controller.power_on();
}
ros::Subscriber<std_msgs::Int16> on_servo_knee_right_set_sub("servo/knee/right/set", &on_servo_knee_right_set);

// rostopic pub --once /torso_arduino/hip/right/calibrate std_msgs/Empty
void on_hip_right_calibrate(const std_msgs::Empty& msg) {
    enable_hip_servos();
    servo_hip_right_controller.calibrate_position_feedback();
    nh.loginfo("Beginning right hip calibration.");
}
ros::Subscriber<std_msgs::Empty> on_hip_right_calibrate_sub("hip/right/calibrate", &on_hip_right_calibrate);

// rostopic pub --once /torso_arduino/debug std_msgs/Empty
void on_debug(const std_msgs::Empty& msg) {
    log_balance_params();
}
ros::Subscriber<std_msgs::Empty> on_debug_sub("debug", &on_debug);

// rostopic pub --once /torso_arduino/hip/left/calibrate std_msgs/Empty
void on_hip_left_calibrate(const std_msgs::Empty& msg) {
    enable_hip_servos();
    servo_hip_left_controller.calibrate_position_feedback();
    nh.loginfo("Beginning left hip calibration.");
}
ros::Subscriber<std_msgs::Empty> on_hip_left_calibrate_sub("hip/left/calibrate", &on_hip_left_calibrate);

// rostopic pub --once /torso_arduino/imu/reset std_msgs/Empty
void on_imu_reset(const std_msgs::Empty& msg) {
    nh.loginfo("Beginning IMU reset...");
    ag_sensor.reset();
    nh.loginfo("IMU reset!");
}
ros::Subscriber<std_msgs::Empty> on_imu_reset_sub("imu/reset", &on_imu_reset);

// rostopic pub --once /torso_arduino/balancing/set std_msgs/Bool 1
void on_balancing_set(const std_msgs::Bool& msg) {
    nh.loginfo("Setting balance flag...");
    if(msg.data){
        enable_balancing();
    }else{
        disable_balancing();
    }
}
ros::Subscriber<std_msgs::Bool> on_balancing_set_sub("balancing/set", &on_balancing_set);

// rostopic pub --once /torso_arduino/balancing/safety/set std_msgs/Bool 1
void on_balancing_safety_set(const std_msgs::Bool& msg) {
    if(msg.data){
        nh.loginfo("Balance safety shutoff flag enabled.");
    }else{
        nh.loginfo("Balance safety shutoff flag disabled.");
    }
    angle_safety_shutoff_enabled = msg.data;
}
ros::Subscriber<std_msgs::Bool> on_balancing_safety_set_sub("balancing/safety/set", &on_balancing_safety_set);

// rostopic echo /torso_arduino/balancing/button/get
ros::Publisher balancing_button_publisher = ros::Publisher("balancing/button/get", &bool_msg);

// rostopic pub --once /torso_arduino/balancing/button/set std_msgs/Bool 1
void on_balancing_button_set(const std_msgs::Bool& msg) {
    balance_button_enabled = msg.data;
    if(msg.data){
        nh.loginfo("Balance button enabled.");
    }else{
        nh.loginfo("Balance button disabled.");
    }
    bool_msg.data = balance_button_enabled;
    balancing_button_publisher.publish(&bool_msg);
}
ros::Subscriber<std_msgs::Bool> on_balancing_button_set_sub("balancing/button/set", &on_balancing_button_set);

// move mass backwards
// rostopic pub --once /torso_arduino/weight/pos/set std_msgs/Int16 900
// center mass
// rostopic pub --once /torso_arduino/weight/pos/set std_msgs/Int16 1500
// move mas forward
// rostopic pub --once /torso_arduino/weight/pos/set std_msgs/Int16 2100
//void on_weight_position_set(const std_msgs::Int16& msg) {
    //~ servo_weight_shifter.set_position(msg.data);
    //~ servo_weight_shifter.power_on();
    //~ snprintf(buffer, MAX_OUT_CHARS, "Weight position set: %i", servo_weight_shifter.get_target_position());
    //~ nh.loginfo(buffer);
//~ }
//~ ros::Subscriber<std_msgs::Int16> on_weight_position_set_sub("weight/pos/set", &on_weight_position_set);

//rostopic pub --once /torso_arduino/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0, 0, 0]}"
//rostopic pub --once /torso_arduino/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0.95, 0.15, 0.1]}"
//rostopic pub --once /torso_arduino/hip/pid/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[1, 0, 0]}"
void on_hip_pid_set(const std_msgs::Float32MultiArray& msg) {
    hip_pid_controller.Kp = msg.data[0];
    hip_pid_controller.Ki = msg.data[1];
    hip_pid_controller.Kd = msg.data[2];
    hip_pid_controller.reset();
    nh.loginfo("Hip PID parameters changed.");
    
    dtostrf(hip_pid_controller.Kp, 5, 3, x_str);
    dtostrf(hip_pid_controller.Ki, 5, 3, y_str);
    dtostrf(hip_pid_controller.Kd, 5, 3, z_str);
    snprintf(buffer, MAX_OUT_CHARS, "p = %s i = %s d = %s", x_str, y_str, z_str);
    nh.loginfo(buffer);

    // Echo back params as confirmation.
    float32ma_msg.data[0] = hip_pid_controller.Kp;
    float32ma_msg.data[1] = hip_pid_controller.Ki;
    float32ma_msg.data[2] = hip_pid_controller.Kd;
    float32ma_msg.data_length = 3;
    hip_pid_publisher.publish(&float32ma_msg);
    nh.spinOnce();

    save_configuration();
            
}
ros::Subscriber<std_msgs::Float32MultiArray> on_hip_pid_set_sub("hip/pid/set", &on_hip_pid_set);


// rostopic pub --once /torso_arduino/weight/param/set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0, 0, 1, 0, 1]}"
void on_balance_controller_set(const std_msgs::Float32MultiArray& msg) {
    nh.loginfo("Setting weights...");
    balance_controller.set_weights(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    save_configuration();

    // Echo back params as confirmation.
    float32ma_msg.data[0] = balance_controller.w_yeta;
    float32ma_msg.data[1] = balance_controller.w_yeta_dot;
    float32ma_msg.data[2] = balance_controller.w_theta;
    float32ma_msg.data[3] = balance_controller.w_theta_dot;
    float32ma_msg.data[4] = balance_controller.w_zeta;
    float32ma_msg.data_length = 5;
    params_publisher.publish(&float32ma_msg);
    nh.spinOnce();

    log_balance_params();
}
ros::Subscriber<std_msgs::Float32MultiArray> on_balance_controller_set_sub("weight/param/set", &on_balance_controller_set);

// rostopic echo /torso_arduino/state
ros::Publisher state_publisher = ros::Publisher("state", &float32ma_msg);

//float get_yeta(){
    //return servo_hip_right_controller.get_actual_position_degrees() * M_PI/180; // yeta, hip angle, radians
//}

//float get_yeta_dot(){
    //return servo_hip_right_controller.get_velocity() * M_PI/180; // yeta_dot, hip velocity, radians/second
//}

//float get_theta(){
    //return foot_tracker.get_position() * M_PI/180; // theta, foot angle, radians
//}

//float get_theta_dot(){
    //return foot_tracker.get_velocity() * M_PI/180; // theta_dot, foot velocity, radians/second
//}

// These methods return a value bounded between [-1:+1].

float get_yeta_bounded(){
    // Hip angle.
    // Note, hip has a range of 90 degrees.
    float v = servo_hip_right_controller.get_actual_position_degrees(); // bounded between [-45:45]
    v = min(max(v, -45), +45); // bound between [-45:+45]
    v = ((v + 45)/90.)*2. - 1; // remap to [-1:+1]
    return v;
}

float get_yeta_dot_bounded(){
    // Hip angle velocity is in degrees/second.
    // Max velocity at 12V with no load is 0.14sec / 60 deg ungeared => 0.63sec / 90 deg (geared) = 143 degrees/sec
    float v = servo_hip_right_controller.get_velocity(); // bounded [-143:143]
    v = min(max(v, -143), +143); // bound between [-143:+143]
    v = ((v + 143)/(143*2.))*2. - 1; // remap to [-1:+1]
    return v;
}

float get_theta_bounded(){
    // Foot angle.
    float v = foot_tracker.get_position(); // degrees
    v = min(max(v, -90), +90); // bound between [-90:+90]
    v = ((v + 90)/(90*2.))*2. - 1; // remap to [-1:+1]
    return v;
}

float get_theta_dot_bounded(){
    // Foot angle velocity.
    // Max velocity should be 0->90 degrees in as fast as gravity can pull it down, which should be in about 0.5 sec => 90deg/0.5sec = 180deg/sec
    //float v = foot_tracker.get_velocity(); // degree/sec
    //v = min(max(v, -180), +180); // bound between [-180:+180]
    //v = ((v + 180)/(180*2.))*2. - 1; // remap to [-1:+1]
    float v = diff_acc_z;
    v = min(max(v, -10000), +10000); // bound between [-10000:+10000]
    v = ((v + 10000)/(10000*2.))*2. - 1; // remap to [-1:+1]
    return v;
}

void setup() {
    
    float32ma_msg.data = reinterpret_cast<float *>(malloc(sizeof(float)*5));
    float32ma_msg.data_length = 5;
    
    EEPROM_readAnything(0, configuration);
    
    // Needed for the MPU6050.
    Wire.begin();

    // Enable the stardard debugging LED.
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, true);

    // Enable our push button input.
    pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

    nh.getHardware()->setBaud(BAUDRATE);
    nh.initNode();

    nh.subscribe(on_servo_hip_set_sub);
    nh.subscribe(on_balance_controller_set_sub);
    nh.subscribe(on_hip_right_calibrate_sub);
    nh.subscribe(on_hip_left_calibrate_sub);
    nh.subscribe(on_imu_reset_sub);
    nh.subscribe(on_debug_sub);
    nh.subscribe(on_balancing_set_sub);
    nh.subscribe(on_balancing_safety_set_sub);
    nh.subscribe(on_hip_pid_set_sub);
    nh.subscribe(on_balancing_button_set_sub);
    nh.subscribe(on_upright_set_sub);
    nh.subscribe(on_servo_knee_left_set_sub);
    nh.subscribe(on_servo_knee_right_set_sub);

    nh.advertise(state_publisher);
    nh.advertise(balancing_get_publisher);
    nh.advertise(upright_publisher);
    nh.advertise(foot_degrees_publisher);
    nh.advertise(params_publisher);
    nh.advertise(button_publisher);
    nh.advertise(hip_pid_publisher);
    nh.advertise(balancing_button_publisher);

    ag_sensor.initialize();
    if(!ag_sensor.is_ready()){
        nh.loginfo("MPU6050 initialization failed.");
    }
    
    reset_hip_position();
    //hip_pid_controller.reset();
    
    servo_hip_left_controller.set_feedback_positions(
        configuration.servo_hip_left_lower_pos_feedback,
        configuration.servo_hip_left_upper_pos_feedback
    );
    
    servo_hip_right_controller.set_feedback_positions(
        configuration.servo_hip_right_lower_pos_feedback,
        configuration.servo_hip_right_upper_pos_feedback
    );
    
    //TODO:knee feedback set?

    balance_controller.set_weights(
        configuration.w_yeta,
        configuration.w_yeta_dot,
        configuration.w_theta,
        configuration.w_theta_dot,
        configuration.w_zeta
    );
    
    hip_pid_controller.Kp = configuration.Kp;
    hip_pid_controller.Ki = configuration.Ki;
    hip_pid_controller.Kd = configuration.Kd;
    hip_pid_controller.reset();
    
    servo_knee_left_controller.set_power_down_seconds(5);
    servo_knee_right_controller.set_power_down_seconds(5);

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

        bool_msg.data = last_pushbutton_state;
        button_publisher.publish(&bool_msg);
        nh.spinOnce();
        
        if(last_pushbutton_state){
            nh.loginfo("Pushbutton released.");
        }else{
            nh.loginfo("Pushbutton pressed.");
            
            // On depress, toggle balancing flag.
            if(balance_button_enabled){
                if(balancing_enabled){
                    disable_balancing();
                }else{
                    hip_pid_controller.target = foot_tracker.get_position();
                    enable_balancing();
                }
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // Refresh all sensors.

    ag_sensor.update(); // Update MPU6050 data.
    //if(millis() - last_aaread_time >= 100){
        //last_aaread_time = millis();
        //snprintf(buffer, MAX_OUT_CHARS, "acc:%i", ag_sensor.aaReal.z - last_acc_z);
        //nh.loginfo(buffer);
        //nh.spinOnce();
    //}
    diff_acc_z = ag_sensor.aaReal.z - last_acc_z;
    last_acc_z = ag_sensor.aaReal.z;
    servo_hip_left_controller.update();
    servo_hip_right_controller.update();
    servo_knee_left_controller.update();
    servo_knee_right_controller.update();
    pitch_degrees = -ag_sensor.ypr[1] * 180/M_PI;
    foot_tracker.update_position(calculate_foot_degrees(servo_hip_right_controller.get_actual_position_degrees(), pitch_degrees));
    
    ////////////////////////////////////////////////////////////////////////////
    // Reaction to sensors.

    // If our angle is off by more than a set threshold, than it means we've fallen over, so stop updating the servos as a safety mechanism.
    angle_safety_shutoff = abs(foot_tracker.get_position()) > SERVO_SHUTOFF_BACKWARDS_ANGLE;
    if(angle_safety_shutoff_enabled && angle_safety_shutoff && balancing_enabled){
        
        dtostrf(abs(foot_tracker.get_position()), 5, 3, x_str);
        dtostrf(SERVO_SHUTOFF_BACKWARDS_ANGLE, 5, 3, y_str);
        snprintf(buffer, MAX_OUT_CHARS, "Safety shutoff angle exceeded. Foot angle %s has exceeded max angle %s.", x_str, y_str);
        nh.loginfo(buffer);

        disable_balancing();
        nh.spinOnce();
        Serial.flush();
    }
    
    // Publish change in upright.
    if(angle_safety_shutoff != last_angle_safety_shutoff) {
        bool_msg.data = !angle_safety_shutoff;
        upright_publisher.publish(&bool_msg);
        nh.spinOnce();
        Serial.flush();
    }
    
    // Keep our pitch balanced.
    if(balancing_enabled && millis() - servo_last_set_time > 10){ // Update balance 100 times a second. TODO:increase?
        // Get our weight action.
        // Controller returns float in [-1:+1].
        // We scale this to the true maximum control signal, representing a change of position from -1000 to 1000.
        // Note the actual positions are offset between 1000 to 2000.
        //balance_action = balance_controller.step(get_yeta_bounded(), get_yeta_dot_bounded(), get_theta_bounded(), get_theta_dot_bounded());
        //increment_hip_position(int(balance_action * 1000.));
        balance_action = hip_pid_controller.get_value(foot_tracker.get_position());
        if(millis() - foot_log_time >= 1000){
            dtostrf(hip_pid_controller.target, 5, 3, x_str);
            dtostrf(hip_pid_controller.last_actual, 5, 3, y_str);
            dtostrf(balance_action, 5, 3, z_str);
            snprintf(buffer, MAX_OUT_CHARS, "target = %s, actual = %s, balance_action = %s", x_str, y_str, z_str);
            nh.loginfo(buffer);
        }
        set_hip_position(balance_action);
    }

    // Turn off servos if unused for more than a few seconds.
    // This is a safety measure. Even when idling, and not under load, the servos will get quite warm.
    // When under load, the servos can get considerably hot.
    if(servo_hips_active && millis() - servo_last_set_time > 10000){
        disable_hip_servos();
        nh.loginfo("Hip servos disabled due to inactivity.");
    }
    
    // DEBUG output.
    //if(millis() - last_ag_read_time >= 1000){
        //last_ag_read_time = millis();
    
        //dtostrf(ag_sensor.ypr[0] * 180/M_PI, 5, 3, yaw_str);
        //dtostrf(pitch_degrees, 5, 3, pitch_str);//dtostrf(-ag_sensor.ypr[1] * 180/M_PI, 5, 3, pitch_str);
        //dtostrf(ag_sensor.ypr[2] * 180/M_PI, 5, 3, roll_str);
        //dtostrf(ag_sensor.ypr_dot[1] * 180/M_PI*1000, 15, 6, d_str);
        //dtostrf(foot_tracker.get_velocity(), 15, 6, e_str); 

        //snprintf(buffer, MAX_OUT_CHARS,
            //"balancing_enabled=%i, angle_safety_shutoff=%i, hip_angle=%i foot_angle=%i foot_vel=%s yaw=%s pitch=%s roll=%s pitch_dot=%s hl1=%i hl2=%i hr1=%i hr2=%i",
            //balancing_enabled,
            //angle_safety_shutoff,
            //servo_hip_right_controller.get_actual_position_degrees(),
            ////calculate_foot_degrees(servo_hip_right_controller.get_actual_position_degrees(), ag_sensor.ypr[1] * 180/M_PI),
            //foot_tracker.get_position(),
            //e_str,//foot_tracker.get_velocity(),
            //yaw_str, // yaw
            //pitch_str, // pitch
            //roll_str, // roll
            //d_str,
            //servo_hip_left_controller.get_lower_feedback_position(),
            //servo_hip_left_controller.get_upper_feedback_position(),
            //servo_hip_right_controller.get_lower_feedback_position(),
            //servo_hip_right_controller.get_upper_feedback_position()
        //);
        //nh.loginfo(buffer);
        
        ////~ dtostrf(servo_weight_shifter.get_velocity(), 15, 6, yaw_str);
        ////~ snprintf(buffer, MAX_OUT_CHARS,
            ////~ "calibration_state=%i millis()=%lu state_change_time=%lu weight lower=%i weight upper=%i weight pos=%i x_vel=%s",
            ////~ servo_weight_shifter.get_calibrate_state(),
            ////~ millis(),
            ////~ servo_weight_shifter._calibrate_state_change_time,
            ////~ servo_weight_shifter.get_lower_feedback_position(),
            ////~ servo_weight_shifter.get_upper_feedback_position(),
            ////~ servo_weight_shifter.get_actual_position(),
            ////~ yaw_str
        ////~ );
        ////~ nh.loginfo(buffer);
         
    //}
    
    nh.spinOnce();
    Serial.flush();

    // Output a heartbeat count to track how long we've been running.
    if(millis() - last_heartbeat_time >= 10000){
        last_heartbeat_time = millis();
        heartbeat_count += 1;
        snprintf(buffer, MAX_OUT_CHARS, "i=%i", heartbeat_count);
        nh.loginfo(buffer);
        nh.spinOnce();
        Serial.flush();
    }
    
    // Output frequent updates.
    if(millis() - foot_log_time >= 1000){
        // Output foot angle.
        foot_log_time = millis();
        int32_msg.data = foot_tracker.get_position();
        foot_degrees_publisher.publish(&int32_msg);
        nh.spinOnce();
        Serial.flush();
        // Output upright status.
        //bool_msg.data = !angle_safety_shutoff;
        //upright_publisher.publish(&bool_msg);
        //nh.spinOnce();
        //Serial.flush();
        // Output balancing button state.
        //bool_msg.data = balance_button_enabled;
        //balancing_button_publisher.publish(&bool_msg);
        //nh.spinOnce();
        //Serial.flush();
    }

    // If calibration variables have changed, then save them to EEPROM.
    if(!servo_hip_left_controller.saved || !servo_hip_right_controller.saved){
        save_configuration();
    }

    // Publish state.
    if(millis() - last_state_publish_time >= 100){ // publish 10 times a second
        float32ma_msg.data[0] = get_yeta_bounded();
        float32ma_msg.data[1] = get_yeta_dot_bounded();
        float32ma_msg.data[2] = get_theta_bounded();
        float32ma_msg.data[3] = get_theta_dot_bounded();
        float32ma_msg.data[5] = 0;
        state_publisher.publish(&float32ma_msg);
        last_state_publish_time = millis();
    }
    nh.spinOnce();
    Serial.flush();

    // Save short term states for comparison in next iteration.
    last_hip_angle = current_hip_angle;
    disabling_balancing = false;
    last_angle_safety_shutoff = angle_safety_shutoff;
}
