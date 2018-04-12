
#ifndef ServoController_h
#define ServoController_h

#define SERVO_CALIBRATE_POS_DONE 0
#define SERVO_CALIBRATE_POS_GO_CW 1
#define SERVO_CALIBRATE_POS_GO_CCW 2

#define SERVO_FEEDBACK_ITERS 10

#include <Servo.h>

class ServoController{

private:

    int _pin;
    int _feedback_pin = -1;
    int _lower_pos;
    int _upper_pos;
    int _pos;
    int _default_pos;
    unsigned long _pos_set_time = 0;
    
    int _weight_pos_sensor_low = -1;
    int _weight_pos_sensor_high = -1;
    int _calibrate_state = SERVO_CALIBRATE_POS_DONE;
    
    //unsigned last_feedback_read_time = 0;
    
    int _pos_feedback;
    int _last_pos_feedback;
    unsigned long _last_pos_feedback_time = 0; // ms
    int _lower_pos_feedback;
    int _upper_pos_feedback;
    double _pos_velocity = 0;

    int adc_min_value = 10000;
    int adc_max_value = 0;
    int adc_sum = 0;
    int adc_value;

public:

    Servo _servo;
    bool enabled = false;
    bool feedback_enabled = false;
    unsigned long _calibrate_state_change_time = 0;
    
    bool saved = true;

    ServoController(int pin, int lower_pos, int upper_pos, int default_pos=1500, int feedback_pin=0){
        _pin = pin;
        _lower_pos = lower_pos;
        _lower_pos_feedback = lower_pos;
        _upper_pos = upper_pos;
        _upper_pos_feedback = upper_pos;
        _pos = default_pos;
        _default_pos = default_pos;
        _feedback_pin = feedback_pin;
        if(_feedback_pin > 0){
            feedback_enabled = true;
        }
    }
    
    void calibrate_position_feedback(){
        if(!feedback_enabled){
            // Do no calibration if there's nothing to calibrate.
            return;
        }
        _calibrate_state = SERVO_CALIBRATE_POS_GO_CW;
        _calibrate_state_change_time = millis();
    }
    
    void power_on(){
        enabled = true;
        _servo.writeMicroseconds(_pos);
        _servo.attach(_pin);
    }
    
    void power_off(){
        enabled = false;
        _servo.detach();
    }
    
    void set_position(int pos){
        _pos = constrain(pos, _lower_pos, _upper_pos);
        _pos_set_time = millis();
    }
    
    void set_min_position(){
        _pos = _lower_pos;
    }
    
    void set_max_position(){
        _pos = _upper_pos;
    }

    int get_target_position(){
        return _pos;
    }
    
    int get_actual_position(){
        return _pos_feedback;
    }
    
    int get_lower_feedback_position(){
        return _lower_pos_feedback;
    }
    
    int get_upper_feedback_position(){
        return _upper_pos_feedback;
    }
    
    void set_feedback_positions(int lower, int upper){
        if(lower){
            _lower_pos_feedback = lower;
        }
        if(upper){
            _upper_pos_feedback = upper;
        }
    }
    
    int read_position_feedback(){
        // We switch to this 2.56V reference (Mega only) because our servo potentiometer outputs, at most, a 1.7V signal
        // which results in a very 200 position range of readings when using the default 5V reference.
        // By lowering the reference, we increase the accuracy of the ADC reading and double the number of positions.
        analogReference(INTERNAL2V56);
        // Take our reading, performing a simple filter that minimizes noise
        // by taking 5 readings, excluding the lower and upper extremes and then averaging the rest.
        adc_sum = 0;
        adc_min_value = 10000;
        adc_max_value = 0;
        for(int i=0; i<SERVO_FEEDBACK_ITERS; i+=1){
            adc_value = analogRead(_feedback_pin);
            adc_sum += adc_value;
            adc_min_value = min(adc_min_value, adc_value);
            adc_max_value = max(adc_max_value, adc_value);
        }
        adc_sum -= adc_min_value;
        adc_sum -= adc_max_value;
        adc_value = adc_sum/(SERVO_FEEDBACK_ITERS-2);
        // Set it back to default when we're done, so we don't screw up ADC readings for something else that expects 5V.
        analogReference(DEFAULT);
        return adc_value;
    }
    
    double get_velocity(){
        return _pos_velocity;
    }
    
    int get_calibrate_state(){
        return _calibrate_state;
    }
    
    void update(){
        if(_calibrate_state != SERVO_CALIBRATE_POS_DONE){
            // Run calibration routine until complete.
            power_on();
            if(_calibrate_state == SERVO_CALIBRATE_POS_GO_CW){
                // Moving clockwise.
                _pos = _lower_pos;
                _servo.writeMicroseconds(_pos);
                if(millis() - _calibrate_state_change_time >= 3000){
                    // After 3 seconds, even a slow servo should have received an endstop.
                    // so switch to next state.
                    _calibrate_state = SERVO_CALIBRATE_POS_GO_CCW;
                    _calibrate_state_change_time = millis();
                    _lower_pos_feedback = read_position_feedback();
                    saved = false;
                }
            }else if(_calibrate_state == SERVO_CALIBRATE_POS_GO_CCW){
                // Moving counter clockwise.
                _pos = _upper_pos;
                _servo.writeMicroseconds(_pos);
                if(millis() - _calibrate_state_change_time >= 3000){
                    // After 3 seconds, even a slow servo should have received an endstop.
                    // so switch to next state.
                    _calibrate_state = SERVO_CALIBRATE_POS_DONE;
                    _calibrate_state_change_time = millis();
                    _upper_pos_feedback = read_position_feedback();
                    saved = false;
                    _pos = _default_pos;
                    _servo.writeMicroseconds(_pos);
                }
            }
        }else if(enabled){
            _servo.writeMicroseconds(_pos);
            if(feedback_enabled){
                // Note, due to noise on the line, the measure position can vary by +/- 10 positions.
                // e.g. If the "true" position is 1500, we might get a reading at 1490 or 1510 or anywhere in between.
                _pos_feedback = read_position_feedback();
                _pos_feedback = map(_pos_feedback, _lower_pos_feedback, _upper_pos_feedback, _lower_pos, _upper_pos);
                
                // Calculate velocity, in positions/second.
                _pos_velocity = double(_pos_feedback - _last_pos_feedback)/double(millis() - _last_pos_feedback_time)*0.001;
                
                // Save value for next iteration.
                _last_pos_feedback = _pos_feedback;
                _last_pos_feedback_time = millis();
            }
        }
    }

};

#endif
