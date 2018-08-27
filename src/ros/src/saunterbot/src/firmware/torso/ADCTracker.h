
#ifndef ADCTracker_h
#define ADCTracker_h

#include "ResponsiveAnalogRead.h"
#include "Sensor.h"

class ADCTracker: public Sensor{

private:

    int _pin;
    //bool _pullup;
    unsigned long _last_read_time;
    bool _changed = false;
    unsigned long _last_reset_time = 0;
    unsigned long _last_change_time = 0;

public:

    int min_value = 10000;
    int max_value = 0;
    int value = 0;
    int last_value = 0;
    float offset = 0; // accounts for misalignment in sensor scale
    
    ResponsiveAnalogRead analog;

    ADCTracker(int pin, bool pullup=false){
        _pin = pin;
        pinMode(_pin, INPUT); // ensure button pin is an input
        digitalWrite(_pin, LOW); // ensure pullup is off on button pin
        analog = ResponsiveAnalogRead(_pin, true, 0.01);
    }
    
    void reset(){
        _last_reset_time = millis();            
        min_value = 10000;
        max_value = 0;
    }
    
    float get_percent(){
        return constrain(float(last_value - min_value)/float(max_value - min_value)*100, 0.0, 100.0) + (isnan(offset)?0:offset);
    }
    
    virtual void update(void){
        if(millis() - _last_read_time >= 10){ // 100 times a second
            _last_read_time = millis();
            
            //analogRead(_pin); // throw away, to clear sampling capacitor
            //value = analogRead(_pin);
            
            analog.update();
            value = analog.getValue();
            
            if(value != last_value){
                _changed = true;
                _last_change_time = millis();
            }
            
            // To limit noise, only track sensor limits for 10 seconds.
            //if(_last_reset_time && millis() - _last_reset_time < 10000){
                //if(value >= 0){
                    //min_value = min(min_value, value);
                //}
                //max_value = max(max_value, value);
            //}

            last_value = value;//constrain(value, min_value, max_value);
        }
    }

    virtual bool get_and_clear_changed(){
        bool v = _changed;
        if(_changed && millis() - _last_change_time >= 100){ // max report frequency of 10 times a second
            _changed = false;
        }
        return v;
    }

};

#endif
