
#ifndef VelocityTracker_h
#define VelocityTracker_h

class VelocityTracker{
    
private:

    int _last_position = 0;
    unsigned long _last_time = 0;
    float _current_velocity = 0;

public:

    VelocityTracker(){
    }
    
    int get_position(){
        return _last_position;
    }
    
    float get_velocity(){
        if(isnan(_current_velocity)){
            return 0;
        }
        return _current_velocity;
    }
    
    void update_position(int position){
        
        // Calculate velocity, in positions/second.
        _current_velocity = float(position - _last_position)/float(millis() - _last_time)*0.001;
        
        // Save value for next iteration.
        _last_position = position;
        _last_time = millis();
    }

};

#endif
