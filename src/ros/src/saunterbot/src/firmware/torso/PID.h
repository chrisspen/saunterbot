
// Variables for Time Keeper function:
#define PID_LOOP_MS 5

#define PID

class PIDController{
    
    private:
    
        float lastpitch;                // Keeps track of error over time
        
        float targetAngle = 0;          // Can be adjusted according to centre of gravity 

        unsigned long thisTime = 0;
        unsigned long lastTime = 0;
        
        int _lower_limit = -255;
        int _upper_limit = +255;
        
        float _scaling_factor = 1.0;
    
    public:
    
        float Kp = 0;//7;                   // (P)roportional Tuning Parameter
        float Ki = 0;//6;                   // (I)ntegral Tuning Parameter        
        float Kd = 0;//3;                  // (D)erivative Tuning Parameter       
        
        float pTerm = 0;
        float iTerm;                    // Used to accumalate error (intergral)
        float dTerm = 0;
        
        PIDController(int lower_limit, int upper_limit, float scaling_factor=1.0){
            _lower_limit = lower_limit;
            _upper_limit = upper_limit;
            _scaling_factor = scaling_factor;
        }
        
        bool is_ready(){
            // We have to limit ourselves from running too fast, otherwise the timeChange rounds out to zero, causing a division error in the dTerm.
            return millis() - lastTime >= PID_LOOP_MS;
        }
        
        void reset(){
            lastTime = 0;
            pTerm = 0;
            iTerm = 0;
            dTerm = 0;
            lastpitch = 0;
        }

        int get_value(float pitch) {            

            // Calculate time since last time PID was called (~10ms)
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            thisTime = millis();
            double timeChange = double(thisTime - lastTime);

            // Calculate Error
            float error = targetAngle - pitch;

            // Calculate our PID terms
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            pTerm = Kp * error;
            iTerm += Ki * error * timeChange;
            dTerm = Kd * (pitch - lastpitch) / timeChange;

            lastpitch = pitch;
            lastTime = thisTime;

            // Combine terms representing the servo position (range 1000-2000).
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            float PIDValue = pTerm + iTerm - dTerm;

            // Limits PID to max motor speed
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            //if (PIDValue > 255) PIDValue = 255;
            //else if (PIDValue < -255) PIDValue = -255; 
            //PIDValue = constrain(PIDValue, _lower_limit, _upper_limit);
            
            // Scale PID to range when input and output are not the same scale.
            // For example, if the input is degrees, but output is a servo position, and we're trying to tilt,
            // outputing a transformed degree as a servo position will lead to choatic results.
            // However, if we scale the degree to a value that makes it correspond to the servo position scale, the output will be more stable.
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            PIDValue *= _scaling_factor;
            
            // Convert the PID value to the range of the limits.
            // e.g. the PID natively returns a value centered at 0, but if the servo limits are from 50-100, then return a value centered at 75.
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            PIDValue += (_upper_limit - _lower_limit)/2. + _lower_limit;
            PIDValue = constrain(PIDValue, _lower_limit, _upper_limit);

            // Return PID Output
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            return int(PIDValue);
        }
};
