
// Variables for Time Keeper function:
#define PID_LOOP_MS 5

#define PID

class PIDController{
    
    private:

        unsigned long thisTime = 0;
        unsigned long lastTime = 0;
        
        float _lower_limit = -255;
        float _upper_limit = +255;
        
    public:
    
        float last_actual;                // Keeps track of error over time
        
        float target = 0;
    
        float Kp = 0;                   // (P)roportional Tuning Parameter
        float Ki = 0;                   // (I)ntegral Tuning Parameter        
        float Kd = 0;                  // (D)erivative Tuning Parameter       
        
        float pTerm = 0;
        float iTerm;                    // Used to accumalate error (intergral)
        float dTerm = 0;
        
        PIDController(float lower_limit, float upper_limit){
            _lower_limit = lower_limit;
            _upper_limit = upper_limit;
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
            last_actual = 0;
        }

        float get_value(float actual) {            

            // Calculate time since last time PID was called (~10ms)
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            thisTime = millis();
            unsigned long timeChange = thisTime - lastTime;

            // Calculate Error
            float error = target - actual;

            // Calculate our PID terms
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            pTerm = Kp * error;
            iTerm += Ki * error * timeChange;
            dTerm = Kd * (actual - last_actual) / timeChange;

            last_actual = actual;
            lastTime = thisTime;

            // Combine terms representing the servo position (range 1000-2000).
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            float PIDValue = pTerm + iTerm - dTerm;
            
            // Convert the PID value to the range of the limits.
            // This is useful when the PID output is to be used in an offset range, which a simple P-controller alone can't handle
            // e.g. the PID natively returns a value centered at 0, but if the servo limits are from 50-100, then return a value centered at 75.
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            PIDValue += (_upper_limit - _lower_limit)/2. + _lower_limit;
            PIDValue = constrain(PIDValue, _lower_limit, _upper_limit);

            // Return PID Output
            //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            return PIDValue;
        }
};
