
#ifndef BalanceController_h
#define BalanceController_h

#define MOVE_FORWARD 0
#define MOVE_BACKWARD 1

/*
 * Implements a simple boolean neural net controller.
 * */
class BalanceController{

private:


public:

    // Decision weights.
    float w_yeta; // hip angle
    float w_yeta_dot; // hip velocity
    float w_theta; // foot angle
    float w_theta_dot; // foot velocity

    BalanceController(){
        w_yeta = 0; // negative=?, positive=?
        w_yeta_dot = 0;
        w_theta = 0; // negative=leaning forward, positive=leaning backward
        w_theta_dot = 0;
    }
    
    void set_weights(float a, float b, float c, float d){
        w_yeta = a;
        w_yeta_dot = b;
        w_theta = c;
        w_theta_dot = d;
    }
    
    /* 
     * Given input observations, returns integer representing suggested action. 
     * 
     * Actions:
     * 
     *  0 = move weight left
     *  1 = move weight right
     * 
     * */
    int step(float yeta, float yeta_dot, float theta, float theta_dot){
        float s = w_yeta * yeta + w_yeta_dot * yeta_dot + w_theta * theta + w_theta_dot * theta_dot;
        if(s < 0){
            return MOVE_FORWARD;
        }else{
            return MOVE_BACKWARD;
        }
    }

};

#endif
