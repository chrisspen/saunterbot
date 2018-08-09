
#ifndef BalanceController_h
#define BalanceController_h

/*
 * Implements a simple boolean neural net controller.
 * 
 * First layer as 4 nodes, each weighting an input from a joint angle or angle velocity.
 * Second layer as 1 node that sums up all in the first, and then weights it, ouputing the control single for the balancer.
 * 
 * Each weight should be bounded between -1 and +1. Similarly
 * */
class BalanceController{

private:


public:

    // Decision weights.
    // First layer.
    float w_yeta; // hip angle
    float w_yeta_dot; // hip velocity
    float w_theta; // foot angle
    float w_theta_dot; // foot velocity
    // Second layer.
    float w_zeta;

    BalanceController(){
        w_yeta = 0; // negative=?, positive=?
        w_yeta_dot = 0;
        w_theta = 0; // negative=leaning forward, positive=leaning backward
        w_theta_dot = 0;
        w_zeta = 0;
    }
    
    void set_weights(float a, float b, float c, float d, float e){
        w_yeta = a;
        w_yeta_dot = b;
        w_theta = c;
        w_theta_dot = d;
        w_zeta = e;
    }

    // Given input observations, returns integer representing suggested action.
    float step(float yeta, float yeta_dot, float theta, float theta_dot){
        return (w_yeta * yeta + w_yeta_dot * yeta_dot + w_theta * theta + w_theta_dot * theta_dot)/4. * w_zeta;
    }

};

#endif
