
#ifndef BalanceController_h
#define BalanceController_h

/*
 * Implements a simple boolean neural net controller.
 * */
class BalanceController{

private:


public:

    // Decision weights.
    double w_x; // estimated position of balancing weight
    double w_x_dot; // estimated velocity of balancing weight
    double w_theta; // pitch angle of the torso (0 = perfectly upright)
    double w_theta_dot; // angular velocity of the pitch angle

    BalanceController(){
        w_x = 0; // negative=?, positive=?
        w_x_dot = 0;
        w_theta = 0; // negative=leaning forward, positive=leaning backward
        w_theta_dot = 0;
    }
    
    void set_weights(double a, double b, double c, double d){
        w_x = a;
        w_x_dot = b;
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
    int step(double x, double x_dot, double theta, double theta_dot){
        double s = w_x * x + w_x_dot * x_dot + w_theta * theta + w_theta_dot * theta_dot;
        if(s < 0){
            return 0;
        }else{
            return 1;
        }
    }

};

#endif
