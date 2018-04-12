
#ifndef Controller_h
#define Controller_h

// Abstract
class Controller{

public:

    virtual void update();
    //{
        // called to refresh sensor input for pull sensor types
        // OVERRIDE
    //}

    //virtual bool get_and_clear_changed();
    //{
        // Returns true if something has changed since the last call, and if so, resets the flag.
        // Returns false if nothing has changed.
        //return false; // OVERRIDE
    //}

};

#endif
