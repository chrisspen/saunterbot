
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.

   For the Galileo Gen1/2 Boards, there is no INT pin support. Therefore
   the INT pin does not need to be connected, but you should work on getting
   the timing of the program right, so that there is no buffer overflow.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#include "Sensor.h"


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// This function is not required when using the Galileo 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


class AccelGyroSensor: public Sensor{

    private:

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error, -1=not yet run)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector

    public:

        // class default I2C address is 0x68
        // specific I2C addresses may be passed as a parameter here
        // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
        // AD0 high = 0x69
        MPU6050 mpu;
        //MPU6050 mpu(0x69); // <-- use for AD0 high
        
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        AccelGyroSensor(){
        }
        
        bool is_ready(){
            return dmpReady && devStatus == 0;
        }

        void initialize(){
            mpu.initialize();
            devStatus = mpu.dmpInitialize();
            // make sure it worked (returns 0 if so)
            if (devStatus == 0) {
                    
                // calibrated with y-up
                //mpu.setXAccelOffset(-5703);
                //mpu.setYAccelOffset(-540);
                //mpu.setZAccelOffset(1118);
                //mpu.setXGyroOffset(15);
                //mpu.setYGyroOffset(135);
                //mpu.setZGyroOffset(56);
                // calibrated with z-up
                mpu.setXAccelOffset(-3957);
                mpu.setYAccelOffset(-533);
                mpu.setZAccelOffset(-930);
                mpu.setXGyroOffset(24);
                mpu.setYGyroOffset(124);
                mpu.setZGyroOffset(47);
                
                // turn on the DMP, now that it's ready
                //Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
                attachInterrupt(0, dmpDataReady, RISING);
                mpuIntStatus = mpu.getIntStatus();

                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                //Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
            }
        }

        void update(){
            
            // Initialization failed, so do nothing.
            if (!dmpReady) {
                return;
            }
            
            // No new data received, so do nothing.
            if (!mpuInterrupt && fifoCount < packetSize) {
                return;
            }

            // reset interrupt flag and get INT_STATUS byte
            mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();

            // get current FIFO count
            fifoCount = mpu.getFIFOCount();

            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                //Serial.println(F("FIFO overflow!"));

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x02) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                // display real acceleration, adjusted to remove gravity
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                //Serial.print("areal\t");
                //Serial.print(aaReal.x);
                //Serial.print("\t");
                //Serial.print(aaReal.y);
                //Serial.print("\t");
                //Serial.print(aaReal.z);

                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetYawPitchRollOnEnd(ypr, &q, &gravity);
                //Serial.print("\typr\t");
                //Serial.print(ypr[0] * 180/M_PI);
                //Serial.print("\t");
                //Serial.print(ypr[1] * 180/M_PI);
                //Serial.print("\t");
                //Serial.println(ypr[2] * 180/M_PI);

                // blink LED to indicate activity
                //blinkState = !blinkState;
                //digitalWrite(LED_PIN, blinkState);
            }

        }

        bool get_and_clear_changed(){
            //TODO
            return false;
        }

};
