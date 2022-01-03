#include <Vector.h>

#ifndef DRONE_H
#define DRONE_H
class Drone
{
    private:
        Vector3<float> position;
        Vector3<float> rotation;
        Vector3<float> velocity;
        Vector3<float> angularVelocity;
        
        Adafruit_MPU6050 mpu;
    public:
        Drone();
        ~Drone();
        // Vector3 GetPosition();
        // Vector3 GetRotation();
        void setup();
        void loop();
        void calibrate();
};
#endif
