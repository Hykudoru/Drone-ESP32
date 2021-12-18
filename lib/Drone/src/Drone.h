#include <Vector.h>

#ifndef DRONE_H
#define DRONE_H
class Drone
{
    private:
        Vector3 position;
        Vector3 rotation;
        Vector3 velocity;
        Vector3 angularVelocity;
        
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
