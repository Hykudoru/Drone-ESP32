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
        //inertial measurement unit
        Adafruit_MPU6050 mpu;
        //motors
        //Adafruit_DCMotor 
    public:
        Drone();
        //Drone(Adafruit_MPU6050 *mpu);
        //Drone(Adafruit_MPU6050 *mpu, Adafruit_MotorShield *motorShield);
        ~Drone();
        // Vector3 GetPosition();
        // Vector3 GetRotation();
        void init();
        void update();
        void calibrate();
};
#endif
