#include <Vector.h>
#include <WirelessData.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>

#ifndef DRONE_H
#define DRONE_H
class Drone
{
    private:
        Vector3<float> position;
        Vector3<float> rotation;
        Vector3<float> velocity;
        Vector3<float> angularVelocity;

        Vector3<float> prevPosition;
        Vector3<float> prevRotation;
        Vector3<float> prevVelocity;
        Vector3<float> prevAngularVelocity;

        Vector3<float> accelZeroOffset;
        Vector3<float> gyroZeroOffset;

        //inertial measurement unit
        Adafruit_MPU6050 mpu;
        Adafruit_MotorShield motorShield;
        void calibrate(); 
    public:
        //Motors
        Adafruit_DCMotor *m1;// = motorShield.getMotor(1);
        Adafruit_DCMotor *m2;// = motorShield.getMotor(2);
        Adafruit_DCMotor *m3;// = motorShield.getMotor(3);
        Adafruit_DCMotor *m4;// = motorShield.getMotor(4);
        const byte motorMinSpeed = 0;
        const byte motorMaxSpeed = 255;
        byte m1Speed = 0;
        byte m2Speed = 0;
        byte m3Speed = 0;
        byte m4Speed = 0;
        Drone();
        //Drone(Adafruit_MPU6050 *mpu);
        //Drone(Adafruit_MPU6050 *mpu, Adafruit_MotorShield *motorShield);
        ~Drone();
        void Init();
        void Update(JoystickControllerData* input);
        // Vector3<float> GetAcceleration();
        Vector3<float> GetAngularVelocity() {return angularVelocity;}
        Vector3<float> GetVelocity() {return velocity;}
        Vector3<float> GetPosition() {return position;}
        Vector3<float> GetRotation() {return rotation;}
};
#endif
