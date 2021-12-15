struct Vector3
{
    double x;
    double y;
    double z;

    static double zero;
    Vector3(double x = 0, double y = 0, double z = 0) {

    }
};

class Drone
{
private:
    Adafruit_MPU6050 mpu;
    Vector3 position;
    Vector3 rotation;
public:
    Drone();
    ~Drone();
    Vector3 GetPosition();
    Vector3 GetRotation();
    void Setup();
    void Loop();
    void Calibrate();
};
Drone::Drone()
{

}

Drone::~Drone()
{
}