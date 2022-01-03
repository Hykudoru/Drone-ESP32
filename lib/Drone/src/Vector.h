#ifndef VECTOR_H
#define VECTOR_H

template <typename T>
struct Vector3
{
    T x;
    T y;
    T z;

    Vector3();
    Vector3(T xVal, T yVal, T zVal);
    //Vector3(float xVal, float yVal, float zVal);
    //Vector3(double xVal, double yVal, double zVal);
    Vector3(T xyz[]);
    //Vector3(float xyz[]);
    //Vector3(double xyz[]);
    
    Vector3 operator+(const Vector3& other)
    {
        Vector3 vectorSum;
        vectorSum.x = this->x + other.x;
        vectorSum.y = this->y + other.y;
        vectorSum.z = this->z + other.z;
        return vectorSum;
    }

    Vector3 operator-(const Vector3& other)
    {
        Vector3 vectorDiff;
        vectorDiff.x = this->x - other.x;
        vectorDiff.y = this->y - other.y;
        vectorDiff.z = this->z - other.z;
        return vectorDiff;
    }

    Vector3 operator+=(const Vector3& other)
    {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
    }

    Vector3 operator-=(const Vector3& other)
    {
        this->x -= other.x;
        this->y -= other.y;
        this->z -= other.z;
    }

    Vector3 operator*=(const T scalar)
    {
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
    }

    Vector3 operator/=(const T scalar)
    {
        this->x /= scalar;
        this->y /= scalar;
        this->z /= scalar;
    }
};

template <typename T>
Vector3<T>::Vector3()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}
template <typename T>
Vector3<T>::Vector3(T xVal, T yVal, T zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

template <typename T>
Vector3<T>::Vector3(T xyz[])
{
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
}
#endif
