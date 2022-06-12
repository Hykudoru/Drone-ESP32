#ifndef VECTOR_H
#define VECTOR_H

//-------------------------------
// --------- Vector2 ------------
//-------------------------------
template <typename T>
class Vector2
{
    public:
    T x;
    T y;

    Vector2();
    Vector2(T xVal, T yVal);
    Vector2(T xy[]);

    Vector2<T> Add(Vector2<T> other);
    Vector2<T> Scale(T scalar);
    T SqrMagnitude();
    T Magnitude();
    Vector2<T> Normalize();
    Vector2<T> Normalized();

    Vector2 operator+(const Vector2& other)
    {
        Vector2 vectorSum;
        vectorSum.x = (float)this->x + other.x;
        vectorSum.y = (float)this->y + other.y;
        return vectorSum;
    }

    Vector2 operator-(const Vector2& other)
    {
        Vector2 vectorDiff;
        vectorDiff.x = (float)this->x - other.x;
        vectorDiff.y = (float)this->y - other.y;
        return vectorDiff;
    }

    Vector2 operator*(const T& scalar)
    {
        Vector2 scaledVector;
        scaledVector.x = (float)this->x * scalar;
        scaledVector.y = (float)this->y * scalar;
        return scaledVector;
    }

    Vector2 operator+=(const Vector2& other)
    {
        this->x = (float)this->x + other.x;
        this->y = (float)this->y + other.y;
    }

    Vector2 operator-=(const Vector2& other)
    {
        this->x = (float)this->x - other.x;
        this->y = (float)this->y - other.y;
    }

    Vector2 operator*=(const T scalar)
    {
        this->x = (float)this->x * scalar;
        this->y = (float)this->y * scalar;
    }

    Vector2 operator/=(const T scalar)
    {
        if (scalar < 0.00001 && scalar > -0.00001) {
            scalar = 0.00001;
        }

        this->x = (float)this->x / scalar;
        this->y = (float)this->y / scalar;
    }
};
template <typename T>
Vector2<T>::Vector2()
{
    x = 0.0;
    y = 0.0;
}
template <typename T>
Vector2<T>::Vector2(T xVal, T yVal)
{
    x = xVal;
    y = yVal;
}

template <typename T>
Vector2<T>::Vector2(T xy[])
{
    x = xy[0];
    y = xy[1];
}

//-------------------------------
// --------- Vector3 ------------
//-------------------------------
template <typename T>
class Vector3
{
    public:
    T x;
    T y;
    T z;

    Vector3();
    Vector3(T xVal, T yVal, T zVal);
    Vector3(T xyz[]);

    Vector3<T> Add(Vector3<T> other);
    Vector3<T> Scale(T scalar);
    T SqrMagnitude();
    T Magnitude();
    Vector3<T> Normalize();
    Vector3<T> Normalized();

    Vector3 operator+(const Vector3& other)
    {
        Vector3 vectorSum;
        vectorSum.x = (float)this->x + other.x;
        vectorSum.y = (float)this->y + other.y;
        vectorSum.z = (float)this->z + other.z;
        return vectorSum;
    }

    Vector3 operator-(const Vector3& other)
    {
        Vector3 vectorDiff;
        vectorDiff.x = (float)this->x - other.x;
        vectorDiff.y = (float)this->y - other.y;
        vectorDiff.z = (float)this->z - other.z;
        return vectorDiff;
    }

    Vector3 operator*(const T& scalar)
    {
        Vector3 scaledVector;
        scaledVector.x = (float)this->x * scalar;
        scaledVector.y = (float)this->y * scalar;
        scaledVector.z = (float)this->z * scalar;
        return scaledVector;
    }

    Vector3 operator+=(const Vector3& other)
    {
        this->x = (float)this->x + other.x;
        this->y = (float)this->y + other.y;
        this->z = (float)this->z + other.z;
    }

    Vector3 operator-=(const Vector3& other)
    {
        this->x = (float)this->x - other.x;
        this->y = (float)this->y - other.y;
        this->z = (float)this->z - other.z;
    }

    Vector3 operator*=(const T scalar)
    {
        this->x = (float)this->x * scalar;
        this->y = (float)this->y * scalar;
        this->z = (float)this->z * scalar;
    }

    Vector3 operator/=(const T scalar)
    {
        if (scalar < 0.00001 && scalar > -0.00001) {
            scalar = 0.00001;
        }
        
        this->x = (float)this->x / scalar;
        this->y = (float)this->y / scalar;
        this->z = (float)this->z / scalar;
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
