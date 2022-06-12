#include "Vector.h"
#include <cmath>

//-------------------------------
// --------- Vector2 ------------
//-------------------------------
template <typename T>
Vector2<T> Vector2<T>::Add(Vector2<T> other)
{
    x += other.x;
    y += other.y;
    
    return this;
}

template <typename T>
Vector2<T> Vector2<T>::Scale(T scalar)
{
    x *= scalar;
    y *= scalar;
    
    return this;
}

template <typename T>
T Vector2<T>::SqrMagnitude()
{
    return x*x + y*y;
}

template <typename T>
T Vector2<T>::Magnitude()
{
    return sqrt(this->SqrMagnitude());
}

//--------- Normalize() vs Normalized() ---------
// Normalize() returns vector and modifies the original.
// Normalized() returns vector without modifying the original.

template <typename T>
Vector2<T> Vector2<T>::Normalize()
{
    float length = this->Magnitude();
    if (length < 0.00001) {
        x = 0;
        y = 0;
    }
    else {
        x /= length;
        y /= length;
    }
    
    return this;
}

template <typename T>
Vector2<T> Vector2<T>::Normalized()
{
    float length = this->Magnitude();
    return new Vector2<T>((float)x/length, (float)y/length);
}

//-------------------------------
// --------- Vector3 ------------
//-------------------------------
template <typename T>
Vector3<T> Vector3<T>::Add(Vector3<T> other)
{
    x += other.x;
    y += other.y;
    z += other.z;
    
    return this;
}

template <typename T>
Vector3<T> Vector3<T>::Scale(T scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    
    return this;
}

template <typename T>
T Vector3<T>::SqrMagnitude()
{
    return x*x + y*y + z*z;
}
template <typename T>
T Vector3<T>::Magnitude()
{
    return sqrt(this->SqrMagnitude());
}

//--------- Normalize() vs Normalized() ---------
// Normalize() returns vector and modifies the original.
// Normalized() returns vector without modifying the original.

template <typename T>
Vector3<T> Vector3<T>::Normalize()
{
    float length = this->Magnitude();
    if (length < 0.00001) {
        x = 0;
        y = 0;
        z = 0;
    }
    else {
        x /= length;
        y /= length;
        z /= length;
    }
    
    return this;
}

template <typename T>
Vector3<T> Vector3<T>::Normalized()
{
    float length = this->Magnitude();
    return new Vector3<T>((float)x/length, (float)y/length, (float)z/length);
}

template <typename T>
T DotProduct(Vector3<T> a, Vector3<T> b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

// A X B
// Calcs unit vector perpendicular to both a and b vectors
template <typename T>
Vector3<T> CrossProduct(Vector3<T> a, Vector3<T> b)
{
    /*
    long matrix2x3[3][3] = { }
    //row1
    matrix[0][0] = a.x;
    matrix[0][1] = a.y;
    matrix[0][2] = a.z;
    //row2
    matrix[1][0] = b.x;
    matrix[1][1] = b.y;
    matrix[1][2] = b.z;
    */
    
    /*
     |  i | j | k  |
     | a.x a.y a.z |
     | b.x b.y b.z |
    */
    
    Vector3<T> u;
    u.x = (a.y*b.z - a.z*b.y); //i 
    u.y = -(a.x*b.z) + (a.z*b.x);// -(a.x*b.z - a.z*b.x), //-j
    u.z = (a.x*b.y - a.y*b.x); //k
    /* To check is orthogonal: bool isPerpendicular = dotProduct(u, a) == 0 && dotProduct(u, b) == 0;
    */
    return u; 
}