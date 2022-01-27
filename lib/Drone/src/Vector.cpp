#include "Vector.h"

// Vector3<double> Vector3<double>::scale(float scalar)
// {
//     x *= scalar;
//     y *= scalar;
//     z *= scalar;
    
//     return this;
// }

// float Vector3<double>::magnitude()
// {
//     return sqrt(x*x + y*y + z*z);
// }

// Vector3<double> Vector3<double>::normalize()
// {
//     float length = magnitude();
//     x /= length;
//     y /= length;
//     z /= length;
    
//     return this;
// }

// Vector3<double> normalized(Vector3<double> vec)
// {
//     float length = vec.magnitude();
//     return new Vector3<double>(x/length, y/length, z/length);
// }

// template <typename T>
// T dotProduct(T a, T b, theta)
// {
//     return a.normalized*b.normalized*cos(theta);
// }
template <typename T>
T dotProduct(Vector3<T> a, Vector3<T> b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

template <typename T>
// A X B
// Calcs unit vector perpendicular to both a and b vectors
Vector3<T> crossProduct(Vector3<T> a, Vector3<T> b)
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