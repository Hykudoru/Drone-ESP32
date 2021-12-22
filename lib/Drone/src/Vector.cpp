#include "Vector.h"

Vector3::Vector3()
{
    x = 0;
    y = 0;
    z = 0;
}

Vector3::Vector3(float xVal, float yVal, float zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

Vector3::Vector3(float xyz[])
{
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
}

Vector3 Vector3::scale(float scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    
    return this;
}

float Vector3::magnitude()
{
    return sqrt(x*x + y*y + z*z);
}

Vector3 Vector3::normalize()
{
    float length = magnitude();
    x /= length;
    y /= length;
    z /= length;
    
    return this;
}

Vector3 normalized(Vector3 vec)
{
    float length = vec.magnitude();
    return new Vector3(x/length, y/length, z/length);
}

float dotProduct(Vector3 a, Vector3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

float dotProduct(float a, float b, theta)
{
    return a*b*cos(theta);
}

// Calcs unit vector perpendicular to both a and b vectors
Vector3 CrossProduct(Vector3 a, Vector3 b)
{
    /*
    long matrix2x3[2][3] = { }
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
    //Vector product A X B
    Vector3 u = new Vector3();
    u.x = (a.y*b.z - a.z*b.y); //i 
    u.y = -(a.x*b.z) + (a.z*b.x);// -(a.x*b.z - a.z*b.x), //-j
    u.z = (a.x*b.y - a.y*b.x); //k
              };
    /* To check is orthogonal:
    bool isPerpendicular = dotProduct(u, a) == 0 && dotProduct(u, b) == 0;
    */
    return u; 
}
