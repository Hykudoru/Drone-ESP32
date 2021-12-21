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

Vector3 Vector3 add(Vector3 vec)
{
    x += vec.x;
    y += vec.y;
    z += vec.z;
    
    return this;
}

Vector3 Vector3 sub(Vector3 vec)
{
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    
    return this;
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
    length = magnitude();
    x /= length;
    y /= length;
    z /= length;
    
    return this;
}

Vector3 normalized(Vector3)
{
    length = magnitude();
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

Vector3 CrossProduct(Vector3 a, Vector3 b)
{
    /*
    long matrix2x3[2][3] = { }
    //row1
    matrix2x3[0][0] = a.x;
    matrix2x3[0][1] = a.y;
    matrix2x3[0][2] = a.z;
    //row2
    matrix2x3[1][0] = b.x;
    matrix2x3[1][1] = b.y;
    matrix2x3[1][2] = b.z;
    */
    //Vector product A X B
    Vector3 vectorAxB = new Vector3();
    vectorAxB.x = (a.y*b.z - a.z*b.y); //i 
    vectorAxB.y = -(a.x*b.z) + (a.z*b.x);// -(a.x*b.z - a.z*b.x), //-j
    vectorAxB.z = (a.x*b.y - a.y*b.x); //k
                      };
    return vectorAxB;
    /*
         |  i | j | k  |
     A   | a.x a.y a.z |
     B   | b.x b.y b.z |
    */
}
