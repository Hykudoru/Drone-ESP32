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