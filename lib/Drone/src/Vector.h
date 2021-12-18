#ifndef VECTOR_H
#define VECTOR_H

struct Vector3
{
    float xyz[3];
    float x;
    float y;
    float z;

    Vector3();
    Vector3(float xVal, float yVal, float zVal);
    Vector3(float xyz[]);
};
#endif