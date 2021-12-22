#ifndef VECTOR_H
#define VECTOR_H

struct Vector3
{
    float x;
    float y;
    float z;

    Vector3();
    Vector3(float xVal, float yVal, float zVal);
    Vector3(float xyz[]);
    
    Vector3 operator+(const Vector3& vec)
    {
        Vector3 vectorSum;
        vectorSum.x = this->x + vec.x;
        vectorSum.y = this->y + vec.y;
        vectorSum.z = this->z + vec.z;
        return vectorSum;
    }

    Vector3 operator-(const Vector3& vec)
    {
        Vector3 vectorDiff;
        vectorDiff.x = this->x - vec.x;
        vectorDiff.y = this->y - vec.y;
        vectorDiff.z = this->z - vec.z;
        return vectorDiff;
    }
    /*
    Vector3 operator+=(const Vector3& vec)
    {
        this->x += vec.x;
        this->y += vec.y;
        this->z += vec.z;

        return this;
    }
    */
};
#endif
