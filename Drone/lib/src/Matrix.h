#ifndef MATRIX_H
#define MATRIX_H
#include <math.h>
#include <Vector.h>

float Identity[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

class Matrix3x3 
{
public:
    float matrix[3][3];
    Matrix3x3();
};

Matrix3x3 Mult(float *matrixA[3][3], float *matrixB[3][3])
{

}

//      | 1,  0,   0  |
// Rx = | 0, Cos,-Sin |
//      | 0, Sin, Cos |
void RotX(float* matrix3x3[3][3], float theta) 
{
    float Cos = cos(theta);
    float Sin = sin(theta);
    float rotMatrix3x3[3][3] = 
    {
        {1, 0, 0},     
        {0, Cos, Sin},  
        {0, -Sin, Cos}
    };

    float m[3][3];
    for (size_t r = 0; r < 3; r++)
    {
        for (size_t c = 0; c < 3; c++)
        {
            m[r][c] = DotProduct(Vector3<float>(*(matrix3x3)[r]), Vector3<float>(rotMatrix3x3[c]));
        }
    }
}

#endif