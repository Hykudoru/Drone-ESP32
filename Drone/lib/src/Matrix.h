#ifndef MATRIX_H
#define MATRIX_H
#include <Vector.h>

float Identity3x3[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float Zero3x3[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

class Matrix3x3 
{
public:
    float matrix[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};
    Matrix3x3() {}
    Matrix3x3(float matrix3x3[3][3])
    {
        for (size_t r = 0; r < 3; r++)
        {
            for (size_t c = 0; c < 3; c++)
            {
                matrix[r][c] = matrix3x3[r][c];
            }
            
        }
    }
};

Matrix3x3 Multiply(float matrixA[3][3], float matrixB[3][3])
{
    Matrix3x3 result = Matrix3x3();
    for (size_t r = 0; r < 3; r++)
    {
        for (size_t c = 0; c < 3; c++)
        {
            Vector3<float> columnVec;
            columnVec.x = matrixB[0][c];
            columnVec.y = matrixB[1][c];
            columnVec.z = matrixB[2][c];
            result.matrix[r][c] = DotProduct(Vector3<float>(matrixA[r]), columnVec);
        }
    }

    return result;
}

// Rotation Matrix about the X axis
// | 1,   0,      0     |
// | 0, Cos(T), -Sin(T) |
// | 0, Sin(T),  Cos(T) |
Matrix3x3 RotX(float theta) 
{
    float Cos = cos(theta);
    float Sin = sin(theta);

    float standardRotX[3][3] = 
    {
        {1, 0, 0},     
        {0, Cos, -Sin},  
        {0, Sin, Cos}
    };

    Matrix3x3 matrix(standardRotX);
    return matrix;
}
// Rotation Matrix about the Y axis
// | Cos(T),  0,   Sin(T)  |
// |   0,     1,     0     |
// |-Sin(T),  0,   Cos(T)  |
Matrix3x3 RotY(float theta) 
{
    float Cos = cos(theta);
    float Sin = sin(theta);

    float standardRotY[3][3] = 
    {
        {Cos, 0, Sin},     
        {0,   1,  0 },  
        {-Sin,0, Cos}
    };

    Matrix3x3 matrix(standardRotY);
    return matrix;
}

// Rotation Matrix about the Z axis
// | Cos(T), -Sin(T), 0 |
// | Sin(T),  Cos(T), 0 |
// |   0,      0,     1 |
Matrix3x3 RotZ(float theta) 
{
    float Cos = cos(theta);
    float Sin = sin(theta);

    float standardRotZ[3][3] = 
    {
        {Cos, -Sin,  0},     
        {Sin,  Cos,  0},  
        {0,     0,   1}
    };

    Matrix3x3 matrix(standardRotZ);
    return matrix;
}
#endif