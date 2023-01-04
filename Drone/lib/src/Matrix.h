#ifndef MATRIX_H
#define MATRIX_H
#include <Vector.h>

/* 
------Dynamic Allocate/Dealocate------
float* dynArray = new float[rows];
delete[] dynArray;
dynArray = NULL;


------Dynamic Allocate------
float** table = new float*[rows];
for (size_t i = 0; i < rows; i++) 
{
    table[i] = new float[cols];
}

-----Dynamic Dealocate (Notice that the process is actually reversed)-----
for (size_t i = 0; i < rows; i++) 
{
    delete[] table[i]; //delete array
}
delete[] table; //Deallocates the memory (note: we still have the address, which later could be used for something else but doesn't belong to us.
table = NULL;   // Notice the address still exists, so we null the value to prevent holding the address to memory that doesn't belong to us.

*/

/* 
    All matrices are defined as Matrix[row][column]. Matrix multiplication A*B or Multiply(A, B) assumes the left side is the row matrix while the right side is the column matrix.
    Euler conventions: Right-handed, Intrinsic.

    How to use the Matrix library to perform rotations
    -----------------------------------------------------------------
        Z axis rotation: 

        rotation *= Matrix3x3::RotZ(PI/2.0);
    ------------------------------------------------------------------
        Euler rotation:

        rotation *= YPR(180*PI/180, 45*PI/180, 90*PI/180);
    ------------------------------------------------------------------
        Undo Euler rotation:
        
        1st. rotation *= YPR(180*PI/180, 45*PI/180, 90*PI/180);
        2nd. rotation *= RPY(-180*PI/180, -45*PI/180, -90*PI/180);
    ----------------------------------------------------------------
*/

float IDENTITY3x3[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float ZERO3x3[3][3] = {
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

    void Set(float matrix3x3[3][3])
    {
        for (size_t r = 0; r < 3; r++)
        {
            for (size_t c = 0; c < 3; c++)
            {
                this->matrix[r][c] = matrix3x3[r][c];
            }
        }
    }

    Matrix3x3() {}
    Matrix3x3(float matrix3x3[3][3])
    {
        this->Set(matrix3x3);
    }
    
    // A*B
    static Matrix3x3 Multiply(const float matrixA[][3], const float matrixB[][3])
    {
        Matrix3x3 result = Matrix3x3();
        for (size_t r = 0; r < 3; r++)
        {
            Vector3<float> rowVec = Vector3<float>(matrixA[r][0], matrixA[r][1], matrixA[r][2]);
            for (size_t c = 0; c < 3; c++)
            {
                Vector3<float> columnVec;
                columnVec.x = matrixB[0][c];
                columnVec.y = matrixB[1][c];
                columnVec.z = matrixB[2][c];
                result.matrix[r][c] = DotProduct(Vector3<float>(rowVec), columnVec);
            }
        }

        return result;
    }
    
    // Same as matrixA = matrixA * matrixB
    Matrix3x3& operator*=(Matrix3x3 matrixB)
    {
        *this = Multiply(this->matrix, matrixB.matrix);
        return *this;
    }

    Matrix3x3& operator=(float matrix[3][3])
    {
        //*this = Matrix3x3(matrix);
        this->Set(matrix);
        return *this;
    }

    // Rotation Matrix about the X axis (in radians)
    // | 1,   0,      0     |
    // | 0, Cos(T), -Sin(T) |
    // | 0, Sin(T),  Cos(T) |
    static Matrix3x3 RotX(float theta) 
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

    // Rotation Matrix about the Y axis (in radians)
    // | Cos(T),  0,   Sin(T)  |
    // |   0,     1,     0     |
    // |-Sin(T),  0,   Cos(T)  |
    static Matrix3x3 RotY(float theta) 
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

    // Rotation Matrix about the Z axis (in radians)
    // | Cos(T), -Sin(T), 0 |
    // | Sin(T),  Cos(T), 0 |
    // |   0,      0,     1 |
    static Matrix3x3 RotZ(float theta) 
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
};

// A*B
Matrix3x3 operator*(const Matrix3x3& matrixA, const Matrix3x3& matrixB)
{
    Matrix3x3 dot = Matrix3x3::Multiply(matrixA.matrix, matrixB.matrix);
    return dot; 
}

// Roll-Pitch-Yaw x-y'-z''(intrinsic rotation) or z-y-x (extrinsic rotation)
Matrix3x3 RPY(float roll, float pitch, float yaw)
{
  //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotX(roll).matrix, Matrix3x3::RotY(pitch).matrix).matrix, Matrix3x3::RotZ(yaw).matrix);//Multiply(rotation.matrix, RotZ(PI/2.0).matrix);
  Matrix3x3 rotation = (Matrix3x3::RotX(roll) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotZ(yaw);//Multiply(rotation.matrix, RotZ(PI/2.0).matrix);
  return rotation;
}

// Yaw-Pitch-Roll z-y'-x''(intrinsic rotation) or x-y-z (extrinsic rotation)
Matrix3x3 YPR(float roll, float pitch, float yaw)
{
  //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotZ(yaw).matrix, Matrix3x3::RotY(pitch).matrix).matrix, Matrix3x3::RotX(roll).matrix);//Multiply(rotation.matrix, RotZ(PI/2.0).matrix);
  Matrix3x3 rotation = (Matrix3x3::RotZ(yaw) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotX(roll);//Multiply(rotation.matrix, RotZ(PI/2.0).matrix);
  
  return rotation;
}

// //To-Do
// void AngleAxisRotation(float axis[3], float angle)
// {
//     float mag = ((Vector3<float>)axis).Magnitude();
//     for (size_t i = 0; i < 3; i++)
//     {
//         axis[i] *= angle;
//     }
// }
#endif