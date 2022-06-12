#include "Vector.h"
#include <cmath>

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