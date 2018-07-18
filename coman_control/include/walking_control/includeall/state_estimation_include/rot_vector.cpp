// Compute orientation from rotation between input vector and gravity vector

#include "math.h"
#include </usr/include/eigen3/Eigen/Dense>
typedef Eigen::Matrix< double , 3 , 3> Cmatrix;
typedef Eigen::Matrix< double , 3 , 1> Cvector;
typedef Eigen::Quaternion<double> quaternion;

void rot_vector(double rotMatrix[3][3], double orientation[4], double v[3])
{
    double gVec[3] = {0,0,1};

    Cvector v1;
    v1(0) = v[0];
    v1(1) = v[1];
    v1(2) = v[2];

    Cvector v2;
    v2(0) = gVec[0];
    v2(1) = gVec[1];
    v2(2) = gVec[2];

    Cmatrix rotMatrix_m;
    quaternion quat_orient;

    // set rotation from the two vectors
    quat_orient.setFromTwoVectors(v2,v1);

    rotMatrix_m = quat_orient.toRotationMatrix();
    for (int i = 0; i<3; i++)
    {
        orientation[i+1] = quat_orient.vec()[i];
        for (int j = 0; j<3; j++)
        {
            rotMatrix[i][j] = rotMatrix_m(i,j);

        }
    }

    quaternion quat_orient_v(rotMatrix_m);
    orientation[0] = quat_orient_v.w();
    orientation[1] = quat_orient_v.x();
    orientation[2] = quat_orient_v.y();
    orientation[3] = quat_orient_v.z();
}
