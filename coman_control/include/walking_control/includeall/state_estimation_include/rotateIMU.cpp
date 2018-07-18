//Rotate data acquired from IMU to body frame

#include "math.h"
#include </usr/include/eigen3/Eigen/Dense>
typedef Eigen::Matrix< double , 3 , 3> Cmatrix;
typedef Eigen::Matrix< double , 3 , 1> Cvector;


void rotateIMU(double rotAdj[3][3], double imuAccelerations[3])
{
    Cmatrix rotateIMU_m;
    static Cvector imuAccelerations_v;

    for (int m=0; m<3; m++)
    {
        imuAccelerations_v(m) = imuAccelerations[m];
        for(int n=0; n<3; n++)
        {
            rotateIMU_m(m,n)=rotAdj[m][n];
        }
    }    
    imuAccelerations_v = rotateIMU_m*imuAccelerations_v;

    for (int m=0; m<3; m++)
    {
        imuAccelerations[m] = imuAccelerations_v(m);
        for(int n=0; n<3; n++)
        {
            rotAdj[m][n] = rotateIMU_m(m,n);
        }
    }
}
