//Prediction step of the Kalman Filter
//integrating the accelerometer data

#include <math.h>
#include </usr/include/eigen3/Eigen/Dense>

typedef Eigen::Matrix< double , 3 , 3, Eigen::RowMajor> Cmatrix;
typedef Eigen::Matrix< double , 3 , 1> Cvector;

#define GRAV -9.806365

void acc_integration(double velocity[3], double position[3], double bias_acceleration[3], double imuAccelerations[3], double rotOrient[3][3], double dt)
{
    Cmatrix rotMatrix_m;
    Cvector imuAcc_v;
    Cvector accRotated_v;
    double accRotated[3];
    double grav = GRAV;


    for (int i = 0; i<3; i++)
    {
        // subtracting bias
        imuAcc_v(i) = imuAccelerations[i] - bias_acceleration[i];
        for (int j = 0; j<3; j++)
        {
            rotMatrix_m(i,j) = rotOrient[i][j];

        }
    }

    rotMatrix_m.transposeInPlace();

    // rotating the acceleration to world frame
    accRotated_v = rotMatrix_m*imuAcc_v;


    for (int i = 0; i<3; i++)
    {
        accRotated[i] = accRotated_v(i);
    }

    // Position integration
    position[0] = position[0] + dt * velocity[0] + ((dt*dt) / 2) * (accRotated[0]);
    position[1] = position[1] + dt * velocity[1] + ((dt*dt) / 2) * (accRotated[1]);
    position[2] = position[2] + dt * velocity[2] + ((dt*dt) / 2) * (accRotated[2] + grav);

    // Velocity integration
    velocity[0] = velocity[0] + dt * (accRotated[0]);
    velocity[1] = velocity[1] + dt * (accRotated[1]);
    velocity[2] = velocity[2] + dt * (accRotated[2] + grav);
}
