// integration of gyroscope data

#include </usr/include/eigen3/Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic > Cmatrix;
typedef Eigen::Quaternion<double> quaternion;

void or_integration(double orientation[4], double imuAngRates[3], double bias_orientation[3], double rotOrient[3][3], double dt)
{
    double imuAngRatesreal[3];

    //subtracting bias
    imuAngRatesreal[0] = imuAngRates[0] - bias_orientation[0];
    imuAngRatesreal[1] = imuAngRates[1] - bias_orientation[1];
    imuAngRatesreal[2] = imuAngRates[2] - bias_orientation[2];

    double norm_imuAng;
    double k_orient;

    //computation of quaternion associated with angular rates
    norm_imuAng = sqrt( pow((imuAngRatesreal[0]*dt),2) + pow((imuAngRatesreal[1]*dt),2) + pow((imuAngRatesreal[2]*dt),2));
    k_orient = dt* sin(norm_imuAng / 2) / norm_imuAng;

    quaternion orientation_add;
    orientation_add.w() = cos(norm_imuAng / 2);
    orientation_add.x() = k_orient * imuAngRatesreal[0];
    orientation_add.y() = k_orient * imuAngRatesreal[1];
    orientation_add.z() = k_orient * imuAngRatesreal[2];


    Cmatrix rotMatrix_m(3,3);


    quaternion orientation_temp;
    quaternion orientation_vec;
    orientation_vec.w() = orientation[0];
    orientation_vec.x() = orientation[1];
    orientation_vec.y() = orientation[2];
    orientation_vec.z() = orientation[3];

    // quaternion update
    orientation_temp = (orientation_add * orientation_vec).normalized();

    rotMatrix_m = orientation_temp.toRotationMatrix();
    for (int i = 0; i<3; i++)
    {
        for (int j = 0; j<3; j++)
        {
            rotOrient[i][j] = rotMatrix_m(i,j);

        }
    }
    orientation[0] = orientation_temp.w();
    orientation[1] = orientation_temp.vec()[0];
    orientation[2] = orientation_temp.vec()[1];
    orientation[3] = orientation_temp.vec()[2];

}
