// Update the state based on the correction computed in compute_matrix

#include </usr/include/eigen3/Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic > Cmatrix;
typedef Eigen::Quaternion<double> quaternion;
const double EPSILON = 0.00001;
void compute_x(double position[3],double velocity[3], double orientation[4], double p_left_w[3],
                double p_right_w[3], double bias_acceleration[3], double bias_orientation[3],
                double D_x[21])
{

    for (int i=0; i<3; i++)
    {
        position[i] = position[i] + D_x[i];
        velocity[i] = velocity[i] + D_x[3+i];
        p_left_w[i] = p_left_w[i] + D_x[9+i];
        p_right_w[i] = p_right_w[i] + D_x[12+i];
        bias_acceleration[i] = bias_acceleration[i] + D_x[15+i];
        bias_orientation[i] = bias_orientation[i] + D_x[18+i];
    }

    quaternion orientation_vec;
    quaternion orientation_add;
    quaternion orientation_temp;

    double norm_adj;
    double k_orient;


    orientation_vec.w() = orientation[0];
    orientation_vec.x() = orientation[1];
    orientation_vec.y() = orientation[2];
    orientation_vec.z() = orientation[3];


    norm_adj = sqrt(pow((D_x[6]),2) + pow((D_x[7]),2) + pow((D_x[8]),2));
    k_orient = sin(norm_adj / 2) / (norm_adj + EPSILON);

    //update orientation

    orientation_add.w() = cos(norm_adj / 2);
    orientation_add.x() = k_orient * D_x[6];
    orientation_add.y() = k_orient * D_x[7];
    orientation_add.z() = k_orient * D_x[8];

    orientation_temp = (orientation_add * orientation_vec).normalized();
    orientation[0] = orientation_temp.w();
    orientation[1] = orientation_temp.vec()[0];
    orientation[2] = orientation_temp.vec()[1];
    orientation[3] = orientation_temp.vec()[2];


}
