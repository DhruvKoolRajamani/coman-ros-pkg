// Compute predicted kinematics and output the difference between this
// and the one computed via kinematics chain

#include </usr/include/eigen3/Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic > Cmatrix;
typedef Eigen::Matrix< double , 3 , 1> Cvector;

void kinematics(const double p_left_w[3], const double p_right_w[3], const double position[3], double rotOrient[3][3],
                double Dy[6], double p_left_m[3], double p_right_m[3], double p_pelvis_left[3], double p_pelvis_right[3],
                double t_yaw[3][3])
{
    Cmatrix orient(3,3);
    Cmatrix t_yaw_m(3,3);
    Cvector p_left_b;
    Cvector p_right_b;
    Cvector position_v;
    Cvector p_right_w_v;
    Cvector p_left_w_v;
    Cvector p_pelvis_left_v;
    Cvector p_pelvis_right_v;
    for (int m=0; m<3; m++)
    {
        p_right_w_v(m) = p_right_w[m];
        p_left_w_v(m) = p_left_w[m];
        position_v(m) = position[m];
        for(int n=0; n<3; n++)
        {
            orient(m,n)=rotOrient[m][n];
            t_yaw_m(m,n) = t_yaw[m][n];
        }
    }

    // p_pelvis_left_v is the position of the left foot predicted in the world frame
    // from the pelvis without the yaw component (for controller purposes)
    p_pelvis_left_v = t_yaw_m.transpose()*(p_left_w_v-position_v);
    p_pelvis_right_v = t_yaw_m.transpose()*(p_right_w_v-position_v);

    // position of the feet in the body frame
    p_left_b = orient*(p_left_w_v-position_v);
    p_right_b = orient*(p_right_w_v-position_v);

    for (int i=0; i<3; i++)
    {
        p_pelvis_left[i] = -p_pelvis_left_v(i);
        p_pelvis_right[i] = -p_pelvis_right_v(i);
    }

    // error vector
    Dy[0] = p_left_m[0] - p_left_b(0);
    Dy[1] = p_left_m[1] - p_left_b(1);
    Dy[2] = p_left_m[2] - p_left_b(2);
    Dy[3] = p_right_m[0] - p_right_b(0);
    Dy[4] = p_right_m[1] - p_right_b(1);
    Dy[5] = p_right_m[2] - p_right_b(2);
}
