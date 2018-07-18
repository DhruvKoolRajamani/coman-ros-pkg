#include <math.h>
#include <stdlib.h>
#include <stdio.h>
// Eigen headers
#include </usr/include/eigen3/Eigen/Dense>
#include "imu_data.hh"

//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d
#define LEFT 0
#define RIGHT 1

void forward_kinematics_pelvis1(double q_joints[],double Ppelvis[], double Trans [][3], int side)
{
    // Just to be consistent with the notion of q in the controller and libcoman (first ankle pitch and then roll)
    double qtmp = q_joints[4];
    q_joints[4] = q_joints[5];
    q_joints[5] = qtmp;

    double P_cartesian[6];
    double C12, S12, C13, S13, C14, S14, C15, S15, C16, S16, C17, S17;
    double RO_0_414, RO_0_614, RO_0_714, RO_0_914, RO_0_115, RO_0_215, RO_0_315, RO_0_415, RO_0_515, RO_0_615;
    double RO_0_116, RO_0_216, RO_0_316, RO_0_716, RO_0_816, RO_0_916, RO_0_417, RO_0_517, RO_0_617, RO_0_717;
    double RO_0_817, RO_0_917, RO_0_718, RO_0_818, RO_0_918, RL_0_115, RL_0_215, RL_0_315, RL_0_116, RL_0_216;
    double RL_0_316, RL_0_117, RL_0_217, RL_0_317, RL_0_119, RL_0_219, RL_0_319;
    double q[18] = {0};
    double dpt[6][6] = {0};
// coman joints
    q[12] = q_joints[0];
    q[13] = q_joints[1];
    q[14] = q_joints[2];
    q[15] = q_joints[3];
    q[16] = q_joints[4];
    q[17] = q_joints[5];
    if(side==LEFT)
    {
        dpt[2][1] = 0.023;
        dpt[2][2] = 0.0496;
        dpt[3][3] = -0.1024;
        dpt[3][4] = -0.1234;
        dpt[3][5] = -0.201;
        dpt[3][7] = -0.1;
    }
    if(side==RIGHT)
    {
        dpt[2][1] = -0.023;
        dpt[2][2] = -0.0496;
        dpt[3][3] = -0.1024;
        dpt[3][4] = -0.1234;
        dpt[3][5] = -0.201;
        dpt[3][7] = -0.1;
    }
// Trigonometric Variables
    C12 = cos(q[12]);
    S12 = sin(q[12]);
    C13 = cos(q[13]);
    S13 = sin(q[13]);
    C14 = cos(q[14]);
    S14 = sin(q[14]);
    C15 = cos(q[15]);
    S15 = sin(q[15]);
    C16 = cos(q[16]);
    S16 = sin(q[16]);
    C17 = cos(q[17]);
    S17 = sin(q[17]);

    RO_0_414 = S12*S13;
    RO_0_614 = C12*S13;
    RO_0_714 = S12*C13;
    RO_0_914 = C12*C13;
    RO_0_115 = RO_0_414*S14+C12*C14;
    RO_0_215 = C13*S14;
    RO_0_315 = RO_0_614*S14-S12*C14;
    RO_0_415 = RO_0_414*C14-C12*S14;
    RO_0_515 = C13*C14;
    RO_0_615 = RO_0_614*C14+S12*S14;
    RO_0_116 = RO_0_115*C15-RO_0_714*S15;
    RO_0_216 = RO_0_215*C15+S13*S15;
    RO_0_316 = RO_0_315*C15-RO_0_914*S15;
    RO_0_716 = RO_0_115*S15+RO_0_714*C15;
    RO_0_816 = RO_0_215*S15-S13*C15;
    RO_0_916 = RO_0_315*S15+RO_0_914*C15;
    RO_0_417 = RO_0_415*C16+RO_0_716*S16;
    RO_0_517 = RO_0_515*C16+RO_0_816*S16;
    RO_0_617 = RO_0_615*C16+RO_0_916*S16;
    RO_0_717 = -(RO_0_415*S16-RO_0_716*C16);
    RO_0_817 = -(RO_0_515*S16-RO_0_816*C16);
    RO_0_917 = -(RO_0_615*S16-RO_0_916*C16);
    RO_0_718 = RO_0_116*S17+RO_0_717*C17;
    RO_0_818 = RO_0_216*S17+RO_0_817*C17;
    RO_0_918 = RO_0_316*S17+RO_0_917*C17;
    RL_0_115 = RO_0_714*dpt[3][3];
    RL_0_215 = -dpt[3][3]*S13;
    RL_0_315 = RO_0_914*dpt[3][3];
    RL_0_116 = RO_0_714*dpt[3][4];
    RL_0_216 = -dpt[3][4]*S13;
    RL_0_316 = RO_0_914*dpt[3][4];
    RL_0_117 = RO_0_716*dpt[3][5];
    RL_0_217 = RO_0_816*dpt[3][5];
    RL_0_317 = RO_0_916*dpt[3][5];
    RL_0_119 = RO_0_718*dpt[3][7];
    RL_0_219 = RO_0_818*dpt[3][7];
    RL_0_319 = RO_0_918*dpt[3][7];


    P_cartesian[0] = RL_0_115+RL_0_116+RL_0_117+RL_0_119;
    P_cartesian[1] = RL_0_215+RL_0_216+RL_0_217+RL_0_219+dpt[2][1]+dpt[2][2];
    P_cartesian[2] = RL_0_315+RL_0_316+RL_0_317+RL_0_319;
    P_cartesian[3] = RO_0_617;
    P_cartesian[4] = RO_0_718;
    P_cartesian[5] = (RO_0_216*C17) - (RO_0_817*S17);


    // P_cartesian [0] = x
    // P_cartesian [1] = y
    // P_cartesian [2] = z
    // P_cartesian [3] = Roll
    // P_cartesian [4] = pitch
    // P_cartesian [5] = Yaw
    double c1 = cos(P_cartesian[3]);
    double c2 = cos(P_cartesian[4]);
    double c3 = cos(P_cartesian[5]);
    double s1 = sin(P_cartesian[3]);
    double s2 = sin(P_cartesian[4]);
    double s3 = sin(P_cartesian[5]);
    Cmatrix A_FTtoH(3,3); // A_HtoFT converts vectors represented in Hip coordinate system to Foot Coordinate system: A_HtoFT [x]_H = [x]_Ft
    A_FTtoH(0,0) = c2*c3;
    A_FTtoH(0,1) = -c2*s3;
    A_FTtoH(0,2) = s2;
    A_FTtoH(1,0) = c1*s3+c3*s1*s2;
    A_FTtoH(1,1) = c1*c3-s1*s2*s3;
    A_FTtoH(1,2) = -c2*s1;
    A_FTtoH(2,0) = s1*s3-c1*c3*s2;
    A_FTtoH(2,1) = c3*s1+c1*s2*s3;
    A_FTtoH(2,2) = c1*c2;
    Cmatrix A_HtoFT(3,3);
    for (int i = 0; i < 3; i++)
        for (int j =0; j < 3; j++)
        {
            A_HtoFT(i,j) = A_FTtoH(j,i);
        }
    Cvector3 P_FTinH_vec;
    for (int i =0; i < 3; i++)
        P_FTinH_vec(i) = P_cartesian[i];

    imu_data IMU_data;
    double Orientation[2];
    double cdPR1[3][3];
    IMU_data.get_Orientation(Trans, cdPR1,Orientation);
    double thr = Orientation[0];
    double thp = Orientation[1];
    Cmatrix cdPR(3,3);
    for (int i =0; i < 3; i++)
        for (int j =0; j < 3; j++)
            cdPR(i,j) = cdPR1[i][j];
    A_FTtoH = cdPR*A_FTtoH;
    P_FTinH_vec = cdPR*P_FTinH_vec; // P_FT in the inertial frame attached to the Hip
    for (int i =0; i < 3; i++)
        Ppelvis[i] = -P_FTinH_vec(i);
}
