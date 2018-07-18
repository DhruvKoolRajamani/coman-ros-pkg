//This script was developed using the ROBOTRAN software
//including the URDF file of the COMAN models

#include </usr/include/eigen3/Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic > Cmatrix;

void kin_initialize(double q_left[6], double q_right[6], double p_left_m[], double p_right_m[], double rotOrient[3][3], const double position[3])
{

    double dpt[4][33] = {0};
    double C28, C29, C30, C31, C32, C33, C34, C35, C36, C37, C38, C39, S28, S29, S30, S31, S32, S33, S34, S35, S36, S37, S38, S39;
    double     ROcp0_429, ROcp0_629, ROcp0_729, ROcp0_929, ROcp0_130, ROcp0_230, ROcp0_330, ROcp0_430, ROcp0_530, ROcp0_630,
            ROcp0_131, ROcp0_231, ROcp0_331, ROcp0_731, ROcp0_831, ROcp0_931, ROcp0_432, ROcp0_532, ROcp0_632, ROcp0_732,
            ROcp0_832, ROcp0_932, ROcp0_133, ROcp0_233, ROcp0_333, ROcp0_733, ROcp0_833, ROcp0_933, RLcp0_130, RLcp0_230,
            RLcp0_330, RLcp0_131, RLcp0_231, RLcp0_331, RLcp0_132, RLcp0_232, RLcp0_332, ROcp1_435, ROcp1_635, ROcp1_735,
            ROcp1_935, ROcp1_136, ROcp1_236, ROcp1_336, ROcp1_436, ROcp1_536, ROcp1_636, ROcp1_137, ROcp1_237, ROcp1_337,
            ROcp1_737, ROcp1_837, ROcp1_937, ROcp1_438, ROcp1_538, ROcp1_638, ROcp1_738, ROcp1_838, ROcp1_938, ROcp1_139,
            ROcp1_239, ROcp1_339, ROcp1_739, ROcp1_839, ROcp1_939, RLcp1_136, RLcp1_236, RLcp1_336, RLcp1_137, RLcp1_237,
            RLcp1_337, RLcp1_138, RLcp1_238, RLcp1_338, Dz403;

    Cmatrix t_Right(4,4);
    Cmatrix t_Left(4,4);
    Cmatrix t_IMU(4,4);
    Cmatrix p_left_v(3,1);
    Cmatrix p_right_v(3,1);
    Cmatrix position_v(4,1);
    t_IMU.setZero(4,4);
    t_Left.setZero(4,4);
    t_Right.setZero(4,4);
    p_left_v.setZero(3,1);
    p_right_v.setZero(3,1);
    position_v.setZero(4,1);


    dpt[1][1] = 0.0202815;
    dpt[3][1] = 0.119121;
    dpt[2][2] = -0.023;
    dpt[2][3] = 0.023;
    dpt[1][4] = -0.072;
    dpt[2][4] = -0.0068;
    dpt[3][4] = 0.044;
    dpt[3][6] = 0.0475;
    dpt[1][7] = -0.0149765;
    dpt[2][7] = -0.0825;
    dpt[3][7] = 0.157708;
    dpt[1][8] = -0.0149765;
    dpt[2][8] = 0.0825;
    dpt[3][8] = 0.157708;
    dpt[2][9] = -0.0733;
    dpt[3][10] = -0.0448085;
    dpt[1][11] = 0.015;
    dpt[3][11] = -0.135192;
    dpt[1][12] = -0.015;
    dpt[3][12] = -0.1078;
    dpt[3][13] = -0.08688;
    dpt[3][15] = -0.07;
    dpt[2][16] = 0.0733;
    dpt[3][17] = -0.0448085;
    dpt[1][18] = 0.015;
    dpt[3][18] = -0.135192;
    dpt[1][19] = -0.015;
    dpt[3][19] = -0.1078;
    dpt[3][20] = -0.08688;
    dpt[3][22] = -0.07;
    dpt[2][23] = -0.0496;
    dpt[3][24] = -0.1024;
    dpt[3][25] = -0.1234;
    dpt[3][26] = -0.201;
    dpt[2][29] = 0.0496;
    dpt[3][30] = -0.1024;
    dpt[3][31] = -0.1234;
    dpt[3][32] = -0.201;

    // = = Block_0_0_0_0_0_5 = =

    // Trigonometric Variables

    C28 = cos(q_right[0]);
    S28 = sin(q_right[0]);
    C29 = cos(q_right[1]);
    S29 = sin(q_right[1]);
    C30 = cos(q_right[2]);
    S30 = sin(q_right[2]);
    C31 = cos(q_right[3]);
    S31 = sin(q_right[3]);
    C32 = cos(q_right[4]);
    S32 = sin(q_right[4]);
    C33 = cos(q_right[5]);
    S33 = sin(q_right[5]);

    // = = Block_0_0_0_0_0_6 = =

    // Trigonometric Variables

    C34 = cos(q_left[0]);
    S34 = sin(q_left[0]);
    C35 = cos(q_left[1]);
    S35 = sin(q_left[1]);
    C36 = cos(q_left[2]);
    S36 = sin(q_left[2]);
    C37 = cos(q_left[3]);
    S37 = sin(q_left[3]);
    C38 = cos(q_left[4]);
    S38 = sin(q_left[4]);
    C39 = cos(q_left[5]);
    S39 = sin(q_left[5]);
    Dz403 = dpt[3][4];

    // Sensor 1 // Right Foot

    ROcp0_429 = S28*S29;
    ROcp0_629 = C28*S29;
    ROcp0_729 = S28*C29;
    ROcp0_929 = C28*C29;
    ROcp0_130 = C28*C30+ROcp0_429*S30;
    ROcp0_230 = C29*S30;
    ROcp0_330 = -S28*C30+ROcp0_629*S30;
    ROcp0_430 = -C28*S30+ROcp0_429*C30;
    ROcp0_530 = C29*C30;
    ROcp0_630 = S28*S30+ROcp0_629*C30;
    ROcp0_131 = ROcp0_130*C31-ROcp0_729*S31;
    ROcp0_231 = ROcp0_230*C31+S29*S31;
    ROcp0_331 = ROcp0_330*C31-ROcp0_929*S31;
    ROcp0_731 = ROcp0_130*S31+ROcp0_729*C31;
    ROcp0_831 = ROcp0_230*S31+-S29*C31;
    ROcp0_931 = ROcp0_330*S31+ROcp0_929*C31;
    ROcp0_432 = ROcp0_430*C32+ROcp0_731*S32;
    ROcp0_532 = ROcp0_530*C32+ROcp0_831*S32;
    ROcp0_632 = ROcp0_630*C32+ROcp0_931*S32;
    ROcp0_732 = -ROcp0_430*S32+ROcp0_731*C32;
    ROcp0_832 = -ROcp0_530*S32+ROcp0_831*C32;
    ROcp0_932 = -ROcp0_630*S32+ROcp0_931*C32;
    ROcp0_133 = ROcp0_131*C33-ROcp0_732*S33;
    ROcp0_233 = ROcp0_231*C33-ROcp0_832*S33;
    ROcp0_333 = ROcp0_331*C33-ROcp0_932*S33;
    ROcp0_733 = ROcp0_131*S33+ROcp0_732*C33;
    ROcp0_833 = ROcp0_231*S33+ROcp0_832*C33;
    ROcp0_933 = ROcp0_331*S33+ROcp0_932*C33;
    RLcp0_130 = ROcp0_729*dpt[3][24];
    RLcp0_230 = -S29*dpt[3][24];
    RLcp0_330 = ROcp0_929*dpt[3][24];
    RLcp0_131 = ROcp0_729*dpt[3][25];
    RLcp0_231 = -S29*dpt[3][25];
    RLcp0_331 = ROcp0_929*dpt[3][25];
    RLcp0_132 = ROcp0_731*dpt[3][26];
    RLcp0_232 = ROcp0_831*dpt[3][26];
    RLcp0_332 = ROcp0_931*dpt[3][26];

    t_Right(0,3) = RLcp0_130+RLcp0_131+RLcp0_132;
    t_Right(1,3) = dpt[2][2]+dpt[2][23]+RLcp0_230+RLcp0_231+RLcp0_232;
    t_Right(2,3) = RLcp0_330+RLcp0_331+RLcp0_332;
    t_Right(3,3) = 1;
    t_Right(0,0) = ROcp0_133;
    t_Right(0,1) = ROcp0_233;
    t_Right(0,2) = ROcp0_333;
    t_Right(1,0) = ROcp0_432;
    t_Right(1,1) = ROcp0_532;
    t_Right(1,2) = ROcp0_632;
    t_Right(2,0) = ROcp0_733;
    t_Right(2,1) = ROcp0_833;
    t_Right(2,2) = ROcp0_933;


    // Sensor 2 // Left Foot

    ROcp1_435 = S34*S35;
    ROcp1_635 = C34*S35;
    ROcp1_735 = S34*C35;
    ROcp1_935 = C34*C35;
    ROcp1_136 = C34*C36+ROcp1_435*S36;
    ROcp1_236 = C35*S36;
    ROcp1_336 = -S34*C36+ROcp1_635*S36;
    ROcp1_436 = -C34*S36+ROcp1_435*C36;
    ROcp1_536 = C35*C36;
    ROcp1_636 = S34*S36+ROcp1_635*C36;
    ROcp1_137 = ROcp1_136*C37-ROcp1_735*S37;
    ROcp1_237 = ROcp1_236*C37+S35*S37;
    ROcp1_337 = ROcp1_336*C37-ROcp1_935*S37;
    ROcp1_737 = ROcp1_136*S37+ROcp1_735*C37;
    ROcp1_837 = ROcp1_236*S37-S35*C37;
    ROcp1_937 = ROcp1_336*S37+ROcp1_935*C37;
    ROcp1_438 = ROcp1_436*C38+ROcp1_737*S38;
    ROcp1_538 = ROcp1_536*C38+ROcp1_837*S38;
    ROcp1_638 = ROcp1_636*C38+ROcp1_937*S38;
    ROcp1_738 = -ROcp1_436*S38+ROcp1_737*C38;
    ROcp1_838 = -ROcp1_536*S38+ROcp1_837*C38;
    ROcp1_938 = -ROcp1_636*S38+ROcp1_937*C38;
    ROcp1_139 = ROcp1_137*C39-ROcp1_738*S39;
    ROcp1_239 = ROcp1_237*C39-ROcp1_838*S39;
    ROcp1_339 = ROcp1_337*C39-ROcp1_938*S39;
    ROcp1_739 = ROcp1_137*S39+ROcp1_738*C39;
    ROcp1_839 = ROcp1_237*S39+ROcp1_838*C39;
    ROcp1_939 = ROcp1_337*S39+ROcp1_938*C39;
    RLcp1_136 = ROcp1_735*dpt[3][30];
    RLcp1_236 = -S35*dpt[3][30];
    RLcp1_336 = ROcp1_935*dpt[3][30];
    RLcp1_137 = ROcp1_735*dpt[3][31];
    RLcp1_237 = -S35*dpt[3][31];
    RLcp1_337 = ROcp1_935*dpt[3][31];
    RLcp1_138 = ROcp1_737*dpt[3][32];
    RLcp1_238 = ROcp1_837*dpt[3][32];
    RLcp1_338 = ROcp1_937*dpt[3][32];

    t_Left(0,3) = RLcp1_136+RLcp1_137+RLcp1_138;
    t_Left(1,3) = dpt[2][3]+dpt[2][29]+RLcp1_236+RLcp1_237+RLcp1_238;
    t_Left(2,3) = RLcp1_336+RLcp1_337+RLcp1_338;
    t_Left(3,3) = 1;
    t_Left(0,0) = ROcp1_139;
    t_Left(0,1) = ROcp1_239;
    t_Left(0,2) = ROcp1_339;
    t_Left(1,0) = ROcp1_438;
    t_Left(1,1) = ROcp1_538;
    t_Left(1,2) = ROcp1_638;
    t_Left(2,0) = ROcp1_739;
    t_Left(2,1) = ROcp1_839;
    t_Left(2,2) = ROcp1_939;

    // Sensor 3 // IMU

    t_IMU(0,3) = -dpt[1][4];
    t_IMU(1,3) = -dpt[2][4];
    t_IMU(2,3) = -Dz403;
    t_IMU(3,3) = 1;
    t_IMU(0,0) = 1;
    t_IMU(0,1) = 0;
    t_IMU(0,2) = 0;
    t_IMU(1,0) = 0;
    t_IMU(1,1) = 1;
    t_IMU(1,2) = 0;
    t_IMU(2,0) = 0;
    t_IMU(2,1) = 0;
    t_IMU(2,2) = 1;

    // Computations

    for (int i=0; i<3; i++)
    {
        position_v(i) = position[i];
    }
    position_v(3) = 1;

    Cmatrix t_Left_t = t_IMU*t_Left;
    Cmatrix t_Right_t = t_IMU*t_Right;
    Cmatrix p_left_v_s = t_Left_t*position_v;
    Cmatrix p_right_v_s = t_Right_t*position_v;
    Cmatrix r_Left_inv(3,3);
    Cmatrix r_Right_inv(3,3);

    for (int i=0; i<3; i++)
    {
        p_left_v(i) = p_left_v_s(i);
        p_right_v(i) = p_right_v_s(i);
        for (int j=0; j<3; j++)
        {
            r_Left_inv(i,j) = t_Left_t(i,j);
            r_Right_inv(i,j) = t_Right_t(i,j);
        }
    }

    Cmatrix cdPR(3,3);
    Cmatrix cd(3,3);
    cd(0,0) = rotOrient[0][0];
    cd(0,1) = rotOrient[0][1];
    cd(0,2) = rotOrient[0][2];
    cd(1,0) = rotOrient[1][0];
    cd(1,1) = rotOrient[1][1];
    cd(1,2) = rotOrient[1][2];
    cd(2,0) = rotOrient[2][0];
    cd(2,1) = rotOrient[2][1];
    cd(2,2) = rotOrient[2][2];

    double thy;

    if (((fabs(cd(0,2))-1.) >= -1e-15)  )
        thy = 0;
    else
        thy = atan2(-cd(0,1),cd(0,0)); //yaw

    Cmatrix cdy(3,3); // inverse of the yaw rotation matrix
    cdy(0,0) = cos(thy);
    cdy(0,1) = sin(thy);
    cdy(0,2) = 0;
    cdy(1,0) = -sin(thy);
    cdy(1,1) = cos(thy);
    cdy(1,2) = 0;
    cdy(2,0) = 0;
    cdy(2,1) = 0;
    cdy(2,2) = 1;

    cdPR = cdy*cd;

    // Correction on foot orientation was not applied

    p_left_v =  r_Left_inv*p_left_v;
    p_right_v = r_Right_inv*p_right_v;
    for (int i =0; i < 3; i++)
    {
        p_left_m[i] =  p_left_v(i);
        p_right_m[i] = p_right_v(i);
    }


}

