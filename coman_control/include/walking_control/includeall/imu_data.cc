// Eigen headers
#include </usr/include/eigen3/Eigen/Dense>
#include "imu_data.hh"

//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d
void imu_data::get_Orientation(double Trans [][3], double cdPR[][3], double Orientation[])
{
    Cmatrix cd(3,3);
    cd(0,0) = Trans[0][0];
    cd(0,1) = Trans[0][1];
    cd(0,2) = Trans[0][2];
    cd(1,0) = Trans[1][0];
    cd(1,1) = Trans[1][1];
    cd(1,2) = Trans[1][2];
    cd(2,0) = Trans[2][0];
    cd(2,1) = Trans[2][1];
    cd(2,2) = Trans[2][2];

    double thr,thp,thy,temp[10];

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

    Cmatrix cthp0(3,3); // inverse of the initial pitch rotation matrix (the imu is not completely upright!)
#ifdef REAL_ROBOT
    double thp0_cal = 0.10;
#endif
#ifndef REAL_ROBOT
    double thp0_cal = 0;
#endif

    cthp0(0,0) = cos(thp0_cal);
    cthp0(0,1) = 0;
    cthp0(0,2) = sin(thp0_cal);
    cthp0(1,0) = 0;
    cthp0(1,1) = 1;
    cthp0(1,2) = 0;
    cthp0(2,0) = -sin(thp0_cal);
    cthp0(2,1) = 0;
    cthp0(2,2) = cos(thp0_cal);

    Cmatrix cdPR_temp(3,3);
    cdPR_temp = cthp0*cdy*cd;
    cdPR[0][0] = cdPR_temp(0,0);
    cdPR[0][1] = cdPR_temp(0,1);
    cdPR[0][2] = cdPR_temp(0,2);
    cdPR[1][0] = cdPR_temp(1,0);
    cdPR[1][1] = cdPR_temp(1,1);
    cdPR[1][2] = cdPR_temp(1,2);
    cdPR[2][0] = cdPR_temp(2,0);
    cdPR[2][1] = cdPR_temp(2,1);
    cdPR[2][2] = cdPR_temp(2,2);

    if (((fabs(cdPR[0][2])-1.) >= -1e-15)  )
    {
        thr = atan2(cdPR[2][1],cdPR[1][1]);
        if ((cdPR[0][2] > 0.)  )
            temp[0] = 1.5707963267949;
        else
            temp[0] = -1.5707963267949;

        thp = temp[0];
        thr = 0.;
    }
    else
    {
        thr = atan2(-cdPR[1][2],cdPR[2][2]); //roll
        thp = asin(cdPR[0][2]); //pitch
    }
    Orientation[0] = thr;
    Orientation[1] = thp;
}

void imu_data::get_AngRates(double ImuAngRates[], double cdPR1[][3], double AngRates[])
{
    Cvector3 AngRatesTemp;
    AngRatesTemp(0) = ImuAngRates[0];
    AngRatesTemp(1) = ImuAngRates[1];
    AngRatesTemp(2) =ImuAngRates[2];
    Cmatrix cdPR(3,3);
    cdPR(0,0) = cdPR1[0][0];
    cdPR(0,1) = cdPR1[0][1];
    cdPR(0,2) = cdPR1[0][2];
    cdPR(1,0) = cdPR1[1][0];
    cdPR(1,1) = cdPR1[1][1];
    cdPR(1,2) = cdPR1[1][2];
    cdPR(2,0) = cdPR1[2][0];
    cdPR(2,1) = cdPR1[2][1];
    cdPR(2,2) = cdPR1[2][2];

    AngRatesTemp = cdPR*AngRatesTemp;
    AngRates[0] = AngRatesTemp[0];
    AngRates[1] = AngRatesTemp[1];
    AngRates[2] = AngRatesTemp[2];
}

void imu_data::get_Accelerations(double ImuAccelerations[], double cdPR1[][3], double Accelerations[])
{
    Cvector3 AccelerationsTemp;
    AccelerationsTemp(0) = ImuAccelerations[0];
    AccelerationsTemp(1) = ImuAccelerations[1];
    AccelerationsTemp(2) =ImuAccelerations[2];
    Cmatrix cdPR(3,3);
    cdPR(0,0) = cdPR1[0][0];
    cdPR(0,1) = cdPR1[0][1];
    cdPR(0,2) = cdPR1[0][2];
    cdPR(1,0) = cdPR1[1][0];
    cdPR(1,1) = cdPR1[1][1];
    cdPR(1,2) = cdPR1[1][2];
    cdPR(2,0) = cdPR1[2][0];
    cdPR(2,1) = cdPR1[2][1];
    cdPR(2,2) = cdPR1[2][2];

    AccelerationsTemp = cdPR*AccelerationsTemp;
    Accelerations[0] = AccelerationsTemp[0];
    Accelerations[1] = AccelerationsTemp[1];
    Accelerations[2] = AccelerationsTemp[2];
}



