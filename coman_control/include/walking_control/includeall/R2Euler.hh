#include </usr/include/eigen3/Eigen/Dense>
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;

void R2Euler(double trans[][3], double orientation[]){
    Cmatrix cd(3,3);
    cd.setZero(3, 3);
    cd(0,0) = trans[0][0];
    cd(0,1) = trans[0][1];
    cd(0,2) = trans[0][2];
    cd(1,0) = trans[1][0];
    cd(1,1) = trans[1][1];
    cd(1,2) = trans[1][2];
    cd(2,0) = trans[2][0];
    cd(2,1) = trans[2][1];
    cd(2,2) = trans[2][2];

    double thr = 0,thp = 0,thy = 0,temp = 0;

    if (((fabs(cd(0,2))-1.) >= -1e-15)  )
        thy = 0;
    else
        thy = atan2(-cd(0,1),cd(0,0)); //yaw


    Cmatrix cdy(3,3); // inverse of the yaw rotation matrix
    cdy.setZero(3, 3);
    cdy(0,0) = cos(thy);
    cdy(0,1) = sin(thy);
    cdy(0,2) = 0;
    cdy(1,0) = -sin(thy);
    cdy(1,1) = cos(thy);
    cdy(1,2) = 0;
    cdy(2,0) = 0;
    cdy(2,1) = 0;
    cdy(2,2) = 1;

    Cmatrix cdPR(3,3);
    cdPR.setZero(3, 3);
    cdPR = cdy*cd;
    

    if (((fabs(cdPR(0, 2))-1.) >= -1e-15)  )
    {
        thr = atan2(cdPR(2, 1),cdPR(1,1));
        if ((cdPR(0, 2) > 0.)  )
            temp = 1.5707963267949;
        else
            temp = -1.5707963267949;

        thp = temp;
        thr = 0.;
    }
    else
    {
        thr = atan2(-cdPR(1,2),cdPR(2,2)); //roll
        thp = asin(cdPR(0,2)); //pitch
    }
    orientation[0] = thr;
    orientation[1] = thp;
    orientation[2] = thy;
}