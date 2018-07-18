//Perform the update step of the Kalman Filter
//calculating the update vector D_x

#include </usr/include/eigen3/Eigen/Dense>
#include <math.h>


typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic > Cmatrix;
using namespace std;

void skew(double vector[3], double skew_matrix[3][3]);

void compute_matrix(double imuAngRates[3], double bias_orientation[3], double imuAcc[3], double bias_acceleration[3],
                    double rotOrient[3][3], const double p_left_w[3], const double p_right_w[3], const double position[3], double D_x[21],
                    double P[21][21], double Dy[6], double F_left, double F_right, double dt)
{

    //World to body rotation matrix
    Cmatrix rotOrient_m(3,3);

    //Process Covariance
    Cmatrix Qc;

    //Measurement Covariance
    Cmatrix R;

    //State error Jacobian
    Cmatrix F;

    //Output error Jacobian
    Cmatrix H;

    //Noise Jacobian
    Cmatrix Lc;

    //Error vector
    Cmatrix Dy_m(6,1);

    //State Covariance
    Cmatrix P_m(21,21);
    Cmatrix ID;

    Cmatrix pos_l_m(3,1);
    Cmatrix pos_r_m(3,1);
    Cmatrix accRotated_m(3,1);
    Cmatrix imuAcc_m(3,1);
    Cmatrix pos_l_temp(3,1);
    Cmatrix pos_r_temp(3,1);

    double skew_ang[3][3];
    double skew_pos_l[3][3];
    double skew_pos_r[3][3];
    double skew_acc[3][3];
    double pos_l[3];
    double pos_r[3];
    double accRotated[3];
    double imuAngRatescorr[3];

    //percentage of the weight sensed by each foot
    double force_left = F_left/(F_left+F_right);
    double force_right = F_right/(F_left+F_right);

    ID.setIdentity(21,21);
    Lc.setZero(21,21);
    H.setZero(6,21);
    F.setIdentity(21,21);
    R.setZero(6,6);
    Qc.setZero(21,21);

    // Set P and Dy to matrix //

    for(int i=0; i<21; i++)
    {
        for(int j=0; j<21;j++)
        {
            P_m(i,j) = P[i][j];
        }
    }

    for (int i=0; i<6; i++)
    {
        Dy_m(i) = Dy[i];
    }

    // -------------------- //

    // Set CoVariance given by side variable //

    if (force_left<0.45)
    {
        for (int i = 0 ; i < 3 ; i++)
        {
            for (int j = 0 ; j<3 ; j++)
            {
                if (i==j)
                {
                    Qc(9+i,9+i) = pow(10,6);
                    R(i,j) = pow(10,6)/dt;
                }
            }
        }
    }
    else
    {
        for (int i = 0 ; i < 3 ; i++)
        {
            for (int j = 0 ; j<3 ; j++)
            {
                if (i==j)
                {
                    Qc(9+i,9+i) = pow(10,-6);
                    R(i,j) = pow(10,-4)/dt;
                }
            }
        }
    }
    if (force_right<0.45)
    {
        for (int i = 0 ; i < 3 ; i++)
        {
            for (int j = 0 ; j<3 ; j++)
            {
                if (i==j)
                {
                    Qc(12+i,12+i) = pow(10,6);
                    R(3+i,3+j) = pow(10,6)/dt;
                }
            }
        }
    }
    else
    {
        for (int i = 0 ; i < 3 ; i++)
        {
            for (int j = 0 ; j<3 ; j++)
            {
                if (i==j)
                {
                    Qc(12+i,12+i) = pow(10,-6);
                    R(3+i,3+j) = pow(10,-4)/dt;
                }
            }
        }
    }


    // ---------------------------------- //

    // Calculate useful variables //

    for (int i=0; i<3; i++)
    {
        imuAngRatescorr[i] = imuAngRates[i] - bias_orientation[i];
        imuAcc_m(i) = imuAcc[i] - bias_acceleration[i];
        pos_l_temp(i) = p_left_w[i] - position[i];
        pos_r_temp(i) = p_right_w[i] -  position[i];
        for (int j=0; j<3; j++)
        {
           rotOrient_m(i,j) = rotOrient[i][j];
        }
    }

    accRotated_m = rotOrient_m.transpose()*imuAcc_m;
    pos_l_m = rotOrient_m*pos_l_temp;
    pos_r_m = rotOrient_m*pos_r_temp;

    for (int i=0; i<3; i++)
    {
        accRotated[i] = accRotated_m(i);
        pos_l[i] = pos_l_m(i);
        pos_r[i] = pos_r_m(i);
    }

    skew(imuAngRatescorr,skew_ang); //ok
    skew(pos_l, skew_pos_l);
    skew(pos_r, skew_pos_r);
    skew(accRotated, skew_acc); //ok

    // -------------------------------- //

    Cmatrix skM(3,3);
    for (int i(0); i < 3; i++){
        for (int j(0); j < 3; j++){
            skM(i, j) = skew_pos_l[i][j];
        }
    }

    Cmatrix zVec;
    unsigned int nRow = skM.rows();
    zVec.setZero(nRow, 1);
    Cmatrix xVec = skM.colPivHouseholderQr().solve(zVec);


    // Compute F H and L matrix //


    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3;j++)
        {
            F(6+i,6+j) = - skew_ang[i][j] * dt; //ok
            F(3+i,6+j) = skew_acc[i][j]*dt; //ok
            F(3+i,15+j)= - rotOrient[j][i] * dt; //ok

            H(i,j) = -rotOrient[i][j]; //ok
            H(3+i,j) = -rotOrient[i][j]; //ok
            H(i,9+j) = rotOrient[i][j];  //ok
            H(3+i, 12+j) = rotOrient[i][j]; //ok
            H(i, 6+j) = skew_pos_l[i][j];//ok check
            H(3+i, 6+j) = skew_pos_r[i][j];//ok check

            Lc(3+i,3+j) = -rotOrient[j][i]; //ok
            Lc(9+i,9+j) = rotOrient[j][i]; //ok
            Lc(12+i,12+j) = rotOrient[j][i]; //ok

            if (i==j)
            {
                Qc(3+i,3+j) = 6.084e-7;
                Qc(6+i,6+j) = 2.73529e-7;
                Qc(15+i,15+j) = 1e-8;
                Qc(18+i,18+j) = 3.81924e-7;

                F(i,3+j) = dt; //ok
                F(i,j) = 1; //ok
                F(3+i,3+j) = 1; //ok
                F(6+i,6+j) = 1; //ok
                F(9+i,9+j) = 1; //ok
                F(12+i,12+j) = 1; //ok
                F(15+i,15+j) = 1; //ok
                F(18+i,18+j) = 1;
//                F(6+i,18+j)=-dt;

                Lc(6+i,6+j) = -1;
                Lc(15+i,15+j) = 1;
                Lc(18+i,18+j) = 1;
            }
        }
    }

    // Update step


    Cmatrix LcTrans = Lc.transpose(), FTrans = F.transpose(), HTrans = H.transpose();
    Cmatrix Qk = F*Lc*Qc*LcTrans*FTrans;
    Cmatrix P_m_t = F*P_m*FTrans+Qk;
    Cmatrix S_m = H*P_m_t*HTrans+R;
    Cmatrix S_t = S_m.transpose();
    Cmatrix P_m_tTrans = P_m_t.transpose();
    Cmatrix K_t = S_t.colPivHouseholderQr().solve(H*P_m_tTrans);
    Cmatrix K = K_t.transpose();
    Cmatrix D_x_m = K*Dy_m;
    Cmatrix P_m_temp = (ID-K*H)*P_m_t;

    for (int i=0; i<21; i++)
    {
        D_x[i] = D_x_m(i);
        for (int j=0; j<21; j++)
        {
            P[i][j] = P_m_temp(i,j);
        }
    }


}
