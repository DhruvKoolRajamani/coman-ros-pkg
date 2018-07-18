#include <iostream>
#include </usr/include/eigen3/Eigen/Dense>
#include "StFtToPelvisJac.hh"

using namespace std;
using namespace Eigen;
//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;

void StFtToPelvisFK(double q[], double dq[], bool stSide, double pPelvis[], double vPelvis[])
{
    Cmatrix jacM(3, 6), dqM(6, 1), vPelvisM(3, 1);
    double jac[3][6] = {0};
    StFtToPelvisJac(q, stSide, pPelvis, jac);
    for (int j = 0; j < 6; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            jacM(i, j) = jac[i][j];
        }
        dqM(j, 0) = dq[j];
    }
    vPelvisM = jacM * dqM;
    for (int i = 0; i < 3; i++)
    {
        vPelvis[i] = vPelvisM(i, 0);
    }
}
