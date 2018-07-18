/* This functions takes the desired Swing Foot Position and the current
 * swing leg angles (qSw), the swSide (left or right is at stance, swSide = 1--> right foot is swing) and
 * returns the hip angles (qHipSwDes) and hip angular velocities (dqHipSwDes) that can yield swFtPosDes and
 * dswFtPosDes.
 */

#include <iostream>
#include "SwFtPosToHip.hh"
#include "SwFtErrJac.hh"
#include "SwFtErrJac2.hh"

#include </usr/include/eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;


#define N_OF_ITERAIONS 20
#define EPSILON 0.001
#define ALPHA 1
#define N_C 2 // Number of constraints

//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d

void SwFtPosToHip(double swFtPosDes[], double dswFtPosDes[], double qSw[], bool swSide, double qHipSwDes[], double dqHipSwDes[])
{

    double jac[N_C][N_C], h[N_C], swFtPos[3];
    Cmatrix jacM(N_C, N_C), jacInvM(N_C, N_C);
    Cmatrix hM(N_C, 1), dswFtPosDesM(N_C, 1), qM(N_C, 1);
    Cmatrix dqHipSwDesM(N_C, 1);
    Cmatrix jacInvMhM(N_C, 1);
    double q[6];
    int it = 0;
    double hNorm  = 100;

    for (int i = 0; i < 6; i++)
    {
        q[i] = qSw[i];
    }
    for (int i = 0; i < N_C; i++)
    {
        for (int j = 0; j < N_C; j++)
        {
            jacM(i, j) = 0;
            jacInvM(i, j) = 0;
            jac[i][j] = 0;
        }
        hM(i, 0) = 0;
        qM(i, 0) = q[i];
        dswFtPosDesM(i, 0) = dswFtPosDes[i];
    }

    while (hNorm > EPSILON && it < N_OF_ITERAIONS)
    {
        SwFtErrJac2(q, swFtPosDes, swSide, h, jac, swFtPos);
        for (int i = 0; i < N_C; i++)
        {
            for (int j = 0; j < N_C; j++)
            {
                jacM(i, j) = jac[i][j];
            }
            hM(i, 0) = h[i];
        }

        //jacInvM = jacM.inverse(); // This is an alternative approach, though when the jacM is not invertible it could be problamatic.
        jacInvMhM = jacM.jacobiSvd(ComputeThinU | ComputeThinV).solve(hM);
        qM -= ALPHA * jacInvMhM;
        for (int i = 0; i < N_C; i++)
        {
            q[i] = qM(i, 0);
        }
        hNorm = hM.norm();
        it++;
    }

    dqHipSwDesM = jacM.jacobiSvd(ComputeThinU | ComputeThinV).solve(dswFtPosDesM);
    for (int i = 0; i < N_C; i ++)
    {
        qHipSwDes[i] = q[i];
        dqHipSwDes[i] = dqHipSwDesM(i, 0);
    }
    /*
    for (int i = 0; i < N_C; i++)
    {
        cout << qHipSwDes[i] << " ";
    }
    cout << endl;
    */
}
