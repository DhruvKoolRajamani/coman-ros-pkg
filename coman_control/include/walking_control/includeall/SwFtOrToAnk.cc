/* This functions takes the desired Swing Foot orientation and the current
 * swing leg angles (qSw), the swSide (left or right is at stance, swSide = 1--> right foot is swing) and
 * returns the Ankle and Swing leg yaw angles (qDes) and their angular velocities (dqDes) that can yield swFtOrDes and
 * dswFtOrDes.
 */

#include <iostream>
#include "SwFtOrErrJac.hh"
#include "SwFtOrToAnk.hh"

#include </usr/include/eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;


#define N_OF_ITERAIONS 20
#define EPSILON 0.001
#define ALPHA 1
#define N_C 3 // Number of constraints

//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d

void SwFtOrToAnk(double swFtOrDes[], double dswFtOrDes[], double qSw[], bool swSide, double qDes[], double dqDes[])
{

    double jac[N_C][N_C], h[N_C];
    Cmatrix jacM(N_C, N_C), jacInvM(N_C, N_C);
    Cmatrix hM(N_C, 1), dswFtOrDesM(N_C, 1), qM(N_C, 1);
    Cmatrix dqDesM(N_C, 1);
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
        dswFtOrDesM(i, 0) = dswFtOrDes[i];
    }
    qM(0, 0) = q[2];
    qM(1, 0) = q[4];
    qM(2, 0) = q[5];

    while (hNorm > EPSILON && it < N_OF_ITERAIONS)
    {
        SwFtOrErrJac(q, swFtOrDes, swSide, h, jac);
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

        q[2] = qM(0, 0);
        q[4] = qM(1, 0);
        q[5] = qM(2, 0);

        hNorm = hM.norm();
     //  cout << it << " " << hNorm << endl;
        it++;
    }

    dqDesM = jacM.jacobiSvd(ComputeThinU | ComputeThinV).solve(dswFtOrDesM);
    for (int i = 0; i < N_C; i ++)
    {
        qDes[i] = qM(i, 0);
        dqDes[i] = dqDesM(i, 0);
    }   
}
