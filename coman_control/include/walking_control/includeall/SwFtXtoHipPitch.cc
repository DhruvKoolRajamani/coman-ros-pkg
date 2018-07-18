/* This functions takes the desired Swing Foot Position and the current
 * swing leg angles (qSw), the swSide (left or right is at stance, swSide = 1--> right foot is swing) and
 * returns the hip angles (qHipSwDes) and hip angular velocities (dqHipSwDes) that can yield swFtPosDes and
 * dswFtPosDes.
 */

#include <iostream>
#include "SwFtXtoHipPitch.hh"
#include "SwFtXerrJac.hh"

#include </usr/include/eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;


#define N_OF_ITERAIONS 20
#define EPSILON 0.001
#define EPS0 0.00001
#define ALPHA 1
#define N_C 1 // Number of constraints

//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d

void SwFtXtoHipPitch(double swFtXDes, double dswFtXDes, double qSw[], bool swSide, double qHipPitchSwDes[], double dqHipPitchSwDes[])
{

    double jac[N_C] = {0}, h[N_C] = {0}, swFtPos[3] = {0};
    double qM = 0;
    double q[6];
    int it = 0;
    double hNorm  = 100;

    for (int i = 0; i < 6; i++)
    {
        q[i] = qSw[i];
    }
    qM = q[0];
    while (hNorm > EPSILON && it < N_OF_ITERAIONS)
    {
        SwFtXerrJac(q, swFtXDes, swSide, h, jac, swFtPos);
        qM -= ALPHA * (1 / (jac[0] + EPS0)) * h[0];
        q[0] = qM;
        hNorm = h[0] < 0 ? -h[0] : (h[0]);
        it++;
    }
    dqHipPitchSwDes[0] = (1 / (jac[0] + EPS0)) * dswFtXDes;
    qHipPitchSwDes[0] = q[0];
}
