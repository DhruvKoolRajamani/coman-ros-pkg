#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ctime>

#include </usr/include/eigen3/Eigen/Dense>

using namespace Eigen;
//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , 
                        Eigen::Dynamic , 
                        Eigen::Dynamic , 
                        Eigen::RowMajor > Cmatrix;

#include "AvgFilter.hh"
#include "includeall/state_vars.hh"
#include "init_pos.hh"
#define NUM 31
//! The class of Cmatrix abreviated from Eigen
//! The class of Cvector abreviated from Eigen VectorXd
#define Cvector Eigen::VectorXd
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d
#define Cvector4 Eigen::Vector4d
#define zero_v3 Cvector3(0.0,0.0,0.0)

#define M 50
#define M2 10
#define M3 15
#define M4 10
#define M5 5
#define AirTresh 50
#define LEFT 0
#define RIGHT 1
#define LOWER_BODY_N 15
#define UPPER_BODY_N 16



class Control
{

public:
    void LowerBody( double tm, double *Q0, double *qSens,  double *qSensAbs,
                    double *dqSens, double *tauSens, double *forceRightAnkle,
                    double *forceLeftAnkle,  double *torqueRightAnkle,  
                    double *torqueLeftAnkle,  double *forceRightHand, 
                    double *forceLeftHand,  double trans[][3],  double *imuAngRates, 
                    double *imuAccelerations, double *h,  double *dh,  double *hD,  
                    double *dhD,  double *tauDes,  double *vals, double DTm, double *euler );
    //Eigen::ArrayXXf outputtest;

    state_vars varsOut;
    //    state_vars lb_vars;
    void SaveVars(std::ofstream &outputFile);

    const double EPSILON = 0.000001;
    unsigned int whichComan_ = 1;

};

