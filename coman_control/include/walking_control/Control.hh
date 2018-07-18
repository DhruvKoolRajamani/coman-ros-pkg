// Eigen headers
#include </usr/include/eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ctime>

#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;
//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;

#include "AvgFilter.hh"
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
    void LowerBody(double tm, double *Q0, double *qSens,  double *qSensAbs,  double *dqSens, double *tauSens, double *forceRightAnkle,\
                   double *forceLeftAnkle,  double *torqueRightAnkle,  double *torqueLeftAnkle,  double *forceRightHand,\
                   double *forceLeftHand,  double trans[][3],  double *imuAngRates,  double *imuAccelerations,\
                   double *h,  double *dh,  double *hD,  double *dhD,  double *tauDes,  double *vals);
    //Eigen::ArrayXXf outputtest;

    struct state_vars {
    //        std::ofstream &outputFile;
             double tm_;
             unsigned int n_;
             double *qSens_ = {0};
             double *qSensAbs_;
             double *dqSens_;
             double *pPelvis_;
             double *pPelvisAbs_;
             double *vPelvis_;
             double *vPelvisAbs_;
             double *aPelvis_;
             double *forceRightAnkle_;
             double *forceLeftAnkle_;
             double *torqueRightAnkle_;
             double *torqueLeftAnkle_;
             double *forceRightHand_;
             double *forceLeftHand_;
             double *orSwFt_;
             double *dorSwFt_;
             double *pSwFtInH_;
             double *angRates_;
             double *h_;
             double *hD_;
             double *pPelvisFK_;
             double *pPelvisTest_;
             double *vPelvisFk_;
             double *tauSens_;
             double *tauDes_;
             double *tauAnkTorque_;
             double k_;
             double s_;
             double tInStep_;
             double deltaX_;
             double deltaY_;
             double thp_;
             double thpF_init_;
             double thr_;
             unsigned int side_;
             double vxFK_;
             double kv_;
             double px0_;
             double kOrg_;
             double vxDes_;
             double x0_;
             double T_;
             double trans_[3][3];
             double *imuAngRates_;
             double *imuAccelerations_;
             double kR_;
             double kL_;
             double avgFreq_;
             double f0_;
             double last_step_freq_;
             double frontalBias_;
             double vxAbsF_;
             double *qSensAbsMed_;
             double *pPelvisAbsMed_;
            double handRightPos_;
            double forceSensors_;
            double *qSt_;
            int *indexSt_;
            int inCommands_;
            double handRightVelFilt_;
            double velRel_;
            double forceDiff_;
            double velSimpleThreshold_;
            double pxAbsOld_;
            double *pPelvisAbsF_;
            std::vector<double> velTimeWind_;
            state_vars(): tm_(),n_(), qSens_(), qSensAbs_(), dqSens_(), pPelvis_(), pPelvisAbs_(), vPelvis_(), vPelvisAbs_(), aPelvis_(), forceRightAnkle_(), forceLeftAnkle_(), torqueRightAnkle_(), \
                torqueLeftAnkle_(), forceRightHand_(), forceLeftHand_(),orSwFt_(), dorSwFt_(), pSwFtInH_(), angRates_(), h_(), hD_(), pPelvisFK_(), pPelvisTest_(), vPelvisFk_(), tauSens_(),\
                tauDes_(), tauAnkTorque_(), k_(), s_(), tInStep_(), deltaX_(), deltaY_(), thp_(), thpF_init_(),thr_(), side_(), vxFK_(), kv_(),px0_(),kOrg_(), vxDes_(), x0_(), T_(), trans_(),\
                imuAngRates_(),imuAccelerations_(),kR_(),kL_(),avgFreq_(),f0_(),last_step_freq_(),frontalBias_(),vxAbsF_(),qSensAbsMed_(),pPelvisAbsMed_(),handRightPos_(),forceSensors_(),inCommands_(),\
                handRightVelFilt_(), qSt_(), indexSt_(), velRel_(), forceDiff_(), velSimpleThreshold_(), velTimeWind_(), pxAbsOld_(), pPelvisAbsF_(){}
    };

    state_vars varsOut;
    //    state_vars lb_vars;
    void SaveVars(std::ofstream &outputFile);

    const double EPSILON = 0.000001;
    unsigned int whichComan_ = 0;

};

