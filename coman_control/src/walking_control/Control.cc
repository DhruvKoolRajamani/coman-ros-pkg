#include "Control.hh"
#include <numeric>
#include "WhichSide.hh"
#include "forward_kinematics_pelvis1.hh"
#include "forward_kinematics_swing_foot.hh"
#include "imu_data.hh"
#include "bezier.hh"
#include "saturate.hh"
//#include "init_pos.hh"
#include "WalkingController3.hh"
#include <math.h>
#include "StFtToPelvisFK.hh"
#include "SwapArrays.hh"
#include "EraseVectors.hh"
#include "DesiredFtPos.hh"
//#include "ControlLowerBody.hh"
//#include "AvgFilter.hh" to add
#include "MedianFilter.h"
#include "StackAsVector.hh"
#include "state_estimation.h"
#include <climits>
#include "R2Euler.hh"

#include <fstream>

// Eigen headers
#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
//! The class of Cmatrix abreviated from Eigen
typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
//! The class of Cvector abreviated from Eigen VectorXd
#define Cvector Eigen::VectorXd
//! The class of Cvector3 abreviated from Eigen Vector3d
#define Cvector3 Eigen::Vector3d
#define Cvector4 Eigen::Vector4d
#define zero_v3 Cvector3(0.0,0.0,0.0)
#define N 31
#define N_OF_ITERATIONS 10
#define EPSILON 0.001
#define ALPHA 1
#define N_C 3
#define N_J 23
#define ALPHA_ 1



static double qknee0 = 0.2, qP0 = -0.1, qR0 = 0.055*1;
static double RIGHT_ELBOW_YAW = 0, LEFT_ELBOW_YAW = 0;
static double Q_INIT[N] = {0,0.075,0,qP0,qP0,-qR0,0,qknee0,qP0*1.4,qR0,qR0*1,0,qknee0,qP0*1.4,-qR0*1, 0.45, -0.2 , 0.0, -1.75, 0.45, 0.2, 0.0, -1.75, RIGHT_ELBOW_YAW,0.0,0.0,LEFT_ELBOW_YAW,0,0,0,0};
static int count1 = 0, count2 = 0, count3 = 0, count4 = 0, count5 = 0;


// Lower-body variables
double pxDes, kR, kL, forceRightAnkleFx;
double pPelvis[3], vPelvis[3];
double pSwFtInH[3], vSwFtInH[3];
double orSwFt[3], dorSwFt[3];
double qSt[6], qStAbs[6], qStAbsMed[6], dqSt[6], qSw[6], qSensAbsMed[N], tauAnkTorque[2];
double imuOrientation[2], angRates[3], aPelvis[3], cdPR[3][3], thr, thp;
double pPelvisTest[3], pPelvisAbs[3], pPelvisAbsMed[3], vPelvisAbs[3], pPelvisFK[3], vPelvisFk[3];
double t, s, tInStep, tmeTemp;
double kv, x0, vxDes, vxOld;
double dVx= 0; //to check
std::vector<double> qSensAbsVec[N];
double lM;
double forceRightAnkleF[3], forceLeftAnkleF[3], torqueRightAnkleF[3], torqueLeftAnkleF[3];
double kRaw;
std::vector<double> pyMidVec;
double thPelvis[3], swFtPos[3], dswFtPos[3];
double TIME_WALK = 10, T = 0.5, ZERO_S = 0.01, TIME_REWALK = 10, DTm;

imu_data imuData;
static double  pyEnd;
WalkingController3 walkingController3;

#ifndef REAL_ROBOT
static double f_d = 2;
#else
static double f_d = 0;
#endif

static bool flagStopCommand = false;
static double f0AtStop = f_d;
static double timeAtStop = 0, f_dAtStop = f_d;
static double timeInAir = 0;
static double STEP_LENGTH = 0.05; // 0.07
static bool flagStart = false;
static int sg = -1;
static int stepNumber = 1;
static int indexSt[6] = {4,10,11,12,13,14}, indexSw[6] = {3,5,6,7,8,9};
static unsigned int side  = 0, oldSide = side;
static double qSensInit[N];
static double thpF, thrF, thpF_init, thrF_init, thPelvisInit[3], swFtPosInit[3], orSwFtInit[3];
static double px, px0, py, pxswf, pyswf, vx, vy, vxswf, vyswf, ax,  px_old, py_old, pxswf_old, pyswf_old, dthpF, dthrF, swFtPitch, swFtPitch_old, dswFtPitch, swFtRoll, swFtRoll_old, dswFtRoll;
static double k_vec[M5], sumk, forceLeftAnkleZMed, forceRightAnkleZMed;
static double vec_px[M], vec_py[M], vec_pxswf[M5], vec_pyswf[M5], vec_swFtPitch[M3], vec_swFtRoll[M3];
static double k, pxAbs, pxAbsMed, pyAbs, vxAbs, vxAbsF, vyAbs, pxAbs_old, pyAbs_old, vec_pxAbs[M], vec_pyAbs[M], vxFK, avgFreq = f_d;
static double deltaX = 0, deltaY = 0;
static double t0 = 0;
static int inAir = 0;
static bool flagMidStep = false;
static double frontalBias = 0;
static double QswKMid_INIT = 0.8, QswKMid = QswKMid_INIT;
static int firstStopN = 25, stepsToStop = 3;
std::vector<double> freqVec {f_d, f_d};
static double SIDE_INITIAL = 0;
static double f0 = 0;
static bool startWalkingFlag = 0;
static double last_step_freq = f0;
//double handRightPosFilt; // to check if the value persists

static double pPelvisAbsF[3];

// State Estimation-only variables
std::vector<double> accVec[3];
static double avgAcc[3] = {0, 0, 0}, avgOr[3] = {0}, meanAccNorm = 0, position[3] = {0}, p_left_w[3] = {0}, p_right_w[3] = {0}, tm_1, variance[3], P[21][21] = {0}, velocity[3] = {0};
static double biasAcceleration[3] = {0.0011,0.0120,-0.0016};
static double biasOrientation[3] = {0.00662207,0.0109228,-0.00523774};
static double rotOrient[3][3] = {
    {1,0,0},
    {0,1,0},
    {0,0,1}
};
static double stateEstOrientation[4] = {0,0,0,1};

static double p_pelvis_left[3];
static double p_pelvis_right[3];
static double velocity_straight[3];


void Control::LowerBody( double tm, double *Q0, double *qSens, double *qSensAbs, double *dqSens, double *tauSens, double *forceRightAnkle, double *forceLeftAnkle, double *torqueRightAnkle, double *torqueLeftAnkle, double *forceRightHand, double *forceLeftHand, double trans[][3], double *imuAngRates, double *imuAccelerations, double *h, double *dh, double *hD, double *dhD, double *tauDes, double *vals)
{
    StackAsVector(tmVec, tm, M);
    
    DTm = 0.001;

    // Which Coman
    walkingController3.whichComan_ = whichComan_;

    // Convert IMU frame data to global frame data.
    
    double testOrientation[3];
    R2Euler(trans, testOrientation);
    thr = testOrientation[0]; // 0
    thp = testOrientation[1]; // 1
    double thy = testOrientation[2];
    // std::cout<< thp << " : " << thr << " : " << thy << std::endl;

    // imuData.get_Orientation(trans, cdPR, imuOrientation);
    // imuData.get_AngRates(imuAngRates, cdPR, angRates);
    // imuData.get_Accelerations(imuAccelerations, cdPR, aPelvis);
    // thr = imuOrientation[0]; // 0
    // thp = imuOrientation[1]; // 1
    // std::cout<< thp << " : " << thr << std::endl;

    thPelvis[0] = thp;
    thPelvis[1] = thr;
    for (int i = 0; i < N; i++){
        MedianFilter(qSensAbsVec[i], qSensAbsMed[i], qSensAbs[i], M2);
    }
    forward_kinematics_pelvis1(qSt, pPelvis, trans, side);
    forward_kinematics_pelvis1(qSt, pPelvisTest, trans, side);
    forward_kinematics_swing_foot(qSw, pSwFtInH, orSwFt, trans, 1-side);
    forward_kinematics_pelvis1(qStAbs, pPelvisAbs, trans, side);
    forward_kinematics_pelvis1(qStAbsMed, pPelvisAbsMed, trans, side);
    StFtToPelvisFK(qStAbs, dqSt, side, pPelvisFK, vPelvisFk);

    lM = tmVec.size() == 1 ? 1 : tmVec.back() - tmVec[0];
    // lM = vecPxAbs.size() == 0 ? 1 : (DTm * vecPxAbs.size());
    AvgFilter(pxVec, px, pPelvis[0], M);
    AvgFilter(pxAbsVec, pxAbs, pPelvisAbs[0], M);
    AvgFilter(pxAbsMedVec, pxAbsMed, pPelvisAbsMed[0], M);
    pxAbs = pxAbsMed;
    AvgFilter(pyVec, py, pPelvis[1], M);
    AvgFilter(pyAbsVec, pyAbs, pPelvisAbs[1], M);
    AvgFilter(pxswfVec, pxswf, pSwFtInH[0], M4);
    AvgFilter(pyswfVec, pyswf, pSwFtInH[1], M4);
    AvgFilter(vxVec, vx, (px-px_old)/(DTm*M), M2);
    AvgFilter(vxAbsVec, vxAbs, (pxAbs-pxAbs_old)/lM, M2);
    AvgFilter(vxAbsVecF, vxAbsF, (pxAbs-pxAbs_old)/lM, M);
    AvgFilter(vxFkVec, vxFK, vPelvisFk[0], M4);
    AvgFilter(vyVec, vy, (py-py_old)/(DTm*M), M2);
    AvgFilter(vyAbsVec, vyAbs, (pyAbs-pyAbs_old)/(DTm*M), M2);
    AvgFilter(vyswfVec, vyswf, (pyswf-pyswf_old)/(DTm*M5), M5);
    AvgFilter(vxswfVec, vxswf, (pxswf-pxswf_old)/(DTm*M5), M5);
    AvgFilter(axVec, ax, aPelvis[0], M4);
    AvgFilter(thpFVec, thpF, thp, M5);
    AvgFilter(dthpFVec, dthpF, angRates[1], M5);
    AvgFilter(thrFVec, thrF, thr, M5);
    AvgFilter(dthrFVec, dthrF, angRates[0], M5);
    AvgFilter(swFtPitchVec, swFtPitch, orSwFt[1], M4);
    AvgFilter(dswFtPitchVec, dswFtPitch, (swFtPitch-swFtPitch_old)/(DTm*M3), M2);
    AvgFilter(swFtRollVec, swFtRoll, orSwFt[0], M4);
    AvgFilter(dswFtRollVec, dswFtRoll, (swFtRoll-swFtRoll_old)/(DTm*M3), M2);

    for (int i(0); i < 3; i++)
    {
        AvgFilter(frcRVec[i], forceRightAnkleF[i], forceRightAnkle[i], M5);
        AvgFilter(frcLVec[i], forceLeftAnkleF[i], forceLeftAnkle[i], M5);
        AvgFilter(trqRVec[i], torqueRightAnkleF[i], torqueRightAnkle[i], M5);
        AvgFilter(trqLVec[i], torqueLeftAnkleF[i], torqueLeftAnkle[i], M5);
    }

    for (int i(0); i < 3; i++)
    {
        AvgFilter(pPelvisAbsVec[i], pPelvisAbsF[i], pPelvisAbsMed[i], M);
        StackAsVector(pPelvisAbsFVec[i], pPelvisAbsF[i], M);
    }

    // state_estimation(accVec, imuAccelerations, imuAngRates, avgAcc, biasAcceleration, variance, meanAccNorm, rotOrient, position, p_left_w, p_right_w, stateEstOrientation, tm, tm_1, biasOrientation, velocity, P, velocity_straight, p_pelvis_left, p_pelvis_right, forceLeftAnkle[2], forceRightAnkle[2], qSensAbsMed);

    unsigned int M_FORCE_FILT;
    double K_FXY;

    if (whichComan_ == 1){
        M_FORCE_FILT = 1;
        K_FXY = 1;
    }
    else{
        M_FORCE_FILT = 2 * M2;
        K_FXY = 0; // this coefficient is to make the effect of FX and FY of the ankle force sensors zero in COMAN2 (IIT coman) because they are very noisy with very high spikes
    }

    static double kF;
    double forceLeftAnkleZ = forceLeftAnkleF[2];
    double forceRightAnkleZ = forceRightAnkleF[2];
    MedianFilter(forceLeftAnkleZVec, forceLeftAnkleZMed, forceLeftAnkleZ, M_FORCE_FILT); // coman 2
    MedianFilter(forceRightAnkleZVec, forceRightAnkleZMed, forceRightAnkleZ, M_FORCE_FILT); // coman 2
    double f1 = pow((pow(K_FXY*forceRightAnkle[0],2)+pow(K_FXY*forceRightAnkle[1],2)+pow(forceRightAnkleZ,2)),0.5); // coman 2
    double f2 = pow((pow(K_FXY*forceLeftAnkle[0],2)+pow(K_FXY*forceLeftAnkle[1],2)+pow(forceLeftAnkleZ,2)),0.5); // coman 2
    kRaw = f1 / (f1 + f2 + EPSILON);

    AvgFilter(kVec, kF, kRaw, M3);
    // kF = 0.5 * cos(2 * M_PI * (tm + 0.25)) + 0.5;
    kR = kF;
    kL = 1 - kF;
    if (f_d == 0){
        kR = 1;
        kL = 1;
    }

    dorSwFt[1] = tInStep > DTm * M4 ? dswFtPitch : 0;
    thp = thpF;
    thr = thrF;
    orSwFt[1] = swFtPitch;
    thPelvis[0] = thp;
    thPelvis[1] = thr;
    pSwFtInH[0] = pxswf;
    pSwFtInH[1] = pyswf;

    count1 = (count1+1)%M;
    count2 = (count2+1)%M2;
    count3 = (count3+1)%M3;
    count4 = (count4+1)%M4;
    count5 = (count5+1)%M5;
    px_old = vec_px[count1];
    vec_px[count1] = px;
    pxAbs_old = vec_pxAbs[count1];
    if (vecPxAbs.size() < M){
        vecPxAbs.push_back(pxAbs);
        pxAbs_old = vecPxAbs[0];
    }
    else{
        pxAbs_old = vecPxAbs[count1];
        vecPxAbs[count1] = pxAbs;
    }
    vec_pxAbs[count1] = pxAbs;
    py_old = vec_py[count1];
    vec_py[count1] = py;
    pyAbs_old = vec_pyAbs[count1];
    vec_pyAbs[count1] = pyAbs;
    pxswf_old = vec_pxswf[count5];
    vec_pxswf[count5] = pxswf;
    pyswf_old = vec_pyswf[count5];
    vec_pyswf[count5] = pyswf;
    swFtPitch_old = vec_swFtPitch[count3];
    vec_swFtPitch[count3] = swFtPitch;
    swFtRoll_old = vec_swFtRoll[count3];
    vec_swFtRoll[count3] = swFtRoll;

    pxAbs_old = pPelvisAbsFVec[0][0];
    pyAbs_old = pPelvisAbsFVec[1][0];
//  pxswf_old = pxswfVec[0];
//  pyswf_old = pyswfVec[0];
//  swFtPitch_old = swFtPitchVec[0];
//  swFtRoll_old = swFtRollVec[0];

    pPelvis[0] = px;
    pPelvis[1] = py;
    vPelvis[0] = vx;
    vPelvis[1] = vy;
    pPelvisAbs[0] = pxAbs;
    pPelvisAbsMed[0] = pxAbsMed;
    pPelvisAbs[1] = pyAbs;
    vPelvisAbs[0] = vxAbs;
    vPelvisAbs[1] = vyAbs;

//  if (side == 1){
//      pPelvisAbs[0] = p_pelvis_right[0] + 0.07;
//      pxAbs = p_pelvis_right[0] + .07;
//  }
//  else{
//      pPelvisAbs[0] = p_pelvis_left[0] + 0.07;
//      pxAbs = p_pelvis_left[0] + 0.07;
//  }

    const double H = 0.05;
    double xCopR = forceRightAnkleF[2] < 50 ? -(torqueRightAnkleF[1] + forceRightAnkleF[0] * H) / 50 : -(torqueRightAnkleF[1] + forceRightAnkleF[0] * H) / forceRightAnkleF[2];
    double xCopL = forceLeftAnkleF[2] < 50 ? -(torqueLeftAnkleF[1] + forceLeftAnkleF[0] * H) / 50 :  -(torqueLeftAnkleF[1] + forceLeftAnkleF[0] * H) / forceLeftAnkleF[2];

#ifndef REAL_ROBOT
t = tm - TIME_WALK;
double tw = 0;
if (t >= TIME_WALK && t <= TIME_WALK + 0.1){
    f_d = 1 / T;
    tw = t - TIME_WALK;
    f0 = tw <= 0.1 ? 10 * tw * f_d : f_d; // fast but continuously go to f_d from f0  = 0;
}
// if tm > 2 * TIME_WALK start walking
if (t >= TIME_WALK && startWalkingFlag == 0){
    startWalkingFlag = 1;
    for (int i = 0; i < N; i++){
        qSensInit[i] = qSens[i];
    }
    for (int i = 0; i < 3; i++){
        thPelvisInit[i] = thPelvis[i];
        swFtPosInit[i] = pSwFtInH[i];
        orSwFtInit[i] = orSwFt[i];
    }
    thpF_init = thpF;
    thrF_init = thrF;
    px0 = pxAbs;
}


if (flagMidStep == false && vyAbs > 0){
    pyMidVec.push_back(pyAbs);
    if (pyMidVec.size() > 2){
        pyMidVec.erase(pyMidVec.begin());
        frontalBias = (pyMidVec[0] + pyMidVec[1]) / 2;
    }
    flagMidStep = true;

}

static double begsteptime = TIME_WALK;
if (side != oldSide){
    begsteptime = tm;
    ++stepNumber;
    SwapArrays(indexSt, indexSw, 6);
    EraseVectors();
    flagMidStep = false;
    QswKMid = QswKMid_INIT;
}
tInStep = tm - begsteptime;
for (int i = 0; i < 6; i++){
    qSt[i] = qSens[indexSt[i]];
    dqSt[i] = dqSens[indexSt[i]];
    qStAbs[i] = qSensAbs[indexSt[i]];
    qStAbsMed[i] = qSensAbsMed[indexSt[i]];
    qSw[i] = qSens[indexSw[i]];
}
// Find initial values at the beginning of the step
if (tInStep < ZERO_S){
    for (int i = 0; i < N; i++){
        qSensInit[i] = qSens[i];
    }
    for (int i = 0; i < 3; i++){
        thPelvisInit[i] = thPelvis[i];
        swFtPosInit[i] = pSwFtInH[i];
        orSwFtInit[i] = orSwFt[i];
    }
    thpF_init = thpF;
    thrF_init = thrF;
    px0 = pxAbs;
}

// Determine the stance side and sign (sg)
oldSide = side;
WhichSide(kF, s, side, sg);
if (side != oldSide){
    //            if (stepNumber >= 15 && stepNumber < 30){
    //                STEP_LENGTH += 0.002;
    //                f_d += .02;
    //                if (stepNumber == 15){
    //                    freqVec[0] = f_d;
    //                    freqVec[1] = f_d;
    //                    QswKMid += 0.1;
    //                }
    //                f0 += 0.02;
    //                Q_INIT[5] = -0.04;
    //                Q_INIT[10] = 0.04;
    //            }
    //            if (stepNumber >= 30 && stepNumber < 35){
    //                STEP_LENGTH += DTm;
    //                f_d +=.02;
    //                if (stepNumber == 30){
    //                    freqVec[0] = f_d;
    //                    freqVec[1] = f_d;
    //                    QswKMid += 0.1;
    //                }
    //                f0 += 0.02;
    //                Q_INIT[5] = -0.03;
    //                Q_INIT[10] = 0.03;
    //            }
    if (stepNumber == firstStopN){
        STEP_LENGTH = 2 * STEP_LENGTH / 3;
    }
    if (stepNumber == firstStopN + 1){
        STEP_LENGTH = STEP_LENGTH / 3;
    }
    if (stepNumber == firstStopN + 2){
        STEP_LENGTH = 0;
    }
    x0 = (stepNumber > 2 && t > 1) ? STEP_LENGTH : 0;
    vxDes = STEP_LENGTH == 0 ? 0: 2 * x0 * f_d;
    pyEnd = pyAbs;
    if (stepNumber > 1 && tInStep > 0.15){
        freqVec.push_back(1 / tInStep);
        freqVec.erase(freqVec.begin());
        avgFreq = (freqVec[0] + freqVec[1]) / 2;
        last_step_freq = freqVec[1];
        f0 = f0 - 0.1 * (avgFreq - f_d);  // for fd = 2.5 the coeff was set to 0.05
        f0 = saturate(f0, 3, 1.35); // added check ****
    }
    if (stepNumber >= firstStopN + stepsToStop + 2 && abs(pSwFtInH[0] + pPelvisAbs[0]) < 0.03 && abs(vxAbsF) < 0.1 && s > 0.25 && flagStopCommand == false){
        flagStopCommand = true;
        f0AtStop = f0;
        f_dAtStop = f_d;
        timeAtStop = tm;
    }
}

if (flagStopCommand == true){
    f_d = saturatel(f_dAtStop - 10 * (tm - timeAtStop), 0);
    // drives the stance outputs to the end of the step (s = 1)
    if (f_d == 0 && tm - timeAtStop > 2 * T){
        f0 = f_d;
    }
}

if (f0 == 0 && tm > timeAtStop + 3 * T){   // *** changed 2-3
    if (pPelvisAbs[0] > 0.1 || pPelvisAbs[0] < -0.06 || kF < 0.1 || kF > 0.9){
        f_d = 1 / T;
        f0 = f_d;
        STEP_LENGTH = 0;
        stepNumber = 1;
        firstStopN = 2;
        side = kF < 0.5 ? 0 : 1;
        if (pPelvisAbs[0] < -0.06){
            QswKMid = 1.3;
        }
        else
        {
            QswKMid = 1.1;
        }
        flagStopCommand = false;
    }
}

s = saturate((tInStep) * f0,1,0);
//        kR = pow(kF, pow(f0 / 3, 0.3));
//        kL = pow(1 - kF, pow(f0 / 3, 0.3));
if (tInStep > DTm*(M4+M2+M5)){
    //            pSwFtInH[0] = pxswf;
    //            pSwFtInH[1] = pyswf;
    vSwFtInH[0] = vxswf;
    vSwFtInH[1] = vyswf;
}
else if (tInStep > DTm*(M5+M4)){
    vSwFtInH[0] = (pxswf-pxswf_old)/(DTm*M5);
    dswFtPitch = (swFtPitch-swFtPitch_old)/(DTm*M5);
    dswFtRoll = (swFtRoll-swFtRoll_old)/(DTm*M5);
}
for (int i = 0; i < 3; i++){
    swFtPos[i] = pSwFtInH[i];
    dswFtPos[i] = vSwFtInH[i];
}

// High-level foot adjustment
DesiredFtPos(pxAbs, pyAbs, tInStep, px0, vxDes, vxAbs, vxAbsF, vyAbs, sg, deltaX, deltaY, kv, frontalBias, s, T);

if (forceRightAnkleZMed < AirTresh && forceLeftAnkleZMed < AirTresh){
    stepNumber = 1;
    side = SIDE_INITIAL;
    oldSide = side;
    inAir = 1;
    if (timeInAir == 0){
        timeInAir = tm;
        for (int i = 0; i < N; i++){
            Q0[i] = qSens[i];
        }
    }
    init_pos(tm - timeInAir, Q0, Q_INIT, qSens, dqSens, tauDes, vals, whichComan_);
    //   cout << tm - timeInAir << endl;
}
else{
    if (inAir == 1){
        t0 = tm;
        inAir = 0;
        timeInAir = 0;
    }
    if (tm - t0 < TIME_REWALK){
        init_pos(tm - t0, Q_INIT, Q_INIT, qSens, dqSens, tauDes, vals, whichComan_);
        // cout << tm - t0 << endl;
    }
    else{
        walkingController3.EvalOutputs(s, f0, Q_INIT, qSens, dqSens, kR, kL, indexSt, indexSw, thp, dthpF, thr, dthrF, x0, deltaX, deltaY, qSensInit,
                                        swFtPosInit, thpF_init, thrF_init, px0, pSwFtInH, vSwFtInH, orSwFt, dorSwFt, orSwFtInit, h, dh, hD, dhD, STEP_LENGTH, QswKMid);
        walkingController3.EvalTorques(s,tInStep, f_d, f0, x0, px0, Q_INIT, qSens, dqSens, kR, kL, orSwFt, tauAnkTorque, forceRightAnkleF, forceLeftAnkleF, torqueRightAnkleF, torqueLeftAnkleF,
                                        pPelvisAbs, vxAbsF, h, dh, hD, dhD, tauDes, vals); // *** no need to send whichcoman command?
    }
}
#else
t = tm - TIME_WALK;

// cout<<t<<endl;

double tw = 0;
if (t >= TIME_WALK && t <= TIME_WALK + 0.1){
    f_d = 1 / T;
    tw = t - TIME_WALK;
    f0 = tw <= 0.1 ? 10 * tw * f_d : f_d; // fast but continuously go to f_d from f0  = 0;
}
// if tm > 2 * TIME_WALK start walking
if (t >= TIME_WALK && startWalkingFlag == 0){
    startWalkingFlag = 1;
    for (int i = 0; i < N; i++){
        qSensInit[i] = qSens[i];
    }
    for (int i = 0; i < 3; i++){
        thPelvisInit[i] = thPelvis[i];
        swFtPosInit[i] = pSwFtInH[i];
        orSwFtInit[i] = orSwFt[i];
    }
    thpF_init = thpF;
    thrF_init = thrF;
    px0 = pxAbs;
}


if (flagMidStep == false && vyAbs > 0){
    pyMidVec.push_back(pyAbs);
    if (pyMidVec.size() > 2){
        pyMidVec.erase(pyMidVec.begin());
        frontalBias = (pyMidVec[0] + pyMidVec[1]) / 2;
    }
    flagMidStep = true;

}

static double begsteptime = TIME_WALK;
if (side != oldSide){
    begsteptime = tm;
    ++stepNumber;
    SwapArrays(indexSt, indexSw, 6);
    EraseVectors();
    flagMidStep = false;
    QswKMid = QswKMid_INIT;
}
tInStep = tm - begsteptime;
for (int i = 0; i < 6; i++){
    qSt[i] = qSens[indexSt[i]];
    dqSt[i] = dqSens[indexSt[i]];
    qStAbs[i] = qSensAbs[indexSt[i]];
    qStAbsMed[i] = qSensAbsMed[indexSt[i]];
    qSw[i] = qSens[indexSw[i]];
}
// Find initial values at the beginning of the step
if (tInStep < ZERO_S){
    for (int i = 0; i < N; i++){
        qSensInit[i] = qSens[i];
    }
    for (int i = 0; i < 3; i++){
        thPelvisInit[i] = thPelvis[i];
        swFtPosInit[i] = pSwFtInH[i];
        orSwFtInit[i] = orSwFt[i];
    }
    thpF_init = thpF;
    thrF_init = thrF;
    px0 = pxAbs;
}

// Determine the stance side and sign (sg)
oldSide = side;
WhichSide(kF, s, side, sg);
if (side != oldSide){
    //            if (stepNumber >= 15 && stepNumber < 30){
    //                STEP_LENGTH += 0.002;
    //                f_d += .02;
    //                if (stepNumber == 15){
    //                    freqVec[0] = f_d;
    //                    freqVec[1] = f_d;
    //                    QswKMid += 0.1;
    //                }
    //                f0 += 0.02;
    //                Q_INIT[5] = -0.04;
    //                Q_INIT[10] = 0.04;
    //            }
    //            if (stepNumber >= 30 && stepNumber < 35){
    //                STEP_LENGTH += DTm;
    //                f_d +=.02;
    //                if (stepNumber == 30){
    //                    freqVec[0] = f_d;
    //                    freqVec[1] = f_d;
    //                    QswKMid += 0.1;
    //                }
    //                f0 += 0.02;
    //                Q_INIT[5] = -0.03;
    //                Q_INIT[10] = 0.03;
    //            }
    if (stepNumber == firstStopN){
        STEP_LENGTH = 2 * STEP_LENGTH / 3;
    }
    if (stepNumber == firstStopN + 1){
        STEP_LENGTH = STEP_LENGTH / 3;
    }
    if (stepNumber == firstStopN + 2){
        STEP_LENGTH = 0;
    }
    x0 = (stepNumber > 2 && t > 1) ? STEP_LENGTH : 0;
    vxDes = STEP_LENGTH == 0 ? 0: 2 * x0 * f_d;
    pyEnd = pyAbs;
    if (stepNumber > 1 && tInStep > 0.15){
        freqVec.push_back(1 / tInStep);
        freqVec.erase(freqVec.begin());
        avgFreq = (freqVec[0] + freqVec[1]) / 2;
        last_step_freq = freqVec[1];
        f0 = f0 - 0.1 * (avgFreq - f_d);  // for fd = 2.5 the coeff was set to 0.05
        f0 = saturate(f0, 3, 1.35); // added check ****
    }
    if (stepNumber >= firstStopN + stepsToStop + 2 && abs(pSwFtInH[0] + pPelvisAbs[0]) < 0.03 && abs(vxAbsF) < 0.1 && s > 0.25 && flagStopCommand == false){
        flagStopCommand = true;
        f0AtStop = f0;
        f_dAtStop = f_d;
        timeAtStop = tm;
    }
}

if (flagStopCommand == true){
    f_d = saturatel(f_dAtStop - 10 * (tm - timeAtStop), 0);
    // drives the stance outputs to the end of the step (s = 1)
    if (f_d == 0 && tm - timeAtStop > 2 * T){
        f0 = f_d;
    }
}

if (f0 == 0 && tm > timeAtStop + 3 * T){
    if (pPelvisAbs[0] > 0.1 || pPelvisAbs[0] < -0.06 || kF < 0.1 || kF > 0.9){
        f_d = 1 / T;
        f0 = f_d;
        STEP_LENGTH = 0;
        stepNumber = 1;
        firstStopN = 2;
        side = kF < 0.5 ? 0 : 1;
        if (pPelvisAbs[0] < -0.06){
            QswKMid = 1.3;
        }
        else
        {
            QswKMid = 1.1;
        }
        flagStopCommand = false;
    }
}

s = saturate((tInStep) * f0,1,0);
//        kR = pow(kF, pow(f0 / 3, 0.3));
//        kL = pow(1 - kF, pow(f0 / 3, 0.3));
if (tInStep > DTm*(M4+M2+M5)){
    //            pSwFtInH[0] = pxswf;
    //            pSwFtInH[1] = pyswf;
    vSwFtInH[0] = vxswf;
    vSwFtInH[1] = vyswf;
}
else if (tInStep > DTm*(M5+M4)){
    vSwFtInH[0] = (pxswf-pxswf_old)/(DTm*M5);
    dswFtPitch = (swFtPitch-swFtPitch_old)/(DTm*M5);
    dswFtRoll = (swFtRoll-swFtRoll_old)/(DTm*M5);
}
for (int i = 0; i < 3; i++){
    swFtPos[i] = pSwFtInH[i];
    dswFtPos[i] = vSwFtInH[i];
}

// High-level foot adjustment
DesiredFtPos(pxAbs, pyAbs, tInStep, px0, vxDes, vxAbs, vxAbsF, vyAbs, sg, deltaX, deltaY, kv, frontalBias, s, T);

if (forceRightAnkleZMed < AirTresh && forceLeftAnkleZMed < AirTresh){
    stepNumber = 1;
    side = SIDE_INITIAL;
    oldSide = side;
    inAir = 1;
    if (timeInAir == 0){
        timeInAir = tm;
        for (int i = 0; i < N; i++){
            Q0[i] = qSens[i];
        }
    }
    init_pos(tm - timeInAir, Q0, Q_INIT, qSens, dqSens, tauDes, vals, whichComan_);
    //   cout << tm - timeInAir << endl;
}
else{
    if (inAir == 1){
        t0 = tm;
        inAir = 0;
        timeInAir = 0;
    }
    if (tm - t0 < TIME_REWALK){
        init_pos(tm - t0, Q_INIT, Q_INIT, qSens, dqSens, tauDes, vals, whichComan_);
        // cout << tm - t0 << endl;
    }
    else{
        walkingController3.EvalOutputs(s, f0, Q_INIT, qSens, dqSens, kR, kL, indexSt, indexSw, thp, dthpF, thr, dthrF, x0, deltaX, deltaY, qSensInit,
                                        swFtPosInit, thpF_init, thrF_init, px0, pSwFtInH, vSwFtInH, orSwFt, dorSwFt, orSwFtInit, h, dh, hD, dhD, STEP_LENGTH, QswKMid);
        walkingController3.EvalTorques(s,tInStep, f_d, f0, x0, px0, Q_INIT, qSens, dqSens, kR, kL, orSwFt, tauAnkTorque, forceRightAnkleF, forceLeftAnkleF, torqueRightAnkleF, torqueLeftAnkleF,
                                        pPelvisAbs, vxAbsF, h, dh, hD, dhD, tauDes, vals);
    }
}
#endif
    // Var for Output
        varsOut.tm_=tm;
        varsOut.n_=N;
        varsOut.qSens_=qSens;
        varsOut.qSensAbs_=qSensAbs;
        varsOut.dqSens_=dqSens;
        varsOut.pPelvis_=pPelvis;
        varsOut.pPelvisAbs_=pPelvisAbs;
        varsOut.vPelvis_=vPelvis;
        varsOut.vPelvisAbs_=vPelvisAbs;
        varsOut.aPelvis_=aPelvis;
        varsOut.forceRightAnkle_=forceRightAnkle;
        varsOut.forceLeftAnkle_=forceLeftAnkle;
        varsOut.torqueRightAnkle_=torqueRightAnkle;
        varsOut.torqueLeftAnkle_=torqueLeftAnkle;
        varsOut.forceRightHand_=forceRightHand;
        varsOut.forceLeftHand_=forceLeftHand;
        varsOut.orSwFt_=orSwFt;
        varsOut.dorSwFt_=dorSwFt;
        varsOut.pSwFtInH_=pSwFtInH;
        varsOut.angRates_=angRates;
        varsOut.h_=h;
        varsOut.hD_=hD;
        varsOut.pPelvisFK_=pPelvisFK;
        varsOut.pPelvisTest_=pPelvisTest;
        varsOut.vPelvisFk_=vPelvisFk;
        varsOut.tauSens_=tauSens;
        varsOut.tauDes_=tauDes;
        varsOut.tauAnkTorque_=tauAnkTorque;
        varsOut.k_=kF;
        varsOut.s_=s;
        varsOut.tInStep_=tInStep;
        varsOut.deltaX_=deltaX;
        varsOut.deltaY_=deltaY;
        varsOut.thp_=thp;
        varsOut.thpF_init_=thpF_init;
        varsOut.thr_=thr;
        varsOut.side_=side;
        varsOut.vxFK_=vxFK;
        varsOut.kv_=kv;
        varsOut.px0_=px0;
        varsOut.kOrg_=kRaw;
        varsOut.vxDes_=vxDes;
        varsOut.x0_=x0;
        varsOut.T_=T;
        memcpy(varsOut.trans_, trans, sizeof(varsOut.trans_));// check values
        //varsOut.trans_=trans;  //???? error with types???
        varsOut.imuAngRates_=imuAngRates;
        varsOut.imuAccelerations_=imuAccelerations;
        varsOut.kR_=kR;
        varsOut.kL_=kL;
        varsOut.avgFreq_=avgFreq;
        varsOut.f0_=f0;
        varsOut.last_step_freq_=last_step_freq;
        varsOut.frontalBias_=frontalBias;
        varsOut.vxAbsF_=vxAbsF;
        varsOut.qSensAbsMed_=qSensAbsMed;
        varsOut.pPelvisAbsMed_=pPelvisAbsMed;
        varsOut.qSt_=qSt;
        varsOut.indexSt_=indexSt;
        varsOut.pxAbsOld_=pxAbs_old;
        varsOut.pPelvisAbsF_=pPelvisAbsF;
}



void Control::SaveVars(std::ofstream &outputFile){
    // Save Data  tme, qSens, qSensAbs, dqSens, tauSens, forceRightAnkle, forceLeftAnkle, torqueRightAnkle, torqueLeftAnkle, forceRightHand, forceLeftHand,forceSensors, trans, imuAngRates, imuAccelerations

    outputFile << varsOut.tm_;
            // start_id = 2
            for (int i = 0; i < N; i++){
                outputFile << " " << varsOut.qSens_[i];
            }
            //  start_id = 2 + n
            for (int i = 0; i < N; i++){
                outputFile << " " << varsOut.qSensAbs_[i];
            }
            // start_id = 2 + 2N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << varsOut.dqSens_[i];
            }
            // start_id = 2 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.pPelvis_[i];
            }
            // start_id = 5 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.pPelvisAbs_[i];
            }
            // start_id = 8 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.vPelvis_[i];
            }
            // start_id = 11 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.vPelvisAbs_[i];
            }
            // start_id = 14 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.aPelvis_[i];
            }
            // start_id = 17 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.forceRightAnkle_[i];
            }
            // start_id = 20 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.forceLeftAnkle_[i];
            }
            // start_id = 23 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.torqueRightAnkle_[i];
            }
            // start_id = 26 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.torqueLeftAnkle_[i];
            }
            // start_id = 29 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.forceRightHand_[i];
            }
            // start_id = 32 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.forceLeftHand_[i];
            }
            // start_id = 35 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.orSwFt_[i];
            }
            // start_id = 38 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.dorSwFt_[i];
            }
            // start_id = 41 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.pSwFtInH_[i];
            }
            // start_id = 44 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.angRates_[i];
            }
            // start_id = 47 + 3N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << varsOut.h_[i];
            }
            // start_id = 47 + 4N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << varsOut.hD_[i];
            }
            // start_id = 47 + 5N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.pPelvisFK_[i];
            }
            // start_id = 50 + 5N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.pPelvisTest_[i];
            }
            // start_id = 53 + 5N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.vPelvisFk_[i];
            }
            // start_id = 56 + 5N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << varsOut.tauSens_[i];
            }
            // start_id = 56 + 6N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << varsOut.tauDes_[i];
            }
            // start_id = 56 + 7N
            for (int i = 0; i < 2; i++)
            {
                outputFile << " " << varsOut.tauAnkTorque_[i];
            }
            // start_id = 58 + 7N
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    outputFile << " " << varsOut.trans_[i][j];
                }
            }
            // start_id = 67 + 7N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.imuAngRates_[i];
            }
            // start_id = 70 + 7N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << varsOut.imuAccelerations_[i];
            }
            // start_id = 73 + 7N
            for (int i = 0; i < N; i++){
                outputFile << " " << varsOut.qSensAbsMed_[i];
            }
            // start_id = 73 + 8N
            for (int i = 0; i < 3; i++){
                outputFile << " " << varsOut.pPelvisAbsMed_[i];
            }
            for (int i = 0; i < 3; i++){
                outputFile << " " << varsOut.pPelvisAbsF_[i];
            }
    //        for (int i = 0; i < 800; i++){
    //            outputFile << " " << varsOut.velTimeWind_[i];
    //        }
            outputFile // start_id = 76 + 8N
                      << " " << varsOut.k_ // 76 + 8N
                      << " " << varsOut.s_ // 77 + 8N
                      << " " << varsOut.tInStep_ // 78 + 8N
                      << " " << varsOut.deltaX_ // 79 + 8N
                      << " " << varsOut.deltaY_ // 80 + 8N
                      << " " << varsOut.thp_ // 81 + 8N
                      << " " << varsOut.thpF_init_ // 82 + 8N
                      << " " << varsOut.thr_ // 83 + 8N
                      << " " << varsOut.side_ // 84 + 8N
                      << " " << varsOut.vxFK_ // 85 + 8N
                      << " " << varsOut.kv_ // 86 + 8N
                      << " " << varsOut.px0_ // 87 + 8N
                      << " " << varsOut.kOrg_ // 88 + 8N
                      << " " << varsOut.vxDes_ * varsOut.tInStep_ + varsOut.px0_ // 89 + 8N
                      << " " << varsOut.x0_ // 87 + 7N
                      << " " << varsOut.T_
                      << " " << varsOut.kR_
                      << " " << varsOut.kL_
                      << " " << varsOut.avgFreq_
                      << " " << varsOut.f0_
                      << " " << varsOut.last_step_freq_
                      << " " << varsOut.frontalBias_
                      << " " << varsOut.vxAbsF_
                      << " " << varsOut.pxAbsOld_
                      << std::endl;
}
