#include "WalkingController3.hh"
#include "bezier.hh"
#include <cmath>
#include <iostream>
#include "SwFtPosToHip.hh"
#include "SwFtXtoHipPitch.hh"
#include "SwFtOrToAnk.hh"

#ifndef REAL_ROBOT
    #define N 31
#else
    #define N 23
#endif

#define INV_KIN_FTX
// #define INV_KIN_FTY
#define INV_KIN_FTOR

using namespace std;
WalkingController3::WalkingController3()
{

}

void WalkingController3::EvalOutputs(double s, double f0, double Q_INIT[], double qSens[], double dqSens[], double kR, double kL, int indexSt[], int indexSw[],double thpF,
                                     double dthpF, double thr, double dthrF, double x0, double deltaX, double deltaHipPitchvy, double qSensInit[], double pSw_init[],
                                     double thpF_init, double thrF_init, double px0, double pSwFtInH[], double vSwFtInH[], double orSwFt[], double dorSwFt[],
                                     double orSwFtInit[], double h[], double dh[], double hD[], double dhD[], double STEP_LENGTH, double QswKMid)
{

    double qSw[6] = {qSens[3], qSens[5], qSens[6], qSens[7], qSens[8], qSens[9]}, qHipSwDes[2], dqHipSwDes[2];
    double pSwFtInHDes[3], orSwFtDes[3], swFtPosDes[2] = {0}, dswFtPosDes[2] = {0};
    bool swSide = 0;
    for (int i = 0; i < 6; i++){
        qSw[i] = qSens[indexSw[i]];
    }
    double calSwRoll = 0.0;
    const double L_KNEE_BIAS = -0.0;

    const double TORSO_YAW_D = 0, TORSO_PITCH_D = Q_INIT[1], TORSO_ROLL_D = Q_INIT[2], THP_D = 0, THR_D = 0;
    const double R_LEG_YAW_D = 0, L_LEG_YAW_D = 0, RST_KNEE_FINAL_D = 0.15, LST_KNEE_FINAL_D = 0.15 + L_KNEE_BIAS, RSW_KNEE_FINAL_D = 0.17 + 0.6 * STEP_LENGTH,
                 LSW_KNEE_FINAL_D = 0.17 + 0.6 * STEP_LENGTH + L_KNEE_BIAS, SW_FT_PITCH_D = 0;
    const double R_ST_ANKLE_ROLL_D = -Q_INIT[5], R_SW_FT_ROLL_D = 0;
    const double L_ST_ANKLE_ROLL_D = -Q_INIT[10] - calSwRoll, L_SW_FT_ROLL_D = 0;

    double stAnkCoef = 1;
    double RST_ANKLE_PITCH_D = - stAnkCoef * (x0 + 0.24 * qSens[7]) / (0.44);
    double LST_ANKLE_PITCH_D = - stAnkCoef * (x0 + 0.24 * qSens[12]) / (0.44);
//#else
//    const double ST_ANKLE_PITCH_D = -0.15; //TODO: note this was used for simulations only should I add REAL_ROBOT variable?
//#endif

    int legFlag = 1; // Right Leg is at stance
    if (indexSw[3] == 7){
        legFlag = -1;
        swSide = 1;
    }
    const double Y0 = 0.1;

    bezier bezierPoly5;
    bezierPoly5.degree = 5;

    double hTorsoYawD, dhTorsoYawD, hTorsoYaw, dhTorsoYaw, hTorsoYawInit;
    hTorsoYaw    = qSens[0];
    dhTorsoYaw   = dqSens[0];
    hTorsoYawInit = qSensInit[0];
    double hTorsoYawAlphaD[6] = {hTorsoYawInit, 0.5 * (hTorsoYawInit + TORSO_YAW_D), TORSO_YAW_D, TORSO_YAW_D, TORSO_YAW_D, TORSO_YAW_D};

    hTorsoYawD = bezierPoly5.get_bezier(hTorsoYawAlphaD, s);
    dhTorsoYawD = bezierPoly5.get_derv_bezier(hTorsoYawAlphaD, s);
    h[0] = hTorsoYaw;
    dh[0] = dhTorsoYaw;
    hD[0] = hTorsoYawD;
    dhD[0] = dhTorsoYawD;

    double hTorsoPitchD, dhTorsoPitchD, hTorsoPitch, dhTorsoPitch, hTorsoPitchInit;
    hTorsoPitch    = qSens[1];
    dhTorsoPitch   = dqSens[1];
    hTorsoPitchInit = qSensInit[1];
    double hTorsoPitchAlphaD[6] = {hTorsoPitchInit, 0.5 * (hTorsoPitchInit + TORSO_PITCH_D), TORSO_PITCH_D, TORSO_PITCH_D, TORSO_PITCH_D, TORSO_PITCH_D};

    hTorsoPitchD = bezierPoly5.get_bezier(hTorsoPitchAlphaD, s);
    dhTorsoPitchD = bezierPoly5.get_derv_bezier(hTorsoPitchAlphaD, s);
    h[1] = hTorsoPitch;
    dh[1] = dhTorsoPitch;
    hD[1] = hTorsoPitchD;
    dhD[1] = dhTorsoPitchD;

    double hTorsoRollD, dhTorsoRollD, hTorsoRoll, dhTorsoRoll, hTorsoRollInit;
    hTorsoRoll    = qSens[2];
    dhTorsoRoll   = dqSens[2];
    hTorsoRollInit = qSensInit[2];
    double hTorsoRollAlphaD[6] = {hTorsoRollInit, 0.5 * (hTorsoRollInit + TORSO_ROLL_D), TORSO_ROLL_D, TORSO_ROLL_D, TORSO_ROLL_D, TORSO_ROLL_D};

    hTorsoRollD = bezierPoly5.get_bezier(hTorsoRollAlphaD, s);
    dhTorsoRollD = bezierPoly5.get_derv_bezier(hTorsoRollAlphaD, s);
    h[2] = hTorsoRoll;
    dh[2] = dhTorsoRoll;
    hD[2] = hTorsoRollD;
    dhD[2] = dhTorsoRollD;

    double hSwFtXD, dhSwFtXD, hSwFtX, dhSwFtX, hSwFtXInit;
    double swFtXD = x0 + deltaX;
    hSwFtX = pSwFtInH[0];
    dhSwFtX = vSwFtInH[0];
    hSwFtXInit = pSw_init[0];
    double hSwFtXAlphaD[6] = {hSwFtXInit, (1 / 2 * hSwFtXInit + 1 / 2 * swFtXD), (1 / 2 * hSwFtXInit + 1 / 2 * swFtXD), (1 / 3 * hSwFtXInit + 2 / 3 * swFtXD), swFtXD, swFtXD};
    hSwFtXD = bezierPoly5.get_bezier(hSwFtXAlphaD, s);
    dhSwFtXD = bezierPoly5.get_derv_bezier(hSwFtXAlphaD, s);

    double hPelvisPitchD, dhPelvisPitchD, hPelvisPitch, dhPelvisPitch, hPelvisPitchInit;
    hPelvisPitch = thpF;
    dhPelvisPitch = dthpF;
    hPelvisPitchInit = thpF_init;
    double hPelvisPitchAlphaD[6] = {hPelvisPitchInit, 0.5 * (hPelvisPitchInit + THP_D), THP_D, THP_D, THP_D, THP_D};
    hPelvisPitchD = bezierPoly5.get_bezier(hPelvisPitchAlphaD, s);
    dhPelvisPitchD = bezierPoly5.get_derv_bezier(hPelvisPitchAlphaD, s);

    double hSwFtYD, dhSwFtYD, hSwFtY, dhSwFtY, hSwFtYInit;
    const double swFtYD = legFlag * Y0 + deltaHipPitchvy; //TEMP
    hSwFtY = pSwFtInH[1];
    dhSwFtY = vSwFtInH[1]; //TEMP
    hSwFtYInit = pSw_init[1];
    double hSwFtYAlphaD[6] = {hSwFtYInit, 0.5 * (hSwFtYInit + 1.1 * swFtYD), 1.1 * swFtYD, swFtYD, swFtYD, swFtYD};
    hSwFtYD = bezierPoly5.get_bezier(hSwFtYAlphaD, s);
    dhSwFtYD = bezierPoly5.get_derv_bezier(hSwFtYAlphaD, s);

    double hPelvisRollD, dhPelvisRollD, hPelvisRoll, dhPelvisRoll, hPelvisRollInit;
    hPelvisRoll = thr;
    dhPelvisRoll = dthrF;
    hPelvisRollInit = thrF_init;
    double hPelvisRollAlphaD[6] = {hPelvisRollInit, 0.5 * (hPelvisRollInit + THR_D), THR_D, THR_D, THR_D, THR_D};
    hPelvisRollD = bezierPoly5.get_bezier(hPelvisRollAlphaD, s);
    dhPelvisRollD = bezierPoly5.get_derv_bezier(hPelvisRollAlphaD, s);

    swFtPosDes[0] = hSwFtXD;
    dswFtPosDes[0] = dhSwFtXD;
    swFtPosDes[1] = hSwFtYD;
    dswFtPosDes[1] = dhSwFtYD;
#if defined(INV_KIN_FTX) && defined(INV_KIN_FTY)
    SwFtPosToHip(swFtPosDes, dswFtPosDes, qSw, swSide, qHipSwDes, dqHipSwDes);
#endif
#if defined(INV_KIN_FTX)
    SwFtXtoHipPitch(swFtPosDes[0], dswFtPosDes[0], qSw, swSide, qHipSwDes, dqHipSwDes);
#endif
    double hSwHipPitchD = qHipSwDes[0];
    double dhSwHipPitchD = dqHipSwDes[0];
    double hSwHipRollD = qHipSwDes[1];
    double dhSwHipRollD = dqHipSwDes[1];
#ifdef INV_KIN_FTX
    h[3] = -kR * hPelvisPitch + (1 - kR) * qSens[3];
    dh[3] = -kR * dhPelvisPitch + (1 - kR) * dqSens[3];
    hD[3] = -kR * hPelvisPitchD + (1 - kR) * hSwHipPitchD;
    dhD[3] = -kR * dhPelvisPitchD + (1 - kR) * dhSwHipPitchD;

    h[4] = -kL * hPelvisPitch + (1 - kL) * qSens[4];
    dh[4] = -kL * dhPelvisPitch + (1 - kL) * dqSens[4];
    hD[4] = -kL * hPelvisPitchD + (1 - kL) * hSwHipPitchD;
    dhD[4] = -kL * dhPelvisPitchD + (1 - kL) * dhSwHipPitchD;
#else
    h[3] = -k * hPelvisPitch - (1 - k) * hSwFtX;
    dh[3] = -k * dhPelvisPitch - (1 - k) * dhSwFtX;
    hD[3] = -k * hPelvisPitchD - (1 - k) * hSwFtXD;
    dhD[3] = -k * dhPelvisPitchD - (1 - k) * dhSwFtXD;

    h[4] = -(1 - k) * hPelvisPitch - k * hSwFtX;
    dh[4] = -(1 - k) * dhPelvisPitch - k * dhSwFtX;
    hD[4] = -(1 - k) * hPelvisPitchD - k * hSwFtXD;
    dhD[4] = -(1 - k) * dhPelvisPitchD - k * dhSwFtXD;
#endif


#ifdef INV_KIN_FTY
    h[5] = k * (qSens[5] - hPelvisRoll)+ (1 - k) * qSens[5];
    dh[5] = k * (dqSens[5] - dhPelvisRoll) + (1 - k) * dqSens[5];
    hD[5] = k * (Q_INIT[5] - hPelvisRollD)+ (1 - k) * hSwHipRollD;
    dhD[5] = k * (0 - dhPelvisRollD) + (1 - k) * dhSwHipRollD;

    h[10] = (1 - k) * (qSens[10] - hPelvisRoll) + k * qSens[10];
    dh[10] = (1 - k) * (dqSens[10] - dhPelvisRoll) + k * dqSens[10];
    hD[10] = (1 - k) * (Q_INIT[10] - hPelvisRollD) + k * hSwHipRollD;
    dhD[10] = (1 - k) * (0 - dhPelvisRollD) + k * dhSwHipRollD;
#else
    h[5] = kR * (qSens[5] - hPelvisRoll) + (1 - kR) * (qSens[5] - hPelvisRoll); // other version: replace qSense[5] by hSwFtY and having -hPelvisRoll in the swing phase is weired to be examined later why
    dh[5] = kR * (dqSens[5] - dhPelvisRoll) + (1 - kR) * (dqSens[5]);
    hD[5] = kR * (Q_INIT[5] - hPelvisRollD) + (1 - kR) * (Q_INIT[5] + deltaHipPitchvy);
    dhD[5] = kR * (0 - dhPelvisRollD) + (1 - kR) * 0;

    h[10] = kL * (qSens[10] - hPelvisRoll) + (1 - kL) * (qSens[10] - hPelvisRoll);
    dh[10] = kL * (dqSens[10] - dhPelvisRoll) + (1 - kL) * (dqSens[10]);
    hD[10] = kL * (Q_INIT[10] - hPelvisRollD) + (1 - kL) * (Q_INIT[10] + deltaHipPitchvy + calSwRoll);
    dhD[10] = kL * (0 - dhPelvisRollD) + (1 - kL) * 0;
#endif

    double hRlegYawD, dhRlegYawD, hRlegYaw, dhRlegYaw, hRlegYawInit;
    hRlegYaw = qSens[6];
    dhRlegYaw = dqSens[6];
    hRlegYawInit = qSensInit[6];
    double hRlegYawAlphaD[6] = {hRlegYawInit, 0.5 * (hRlegYawInit + R_LEG_YAW_D), R_LEG_YAW_D, R_LEG_YAW_D, R_LEG_YAW_D, R_LEG_YAW_D};
    hRlegYawD = bezierPoly5.get_bezier(hRlegYawAlphaD, s);
    dhRlegYawD = bezierPoly5.get_derv_bezier(hRlegYawAlphaD, s);

    h[6] = hRlegYaw;
    dh[6] = dhRlegYaw;
    hD[6] = hRlegYawD;
    dhD[6] = dhRlegYawD;

    double hLlegYawD, dhLlegYawD, hLlegYaw, dhLlegYaw, hLlegYawInit;
    hLlegYaw = qSens[11];
    dhLlegYaw = dqSens[11];
    hLlegYawInit = qSensInit[11];
    double hLlegYawAlphaD[6] = {hLlegYawInit, 0.5 * (hLlegYawInit + L_LEG_YAW_D), L_LEG_YAW_D, L_LEG_YAW_D, L_LEG_YAW_D, L_LEG_YAW_D};
    hLlegYawD = bezierPoly5.get_bezier(hLlegYawAlphaD, s);
    dhLlegYawD = bezierPoly5.get_derv_bezier(hLlegYawAlphaD, s);

    h[11] = hLlegYaw;
    dh[11] = dhLlegYaw;
    hD[11] = hLlegYawD;
    dhD[11] = dhLlegYawD;

    double hRStKneeD, dhRStKneeD, hRStKneeInit;
    hRStKneeInit = qSensInit[7];
    double hRStKneeAlphaD[6] = {hRStKneeInit, 0.5 * (hRStKneeInit + RST_KNEE_FINAL_D), RST_KNEE_FINAL_D, RST_KNEE_FINAL_D, RST_KNEE_FINAL_D, RST_KNEE_FINAL_D};
    hRStKneeD = bezierPoly5.get_bezier(hRStKneeAlphaD, s);
    dhRStKneeD = bezierPoly5.get_derv_bezier(hRStKneeAlphaD, s);

    double hLStKneeD, dhLStKneeD, hLStKneeInit;
    hLStKneeInit = qSensInit[12];
    double hLStKneeAlphaD[6] = {hLStKneeInit, 0.5 * (hLStKneeInit + LST_KNEE_FINAL_D), LST_KNEE_FINAL_D, LST_KNEE_FINAL_D, LST_KNEE_FINAL_D, LST_KNEE_FINAL_D};
    hLStKneeD = bezierPoly5.get_bezier(hLStKneeAlphaD, s);
    dhLStKneeD = bezierPoly5.get_derv_bezier(hLStKneeAlphaD, s);

    double hRSwKneeD, dhRSwKneeD, hRSwKneeInit;
    hRSwKneeInit = qSensInit[7];
    double hRSwKneeAlphaD[6] = {hRSwKneeInit,0.75 * QswKMid,QswKMid,0.5,RSW_KNEE_FINAL_D,RSW_KNEE_FINAL_D};
    hRSwKneeD = bezierPoly5.get_bezier(hRSwKneeAlphaD, s);
    dhRSwKneeD = bezierPoly5.get_derv_bezier(hRSwKneeAlphaD, s);

    double hLSwKneeD, dhLSwKneeD, hLSwKneeInit;
    hLSwKneeInit = qSensInit[12];
    double hLSwKneeAlphaD[6] = {hLSwKneeInit,0.75 * QswKMid,QswKMid,0.5,LSW_KNEE_FINAL_D,LSW_KNEE_FINAL_D};
    hLSwKneeD = bezierPoly5.get_bezier(hLSwKneeAlphaD, s);
    dhLSwKneeD = bezierPoly5.get_derv_bezier(hLSwKneeAlphaD, s);

    h[7] = qSens[7];
    dh[7] = dqSens[7];
    hD[7] = kR * hRStKneeD + (1 - kR) * hRSwKneeD; // If + * (1-kR) = 0 then stepping for short burst after 1st loop is over.
    dhD[7] = kR * dhRStKneeD + (1 - kR) * dhRSwKneeD;

    h[12] = qSens[12];
    dh[12] = dqSens[12];
    hD[12] = (1 - kL) * hLSwKneeD + kL * hLStKneeD;
    dhD[12] = (1 - kL) * dhLSwKneeD + kL * dhLStKneeD;

    double hRStAnkPitchD, dhRStAnkPitchD, hRStAnkPitch, dhRStAnkPitch, hRStAnkPitchInit;
    hRStAnkPitch = qSens[8];
    dhRStAnkPitch = dqSens[8];
    hRStAnkPitchInit = qSensInit[8];
    double hRStAnkPitchAlphaD[6] = {hRStAnkPitchInit, 0.5 * (hRStAnkPitchInit + RST_ANKLE_PITCH_D),  1 / 4 * hRStAnkPitchInit + 3 / 4 * RST_ANKLE_PITCH_D, RST_ANKLE_PITCH_D, RST_ANKLE_PITCH_D, RST_ANKLE_PITCH_D};
    hRStAnkPitchD = bezierPoly5.get_bezier(hRStAnkPitchAlphaD, s);
    dhRStAnkPitchD = bezierPoly5.get_derv_bezier(hRStAnkPitchAlphaD, s);

    double hLStAnkPitchD, dhLStAnkPitchD, hLStAnkPitch, dhLStAnkPitch, hLStAnkPitchInit;
    hLStAnkPitch = qSens[13];
    dhLStAnkPitch = dqSens[13];
    hLStAnkPitchInit = qSensInit[13];
    double hLStAnkPitchAlphaD[6] = {hLStAnkPitchInit, 0.5 * (hLStAnkPitchInit + LST_ANKLE_PITCH_D), 1 / 3 * hLStAnkPitchInit + 3 / 4 * LST_ANKLE_PITCH_D, LST_ANKLE_PITCH_D, LST_ANKLE_PITCH_D, LST_ANKLE_PITCH_D};
    hLStAnkPitchD = bezierPoly5.get_bezier(hLStAnkPitchAlphaD, s);
    dhLStAnkPitchD = bezierPoly5.get_derv_bezier(hLStAnkPitchAlphaD, s);

    double hSwFtPitchD, dhSwFtPitchD, hSwFtPitch, dhSwFtPitch, hSwFtPitchInit;
    hSwFtPitch = orSwFt[1];
    dhSwFtPitch = dqSens[indexSw[4]]; // or dorSwFt[1]?
    hSwFtPitchInit = orSwFtInit[1];
    double hSwFtPitchAlphaD[6] = {hSwFtPitchInit, 0.5 * (hSwFtPitchInit + SW_FT_PITCH_D), SW_FT_PITCH_D, SW_FT_PITCH_D, SW_FT_PITCH_D, SW_FT_PITCH_D};
    hSwFtPitchD = bezierPoly5.get_bezier(hSwFtPitchAlphaD, s);
    dhSwFtPitchD = bezierPoly5.get_derv_bezier(hSwFtPitchAlphaD, s);

    h[8] = kR * qSens[8] + (1 - kR) * hSwFtPitch;
    dh[8] = kR * dqSens[8] + (1 - kR) * dqSens[8];
    hD[8] = kR * hRStAnkPitchD + (1 - kR) * hSwFtPitchD;
    dhD[8] = kR * dhRStAnkPitchD + (1 - kR) * dhSwFtPitchD;

    h[13] = kL * qSens[13] + (1 - kL) * hSwFtPitch;
    dh[13] = kL * dqSens[13] + (1 - kL) * dqSens[13];
    hD[13] = kL * hLStAnkPitchD + (1 - kL) * hSwFtPitchD;
    dhD[13] = kL * dhLStAnkPitchD + (1 - kL) * dhSwFtPitchD;

    double hRStAnkRollD, dhRStAnkRollD, hRStAnkRoll, dhRStAnkRoll, hRStAnkRollInit;
    hRStAnkRoll = qSens[9];
    dhRStAnkRoll = dqSens[9];
    hRStAnkRollInit = qSensInit[9];
    double hRStAnkRollAlphaD[6] = {hRStAnkRollInit, 0.5 * (hRStAnkRollInit + R_ST_ANKLE_ROLL_D), R_ST_ANKLE_ROLL_D, R_ST_ANKLE_ROLL_D, R_ST_ANKLE_ROLL_D, R_ST_ANKLE_ROLL_D};
    hRStAnkRollD = bezierPoly5.get_bezier(hRStAnkRollAlphaD, s);
    dhRStAnkRollD = bezierPoly5.get_derv_bezier(hRStAnkRollAlphaD, s);

    double hRSwFtRollD, dhRSwFtRollD, hRSwFtRoll, dhRSwFtRoll, hRSwFtRollInit;
    hRSwFtRoll = orSwFt[0];
    dhRSwFtRoll = dqSens[9]; // or dorSwFt[1]?
    hRSwFtRollInit = orSwFtInit[0];
    double hRSwFtRollAlphaD[6] = {hRSwFtRollInit, 0.5 * (hRSwFtRollInit + R_SW_FT_ROLL_D), R_SW_FT_ROLL_D, R_SW_FT_ROLL_D, R_SW_FT_ROLL_D, R_SW_FT_ROLL_D};
    hRSwFtRollD = bezierPoly5.get_bezier(hRSwFtRollAlphaD, s);
    dhRSwFtRollD = bezierPoly5.get_derv_bezier(hRSwFtRollAlphaD, s);

    h[9] = kR * hRStAnkRoll + (1 - kR) * hRSwFtRoll;
    dh[9] = kR * dhRStAnkRoll + (1 - kR) * dhRSwFtRoll;
    hD[9] = kR * hRStAnkRollD + (1 - kR) * hRSwFtRollD;
    dhD[9] = kR * dhRStAnkRollD + (1 - kR) * dhRSwFtRollD;

    double hLStAnkRollD, dhLStAnkRollD, hLStAnkRoll, dhLStAnkRoll, hLStAnkRollInit;
    hLStAnkRoll = qSens[14];
    dhLStAnkRoll = dqSens[14];
    hLStAnkRollInit = qSensInit[14];
    double hLStAnkRollAlphaD[6] = {hLStAnkRollInit, 0.5 * (hLStAnkRollInit + L_ST_ANKLE_ROLL_D), L_ST_ANKLE_ROLL_D, L_ST_ANKLE_ROLL_D, L_ST_ANKLE_ROLL_D, L_ST_ANKLE_ROLL_D};
    hLStAnkRollD = bezierPoly5.get_bezier(hLStAnkRollAlphaD, s);
    dhLStAnkRollD = bezierPoly5.get_derv_bezier(hLStAnkRollAlphaD, s);

    double hLSwFtRollD, dhLSwFtRollD, hLSwFtRoll, dhLSwFtRoll, hLSwFtRollInit;
    hLSwFtRoll = orSwFt[0];
    dhLSwFtRoll = dqSens[14]; // or dorSwFt[1]?
    hLSwFtRollInit = orSwFtInit[0];
    double hLSwFtRollAlphaD[6] = {hLSwFtRollInit, 0.5 * (hLSwFtRollInit + L_SW_FT_ROLL_D), L_SW_FT_ROLL_D, L_SW_FT_ROLL_D, L_SW_FT_ROLL_D, L_SW_FT_ROLL_D};
    hLSwFtRollD = bezierPoly5.get_bezier(hLSwFtRollAlphaD, s);
    dhLSwFtRollD = bezierPoly5.get_derv_bezier(hLSwFtRollAlphaD, s);

    h[14] = kL * hLStAnkRoll + (1 - kL) * hLSwFtRoll;
    dh[14] = kL * dhLStAnkRoll + (1 - kL) * dhLSwFtRoll;
    hD[14] = kL * hLStAnkRollD + (1 - kL) * hLSwFtRollD;
    dhD[14] = kL * dhLStAnkRollD + (1 - kL) * dhLSwFtRollD;


#ifdef INV_KIN_FTOR
    double swRFtOrDes[3] = {0}, dswRFtOrDes[3] = {0};
    swRFtOrDes[0] = hSwFtPitchD;
    swRFtOrDes[1] = hRSwFtRollD;
    swRFtOrDes[2] = 0;
    dswRFtOrDes[0] = dhSwFtPitchD;
    dswRFtOrDes[1] = dhRSwFtRollD;
    dswRFtOrDes[2] = 0;
    double qRSwAnkDes[3], dqRSwAnkDes[3];
    double qRSw[6] = {qSens[3], qSens[5], qSens[6], qSens[7], qSens[8], qSens[9]};
    SwFtOrToAnk(swRFtOrDes, dswRFtOrDes, qRSw, 1, qRSwAnkDes, dqRSwAnkDes);

    double swLFtOrDes[3] = {0}, dswLFtOrDes[3] = {0};
    swLFtOrDes[0] = hSwFtPitchD;
    swLFtOrDes[1] = hLSwFtRollD;
    swLFtOrDes[2] = 0;
    dswLFtOrDes[0] = dhSwFtPitchD;
    dswLFtOrDes[1] = dhLSwFtRollD;
    dswLFtOrDes[2] = 0;
    double qLSwAnkDes[3], dqLSwAnkDes[3];
    double qLSw[6] = {qSens[4], qSens[10], qSens[11], qSens[12], qSens[13], qSens[14]};
    SwFtOrToAnk(swLFtOrDes, dswLFtOrDes, qLSw, 0, qLSwAnkDes, dqLSwAnkDes);

//    h[6] h[11] ?

    h[8] = kR * qSens[8] + (1 - kR) * qSens[8];
    dh[8] = kR * dqSens[8] + (1 - kR) * dqSens[8];
    hD[8] = kR * hRStAnkPitchD + (1 - kR) * qRSwAnkDes[1];
    dhD[8] = kR * dhRStAnkPitchD + (1 - kR) * dqRSwAnkDes[1];

    h[13] = kL * qSens[13] + (1 - kL) * qSens[13];
    dh[13] = kL * dqSens[13] + (1 - kL) * dqSens[13];
    hD[13] = kL * hLStAnkPitchD + (1 - kL) * qLSwAnkDes[1];
    dhD[13] = kL * dhLStAnkPitchD + (1 - kL) * dqLSwAnkDes[1];

    h[9] = kR * hRStAnkRoll + (1 - kR) * qSens[9];
    dh[9] = kR * dhRStAnkRoll + (1 - kR) * dqSens[9];
    hD[9] = kR * hRStAnkRollD + (1 - kR) * qRSwAnkDes[2];
    dhD[9] = kR * dhRStAnkRollD + (1 - kR) * dqRSwAnkDes[2];

    h[14] = kL * hLStAnkRoll + (1 - kL) * qSens[14];
    dh[14] = kL * dhLStAnkRoll + (1 - kL) * dqSens[14];
    hD[14] = kL * hLStAnkRollD + (1 - kL) * qLSwAnkDes[2];
    dhD[14] = kL * dhLStAnkRollD + (1 - kL) * dqLSwAnkDes[2];
#endif
    double R_ARM_PITCH_D = -legFlag * 2 * (x0 + deltaX);
    double L_ARM_PITCH_D = legFlag * 2 * (x0 + deltaX);
    double hRArmPitchD, dhRArmPitchD, hLArmPitchD, dhLArmPitchD;
    double hRArmPitchInit = qSensInit[15];
    double hRArmPitchAlphaD[6] = {hRArmPitchInit,  1 / 2 * hRArmPitchInit + 1 / 2 * R_ARM_PITCH_D,\
                                  1 / 2 * hRArmPitchInit + 1 /2 * R_ARM_PITCH_D, 1 / 3 * hRArmPitchInit + 2 / 3 * R_ARM_PITCH_D,\
                                  0 * hRArmPitchInit + 1 * R_ARM_PITCH_D, R_ARM_PITCH_D};
    hRArmPitchD = bezierPoly5.get_bezier(hRArmPitchAlphaD, s);
    dhRArmPitchD = bezierPoly5.get_derv_bezier(hRArmPitchAlphaD, s);

    double hLArmPitchInit = qSensInit[19];
    double hLArmPitchAlphaD[6] = {hLArmPitchInit,  1 / 2 * hLArmPitchInit + 1 / 2 * L_ARM_PITCH_D,\
                                  1 / 2 * hLArmPitchInit + 1 / 2 * L_ARM_PITCH_D, 1 / 3 * hLArmPitchInit + 2 / 3 * L_ARM_PITCH_D,\
                                  0 * hLArmPitchInit + 1 * L_ARM_PITCH_D, L_ARM_PITCH_D};
    hLArmPitchD = bezierPoly5.get_bezier(hLArmPitchAlphaD, s);
    dhLArmPitchD = bezierPoly5.get_derv_bezier(hLArmPitchAlphaD, s);

    h[15] = qSens[15] - Q_INIT[15];
    dh[15] = dqSens[15];
    hD[15] = hRArmPitchD;
    dhD[15] = dhRArmPitchD;

    h[19] = qSens[19] - Q_INIT[19];
    dh[19] = dqSens[19];
    hD[19] = hLArmPitchD;
    dhD[19] = dhLArmPitchD;
}

void WalkingController3::EvalTorques(double s, double tInStep, double f_d, double f0, double x0, double px0, double Q_INIT[], double qSens[], double dqSens[], double kR, double kL, double orSwFt[],
                                     double tauAnkTorque[], double forceRightAnkle[], double forceLeftAnkle[], double torqueRightAnkle[], double torqueLeftAnkle[],
                                     double pPelvis[], double vxAbsF, double h[], double dh[], double hD[], double dhD[], double tauDes[], double vals[]){
    double y[N];  // desired output to be driven to zero
    double dy[N]; // derivative of the desired output to be driven to zero
    double Kp[N], Kd[N], I[N];
    
    #ifndef REAL_ROBOT
    Kp[23] = 5;
    Kd[23] = 0;
    Kp[24] = 5;
    Kd[24] = 0;
    Kp[25] = 5;
    Kd[25] = 0;
    Kp[26] = 0;
    Kd[26] = 0;
    Kp[27] = 5;
    Kd[27] = 0;
    Kp[28] = 5;
    Kd[28] = 0;
    Kp[29] = 0;
    Kd[29] = 0;
    Kp[30] = 0;
    Kd[30] = 0;
    Kp[31] = 0;
    Kd[31] = 0;

    for (int i = 0; i < 23; i++){
        Kp[i] = 300;
        Kd[i] = 0;
    }
    #else

    
    for (int i = 0; i < N; i++){
        Kp[i] = 500;
        Kd[i] = 15;
    }

    #endif

    Kp[3] = 70+160*pow(kR,3);
    Kd[3] = 15-12.5*pow(kR,3);
    Kp[4] = 70+160*pow(kL,3);//180+70*pow(1-k,3);
    Kd[4] = 15-12.5*pow(kL,3);//25-12.5*pow(1-k,3);
#ifdef INV_KIN_FTX
    Kp[3] = 200 + 130 * kR;
    Kd[3] = 7 + 0 * kR;
    Kp[4] = 285 + 130 * kL;//180+70*pow(1-k,3);
    Kd[4] = 10 - 2 * kL;//25-12.5*pow(1-k,3);

    Kp[7] = 500;
    Kd[7] = 10;
    Kp[12] = 500;
    Kd[12] = 10;

//    Kp[8] = 200;
//    Kd[8] = 6;
//    Kp[13] = 200;
//    Kd[13] = 6;

    Kp[9] = 270-70*kR;
    Kd[9] = 17.5-7.5*kR;
    Kp[14] = 270-70*kL;//180+70*pow(1-k,3);
    Kd[14] = 17.5-7.5*kL;
#endif


    for (int i = 0; i < 15; i++){
        y[i] = h[i] - hD[i];
        dy[i] = dh[i] - dhD[i];
    }

    for (int i = 15; i < N; i++){
        y[i] = qSens[i]-Q_INIT[i];
        dy[i] = dqSens[i]-0;
    }

//    y[15] = h[15] - hD[15];
//    dy[15] = dh[15] - dhD[15];

//    y[19] = h[19] - hD[19];
//    dy[19] = dh[19] - dhD[19];


#ifdef REAL_ROBOT
     for (int i = 23; i < N; i++){
        Kd[i] = 0;
     }
    Kd[23] = 10;
    Kp[23] = 100;
    Kp[24] = -50;
    Kp[25] = 50;
    Kp[26] = 100;
    Kd[26] = 10;
    Kp[27] = 50;
    Kp[28] = 50;
    Kp[29] = -50;
    Kp[30] = -100;
#else
    for (int i = 0; i < N; i++)
    {
        Kp[i] = 300;
        Kd[i] = 1.5;
    }
Kp[3] = 300 + 0 * pow(kR, 3);
Kd[3] = 2.3;
Kp[4] = 300 + 0 * pow(kL, 3);
Kd[4] = 2.3;

#endif

    double VelZero[N];
    for (int i = 0; i < N; i++){
        VelZero[i] = 0;
    }
    VelZero[24] = 1;
    VelZero[25] = 1;
    VelZero[27] = 1;
    VelZero[28] = 1;
    VelZero[29] = 1;
    VelZero[30] = 1;

double MAX_VOLTAGE[N];
    for (int i = 0; i < 15; i++){
        MAX_VOLTAGE[i] = 12;  // for fd = 2.5 this was set to 15 volts
    }
    for (int i = 15; i < 23; i++){
        MAX_VOLTAGE[i] = 6;
    }
    for (int i = 23; i < N; i++){
        MAX_VOLTAGE[i] = 6;
    }

    const double H = 0.05;
    double xCopR = forceRightAnkle[2] < 50 ? -(torqueRightAnkle[1] + forceRightAnkle[0] * H) / 50 : -(torqueRightAnkle[1] + forceRightAnkle[0] * H) / forceRightAnkle[2];
    double xCopL = forceLeftAnkle[2] < 50 ? -(torqueLeftAnkle[1] + forceLeftAnkle[0] * H) / 50 :  -(torqueLeftAnkle[1] + forceLeftAnkle[0] * H) / forceLeftAnkle[2];
    double p_d = f0 == 0 ? 0 : 0;
    double t = f0 == 0 ? 0 : s / f0;
    double pxx0 = f0 == 0 ? 0 : px0;
    double vdes = (-px0 + x0) * f_d;
    double kVdes = x0 == 0 ? 0 : 1;
    double deltaPR = (pPelvis[0] - 0 - kVdes*(pxx0 + vdes*t));
    double deltaPL = (pPelvis[0] - 0 - kVdes*(pxx0 + vdes*t));
    double deltaV = (vxAbsF - vdes);
    double KpAR = deltaPR > 0 ? 370  : 370;
    double KpAL = deltaPL > 0 ? 370  : 370;
    double trqFeedFwd = 100 * x0 - 11.5; // 1 * f0 worked nicely for T = 0.5714, but for T = 0.4, 0.5 * f0 seems to work better maybe a nonlinear function is needed?
    if (x0 == 0){
        KpAR = deltaPR > 0 ? 370  : 370;
        KpAL = deltaPL > 0 ? 370  : 370;
    }
    double KpC = pPelvis[0] > 0 ?  30000000 : 0;
    if (x0 == 0){
        KpC = KpC * 0;
    }
    double KpL = 150;
    double purePosTrq = KpC * pow(pPelvis[0], 5) > 12.5 ? 12.5 : KpC * pow(pPelvis[0], 5);
    double tau8torque_d = 1 * (pPelvis[2] * forceRightAnkle[0] - pPelvis[0] * forceRightAnkle[2]) + (KpAR * deltaPR  + 30 * deltaV + purePosTrq) + trqFeedFwd;
    double tau13torque_d = 1 * (pPelvis[2] * forceLeftAnkle[0] - pPelvis[0] * forceLeftAnkle[2]) + (KpAL * deltaPL + 30 * deltaV + purePosTrq) + trqFeedFwd;
    double tau9torque_d = 1 * (pPelvis[1] * forceRightAnkle[2] - pPelvis[2] * forceRightAnkle[1]) - KpL * (pPelvis[1] - 0.1);
    double tau14torque_d = 1 * (pPelvis[1] * forceLeftAnkle[2] - pPelvis[2] * forceLeftAnkle[1]) - KpL * (pPelvis[1] + 0.1);

    #ifndef REAL_ROBOT
    double swP = 1; // 0.3
    double swT = 0; // 1

    for (int i = 0; i < N; i++){
        tauDes[i] = -Kp[i]*(y[i])-Kd[i]*(dy[i]);
        if (s >= 0){
            if (i == 8){
                tauDes[8] = ((1 - kR) * tauDes[8] + swP * kR * tauDes[8]) + swT * kR * (tau8torque_d + 0 * (torqueRightAnkle[2] + tau8torque_d));
                //            tauDes[8] = kR * tau8torque;
            }
            if (i == 13){
                tauDes[13] = ((1 - kL) * tauDes[13] + swP * kL * tauDes[13]) + swT * kL * (tau13torque_d + 0 * (torqueLeftAnkle[2] + tau13torque_d));
                //            tauDes[13] = kL * tau13torque;
            }
            //        if (i == 9){
            //            tauDes[9] = ((1 - kR) * tauDes[9] + 0.75 * kR * tauDes[9]) + 1 * kR * (tau9torque_d);
            //        }
            //        if (i == 14){
            //            tauDes[14] = ((1 - kL) * tauDes[14] + 0.75 * kL * tauDes[14]) + 1 * kL * (tau14torque_d);
            //        }
        }
        I[i] = (tauDes[i]+0*0.25+0)/(0.055*100);
        I[i] = I[i]>8 ? 8: (I[i]<-8 ? -8 : I[i]);
        vals[i] = 0.055*100*(1-VelZero[i])*dqSens[i]+2.5*I[i]; // 100 is the grear ratio
        vals[i] = vals[i] > MAX_VOLTAGE[i] ? MAX_VOLTAGE[i]: (vals[i] < -MAX_VOLTAGE[i] ? -MAX_VOLTAGE[i] : vals[i]);
    }
   // cout << swFtYD << " " << hSwFtY << " hswFtyD " << hSwFtYD << "TauDes5" << tauDes[5] << endl;
   tauAnkTorque[0] = kR * tau8torque_d;
   tauAnkTorque[1] = kL * tau13torque_d;

   #else
   double swP = 0.3; // 0.3
    double swT = 1; // 1

    for (int i = 0; i < N; i++){
        tauDes[i] = -Kp[i]*(y[i])-Kd[i]*(dy[i]);
        if (s >= 0){
            if (i == 8){
                tauDes[8] = ((1 - kR) * tauDes[8] + swP * kR * tauDes[8]) + swT * kR * (tau8torque_d + 0 * (torqueRightAnkle[2] + tau8torque_d));
                //            tauDes[8] = kR * tau8torque;
            }
            if (i == 13){
                tauDes[13] = ((1 - kL) * tauDes[13] + swP * kL * tauDes[13]) + swT * kL * (tau13torque_d + 0 * (torqueLeftAnkle[2] + tau13torque_d));
                //            tauDes[13] = kL * tau13torque;
            }
            //        if (i == 9){
            //            tauDes[9] = ((1 - kR) * tauDes[9] + 0.75 * kR * tauDes[9]) + 1 * kR * (tau9torque_d);
            //        }
            //        if (i == 14){
            //            tauDes[14] = ((1 - kL) * tauDes[14] + 0.75 * kL * tauDes[14]) + 1 * kL * (tau14torque_d);
            //        }
        }
        I[i] = (tauDes[i]+0*0.25+0)/(0.055*100);
        I[i] = I[i]>8 ? 8: (I[i]<-8 ? -8 : I[i]);
        vals[i] = 0.055*100*(1-VelZero[i])*dqSens[i]+2.5*I[i]; // 100 is the grear ratio
        vals[i] = vals[i] > MAX_VOLTAGE[i] ? MAX_VOLTAGE[i]: (vals[i] < -MAX_VOLTAGE[i] ? -MAX_VOLTAGE[i] : vals[i]);
    }
   // cout << swFtYD << " " << hSwFtY << " hswFtyD " << hSwFtYD << "TauDes5" << tauDes[5] << endl;
   tauAnkTorque[0] = kR * tau8torque_d;
   tauAnkTorque[1] = kL * tau13torque_d;
   #endif
}
