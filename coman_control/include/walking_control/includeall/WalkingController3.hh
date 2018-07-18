#ifndef WALKINGCONTROLLER3_H
#define WALKINGCONTROLLER3_H

class WalkingController3
{
public:
    WalkingController3();
    void EvalOutputs(double s, double f0, double Q_INIT[], double qSens[], double dqSens[], double kR, double kL, int indexSt[], int indexSw[], double thpF,
                     double dthpF, double thr, double dthrF, double x0, double deltaX, double deltaHipPitchvy, double qSensInit[], double pSw_init[],
                     double thpF_init, double thrF_init, double px0, double P_SWFTinH[], double vSwFtInH[], double Or_SWFTinH[], double dorSwFt[],
                     double orSwFtInit[], double h[], double dh[], double hD[], double dhD[], double STEP_LENGTH, double QswKMid);

    void EvalTorques(double s, double tInStep, double f_d, double f0, double STEP_LENGTH, double px0, double Q_INIT[], double qSens[], double dqSens[], double kR, double kL,  double orSwFt[],
                                         double tauAnkTorque[], double forceRightAnkle[], double forceLeftAnkle[], double torqueRightAnkle[], double torqueLeftAnkle[], double pPelvis[], double vxAbsF, double h[], double dh[], double hD[], double dhD[], double tauDes[], double vals[]);

    unsigned int whichComan_ = 0;
};

#endif // WALKINGCONTROLLER2_H
