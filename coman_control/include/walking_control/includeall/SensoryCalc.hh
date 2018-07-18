#include "saturate.hh"

#ifdef REAL_ROBOT
#include <comanepfl/robot.hh>
#else
#include <coman/webots/robotwebots.hh>
#endif
#ifdef REAL_ROBOT
#define N 31
#else
#define N 23
#endif
const double RIGHT_KNEE_BIAS = 0;
const double RIGHT_ANK_PITCH_BIAS = -0.02;
const double LEFT_ANK_PITCH_BIAS = 0.04;
const double EPSILON = 0.000001;

class SensoryCalc
{
public:
    SensoryCalc(coman::SensorData sData): S_(sData){}
    double GetTime();
    void Get_q(double *q);
    void ApplyBias_q(double *q);
    void Get_qAbs(double *qAbs);
    void Get_dq(double *dq);
    void Get_tau(double *tau);
    void Get_imuTrans(double trans[][3]);
    void Get_imuAngRate(double *imuAngRates);
    void Get_imuAccelerations(double *imuAccelerations);
    void Get_ankForces(double *forceRightAnkle, double *forceLeftAnkle, double *torqueRightAnkle, double *torqueLeftAnkle, double *forceRightHand, double *forceLeftHand);
    void Get_all(double *q, double *dq, double *qAbs, double *tau, double trans[][3], double *imuAngRates, double *imuAccelerations);
private:
    coman::SensorData S_;
};

double SensoryCalc::GetTime()
{
    return S_.TimeStamp.BaseValue();
}

void SensoryCalc::Get_q(double *q)
{
    for (int i = 0; i < N; i++)
    {
        q[i] = S_.MotorSensors[i].Position.Convert(coman::Units::Radian).Value;
    }
}
void SensoryCalc::ApplyBias_q(double *q)
{
    q[7] += RIGHT_KNEE_BIAS;
    q[8] += RIGHT_ANK_PITCH_BIAS;
    q[13] += LEFT_ANK_PITCH_BIAS;
}

void SensoryCalc::Get_qAbs(double *qAbs)
{
    for (int i = 0; i < N; i++)
    {
        qAbs[i] = S_.MotorSensors[i].AbsolutePosition.Convert(coman::Units::Radian).Value;
    }
}
void SensoryCalc::Get_dq(double *dq)
{
    for (int i = 0; i < N; i++)
    {
        dq[i] = S_.MotorSensors[i].Velocity.Convert(coman::Units::Radian).Value;
    }
}
void SensoryCalc::Get_tau(double *tau)
{
    for (int i = 0; i < N; i++)
    {
        tau[i] = S_.MotorSensors[i].Torque.Convert(coman::Units::Newton).Value;
    }
}
void SensoryCalc::Get_imuTrans(double trans[][3])
{
    trans[0][0] = S_.ImuSensor.Orientation[0].Value;
    trans[0][1] = S_.ImuSensor.Orientation[3].Value;
    trans[0][2] = S_.ImuSensor.Orientation[6].Value;
    trans[1][0] = S_.ImuSensor.Orientation[1].Value;
    trans[1][1] = S_.ImuSensor.Orientation[4].Value;
    trans[1][2] = S_.ImuSensor.Orientation[7].Value;
    trans[2][0] = S_.ImuSensor.Orientation[2].Value;
    trans[2][1] = S_.ImuSensor.Orientation[5].Value;
    trans[2][2] = S_.ImuSensor.Orientation[8].Value;
}

void SensoryCalc::Get_imuAngRate(double *imuAngRates)
{
    imuAngRates[0] = S_.ImuSensor.AngularRate[0].Value;
    imuAngRates[1] = S_.ImuSensor.AngularRate[1].Value;
    imuAngRates[2] = S_.ImuSensor.AngularRate[2].Value;
}

void SensoryCalc::Get_imuAccelerations(double *imuAccelerations)
{
    imuAccelerations[0] = S_.ImuSensor.Acceleration[0].Value;
    imuAccelerations[1] = S_.ImuSensor.Acceleration[1].Value;
    imuAccelerations[2] = S_.ImuSensor.Acceleration[2].Value;
}

void SensoryCalc::Get_ankForces(double *forceRightAnkle, double *forceLeftAnkle, double *torqueRightAnkle, double *torqueLeftAnkle, double *forceRightHand, double *forceLeftHand)
{
    for (int i =0; i < 3; i++)
    {
        forceRightAnkle[i] = S_.ForceTorqueSensors[coman::Sensors::RIGHT_FOOT_FT_SENSOR].Force[i].Convert(coman::Units::Newton).Value;
        torqueRightAnkle[i] = S_.ForceTorqueSensors[coman::Sensors::RIGHT_FOOT_FT_SENSOR].Torque[i].Convert(coman::Units::Newton).Value;
        forceLeftAnkle[i] = S_.ForceTorqueSensors[coman::Sensors::LEFT_FOOT_FT_SENSOR].Force[i].Convert(coman::Units::Newton).Value;
        torqueLeftAnkle[i] = S_.ForceTorqueSensors[coman::Sensors::LEFT_FOOT_FT_SENSOR].Torque[i].Convert(coman::Units::Newton).Value;
#ifdef REAL_ROBOT
            forceRightHand[i] = S_.ForceTorqueSensors[coman::Sensors::RIGHT_HAND_FT_SENSOR].Force[i].Convert(coman::Units::Newton).Value;
            forceLeftHand[i] = S_.ForceTorqueSensors[coman::Sensors::LEFT_HAND_FT_SENSOR].Force[i].Convert(coman::Units::Newton).Value;
#endif
    }
//    double forceLeftAnkleZ = forceLeftAnkle[2];
//    double forceRightAnkleZ = forceRightAnkle[2];

//    double f1 = pow((pow(forceRightAnkle[0],2)+pow(forceRightAnkle[1],2)+pow(forceRightAnkleZ,2)),0.5);
//    double f2 = pow((pow(forceLeftAnkle[0],2)+pow(forceLeftAnkle[1],2)+pow(forceLeftAnkleZ,2)),0.5);
//    k = f1 / (f1 + f2 + EPSILON);
}

void SensoryCalc::Get_all(double *q, double *dq, double *qAbs, double *tau, double trans[][3], double *imuAngRates, double *imuAccelerations){
    Get_q(q);
    Get_dq(dq);
    Get_qAbs(qAbs);
    Get_tau(tau);
    Get_imuTrans(trans);
    Get_imuAngRate(imuAngRates);
    Get_imuAccelerations(imuAccelerations);
}





