//----------------------------------------------------------------------------------//
//                          STATE ESTIMATION FOR COMAN ROBOT                        //
//----------------------------------------------------------------------------------//
//                     This software was developed in BioRob Laboratory             //
//                         Ecole Polytechnique Federale de Lausanne                 //
// In collaboration with University of Pisa and Sant'Anna School of Advanced Studies//
// ---------------------------------------------------------------------------------//
//       Developed by Salvatore Alberto Buccellato  - albertobuccellato@gmail.com   //
//                              Supervised by Hamed Razavi                          //
// ---------------------------------------------------------------------------------//
// The purpose of this software is to estimate the coman global pose relying on data//
//                         acquired from IMU and joint encoders                     //
//----------------------------------------------------------------------------------//
#include<vector>
#include<math.h>
#define M5 5
#define M_LESS 500 // Time needed to initialize orientation
#define PITCH_ADJ -0.092 // Robot to IMU angle
#define N 31
#include "rotateIMU.hh"
#include "rot_vector.hh"
#include "forward_kinematics.hh"
#include "kin_initialize.hh"
#include "motionless_detect.h"
#include "or_integration.hh"
#include "acc_integration.hh"
#include "kinematics.hh"
#include "compute_matrix.hh"
#include "compute_x.hh"
#include <iostream>

static bool motionlessFlag = false;
static bool bias_bool = false;
static std::vector<double> orVec[3];
void state_estimation(std::vector<double> acc_vec[3], double imuAccelerations[3],
                      double imuAngRates[3], double avg_acc[3], double bias_acceleration[3], double variance[3],
                    double &mean_norm, double rotOrient[3][3], double position[3],
                    double p_left_w[3], double p_right_w[3],double orientation[4], double tm, double &tm_1,
                    double bias_orientation[3], double velocity[3], double P[21][21], double velocity_straight[3], double p_pelvis_left[3],
                    double p_pelvis_right[3], double F_left, double F_right, double qSensAbsMed[31])
{
        //defining kinematics joints in order to be consistent with the model
        double qLeftAbs[6];
        double qRightAbs[6];
        static int indexLeft[6] = {4,10,11,12,14,13}, indexRight[6] = {3,5,6,7,9,8};

        //time elapsed between two data acquisitions
        double dt = tm-tm_1;

        double p_left_m[3];
        double p_right_m[3];
        double D_x[21];
        double Dy[6];
        double pitch_adj = PITCH_ADJ;

        //rotation matrix from IMU to robot
        double rotAdj[3][3] = {
            {cos(pitch_adj),0,sin(pitch_adj)},
            {0,1,0},
            {-sin(pitch_adj),0,cos(pitch_adj)}
        };
        double t_yaw[3][3] = {0};

        //correction on axis convention of IMU angular rates
        imuAngRates[0] = - imuAngRates[0];
        imuAngRates[1] = - imuAngRates[1];
        imuAngRates[2] = - imuAngRates[2];

        tm_1 = tm;

        // rotating IMU data to robot body frame
        rotateIMU(rotAdj, imuAccelerations);
        rotateIMU(rotAdj, imuAngRates);

        // rotating IMU bias on acceleration
        if (bias_bool == false)
        {
            rotateIMU(rotAdj, bias_acceleration);
            rotateIMU(rotAdj, bias_orientation);
            bias_bool = true;
        }

        if (motionlessFlag == false && tm>10)
        {

            for (int i=0; i<3; i++)
            {
                // detecting a period without movement: this is used to initialize the orientation
                // relying on gravity
                motionless_detect(acc_vec[i], variance[i], avg_acc[i], imuAccelerations[i], M_LESS);
            }

            if (variance[0] < 0.00002 && variance[1] < 0.00002 && variance[2] < 0.00002 && acc_vec[0].size() == M_LESS)
            {
                for (int i = 0; i < 6; i++)
                {
                    // import data filter with median filter
                    qLeftAbs[i] = qSensAbsMed[indexLeft[i]];
                    qRightAbs[i] = qSensAbsMed[indexRight[i]];

                }

                double mean_x = avg_acc[0] - bias_acceleration[0];
                double mean_y = avg_acc[1] - bias_acceleration[1];
                double mean_z = avg_acc[2] - bias_acceleration[2];

                // compute norm of the accelerometer vector
                mean_norm = sqrt(mean_x*mean_x+mean_y*mean_y+mean_z*mean_z);

                double imuAccNorm[3] = {avg_acc[0]/mean_norm,avg_acc[1]/mean_norm,avg_acc[2]/mean_norm};

                // calculate initial orientation from gravity
                // rotOrient is the world to body rotation matrix
                rot_vector(rotOrient, orientation, imuAccNorm);

                for (int i=0;i<3;i++)
                {
                    // initialize acceleration
                    bias_acceleration[i] = bias_acceleration[i]/mean_norm*9.806365;
                }
                std::cout << mean_norm << std::endl;

                // compute kinematic chain
                forward_kinematics(qLeftAbs,qRightAbs,p_left_m,p_right_m,rotOrient, t_yaw);

                // compute initial pelvis position from kinematic chain
                position[0]=0;
                position[1]=0;
                position[2]= -p_left_m[2];

                // initialize feet position
                kin_initialize(qLeftAbs,qRightAbs,p_left_w,p_right_w,rotOrient,position);

                motionlessFlag = true;
            }
        }
        if (motionlessFlag == true)
        {

            // scale IMU accelerometer data
            imuAccelerations[0] = imuAccelerations[0]/mean_norm*9.806365;
            imuAccelerations[1] = imuAccelerations[1]/mean_norm*9.806365;
            imuAccelerations[2] = imuAccelerations[2]/mean_norm*9.806365;


            for (int i = 0; i < 6; i++)
            {
                qLeftAbs[i] = qSensAbsMed[indexLeft[i]];
                qRightAbs[i] = qSensAbsMed[indexRight[i]];

            }

            // Prediction step of the Kalman Filter (integration of IMU data)
            or_integration(orientation, imuAngRates, bias_orientation , rotOrient , dt);
            acc_integration(velocity, position, bias_acceleration, imuAccelerations, rotOrient, dt);

            // Compute kinematic chain (measurement)
            forward_kinematics(qLeftAbs,qRightAbs,p_left_m,p_right_m,rotOrient,t_yaw);

            // Prediction of kinematics and computation of difference between the two values
            kinematics(p_left_w,p_right_w,position,rotOrient,Dy,p_left_m,p_right_m, p_pelvis_left, p_pelvis_right, t_yaw);

            // Update step of Kalman Filter
            compute_matrix(imuAngRates,bias_orientation,imuAccelerations,bias_acceleration,
            rotOrient,p_left_w,p_right_w,position,D_x,P,Dy,F_left, F_right, dt);
            compute_x(position,velocity,orientation,p_left_w,p_right_w,bias_acceleration,
                      bias_orientation, D_x);

            // computing velocity without yaw component (for controller purposes)
            for (int i=0;i<3;i++)
            {
                velocity_straight[i] = t_yaw[0][i]*velocity[0] + t_yaw[1][i]*velocity[1] + t_yaw[2][i]*velocity[2];
            }
        }
}
