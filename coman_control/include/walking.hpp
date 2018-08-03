#pragma once

#include "coman_control.hpp"
#include "walking_control/init_pos.hh"
#include "walking_control/Control.hh"
#include "R2Euler.hh"

#include <fstream>
#include <boost/bind.hpp>

using namespace geometry_msgs;

void getRobotParams( ros::NodeHandlePtr nhParams, vector< string > namespaces, vector< string > LogPath, int *islands, vector< int > vRobots, vector< int > n )
{
    bool bIsl = false, bRob = false, bNs = false, bLog = false, bNum = false;
    int robots, _n, isl;

    vector< int > temp;
    vector< string > namespaceNames;

    bIsl = nhParams->getParam( "/islands", isl );
    *islands = isl;

    if ( bIsl )
    {
        for ( int i= 1; i< (*islands) + 1; i++ )
        {
            bRob = nhParams->getParam( "/island_" + to_string( i ) + "/robots", robots );
            vRobots.push_back( robots );

            for ( int j= 1; j< robots+1; j++ )
            {
                namespaceNames.push_back( "/namespace_" + to_string( i ) + "_" + to_string( j ) );
            }
        }

        for ( int i= 0; i< namespaceNames.size(); i++ )
        {
            string ns, joint, logs, rLogs;
            bNs = nhParams -> getParam( namespaceNames[i], ns );
            
            namespaces.push_back( ns );

            joint = ns + "n_joints";
            logs = ns + "log_path";
            
            bNum = nhParams -> getParam( joint, _n );
            temp.push_back( _n );
            
            bLog = nhParams -> getParam( logs, rLogs );
            LogPath.push_back( rLogs );
        }
    }

    n = temp;

    cout << n.back() << " : " << LogPath.back() << " : " << endl;
}

void init ( double torques[] )
{
    for ( int i= 0; i< NUM; i++ )
    {        
        torques[i] = 0.0;
    }
}

void calcQuaternion( const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *outQ )
{
    double norm = sqrt( q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w );

    outQ->x = q.x/norm;
    outQ->y = q.y/norm;
    outQ->z = q.z/norm;
    outQ->w = q.w/norm;
}

static void toEulerAngle( const geometry_msgs::Quaternion q, double *roll, double *pitch, double *yaw )
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	*roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		*pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		*pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	*yaw = atan2(siny, cosy);
}

void toTrans(double E_imu[3], double Trans[3][3])
{
    double thp, thr, thy;

    thr = E_imu[0];
    thp = E_imu[1];
    thy = E_imu[2];

    double c1 = cos(-thr); // 1
    double c2 = cos(-thp); // 2
    double c3 = cos(-thy); // 3
    double s1 = sin(-thr);
    double s2 = sin(-thp);            
    double s3 = sin(-thy);

    Trans[0][0] = c2*c3;
    Trans[0][1] = c1*s3 + c3*s1*s2;
    Trans[0][2] = s1*s3 - c1*c3*s2;
    Trans[1][0] = -c2*s3;
    Trans[1][1] = c1*c3 - s1*s2*s3;
    Trans[1][2] = c3*s1 + c1*s2*s3;
    Trans[2][0] = s2;
    Trans[2][1] = -c2*s1;
    Trans[2][2] = c1*c2;
}

void CalcRots( const geometry_msgs::Quaternion q, double Trans[][3], double *roll, double *pitch, double *yaw )
{
    double tempTrans[3][3];

    double q02 = q.w*q.w;
    double q12 = q.x*q.x;
    double q22 = q.y*q.y;
    double q32 = q.z*q.z;

    double q0 = q.w;
    double q1 = q.x;
    double q2 = q.y;
    double q3 = q.z;

    tempTrans[0][0] = 2*( q02 + q12 - 1/2 );
    tempTrans[0][1] = 2*( q1*q2 + q0*q3 );
    tempTrans[0][2] = 2*( q1*q3 - q0*q2 );
    tempTrans[1][0] = 2*( q1*q2 - q0*q3 );
    tempTrans[1][1] = 2*( q02 + q22 - 1/2 );
    tempTrans[1][2] = 2*( q2*q3 + q0*q1 );
    tempTrans[2][0] = 2*( q1*q3 + q0*q2 );
    tempTrans[2][1] = 2*( q2*q3 - q0*q1 );
    tempTrans[2][2] = 2*( q02 + q32 - 1/2 );

    *roll = asin( -tempTrans[0][2] );
    *pitch = asin( tempTrans[1][2]/tempTrans[2][2] );
    *yaw = atan2( tempTrans[0][1], tempTrans[0][0] );

    double c1 = cos(*roll); // 1
    double c2 = cos(*pitch); // 2
    double c3 = cos(*yaw); // 3
    double s1 = sin(*roll);
    double s2 = sin(*pitch);            
    double s3 = sin(*yaw);

    Trans[0][0] = c2*c3;
    Trans[0][1] = -c1*s3 + c3*s1*s2;
    Trans[0][2] = s1*s3 + c1*c3*s2;
    Trans[1][0] = c2*s3;
    Trans[1][1] = c1*c3 + s1*s2*s3;
    Trans[1][2] = -c3*s1 + c1*s2*s3;
    Trans[2][0] = -s2;
    Trans[2][1] = c2*s1;
    Trans[2][2] = c1*c2;
}

void SaveVars(
            std::ofstream &outputFile, double tm_, double *qSens, double *dqSens, 
            double *forceRightAnkle, double *forceLeftAnkle, double *torqueRightAnkle,
            double *torqueLeftAnkle, double *forceRightHand, double *forceLeftHand,
            double imuAngRates[], double imuAccelerations[], double tauDes[], 
            double trans[][3], double thp, double thr, int n
            )
{
    // Save Data  tme, qSens, qSensAbs, dqSens, tauSens, forceRightAnkle, forceLeftAnkle, torqueRightAnkle, torqueLeftAnkle, forceRightHand, forceLeftHand,forceSensors, trans, imuAngRates, imuAccelerations

    int N = n;

    outputFile << tm_;
            // start_id = 2
            for (int i = 0; i < N; i++){
                outputFile << " " << qSens[i];
            }
            // start_id = 2 + N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << dqSens[i];
            }
            // 2 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << forceRightAnkle[i];
            }
            // start_id = 5 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << forceLeftAnkle[i];
            }
            // start_id = 8 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << torqueRightAnkle[i];
            }
            // start_id = 11 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << torqueLeftAnkle[i];
            }
            // start_id = 14 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << forceRightHand[i];
            }
            // start_id = 17 + 2N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << forceLeftHand[i];
            }
            // start_id = 20 + 2N
            for (int i = 0; i < N; i++)
            {
                outputFile << " " << tauDes[i];
            }
            // start_id = 20 + 3N
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    outputFile << " " << trans[i][j];
                }
            }
            // start_id = 29 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << imuAngRates[i];
            }
            // start_id = 32 + 3N
            for (int i = 0; i < 3; i++)
            {
                outputFile << " " << imuAccelerations[i];
            }
            outputFile // start_id = 35 + 3N
                      << " " << thp
                      << " " << thr // 36 + 3N
                      << std::endl;
}

