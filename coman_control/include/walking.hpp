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

static void toEulerAngle(const geometry_msgs::Quaternion q, double *roll, double *pitch, double *yaw, double Trans[][3] )
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

    Trans[0][0] = 2*((q.x*q.x) + (q.w*q.w)) - 1;
    Trans[0][1] = 2*((q.x*q.y) - (q.w*q.z));
    Trans[0][2] = 2*((q.x*q.z) + (q.w*q.y));
    Trans[1][0] = 2*((q.x*q.y) + (q.w*q.z));
    Trans[1][1] = 2*((q.w*q.w) + (q.y*q.y)) - 1;
    Trans[1][2] = 2*((q.y*q.z) - (q.w*q.x));
    Trans[2][0] = 2*((q.x*q.z) - (q.w*q.y));
    Trans[2][1] = 2*((q.y*q.z) + (q.w*q.x));
    Trans[2][2] = 2*((q.w*q.w) + (q.z*q.z)) - 1;
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
