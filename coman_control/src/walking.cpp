#include "coman_control.hpp"
#include "walking_control/init_pos.hh"
#include "walking_control/Control.hh"
#include "R2Euler.hh"

#include <boost/bind.hpp>

using namespace geometry_msgs;

bool                            isInit = true;

const double                    TIME2WALK=10;

unsigned int                    whichComan_ = 1;

static int                      n;

static double                   dt = 0;
static double                   begin_time = -1.0;
static double                   _tm = 0;
static double                   Q0[NUM];
static double                   qknee0 = 0.2, qP0 = -0.1, qR0 = 0.055*1;
const double                    RIGHT_ELBOW_YAW = 0, LEFT_ELBOW_YAW = 0, HIP_YAW = 0;
static double                   qInit[] = {
                                0, 0.075, 0, qP0, qP0, -qR0, HIP_YAW, // 0 - 6
                                qknee0, qP0*1.4, qR0, qR0*1, HIP_YAW, // 7 - 11
                                qknee0, qP0*1.4, -qR0*1, // 12 - 14
            /* 15 onwards */    0.45, -0.2, 0.0, -1.75, 0.45, // 15 - 19
                                0.2, 0.0, -1.75, RIGHT_ELBOW_YAW, // 20 - 23
                                0.0, 0.0, LEFT_ELBOW_YAW, 0, 0, 0, 0 // 24 - 30
                                };

double                          vals[NUM], qSens[NUM], dqSens[NUM], 
                                tauSens[NUM], qSensAbs[NUM], tauDes[NUM];
double                          Trans[3][3];
double                          ImuAngRates[3], ImuAccelerations[3];
double                          forceRightAnkle[3], torqueRightAnkle[3], 
                                forceLeftAnkle[3], torqueLeftAnkle[3], 
                                forceRightHand[3], forceLeftHand[3];
double                          h[NUM], dh[NUM], hD[NUM], dhD[NUM];
double                          thr, thy, thp, euler[3], testOrientation[3];

vector< double >                vTime;

geometry_msgs::Quaternion       Q_imu;
geometry_msgs::Vector3          V_imu, A_imu;

Control                         control;


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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coman_control");
    ros::init(argc, argv, "coman_feedback");

    ros::NodeHandlePtr nhMain = 
        ros::NodeHandlePtr(new ros::NodeHandle("/coman_control"));
    ros::NodeHandlePtr nhSub1 = 
        ros::NodeHandlePtr(new ros::NodeHandle("~"));

    ros::Rate loop_rate = 1000;

    n = NUM;

    // Initialize controller
    coman_control controller(n, nhMain);
    
    // subcriber to joint states
    controller.jointStateFeedback( nhSub1 );
    //subsriber to get imu values
    controller.imuFeedback( nhSub1 );
    // subscriber to ft sensor
    controller.ftSensorFeedback( nhSub1 );

    while (nhMain->ok())
    {
        // Get Simulation Time
        _tm = controller.getSimulationTime();

        // store simulation time for future use.
        vTime.push_back( _tm );

        if( dt != 0 )
            dt = _tm - vTime.back();
        else
            dt += _tm;
        
        // Get IMU Feedback
        controller.getImuFeedback( &Q_imu, &V_imu, &A_imu );

        // Get force torque sensor Feedback
        controller.getftSensorFeedback( 
                    forceRightAnkle, 
                    forceLeftAnkle, 
                    forceRightHand, 
                    forceLeftHand, 
                    torqueRightAnkle, 
                    torqueLeftAnkle 
                    );

        // Get Joint State Sensor Feedback -> Add efforts if torque control is
        // wanted.
        controller.getStateFeedback( qSens, dqSens );

        // For init_pos
        if ( begin_time == -1 )
        {
            // Initialize tauDes to 0
            init( tauDes );

            // set begin_time
            begin_time = _tm;

            // Set QInit to qSens for init_pos
            for ( int i = 0; i < NUM; i++ )
            {
                Q0[i] = qSens[i]; // TODO: Change qSens to vector
            }
        }

        _tm -= begin_time;

        /* Debug IMU Data */
        // cout << dt << " : Magnitude should be 1 = " 
        //      << (Q_imu.x*Q_imu.x) + (Q_imu.y*Q_imu.y) + 
        //      (Q_imu.z*Q_imu.z) + (Q_imu.w*Q_imu.w) << " : Vel : " 
        //      << V_imu.x << endl;

        toEulerAngle( Q_imu, &thr, &thp, &thy, Trans );

        euler[0] = thr;
        euler[1] = thp;
        euler[2] = thy;

        // R2Euler(Trans, testOrientation);

        // toTrans( euler, Trans );

        // cout << "thpQ : " << thp << " : thrQ : " << thr << " : thp : " << testOrientation[1] << " : thr : " << testOrientation[0] << endl;  

        ImuAngRates[0] = V_imu.x;
        ImuAngRates[1] = V_imu.y;
        ImuAngRates[2] = V_imu.z;

        ImuAccelerations[0] = A_imu.x;
        ImuAccelerations[1] = A_imu.y;
        ImuAccelerations[2] = A_imu.z;

        // Enter init_pos
        if ( _tm < TIME2WALK && begin_time != -1 )
        {
            init_pos( _tm, Q0, qInit, qSens, dqSens, tauDes, whichComan_ );
        }
        else
        {
            control.LowerBody(
                        _tm, Q0, qSens, qSensAbs, dqSens, tauSens, 
                        forceRightAnkle, forceLeftAnkle, torqueRightAnkle, 
                        torqueLeftAnkle, forceRightHand, forceLeftHand, 
                        Trans, ImuAngRates, ImuAccelerations, h, dh, 
                        hD, dhD, tauDes, vals, dt, euler
                        );
        }

        for ( int i= 0; i< NUM; i++ )
        {
            double temp = tauDes[i];
            if( temp < 1000000000 && temp > -1000000000 )
                tauDes[i] = temp;
            else
                tauDes[i] = 0.0;
        }

        // publish joint effort commands
        controller.jointEffortControllers( tauDes );

        // dt is a variable to see loop rate size in secs in case loop_rate is
        // too fast and is making the code skip hooks.

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
