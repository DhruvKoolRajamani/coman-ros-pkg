#include "coman_control.hpp"
#include "walking_control/init_pos.hh"

#include <boost/bind.hpp>

using namespace geometry_msgs;

const double RIGHT_ELBOW_YAW = 0, LEFT_ELBOW_YAW = 0;
const double TIME2WALK=10;

unsigned int whichComan_ = 1;

static int n;

static double dt = 0;
static double begin_time = -1;
static double _tm = 0;
static double Q0[NUM];
static double qknee0 = 0.2, qP0 = -0.1, qR0 = 0.055*1;
static double qInit[] = {
    0, 0.075, 0, qP0, qP0, -qR0, 0,
    qknee0, qP0*1.4, qR0, qR0*1, 0,
    qknee0, qP0*1.4, -qR0*1, 0.45,
    -0.2, 0.0, -1.75, 0.45, 0.2, 0.0,
    -1.75, RIGHT_ELBOW_YAW, 0.0, 0.0,
    LEFT_ELBOW_YAW, 0, 0, 0, 0
    };

// static double qInit[NUM];

double vals[NUM], qSens[NUM], dqSens[NUM], tauSens[NUM], qSensAbs[NUM], tauDes[NUM];
double Trans[3][3];
double ImuAngRates[3], ImuAccelerations[3];
double forceRightAnkle[3], torqueRightAnkle[3], forceLeftAnkle[3], torqueLeftAnkle[3], forceRightHand[3], forceLeftHand[3];
double h[NUM], dh[NUM], hD[NUM], dhD[NUM];

vector< double > vTime;

void init ( double torques[] )
{
    for ( int i= 0; i< NUM; i++ )
    {        
        torques[i] = 0.0;
        // qInit[i] = 0.0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coman_control");
    ros::init(argc, argv, "coman_feedback");

    ros::NodeHandlePtr nhMain = ros::NodeHandlePtr(new ros::NodeHandle("/coman_control"));
    ros::NodeHandlePtr nhSub1 = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    ros::Rate loop_rate = 1000;

    if( nhMain->getParam( "/island_1/robot_1/n_joints", n ) )
    {
        // Initialize controller
        coman_control controller(n, nhMain);

        geometry_msgs::Quaternion Q_imu;
        geometry_msgs::Vector3 V_imu, A_imu;

        // cout << dt << " : ";
        // subcriber to joint states
        controller.jointStateFeedback( nhSub1 );
        //subsriber to get imu values
        controller.imuFeedback( nhSub1 );
        // subscriber to ft sensor
        controller.ftSensorFeedback( nhSub1 );

        while (nhMain->ok())
        {
            if ( begin_time == -1 )
            {
                sleep(0.01);
            }

            _tm = controller.getSimulationTime();

            Q_imu = controller.getImuQuat();
            V_imu = controller.getImuVel();
            A_imu = controller.getImuAcc();

            // cout << dt << " : Magnitude should be 1 = " << (Q_imu.x*Q_imu.x) + (Q_imu.y*Q_imu.y) + (Q_imu.z*Q_imu.z) + (Q_imu.w*Q_imu.w) << " : " << V_imu.x << endl;

            Trans[0][0] = 2*((Q_imu.x*Q_imu.x) + (Q_imu.w*Q_imu.w)) - 1;
            Trans[0][1] = 2*((Q_imu.x*Q_imu.y) - (Q_imu.w*Q_imu.z));
            Trans[0][2] = 2*((Q_imu.x*Q_imu.z) + (Q_imu.w*Q_imu.y));
            Trans[1][0] = 2*((Q_imu.x*Q_imu.y) + (Q_imu.w*Q_imu.z));
            Trans[1][1] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.y*Q_imu.y)) - 1;
            Trans[1][2] = 2*((Q_imu.y*Q_imu.z) - (Q_imu.w*Q_imu.x));
            Trans[2][0] = 2*((Q_imu.x*Q_imu.z) - (Q_imu.w*Q_imu.y));
            Trans[2][1] = 2*((Q_imu.y*Q_imu.z) + (Q_imu.w*Q_imu.x));
            Trans[2][2] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.z*Q_imu.z)) - 1;

            ImuAngRates[0] = V_imu.x;
            ImuAngRates[1] = V_imu.y;
            ImuAngRates[2] = V_imu.z;

            ImuAccelerations[0] = A_imu.x;
            ImuAccelerations[1] = A_imu.y;
            ImuAccelerations[2] = A_imu.z;

            controller.getftSensorFeedback( forceRightAnkle, forceLeftAnkle, forceRightHand, forceLeftHand, torqueRightAnkle, torqueLeftAnkle );

            controller.getStateFeedback( qSens, dqSens );

            // double temp;
            // for ( int i= 0; i< NUM; i++ )
            // {
            //     if ( qSens[i] < 0.001 )
            //     {
            //         temp = 0;
            //     }
            //     else
            //         temp = qSens[i];
            //     cout << i << " : " << temp << endl;
            // }

            if( dt != 0 )
                dt = _tm - vTime.back();
            else
                dt += _tm;

            vTime.push_back( _tm );

            if ( begin_time == -1 )
            {
                begin_time = _tm;
                for ( int i = 0; i < NUM; i++ )
                {
                    Q0[i] = qSens[i];
                }
                init( tauDes );
            }

            if ( _tm < TIME2WALK )
            {
                init_pos( _tm, Q0, qInit, qSens, dqSens, tauDes, whichComan_ );
            }

            // cout << dt << endl;

            // publish joint effort commands
            controller.jointEffortControllers( tauDes );

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
