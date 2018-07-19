#include "coman_control.hpp"
#include "walking_control/init_pos.hh"

#include <boost/bind.hpp>

using namespace geometry_msgs;

bool                            isInit = true;

const double                    RIGHT_ELBOW_YAW = 0, LEFT_ELBOW_YAW = 0;
const double                    TIME2WALK=10;

unsigned int                    whichComan_ = 1;

static int                      n;

static double                   dt = 0;
static double                   begin_time = -1.0;
static double                   _tm = 0;
static double                   Q0[NUM];
static double                   qknee0 = 0.2, qP0 = -0.1, qR0 = 0.055*1;
static double                   qInit[] = {
                                0, 0.075, 0, qP0, qP0, -qR0, 0,
                                qknee0, qP0*1.4, qR0, qR0*1, 0,
                                qknee0, qP0*1.4, -qR0*1, 0.45,
                                -0.2, 0.0, -1.75, 0.45, 0.2, 0.0,
                                -1.75, RIGHT_ELBOW_YAW, 0.0, 0.0,
                                LEFT_ELBOW_YAW, 0, 0, 0, 0
                                };

double                          vals[NUM], qSens[NUM], dqSens[NUM], 
                                tauSens[NUM], qSensAbs[NUM], tauDes[NUM];
double                          Trans[3][3];
double                          ImuAngRates[3], ImuAccelerations[3];
double                          forceRightAnkle[3], torqueRightAnkle[3], 
                                forceLeftAnkle[3], torqueLeftAnkle[3], 
                                forceRightHand[3], forceLeftHand[3];
double                          h[NUM], dh[NUM], hD[NUM], dhD[NUM];

vector< double >                vTime;

geometry_msgs::Quaternion       Q_imu;
geometry_msgs::Vector3          V_imu, A_imu;


void init ( double torques[] )
{
    for ( int i= 0; i< NUM; i++ )
    {        
        torques[i] = 0.0;
    }
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
        
        // Get IMU Feedback
        controller.getImuFeedback( Q_imu, V_imu, A_imu );

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

        // Enter init_pos
        if ( _tm < TIME2WALK && begin_time != -1 )
        {
            double          y[NUM], dy[NUM], Kp[NUM], Kd[NUM], Q0_temp[NUM];
            double          pos_des, vel_des;
            double          ALPHA = 1;

            for ( int i= 0; i< NUM; i++ )
            {
                Q0_temp[i] = Q0[i];
            }

            Q0_temp[0] = 0;
            Q0_temp[12] = 0;
            
            for (int i = 0; i < 23; i++)
            {
                Kp[i] = 10;
                Kd[i] = 0.1;
            }

            // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15:Kp = 3:Kd = 0,
            //  
            const int JOINT_NUM = 15;
            Kp[JOINT_NUM] = 10;
            Kd[JOINT_NUM] = 0.1;

            for ( int i = 0; i < NUM; i++ )
            {
                if ( i == 15 )
                {
                    pos_des = qInit[i]+(Q0_temp[i]-qInit[i])*exp(-ALPHA * _tm);
                    vel_des = -ALPHA*(Q0_temp[i]-qInit[i])*exp(-ALPHA * _tm);
                    // y[i] = qSens[i]-pos_des;
                    // dy[i] = dqSens[i]-vel_des;
                    // y[i] = 0.0; //qSens[i] - 0.45*(1 - exp(-ALPHA * _tm));
                    // dy[i] = dqSens[i] + ALPHA*0.45*exp(-ALPHA * _tm);

                    y[i] = qSens[i] - qInit[i];
                    dy[i] = (Q0_temp[i] - qInit[i]) * exp( -ALPHA * _tm );                    
                }
                else if ( i == 24 )
                {
                    pos_des = qInit[i]+(Q0[i]-qInit[i])*exp(-ALPHA * _tm);
                    vel_des = -ALPHA*(Q0[i]-qInit[i])*exp(-ALPHA * _tm);
                    // y[i] = qSens[i]-pos_des;
                    // dy[i] = dqSens[i]-vel_des;
                    // y[i] = 0.0; //qSens[i] - 0.45*(1 - exp(-ALPHA * _tm));
                    // dy[i] = dqSens[i] + ALPHA*0.45*exp(-ALPHA * _tm);

                    y[i] = qSens[i] - qInit[i];
                    dy[i] = (Q0_temp[i] - qInit[i]) * exp( -ALPHA * _tm );                  
                }
                else
                {
                    // pos_des = qinit<vector> + 
                    // *(error -> sensor - qinit)*e^(-alpha*_tm)
                    pos_des = qInit[i]+(Q0_temp[i]-qInit[i])*exp(-ALPHA * _tm);
                    vel_des = -ALPHA*(Q0_temp[i]-qInit[i])*exp(-ALPHA * _tm);
                    y[i] = qSens[i]-pos_des;
                    dy[i] = dqSens[i]-vel_des;

                    // y[i] = qInit[i] - qSens[i];
                    // dy[i] = qInit[i] - qSens[i];
                }
                cout << _tm << " : " << i << " : " << qSens[i] << " : " 
                     << Q0_temp[i] << " : " << qInit[i] << " : " 
                     << pos_des << " : " << tauDes[i] << endl;
            }

            for ( int i= 0; i< NUM; i++ )
            {
                double temp;
                temp = -Kp[i]*(y[i]) - Kd[i]*(dy[i]);
                if( temp < 100000000 && temp > -100000000 )
                    tauDes[i] = temp;
                else
                    cout << i << "\n******************************\n";
            }
        }

        /* Debug IMU Data */
        // cout << dt << " : Magnitude should be 1 = " 
        //      << (Q_imu.x*Q_imu.x) + (Q_imu.y*Q_imu.y) + 
        //      (Q_imu.z*Q_imu.z) + (Q_imu.w*Q_imu.w) << " : " 
        //      << V_imu.x << endl;

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

        // publish joint effort commands
        controller.jointEffortControllers( tauDes );

        // dt is a variable to see loop rate size in secs in case loop_rate is
        // too fast and is making the code skip hooks.
        if( dt != 0 )
            dt = _tm - vTime.back();
        else
            dt += _tm;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
