#include "walking.hpp"

bool                            isInit = false;

const double                    TIME2WALK=10;

unsigned int                    whichComan_ = 1;

static int                      n, islands;

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
double                          Trans[3][3], testTrans[3][3];
double                          ImuAngRates[3], ImuAccelerations[3];
double                          forceRightAnkle[3], torqueRightAnkle[3], 
                                forceLeftAnkle[3], torqueLeftAnkle[3], 
                                forceRightHand[3], forceLeftHand[3];
double                          h[NUM], dh[NUM], hD[NUM], dhD[NUM];
double                          thr, thp, thy, euler[3], testOrientation[3];

string                          log_path;

vector< int >                   robots, n_joints;
vector< double >                vTime;
vector< string >                namespaces, LogPath;

geometry_msgs::Quaternion       Q_imu;
geometry_msgs::Vector3          V_imu, A_imu;

Control                         control;

static ifstream                 inputfile;
static ofstream                 outputfile, outputfileInit;

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coman_control");
    ros::init(argc, argv, "coman_feedback");

    ros::NodeHandlePtr nhMain = 
        ros::NodeHandlePtr(new ros::NodeHandle("/coman_control"));
    ros::NodeHandlePtr nhSub1 = 
        ros::NodeHandlePtr(new ros::NodeHandle("~"));
    ros::NodeHandlePtr nhParams = 
        ros::NodeHandlePtr(new ros::NodeHandle("~"));

    ros::Rate loop_rate = 1000;

    // getRobotParams( nhParams, namespaces, LogPath, &islands, robots, n_joints );
    n = NUM;

    nhParams -> getParam( "/log_path", log_path );
    
    string sOutputFile = log_path + "/tempdata.txt";
    outputfile.open( sOutputFile );

    string sInitOutputFile = log_path + "/tempdata_init.txt";
    outputfileInit.open( sInitOutputFile );

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

        calcQuaternion( Q_imu, &Q_imu );

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

        CalcRots( Q_imu, Trans, &thr, &thp, &thy );

        euler[0] = thr; // pitch
        euler[1] = thp; // yaw
        euler[2] = thy; // roll

        cout << "\nthr : " << euler[0] << " : thp : " << euler[1] << " : thy : " << euler[2] << endl;

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
            SaveVars( outputfileInit, _tm, qSens, dqSens, forceRightAnkle, forceLeftAnkle,
                        torqueRightAnkle, torqueLeftAnkle, forceRightHand, forceLeftHand,
                        ImuAngRates, ImuAccelerations, tauDes, Trans, euler[1], euler[0], n );
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
            control.SaveVars( outputfile );            
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
