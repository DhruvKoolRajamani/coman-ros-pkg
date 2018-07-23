#pragma once
#include "ros/ros.h"
#include "ros/spinner.h"
#include "ros/callback_queue.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"

#include <string>
#include <iostream>
#include <time.h>
#include <math.h>
#include <vector>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#define NUM 31

using namespace std;

// typedef const boost::function < void ( const sensor_msgs::JointState & )
//      > jsCallback;

typedef std::map< string, double >      MapType;

MapType                                 jointPositions;
MapType                                 jointVelocities;

class coman_control
{
    public:
    // coman_control(ros::NodeHandle *nh);
    coman_control( int nLinks, ros::NodeHandlePtr node_handle );
    ~coman_control(){}

    void setNodeHandle( ros::NodeHandlePtr node_handle ){ nh = node_handle; }
    
    void getImuFeedback(
                        geometry_msgs::Quaternion *_qImu,
                        geometry_msgs::Vector3 *_vImu,
                        geometry_msgs::Vector3 *_aImu
                        );

    void getftSensorFeedback(
            double _forceRightAnkle[],
            double _forceLeftAnkle[], 
            double _forceRightHand[], 
            double _forceLeftHand[], 
            double _torqueLeftAnkle[], 
            double _torqueRightAnkle[] 
            );
    
    void getStateFeedback( double _qSens[], double _dqSens[] );
    void setStateFeedback();
    
    void jointEffortControllers( double torques[] );

    void jointStateFeedback( ros::NodeHandlePtr n );
    void jointStateCallback( const sensor_msgs::JointState::ConstPtr & msg );

    void f3dForceFeedback();
    void f3dForceCallback( const geometry_msgs::WrenchStamped& msg );

    void imuFeedback( ros::NodeHandlePtr n );
    void imuCallback( const sensor_msgs::ImuConstPtr& msg );

    void ftSensorCallbackRAS( const geometry_msgs::WrenchStamped::ConstPtr& WS );
    void ftSensorCallbackLAS( const geometry_msgs::WrenchStamped::ConstPtr& WS );
    void ftSensorCallbackRFS( const geometry_msgs::WrenchStamped::ConstPtr& WS );
    void ftSensorCallbackLFS( const geometry_msgs::WrenchStamped::ConstPtr& WS );
    void ftSensorFeedback( ros::NodeHandlePtr nSub );

    double getSimulationTime();
    
    private:
    bool                                jointEffortPublish;
    bool                                f3dSubscriberSet = false;
    bool                                imuSubscriberSet = true;

    int                                 n;
    int                                 callbackQueueSize = 1;

    double                              _tm;
    double                              qSens[NUM], dqSens[NUM];
    double                              tauSens[NUM];
    double                              forceRightAnkle[3], 
                                        forceLeftAnkle[3], 
                                        forceRightHand[3], 
                                        forceLeftHand[3], 
                                        torqueLeftAnkle[3], 
                                        torqueRightAnkle[3];

    string                              prefix = "/island_1/robot_1"; // TODO: 
                                            // Load with ROSPARAM
    string                              control_prefix = 
                                            prefix + "/control/config";

    geometry_msgs::Quaternion           qImu;
    geometry_msgs::Vector3              vImu, aImu;

    geometry_msgs::WrenchStamped        rightAnkle, leftAnkle, 
                                        rightHand, leftHand;

    sensor_msgs::JointState             jointState;
    
    ros::Subscriber                     jointStateSubscriber;
    ros::Subscriber                     f3dSubscriber;
    ros::Subscriber                     imuSubscriber;
    ros::Subscriber                     ftSubscriberRAS, 
                                        ftSubscriberLAS, 
                                        ftSubscriberRFS, 
                                        ftSubscriberLFS;

    ros::NodeHandlePtr                  nh;

    vector< string >                    ftSensorTypes = 
                                            { "RFS", "LFS", "RAS", "LAS" };
    vector< string >                    jointEffortList;
    vector< string >                    f3dFeedbackLegsList;
    vector< string >                    revoluteJointsList;
    vector< string >                    ftSensorList;

    vector< ros::Publisher >            vec_pubs_efforts;

    vector< ros::Subscriber >           vec_ftSensor[4];
};