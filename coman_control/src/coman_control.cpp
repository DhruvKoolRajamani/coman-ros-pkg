#include "coman_control.hpp"

coman_control::coman_control( int nLinks, ros::NodeHandlePtr node_handle )
{
    ros::Publisher pubs_efforts;

    string jointName;

    n = nLinks;

    int j = 0;

    setNodeHandle( node_handle );

    for ( int i= 0; i< n; i++ )
    {
        // Initializing vector of strings for joint effort controllers in the spine
        node_handle->getParam(prefix + "/joint/" + to_string(i), jointName);
        revoluteJointsList.push_back(jointName);

        // cout << i << " : " << revoluteJointsList[i] << endl;

        string joint_control_topic_name = control_prefix + "/joint_effort_controller_" + jointName + "/command";
        jointEffortList.push_back(joint_control_topic_name);

        pubs_efforts = nh->advertise< std_msgs::Float64 > ( jointEffortList[i], callbackQueueSize );
        vec_pubs_efforts.push_back( pubs_efforts );

        // JointState feedback
        // if ( f3dSubscriberSet )
        // {
        //     string f3dFeedbackLegs_topic_name = prefix + "/forces/leg_" + to_string(i) + "_" + itSides->c_str() + "_" + "foot";
        //     f3dFeedbackLegsList.push_back( f3dFeedbackLegs_topic_name );
        // }
    }

    vector< string >::iterator itFtSensorTypes;
    for ( itFtSensorTypes= ftSensorTypes.begin(); itFtSensorTypes!= ftSensorTypes.end(); itFtSensorTypes++ )
    {
        string ftSensorTopicName = prefix + "/ft_sensor/" + itFtSensorTypes->c_str();
        ftSensorList.push_back( ftSensorTopicName );
    }
}

void coman_control::jointEffortControllers( double torques[] )
{
    std_msgs::Float64 TmpData;

    int count = 0;
    int j = 0;

    for ( int i= 0; i< NUM; i++ )
    {
        TmpData.data = torques[i];
        vec_pubs_efforts[i].publish(TmpData);
        count++;
    }
}

void coman_control::jointStateCallback( const sensor_msgs::JointState::ConstPtr & JS )
{
    jointState = *JS;
}

void coman_control::jointStateFeedback( ros::NodeHandlePtr n )
{   
    string topic = prefix + "/joint_states";
    jointStateSubscriber = n->subscribe( topic, 31, &coman_control::jointStateCallback, this );
}

void coman_control::f3dForceCallback( const geometry_msgs::WrenchStamped & WS )
{
    ROS_INFO( "stamp.sec: %d", WS.header.stamp.sec );
}

// Should be made async-> on another thread
void coman_control::f3dForceFeedback()
{
    if ( f3dSubscriberSet )
    {
        for ( int i= 0; i< f3dFeedbackLegsList.size(); i++ )
        {
            // cout << f3dFeedbackLegsList[i] << " : " << endl;
            f3dSubscriber = nh->subscribe( f3dFeedbackLegsList[i].c_str(), callbackQueueSize, &coman_control::f3dForceCallback, this );
        }
    }
}

void coman_control::imuCallback( const sensor_msgs::ImuConstPtr& IMU )
{
    qImu = IMU->orientation;
    vImu = IMU->angular_velocity;
    aImu = IMU->linear_acceleration;
}

void coman_control::imuFeedback( ros::NodeHandlePtr n )
{
    if( imuSubscriberSet )
    {
        imuSubscriber = n->subscribe( prefix + "/imu", callbackQueueSize, &coman_control::imuCallback, this );
    }
}

void coman_control::ftSensorCallbackRAS( const geometry_msgs::WrenchStamped::ConstPtr& WS )
{
    forceRightAnkle[0] = WS->wrench.force.x;
    forceRightAnkle[1] = WS->wrench.force.y;
    forceRightAnkle[2] = WS->wrench.force.z;
    torqueRightAnkle[0] = WS->wrench.torque.x;
    torqueRightAnkle[1] = WS->wrench.torque.y;
    torqueRightAnkle[2] = WS->wrench.torque.z;
}

void coman_control::ftSensorCallbackLAS( const geometry_msgs::WrenchStamped::ConstPtr& WS )
{
    forceLeftAnkle[0] = WS->wrench.force.x;
    forceLeftAnkle[1] = WS->wrench.force.y;
    forceLeftAnkle[2] = WS->wrench.force.z;
    torqueLeftAnkle[0] = WS->wrench.torque.x;
    torqueLeftAnkle[1] = WS->wrench.torque.y;
    torqueLeftAnkle[2] = WS->wrench.torque.z;
}

void coman_control::ftSensorCallbackRFS( const geometry_msgs::WrenchStamped::ConstPtr& WS )
{
    forceRightHand[0] = WS->wrench.force.x;
    forceRightHand[1] = WS->wrench.force.y;
    forceRightHand[2] = WS->wrench.force.z;
}

void coman_control::ftSensorCallbackLFS( const geometry_msgs::WrenchStamped::ConstPtr& WS )
{
    forceLeftHand[0] = WS->wrench.force.x;
    forceLeftHand[1] = WS->wrench.force.y;
    forceLeftHand[2] = WS->wrench.force.z;
}

void coman_control::ftSensorFeedback( ros::NodeHandlePtr nSub )
{
    ftSubscriberRFS = nSub->subscribe(
        ftSensorList[0].c_str(),
        callbackQueueSize,
        &coman_control::ftSensorCallbackRFS,
        this
    );
    ftSubscriberLFS = nSub->subscribe(
        ftSensorList[1].c_str(),
        callbackQueueSize,
        &coman_control::ftSensorCallbackLFS,
        this
    );
    ftSubscriberRAS = nSub->subscribe(
        ftSensorList[2].c_str(),
        callbackQueueSize,
        &coman_control::ftSensorCallbackRAS,
        this
    );
    ftSubscriberLAS = nSub->subscribe(
        ftSensorList[3].c_str(),
        callbackQueueSize,
        &coman_control::ftSensorCallbackLAS,
        this
    );
    // ftSubscriberLAS = nh->subscribe< geometry_msgs::WrenchStamped >(
    //     ftSensorList[3].c_str(),
    //     callbackQueueSize,
    //     boost::bind(&coman_control::ftSensorCallbackLAS, this, _1, i)
    // );
}

geometry_msgs::Quaternion coman_control::getImuQuat()
{
    return qImu;
}

geometry_msgs::Vector3 coman_control::getImuVel()
{
    return vImu;
}

geometry_msgs::Vector3 coman_control::getImuAcc()
{
    return aImu;
}

void coman_control::getftSensorFeedback( double _forceRightAnkle[], double _forceLeftAnkle[], double _forceRightHand[], double _forceLeftHand[], double _torqueLeftAnkle[], double _torqueRightAnkle[] )
{
    for ( int i= 0; i< 3; i++ )
    {
        _forceRightAnkle[i] = forceRightAnkle[i];
        _forceLeftAnkle[i] = forceLeftAnkle[i];
        _forceRightHand[i] = forceRightHand[i];
        _forceLeftHand[i] = forceLeftHand[i];
        _torqueRightAnkle[i] = torqueRightAnkle[i];
        _torqueLeftAnkle[i] = torqueLeftAnkle[i];
    }
}

void coman_control::setStateFeedback()
{
    for ( int i= 0; i< NUM; i++ )
    {
        for ( int j = 0; j < jointState.name.size(); j++ )
        {
            if (( revoluteJointsList[i].compare( jointState.name[j] ) ) == 0 )
            {
                qSens[i] = jointState.position[j];
                dqSens[i] = jointState.velocity[j];
            }
        }
        // cout << i << " : " << revoluteJointsList[i] << " : " << qSens[i] << endl;
    }
}

void coman_control::getStateFeedback( double _qSens[], double _dqSens[] )
{
    coman_control::setStateFeedback();

    for ( int i = 0; i < NUM; i++ )
    {
        _qSens[i] = qSens[i];
        _dqSens[i] = dqSens[i];
    }
}

double coman_control::getSimulationTime()
{
    return ros::Time::now().toSec();
}