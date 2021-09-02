#define LEFT_MOTOR_PPR 1079
#define RIGHT_MOTOR_PPR 1079

#define MOTOR_TICKS_CURRENT_INDEX 0
#define MOTOR_TICKS_PREVIOUS_INDEX 1

#define LEFT_MOTOR_INDEX 0
#define RIGHT_MOTOR_INDEX 1

#define LEFT_SONAR_INDEX 0
#define MIDDLE_SONAR_INDEX 1
#define RIGHT_SONAR_INDEX 2

#define MASK_16_BIT 65535

#define NSEC_TO_MIN_DENOM 60000000000


#include "diffbot_hardware_interface/diffbot_hardware_interface.h"

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    LeftWheelTicksSub = nh_.subscribe("LeftWheelTicks", 1000, &ROBOTHardwareInterface::LeftWheelTicksCallback, this);
    RightWheelTicksSub = nh_.subscribe("RightWheelTicks", 1000, &ROBOTHardwareInterface::RightWheelTicksCallback, this);

    LeftWheelVelocityPub = nh_.advertise<std_msgs::Int32>("LeftWheelVelocity", 1000);
    RightWheelVelocityPub = nh_.advertise<std_msgs::Int32>("RightWheelVelocity", 1000);

    LeftWheelCurrentRpmPub = nh_.advertise<std_msgs::Int32>("LeftWheelCurrentRpm", 1000);
    RightWheelCurrentRpmPub = nh_.advertise<std_msgs::Int32>("RightWheelCurrentRpm", 1000);

    LeftWheelTargetRpmPub = nh_.advertise<std_msgs::Int32>("LeftWheelTargetRpm", 1000);
    RightWheelTargetRpmPub = nh_.advertise<std_msgs::Int32>("RightWheelTargetRpm", 1000);

    SonarSub = nh_.subscribe("SonarDistances", 1000, &ROBOTHardwareInterface::SonarDistancesCallback, this);

    sonarDistanceFrames[LEFT_SONAR_INDEX] = "ultra_left";
    sonarDistanceFrames[MIDDLE_SONAR_INDEX] = "ultra_middle";
    sonarDistanceFrames[RIGHT_SONAR_INDEX] = "ultra_right";
    SonarRangePubs[LEFT_SONAR_INDEX] = nh_.advertise<sensor_msgs::Range>("/ultra/left_scan", 1000);
    SonarRangePubs[MIDDLE_SONAR_INDEX] = nh_.advertise<sensor_msgs::Range>("/ultra/middle_scan", 1000);
    SonarRangePubs[RIGHT_SONAR_INDEX] = nh_.advertise<sensor_msgs::Range>("/ultra/right_scan", 1000);


    TestPub = nh_.advertise<std_msgs::Int32>("TestPos", 1000);
		
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
	
	for(int i=0; i<2; i++)
	{
        joint_position_[i] = 0;
        joint_velocity_[i] = 0;
        joint_effort_[i] = 0;

	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);

    SonarDistanceMsgSeq = 0;
}

void ROBOTHardwareInterface::LeftWheelTicksCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int prevMotorTicks = LeftMotorTicks[MOTOR_TICKS_CURRENT_INDEX];
    LeftMotorTicks[MOTOR_TICKS_CURRENT_INDEX] = msg->data;
    double diffTicks = LeftMotorTicks[MOTOR_TICKS_CURRENT_INDEX] - LeftMotorTicks[MOTOR_TICKS_PREVIOUS_INDEX];
    LeftMotorTicks[MOTOR_TICKS_PREVIOUS_INDEX] = LeftMotorTicks[MOTOR_TICKS_CURRENT_INDEX];

    double rad = (diffTicks / LEFT_MOTOR_PPR) * 2 * M_PI;

    joint_position_[LEFT_MOTOR_INDEX] += rad;

    ros::Time timestamp = ros::Time::now();

    ros::Duration timeDiff = timestamp - PreviousTimeStampTicksCallback[LEFT_MOTOR_INDEX];
    int64_t ticksDiff = msg->data - prevMotorTicks;

    std_msgs::Int32 rpmMsg;
    rpmMsg.data = ((diffTicks*60*100)/2)/LEFT_MOTOR_PPR;//(ticksDiff*NSEC_TO_MIN_DENOM) / timeDiff.toNSec();
    LeftWheelCurrentRpmPub.publish(rpmMsg);

    PreviousTimeStampTicksCallback[LEFT_MOTOR_INDEX] = timestamp;
}

void ROBOTHardwareInterface::RightWheelTicksCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int prevMotorTicks = RightMotorTicks[MOTOR_TICKS_CURRENT_INDEX];
    RightMotorTicks[MOTOR_TICKS_CURRENT_INDEX] = msg->data;
    double diffTicks = RightMotorTicks[MOTOR_TICKS_CURRENT_INDEX] - RightMotorTicks[MOTOR_TICKS_PREVIOUS_INDEX];
    RightMotorTicks[MOTOR_TICKS_PREVIOUS_INDEX] = RightMotorTicks[MOTOR_TICKS_CURRENT_INDEX];

    double rad = (diffTicks / RIGHT_MOTOR_PPR) * 2 * M_PI;

    joint_position_[RIGHT_MOTOR_INDEX] += rad;

    ros::Time timestamp = ros::Time::now();

    ros::Duration timeDiff = timestamp - PreviousTimeStampTicksCallback[RIGHT_MOTOR_INDEX];
    int64_t ticksDiff = msg->data - prevMotorTicks;

    std_msgs::Int32 rpmMsg;
    rpmMsg.data = ((diffTicks*60*100)/2)/RIGHT_MOTOR_PPR ;//(ticksDiff*NSEC_TO_MIN_DENOM) / timeDiff.toNSec();
    RightWheelCurrentRpmPub.publish(rpmMsg);

    PreviousTimeStampTicksCallback[RIGHT_MOTOR_INDEX] = timestamp;
}

void ROBOTHardwareInterface::SonarDistancesCallback(const std_msgs::Int64::ConstPtr& msg)
{
    uint seq = SonarDistanceMsgSeq;
    ros::Time nowStamp = ros::Time::now();
    for (int i = 0; i < SonarRangePubs.size(); i++)
    {
        sensor_msgs::Range msgRange;
        msgRange.header.seq = SonarDistanceMsgSeq;
        msgRange.header.stamp = nowStamp;
        msgRange.header.frame_id = sonarDistanceFrames[i];
        msgRange.radiation_type = msgRange.ULTRASOUND;
        msgRange.field_of_view = 0.1;
        msgRange.min_range = 0.05;
        msgRange.max_range = 2;
        msgRange.range = ((float) ( ((msg->data) >> (i * 16)) &  MASK_16_BIT) ) / 100;

        if((msgRange.range < 1.5) && (i != 1))
        {
            SonarRangePubs[i].publish(msgRange);
        }
    }

    SonarDistanceMsgSeq ++; 
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

    std_msgs::Int32 velocityLeft;
    std_msgs::Int32 velocityRight;

    velocityLeft.data = this->rad_per_s_TO_ppm(joint_velocity_command_[LEFT_MOTOR_INDEX], LEFT_MOTOR_PPR);
    velocityRight.data = this->rad_per_s_TO_ppm(joint_velocity_command_[RIGHT_MOTOR_INDEX], RIGHT_MOTOR_PPR);

    LeftWheelVelocityPub.publish(velocityLeft);
    RightWheelVelocityPub.publish(velocityRight);

    std_msgs::Int32 targetRpmLeft;
    std_msgs::Int32 targetRpmRight;

    targetRpmLeft.data = velocityLeft.data / LEFT_MOTOR_PPR;
    targetRpmRight.data = velocityRight.data / RIGHT_MOTOR_PPR ;

    LeftWheelTargetRpmPub.publish(targetRpmLeft);
    RightWheelTargetRpmPub.publish(targetRpmRight);
}

int ROBOTHardwareInterface::rad_per_s_TO_ppm(double rad, int ppr)
{
    int ppm = (rad * ppr * 60) / (2 * M_PI);
    return ppm;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}