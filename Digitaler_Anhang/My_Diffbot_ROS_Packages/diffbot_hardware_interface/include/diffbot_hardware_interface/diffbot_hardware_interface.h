#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "sensor_msgs/Range.h"

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ros::ServiceClient client;
        rospy_tutorials::Floats joints_pub;
        void LeftWheelTicksCallback(const std_msgs::Int32::ConstPtr& msg);
        void RightWheelTicksCallback(const std_msgs::Int32::ConstPtr& msg);
        void SonarDistancesCallback(const std_msgs::Int64::ConstPtr& msg);
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;


        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void write(ros::Duration elapsed_time);

        //three_dof_planar_manipulator::Floats_array joint_read;
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        
        std::string joint_name_[2]={"left_wheel_joint","right_wheel_joint"};  
        double joint_position_[2];
        double joint_velocity_[2];
        double joint_effort_[2];
        double joint_velocity_command_[2];
        
        ros::Subscriber LeftWheelTicksSub;
        ros::Subscriber RightWheelTicksSub;

        ros::Publisher LeftWheelVelocityPub;
        ros::Publisher RightWheelVelocityPub;

        ros::Publisher LeftWheelCurrentRpmPub;
        ros::Publisher RightWheelCurrentRpmPub;

        ros::Publisher LeftWheelTargetRpmPub;
        ros::Publisher RightWheelTargetRpmPub;

        ros::Subscriber SonarSub;
        std::array<ros::Publisher, 3> SonarRangePubs;

        ros::Publisher TestPub;
	
	    double left_motor_pos=0,right_motor_pos=0;
        int left_prev_cmd=0, right_prev_cmd=0;


        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        //boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    private:
        int LeftMotorTicks[2];
        int RightMotorTicks[2];

        std::string sonarDistanceFrames[3];

        uint SonarDistanceMsgSeq;

        ros::Time PreviousTimeStampTicksCallback[2];

        int rad_per_s_TO_ppm(double rad, int ppr);
};