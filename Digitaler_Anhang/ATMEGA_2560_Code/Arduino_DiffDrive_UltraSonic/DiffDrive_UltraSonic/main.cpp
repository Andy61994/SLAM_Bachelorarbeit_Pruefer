#undef main

#define MAX_RPM 200

#define PPR_MOTOR_LEFT 1079 //226
#define PPR_MOTOR_RIGHT 1079 //494

#define MIN_ERROR_MOTOR_LEFT ((-1L) * MAX_RPM * PPR_MOTOR_LEFT)
#define MIN_ERROR_MOTOR_RIGHT ((-1L) * MAX_RPM * PPR_MOTOR_RIGHT)

#define MAX_ERROR_MOTOR_LEFT (1L * MAX_RPM * PPR_MOTOR_LEFT)
#define MAX_ERROR_MOTOR_RIGHT (1L * MAX_RPM * PPR_MOTOR_RIGHT)

#define MIN_CONTROL_VALUE_MOTOR_LEFT -7999
#define MIN_CONTROL_VALUE_MOTOR_RIGHT -7999

#define MAX_CONTROL_VALUE_MOTOR_LEFT 7999
#define MAX_CONTROL_VALUE_MOTOR_RIGHT 7999

#define OBSTACLE_DETECTED_LEFT 1
#define OBSTACLE_DETECTED_MIDDLE 2
#define OBSTACLE_DETECTED_RIGHT 4
#define UNSTUCKED_EVENT 8

#define AVOID_MODE 1
#define UNSTUCK_MODE 2
#define CHAOS_MODE 3
#define GO_FAR_MODE 4
#define TELEOP_MODE 5


#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>

#include "timer.h"
#include "Encoder_Tracker.h"
#include "DC_Motor_Driver.h"
#include "Ultra_Sonic_Range_Finder_Driver.h"
#include "PidController.h";


// Create Motor Instances
DC_Motor_Driver motorLeft(PWM_0, PPR_MOTOR_LEFT);	
DC_Motor_Driver motorRight(PWM_1, PPR_MOTOR_RIGHT);

// Create UltraSonic Instances
Ultra_Sonic_Range_Finder_Driver ultraLeft(INT_2);
Ultra_Sonic_Range_Finder_Driver ultraMiddle(INT_3);
Ultra_Sonic_Range_Finder_Driver ultraRight(INT_5);

// Declare ROS Subscriber Callback Functions
void leftWheelVelCallback(const std_msgs::Int32& msg);
void RightWheelVelCallback(const std_msgs::Int32& msg);


// Declare some utility functions used for Exploration Mode for testing purposes of Sonar Sensors. Reliability was approved
/*
void turnLeft(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight);
void turnRight(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight);
void goStraight(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight);
*/

int main()
{
	init();
	
	//Setup Timers

	uint8_t controlLoopTimerId = declareTimer(20, 1, Elapsed);
	startTimer(controlLoopTimerId);
	
	uint8_t ultraSonarTimerId = declareTimer(200, 1, Elapsed);
	startTimer(ultraSonarTimerId);
	
	//uint8_t stuckTimerId = declareTimer(6000, 0, Elapsed);
	//startTimer(turnTimerId);
	
	//uint8_t chaosTimerId = declareTimer(59000, 0, Elapsed);
	//startTimer(chaosTimerId);
	

	
	
	//Setup ROS Publisher
	ros::NodeHandle  nh;

	std_msgs::Int32 motorLeftMsg;
	std_msgs::Int32 motorRightMsg;
	ros::Publisher motorLeftPublisher("motorLeft", &motorLeftMsg);
	ros::Publisher motorRightPublisher("motorRight", &motorRightMsg);
	
	std_msgs::Int32 motorLeftTicksMsg;
	ros::Publisher motorLeftTicksPub("LeftWheelTicks", &motorLeftTicksMsg);
	
	std_msgs::Int64 UltraDistancesMsg;
	ros::Publisher UltraDistancesPub("SonarDistances", &UltraDistancesMsg);
	
	std_msgs::Int32 motorRightTicksMsg;
	ros::Publisher motorRightTicksPub("RightWheelTicks", &motorRightTicksMsg);
	
	
	//Setup ROS Subscriber	
	std_msgs::Int32 motorLeftVelovityMsg;
	std_msgs::Int32 motorRightVelovityMsg;
	ros::Subscriber<std_msgs::Int32> motorLeftVelSub("LeftWheelVelocity", &leftWheelVelCallback);
	ros::Subscriber<std_msgs::Int32> motorRightVelSub("RightWheelVelocity", &RightWheelVelCallback);
	
	
	// Init the eventStore used in main
	event_type mainStore = createEventStore();


	// Init ROS NodeHandle
	nh.initNode();
	
	nh.advertise(motorLeftTicksPub);
	nh.advertise(motorRightTicksPub);
	nh.advertise(UltraDistancesPub);
	
	nh.subscribe(motorLeftVelSub);
	nh.subscribe(motorRightVelSub);

	//Serial.begin(57900);
	
	motorLeft.setPwmDutyCycle(0);
	motorLeft.startMotor();
	
	motorRight.setPwmDutyCycle(0);
	motorRight.startMotor();
	
	Encoder_Tracker Tracker_MotorLeft = Encoder_Tracker(50, PPR_MOTOR_LEFT);
	PidController PID_MotorLeft = PidController(0.122, 0.200, 0, 20, MIN_ERROR_MOTOR_LEFT, MAX_ERROR_MOTOR_LEFT, MIN_CONTROL_VALUE_MOTOR_LEFT, MAX_CONTROL_VALUE_MOTOR_LEFT); //(0.03, 0.011, 0.0, 20, 0, 7269, 730, 7999)
	
	Encoder_Tracker Tracker_MotorRight = Encoder_Tracker(50, PPR_MOTOR_RIGHT);
	PidController PID_MotorRight = PidController(0.122, 0.200, 0, 20, MIN_ERROR_MOTOR_RIGHT, MAX_ERROR_MOTOR_RIGHT, MIN_CONTROL_VALUE_MOTOR_RIGHT, MAX_CONTROL_VALUE_MOTOR_RIGHT); //(0.050, 0.009, 0.0, 20, 0, 7269, 730, 7999)
	
	motorLeft.setTargetRPM(0);
	motorRight.setTargetRPM(0);

	int8_t turning = 0;
	uint8_t mode = TELEOP_MODE;
	
	while(1)
	{
		if(timerElapsed(controlLoopTimerId))
		{
			clearTimerElapsed(controlLoopTimerId);
			
			int32_t ticksLeft = motorLeft.getEncoderCount();
			int32_t ticksRight = motorRight.getEncoderCount();
			Tracker_MotorLeft.Track(ticksLeft);
			Tracker_MotorRight.Track(ticksRight);			
			
			int32_t error_motor_left = motorLeft.getTargetPPM()-Tracker_MotorLeft.getPPM();
			int32_t error_motor_right = motorRight.getTargetPPM()-Tracker_MotorRight.getPPM();
			
			motorLeft.setPwmDutyCycle(PID_MotorLeft.Control(error_motor_left));
			motorRight.setPwmDutyCycle(PID_MotorRight.Control(error_motor_right)); 
			
			motorLeftTicksMsg.data = ticksLeft;
			motorRightTicksMsg.data = -ticksRight;
			
			motorLeftTicksPub.publish(&motorLeftTicksMsg);
			motorRightTicksPub.publish(&motorRightTicksMsg);
			
			motorLeftMsg.data = Tracker_MotorLeft.getRPM();
			motorRightMsg.data = Tracker_MotorRight.getRPM();
			
			nh.spinOnce();
		}
			
			
			
		
		if (timerElapsed(ultraSonarTimerId))
		{
			clearTimerElapsed(ultraSonarTimerId);
			ultraLeft.pullTrigger();
			ultraMiddle.pullTrigger();
			ultraRight.pullTrigger();
			_delay_us(10);
			ultraLeft.stopTriggering();
			ultraMiddle.stopTriggering();
			ultraRight.stopTriggering();
			
			int64_t distLeft = ultraLeft.getDistance();
			int64_t distMiddle = ultraMiddle.getDistance();
			int64_t distRight = ultraRight.getDistance();
			
			// Save 
			int64_t distancesUltra = distLeft;
			distancesUltra |= (distMiddle << 16);
			distancesUltra |= (distRight << 32);
			
			UltraDistancesMsg.data = distancesUltra;
			
			UltraDistancesPub.publish(&UltraDistancesMsg);
			
			
			//Exploration Mode for testing purposes of Sonar Sensors. Reliability was approved
			/*
			
			if ((distLeft < 20 ) && (distLeft > -1 ) && !eventIsSet(OBSTACLE_DETECTED_LEFT, &mainStore))
			{
				setEvent(OBSTACLE_DETECTED_LEFT, &mainStore);
			}
			
			if ((distMiddle < 20 ) && (distMiddle > -1 ) && !eventIsSet(OBSTACLE_DETECTED_MIDDLE, &mainStore))
			{
				setEvent(OBSTACLE_DETECTED_MIDDLE, &mainStore);
			}
			
			if ((distRight < 20 ) && (distRight > -1 ) && !eventIsSet(OBSTACLE_DETECTED_RIGHT, &mainStore))
			{
				setEvent(OBSTACLE_DETECTED_RIGHT, &mainStore);
			}
			
			if ( ( eventIsSet(OBSTACLE_DETECTED_LEFT, &mainStore) || eventIsSet(OBSTACLE_DETECTED_MIDDLE, &mainStore) || eventIsSet(OBSTACLE_DETECTED_RIGHT, &mainStore) ) && !timerIsActive(stuckTimerId))
			{
				startTimer(stuckTimerId);
			}
			if( ( !eventIsSet(OBSTACLE_DETECTED_LEFT, &mainStore) && !eventIsSet(OBSTACLE_DETECTED_MIDDLE, &mainStore) && !eventIsSet(OBSTACLE_DETECTED_RIGHT, &mainStore) ) && mode != UNSTUCK_MODE)
			{
				cancelTimer(stuckTimerId);
			}
			
			
			switch (mode)
			{
			case AVOID_MODE:
				if (eventIsSet(OBSTACLE_DETECTED_LEFT, &mainStore))
				{
					turnRight(&motorLeft, &motorRight);
					turning = 1;
				}
				else if(eventIsSet(OBSTACLE_DETECTED_MIDDLE, &mainStore) || eventIsSet(OBSTACLE_DETECTED_RIGHT, &mainStore))
				{
					turnLeft(&motorLeft, &motorRight);
					turning = 1;
				}
				else if(turning)
				{
					goStraight(&motorLeft, &motorRight);
					turning = 0;
				}
				break;
			
			case UNSTUCK_MODE:
				break;
				
			case CHAOS_MODE:
				break;
				
			case TELEOP_MODE:
			break;
			}
			
			
			clearEvent(OBSTACLE_DETECTED_LEFT, &mainStore);
			clearEvent(OBSTACLE_DETECTED_MIDDLE, &mainStore);
			clearEvent(OBSTACLE_DETECTED_RIGHT, &mainStore);
			
			/*Serial.print("Sensor_0: ");
			Serial.print(dist_0);
			Serial.print('\n');
			
			Serial.print("Sensor_1: ");
			Serial.print(dist_1);
			Serial.print('\n');
		
			Serial.print("Sensor_2: ");
			Serial.print(dist_2);
			Serial.print('\n');*/
			
			//nh.spinOnce();*/
		}
		/*
			if(timerElapsed(stuckTimerId))
			{
				clearTimerElapsed(stuckTimerId);

				if(mode != UNSTUCK_MODE)
				{
					mode = UNSTUCK_MODE;
					turnRight(&motorLeft, &motorRight);
					turning = 1;
					setTimerPeriod(stuckTimerId, 4000);
					startTimer(stuckTimerId);
				}
				else
				{
					mode = AVOID_MODE;
					setTimerPeriod(stuckTimerId, 6000);
				}
			}
			
			if(timerElapsed(chaosTimerId))
			{
				clearTimerElapsed(chaosTimerId);
				
				if(mode != CHAOS_MODE)
				{
					mode = CHAOS_MODE;
					turnRight(&motorLeft, &motorRight);
					turning = 1;
					setTimerPeriod(chaosTimerId, (rand()%6000));
					startTimer(chaosTimerId);
				}
				else
				{
					mode = AVOID_MODE;
					setTimerPeriod(chaosTimerId, 59000);
					startTimer(chaosTimerId);
				}
			}	*/
	}
	return 0;
}


void leftWheelVelCallback(const std_msgs::Int32& msg)
{
	motorLeft.setTargetPPM(msg.data);
}

void RightWheelVelCallback(const std_msgs::Int32& msg)
{
	motorRight.setTargetPPM(-msg.data);
}

//Part Exploration Mode for testing purposes of Sonar Sensors. Reliability was approved
/*
void turnLeft(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight)
{
	_motorRight->stopMotor();
	_motorLeft->stopMotor();
	_motorRight->setTargetRPM(-15);
	_motorLeft->setTargetRPM(-15);
	_motorRight->startMotor();
	_motorLeft->startMotor();
}

void turnRight(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight)
{
	_motorRight->stopMotor();
	_motorLeft->stopMotor();
	_motorRight->setTargetRPM(15);
	_motorLeft->setTargetRPM(15);
	_motorRight->startMotor();
	_motorLeft->startMotor();
}

void goStraight(DC_Motor_Driver* _motorLeft, DC_Motor_Driver* _motorRight)
{
	_motorRight->stopMotor();
	_motorLeft->stopMotor();
	_motorRight->setTargetRPM(-30);
	_motorLeft->setTargetRPM(30);
	_motorRight->startMotor();
	_motorLeft->startMotor();
} */