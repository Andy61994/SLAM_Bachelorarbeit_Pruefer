/*
 * PidController.h
 *
 * Created: 27.10.2020 16:10:06
 *  Author: Andy
 */ 

#define PID_D_QUEUE_SIZE 2


#ifndef PidController_h
#define PidController_h

#include <avr/io.h>
#include "queue.h"

/**
  * pathMode: used if setPoint gets changed constantly in a high frequency
  * targetMode: used if setPoint should be reached and stabilized quickly
  **/
enum Pid_Mode{targetMode, pathMode};

class PidController
{
public:
	PidController(float _kp, float _ki, float _kd, uint8_t _period, int32_t _minError, int32_t _maxError, int32_t _minControlValue, int32_t _maxControlValue);
	int16_t Control(int32_t error); // compute PID controller
	void setPointChanged(); // to indicate that the set point has been changed
	void setPidMode(Pid_Mode _mode); // set mode
	
	
private:
int32_t PID_Total; // PID_P + PID_D + PID_I 
int32_t previous_Error; 
float Pid_P;
float Pid_D;
float Pid_I;
float kp;
float kd;
float ki;
int32_t MaxError; // Max allowed Error. Used to prevent Windup
int32_t MinError; // Min allowed Error. Used to prevent Windup
int32_t MaxControlValue; // Max Control Output
int32_t MinControlValue;  // Min Control Output
uint8_t period; // period that determines PID computing frequency
uint8_t _setPointChanged; // indicates if the set point has been changed
int32_t Pid_D_N_History; //number of values from the PID_D_History Queue that are used to calculate the weighted PID_D value
int32_t Pid_D_N_History_Sum; //number of values from the PID_D_History Queue summed up by index

Queue Pid_D_History; // Queue containing History of the PID_D Values
int32_t Pid_D_History_Array[PID_D_QUEUE_SIZE];


};



#endif /* PIDCONTROLLER_H_ */