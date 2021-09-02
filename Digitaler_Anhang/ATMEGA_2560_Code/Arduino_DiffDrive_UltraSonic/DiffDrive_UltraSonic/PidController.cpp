/*
 * PidController.cpp
 *
 * Created: 27.10.2020 21:49:05
 *  Author: Andy
 */ 

#define PID_D_N_PATH 2
#define PID_D_N_TARGET 2

#define PID_D_N_TARGET_SUM (PID_D_N_TARGET*(PID_D_N_TARGET+1)/PID_D_N_TARGET)
#define PID_D_N_TARGET_SUM_SQR ((PID_D_N_TARGET*(PID_D_N_TARGET+1)*(2*PID_D_N_TARGET+1))/6)

#define PID_D_N_PATH_SUM (PID_D_N_PATH*(PID_D_N_PATH+1)/PID_D_N_PATH)
#define PID_D_N_PATH_SUM_SQR ((PID_D_N_PATH*(PID_D_N_PATH+1)*(2*PID_D_N_PATH+1))/6)


#include "PidController.h"


PidController::PidController(float _kp, float _ki, float _kd, uint8_t _period, int32_t _minError, int32_t _maxError, int32_t _minControlValue, int32_t _maxControlValue)
{
	Pid_P = 0;
	Pid_D = 0;
	Pid_I = 0;
	PID_Total = 0;
	kp = _kp;
	kd = _kd;
	ki = _ki;
	period = _period;
	MinError = _minError;
	MaxError = _maxError;
	MinControlValue = _minControlValue;
	MaxControlValue = _maxControlValue;
	
	_setPointChanged = 0;
	
	initQueue(&Pid_D_History, Pid_D_History_Array, PID_D_QUEUE_SIZE);
	Pid_D_N_History = PID_D_N_TARGET;
	
	previous_Error = 0;	
}




int16_t PidController::Control(int32_t error)
{
	int32_t errorDifference;
	if(!_setPointChanged)
	{
		errorDifference = error-previous_Error;
	}
	else
	{
		errorDifference = 0;
	}	
	
	Pid_P = kp*error;
	
	int32_t sum = 0;
	
	int32_t currentPid_D = (kd*errorDifference)/period;
	enqeue(currentPid_D, &Pid_D_History);
	

	sum = 0;
	for(uint8_t i = 0; i < Pid_D_N_History; i++)
	{
		sum += ((Pid_D_N_History - i) * (Pid_D_N_History - i)) * (int32_t) getNthElement(&Pid_D_History, i);
	}
	if (!_setPointChanged)
	{
		Pid_D = sum/Pid_D_N_History_Sum;
	} 
	else
	{
		Pid_D = sum/(Pid_D_N_History_Sum-Pid_D_N_History*Pid_D_N_History);	
	}
	
	if(!_setPointChanged)
    {
	    Pid_I = Pid_I + ( ki * error);
    }
	else
	{
		Pid_I = ki * error;
		_setPointChanged = 0;
	}

	
	PID_Total = Pid_P + Pid_D + Pid_I;
	
	
	if(PID_Total < MinError)
	{
		PID_Total = MinError;
	}
	if(PID_Total > MaxError)
	{
		PID_Total = MaxError;
	}

	int16_t PID_Mapped = map(PID_Total, MinError, MaxError, MinControlValue, MaxControlValue);
	previous_Error = error;
	return PID_Mapped;
}

void PidController::setPointChanged()
{
	_setPointChanged = 1;
}

void PidController::setPidMode(Pid_Mode _mode)
{
	switch (_mode)
	{
	case targetMode:
		Pid_D_N_History = PID_D_N_TARGET;
		Pid_D_N_History_Sum = PID_D_N_TARGET_SUM_SQR;
		break;
	
	case pathMode:
	Pid_D_N_History = PID_D_N_PATH;
	Pid_D_N_History_Sum = PID_D_N_PATH_SUM_SQR;
	break;
	}
}

