/*
 * RPM_Tracker.cpp
 *
 * Created: 31.05.2021 18:09:04
 *  Author: User
 */ 

#include "Encoder_Tracker.h"

Encoder_Tracker::Encoder_Tracker(uint8_t _TrackingFreuencyHz, uint16_t _CountsPerRevolution)
{
	TrackingFrequencyHz = _TrackingFreuencyHz;
	CountsPerRevolution = _CountsPerRevolution;
	initQueue(&Enocder_Count_History, Enocder_Count_History_Array, ENCODER_HISTORY_SIZE);
}

void Encoder_Tracker::Track(int32_t currentCount)
{
	enqeue(currentCount, &Enocder_Count_History);
}

int16_t Encoder_Tracker::getRPM()
{
	int32_t countDiff = getNthElement(&Enocder_Count_History, 0) - getNthElement(&Enocder_Count_History, this->Enocder_Count_History.size-1);
	int8_t sign = (countDiff<0) ? -1 : 1;
	countDiff = countDiff * sign;
	int16_t rpm = countToRPM(countDiff);
	
	return rpm * sign;
}

int32_t Encoder_Tracker::getPPM()
{
	int32_t countDiff = getNthElement(&Enocder_Count_History, 0) - getNthElement(&Enocder_Count_History, this->Enocder_Count_History.size-1);
	int8_t sign = (countDiff<0) ? -1 : 1;
	countDiff = countDiff * sign;
	int32_t rpm = countToPPM(countDiff);
	
	return rpm * sign;
}

int32_t Encoder_Tracker::countToPPM(int32_t _countDiff)
{
	int32_t ppm = (((_countDiff) * (((int32_t) this->TrackingFrequencyHz*1000) / (this->Enocder_Count_History.size-1)))*60)/1000;
	
	return ppm;
}

int16_t Encoder_Tracker::countToRPM(int32_t _countDiff)
{
	int16_t rpm = (((_countDiff*1000 / this->CountsPerRevolution) * (((int32_t) this->TrackingFrequencyHz*1000) / (this->Enocder_Count_History.size-1)))*60)/100000;
	
	uint8_t rest = rpm%10;
	
	rpm = (rest<5) ? (rpm - rest) : (rpm + (10-rest));
	rpm = rpm / 10;
	
	return rpm;
}
