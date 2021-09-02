/*
 * RPM_Tracker.h
 *
 * Created: 28.05.2021 12:15:30
 *  Author: User
 */ 

#ifndef RPM_TRACKER_H_
#define RPM_TRACKER_H_

#define ENCODER_HISTORY_SIZE 12

#include <avr/io.h>



#include "queue.h"



class Encoder_Tracker
{
	public:	
	Encoder_Tracker(uint8_t _TrackingFreuencyHz, uint16_t _CountsPerRevolution);
	void Track(int32_t currentCount);
	int16_t getRPM();
	int32_t getPPM();
		
	private:
	uint8_t TrackingFrequencyHz;
	uint16_t CountsPerRevolution;
	Queue Enocder_Count_History;
	int32_t Enocder_Count_History_Array[ENCODER_HISTORY_SIZE];	
	
	int16_t countToRPM(int32_t _countDiff);
	int32_t countToPPM(int32_t _countDiff);
};



#endif /* RPM_TRACKER_H_ */