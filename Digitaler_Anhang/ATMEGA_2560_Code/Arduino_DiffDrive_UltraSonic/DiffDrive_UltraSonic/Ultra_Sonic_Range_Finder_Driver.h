/*
 * Ultra_Sonic_Range_Finder_Driver.h
 *
 * Created: 03.08.2021 18:37:33
 *  Author: User
 */ 


#ifndef ULTRA_SONIC_RANGE_FINDER_DRIVER_H_
#define ULTRA_SONIC_RANGE_FINDER_DRIVER_H_

#define INT_0 0
#define INT_1 1
#define INT_2 2
#define INT_3 3
#define INT_4 4
#define INT_5 5

#include <avr/io.h>

typedef uint8_t int_num;

class Ultra_Sonic_Range_Finder_Driver
{
	public:
	
	Ultra_Sonic_Range_Finder_Driver(int_num INT_N);
	void pullTrigger(); // Send the Trigger Signal. According to Datasheet at least for 10 us
	void stopTriggering(); // Stop the trigger signal
	int16_t getDistance(); // Get Distance in cm
	
	
	private:
	
	volatile int16_t* CurrentDistance;
	volatile uint32_t* TriggerPulled_TimerTimestamp;
	volatile uint32_t* TriggerPulled_MsTimestamp;
	volatile uint8_t* PORTx_Trigger;
	uint8_t PORTxN_Trigger;
	
	void initSensor(uint8_t INTx0,
					volatile uint8_t* DDRx_Trigger, //DDR of Trigger Pin
					volatile uint8_t* DDRx_Echo, //DDR of Echo Pin
					uint8_t DDxN_Trigger, //Flag to set Trigger Pin as Outport
					uint8_t DDxN_Echo, //Flag to set Echo Pin as Inport
					volatile uint8_t* PORTx_Trigger, //Port of Trigger Pin
					volatile uint8_t* PORTx_Echo, //Port of Echo Pins
					uint8_t PORTxN_Trigger, //Flag to set Trigger Pin High/Low
					uint8_t PORTxN_Echo, //Flag to read state of Echo Pin
					volatile uint8_t* EICRx,
					uint8_t ISCn0, // Flag to set at which signal state interrupt is caused
					uint8_t ISCn1, // Flag to set at which signal state interrupt is caused
					volatile int16_t* _CurrentDistance, // pointer to variable that stores current distance measured by sensor
					volatile uint32_t* _TriggerPulled_TimerTimestamp, // TimerCount when trigger was pulles
					volatile uint32_t* _TriggerPulled_MsTimestamp // ms timestamp when trigger was pulled to determine validity
					);
	
};




#endif /* ULTRA_SONIC_RANGE_FINDER_DRIVER_H_ */