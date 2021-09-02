/*
 * timer.h
 *
 * Created: 25.11.2020 14:20:20
 *  Author: Andy
 */ 

#define MAX_TIMERS 5 // max number of timers that can be registered
#define TIMER_START_TICK 5; // 2^16 - 1 - 1600(Tics for 1 microsecond)
#define UNUSED_TIMER_ID 255 // Id of an unused timer (before declaration)

#include "timer.h"
#include <avr/interrupt.h>



// struct representing a timer 


Timer timers[MAX_TIMERS];
uint8_t timerInitialized = 0;
uint8_t timerStructInitialized = 0;

static uint32_t millisec[MAX_TIMERS] = {0}; // ~65s till overflow (max period) uint16 or ~29 days uint32
	
static uint32_t global_ms;

void initMilliISR()
{
	TCNT2 = TIMER_START_TICK; 
	TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
	TCCR2B |= (1<<CS22);
	TIMSK2 |= (1<<TOIE2);
	sei();
};

/*
 * Checks for all active timers if they
 * elapsed and calls there callback function
 */
ISR(TIMER2_OVF_vect)
{
	TCNT2 = TIMER_START_TICK;
	global_ms++;
	
	for( int i = 0; (timers[i].Id != 255) && (i < MAX_TIMERS); i++)
	{
		if(timers[i].isActive)
		{
			millisec[i]++;
			
			if(millisec[i] == timers[i].period)
			{
				timers[i].isActive = timers[i].repeating;
				millisec[i] = 0;
				switch (timers[i].mode)
				{
				case Callback:
				cli();
				timers[i].callback();
				sei();
				break;
				
				case Elapsed:
				timers[i].elapsed = 1;
				break;				
						
				case Event:
				setEvent(timers[i].timerEvent, timers[i].eventStore);
				break;
				
				case Dual:
				setEvent(timers[i].timerEvent, timers[i].eventStore);
				timers[i].callback();
				break;				
				}			
			}
		}
	}
}

uint8_t declareTimer(uint16_t _period, uint8_t _repeating, timerMode _mode, void (*_callback) (), event_type _event, event_type* _eventStore)
{
	uint8_t index = 0;
	Timer *timerPtr;
	
	if(timerStructInitialized == 0)
	{
		for(int i = 0; i<MAX_TIMERS; i++)
		{
			timers[i].Id = 255;
			timers[i].isActive = 0;
		}
		timerStructInitialized = 1;
	}
	
	while(timers[index].Id != UNUSED_TIMER_ID)
	{
		index++;
	}

	timerPtr = &timers[index];
	
	timerPtr->Id = index;
	timerPtr->elapsed = 0;
	timerPtr->callback = _callback;
	timerPtr->timerEvent = _event;
	timerPtr->eventStore = _eventStore;
	timerPtr->mode = _mode;
	timerPtr->period = _period;
	timerPtr->repeating = _repeating;
	
	return timerPtr->Id;
}

void startTimer(uint8_t timerId)
{
	if(timerInitialized == 0)
	{
		initMilliISR();
		global_ms = 0;
		timerInitialized = 1;
	}
	timers[timerId].isActive = 1;
}

void cancelTimer(uint8_t timerId)
{
	timers[timerId].isActive = 0;
	millisec[timerId] = 0;
}

uint8_t timerElapsed(uint8_t timerId)
{
	return timers[timerId].elapsed;
}

void clearTimerElapsed(uint8_t timerId)
{
	timers[timerId].elapsed	= 0;
}

 uint8_t timerIsActive(uint8_t timerId)
 {
	 return timers[timerId].isActive;
 }
 
 void setTimerPeriod(uint8_t timerId, uint16_t _period)
 {
	 cli();
	 timers[timerId].period = _period;
	 sei();
 }
 
uint32_t getGolabal_ms()
 {
	 cli();
	 return global_ms;
	 sei();
 }