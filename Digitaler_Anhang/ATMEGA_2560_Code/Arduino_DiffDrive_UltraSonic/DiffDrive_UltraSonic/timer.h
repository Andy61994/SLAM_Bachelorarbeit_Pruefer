/*
 * timer.h
 *
 * Created: 25.08.2021 16:26:17
 *  Author: User
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <avr/common.h>
#include "events.h"

enum timerMode {Elapsed, Event, Callback, Dual};

struct Timer
{
	uint8_t Id; // Id of the timer
	uint8_t isActive; // indicates if the timer is active (started)
	volatile uint8_t elapsed; // set to 1 after timer has elapsed
	uint16_t period; // period in Microseconds
	uint8_t repeating; // indicates if timer runes once or repeats (not fully implemented)
	timerMode mode;
	void (*callback) (); // function that is called when timer elapsed
	event_type timerEvent;
	event_type* eventStore;
};

uint8_t declareTimer(uint16_t _period, uint8_t _repeating, timerMode _mode, void (*_callback) () = 0, event_type _event = 0, event_type* _eventStore = 0);

void startTimer(uint8_t timerId);

void cancelTimer(uint8_t timerId);

uint8_t timerElapsed(uint8_t timerId);

void clearTimerElapsed(uint8_t timerId);

uint8_t timerIsActive(uint8_t timerId);

void setTimerPeriod(uint8_t timerId, uint16_t _period);

uint32_t getGolabal_ms();

#endif /* TIMER_H_ */