/*
 * events.h
 *
 * Created: 13.11.2020 16:20:11
 *  Author: Andy
 */ 

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include "events.h"

//Event Store to keep an overview if events are used in different areas / classes.
// One Store per "context"
event_type createEventStore()
{
	event_type store = 0;
	return store;
}

void setEvent(event_type event, event_type* eventStore) {
	unsigned char sreg_old = SREG;
	cli();
	(*eventStore) |= event;
	SREG = sreg_old;
}

void clearEvent(event_type event, event_type* eventStore) {
	unsigned char sreg_old = SREG;
	cli();
	(*eventStore) &= ~event;
	SREG = sreg_old;
}

int eventIsSet(event_type event, event_type* eventStore){
	return ((*eventStore)&event);
}
