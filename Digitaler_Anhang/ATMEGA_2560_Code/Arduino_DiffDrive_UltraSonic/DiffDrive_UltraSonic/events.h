/*
 * events.h
 *
 * Created: 25.08.2021 16:20:02
 *  Author: User
 */ 


#ifndef EVENTS_H_
#define EVENTS_H_

typedef volatile uint16_t event_type;

event_type createEventStore();

void setEvent(event_type event, event_type* eventStore);


void clearEvent(event_type event, event_type* eventStore);


int eventIsSet(event_type event, event_type* eventStore);




#endif /* EVENTS_H_ */