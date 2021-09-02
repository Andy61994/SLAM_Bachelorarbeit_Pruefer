/*
 * Ultra_Sonic_Range_Finder_Driver.cpp
 *
 * Created: 04.08.2021 19:57:10
 *  Author: User
 */ 

#define TIMER_1_TOP 6553
#define MAX_TRAVELTIME_SOUND (58*400) // SC_SR04 Can Measure Up to 400cm. Formula: x[cm] = t[?s]/58

#include "Ultra_Sonic_Range_Finder_Driver.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timer.h"




volatile int16_t CurrentDistance_INT_2 = 0;
volatile int16_t CurrentDistance_INT_3 = 0;
volatile int16_t CurrentDistance_INT_5 = 0;

volatile uint32_t BurstSent_Timestamp_INT_2 = 0;
volatile uint32_t BurstSent_Timestamp_INT_3 = 0;
volatile uint32_t BurstSent_Timestamp_INT_5 = 0;

volatile uint32_t BurstSent_Timestamp_Ms_INT_2 = 0;
volatile uint32_t BurstSent_Timestamp_Ms_INT_3 = 0;
volatile uint32_t BurstSent_Timestamp_Ms_INT_5 = 0;



uint32_t count = 0;



Ultra_Sonic_Range_Finder_Driver::Ultra_Sonic_Range_Finder_Driver(int_num INT_N)
{
	switch (INT_N)
	{
		case INT_2:
		initSensor(INT2, &DDRL, &DDRD, DDL1, DDD2, &PORTL, &PORTD, PL1, PD2, &EICRA, ISC20, ISC21, &CurrentDistance_INT_2, &BurstSent_Timestamp_INT_2, &BurstSent_Timestamp_Ms_INT_2);
		break;
		
		case INT_3:
		initSensor(INT3, &DDRL, &DDRD, DDL3, DDD3, &PORTL, &PORTD, PL3, PD3, &EICRA, ISC30, ISC31, &CurrentDistance_INT_3, &BurstSent_Timestamp_INT_3, &BurstSent_Timestamp_Ms_INT_3);
		break;
		
		case INT_5:
		initSensor(INT5, &DDRL, &DDRE, DDL5, DDE5, &PORTL, &PORTE, PL5, PE5, &EICRB, ISC50, ISC51, &CurrentDistance_INT_5, &BurstSent_Timestamp_INT_5, &BurstSent_Timestamp_Ms_INT_5);
		break;		
	}
	//EncoderTracker = _EncoderTracker;
}

void Ultra_Sonic_Range_Finder_Driver::initSensor(uint8_t INTx0,
											volatile uint8_t* DDRx_Trigger,
											volatile uint8_t* DDRx_Echo,
											uint8_t DDxN_Trigger,
											uint8_t DDxN_Echo,
											volatile uint8_t* PORTx_Trigger,
											volatile uint8_t* PORTx_Echo,
											uint8_t PORTxN_Trigger,
											uint8_t PORTxN_Echo,
											volatile uint8_t* EICRx,
											uint8_t ISCn0,
											uint8_t ISCn1,
											volatile int16_t* _CurrentDistance,
											volatile uint32_t* _TriggerPulled_TimerTimestamp,
											volatile uint32_t* _TriggerPulled_MsTimestamp
											)
{
	*DDRx_Trigger |= (1<<DDxN_Trigger); // Set Trigger pin as outport
	*DDRx_Echo &= ~(1<<DDxN_Echo); // Set Echo Pin as inport
	
	this->PORTx_Trigger = PORTx_Trigger,
	this->PORTxN_Trigger = PORTxN_Trigger,
	
	this->CurrentDistance = _CurrentDistance;
	*(this->CurrentDistance) = -1;
	this->TriggerPulled_TimerTimestamp = _TriggerPulled_TimerTimestamp;
	this->TriggerPulled_MsTimestamp = _TriggerPulled_MsTimestamp;
	
	(*EICRx) |= (1<<ISCn0);
	EIMSK |= (1<<INTx0);	
	
	TCCR1B |= (1<<CS12);
}

void Ultra_Sonic_Range_Finder_Driver::pullTrigger()
{
	*(this->PORTx_Trigger) |= (1<<this->PORTxN_Trigger);
}

void Ultra_Sonic_Range_Finder_Driver::stopTriggering()
{
	*(this->PORTx_Trigger) &= ~(1<<this->PORTxN_Trigger);
}

int16_t Ultra_Sonic_Range_Finder_Driver::getDistance()
{
	cli();
	int16_t dist = *(this->CurrentDistance);
	sei();
	return dist;
}

void Echo_INT_ISR(volatile uint8_t* PINx_Echo, uint8_t PINxN_Echo, volatile int16_t* _CurrentDistance, volatile uint32_t* _BurstSent_TimerTimestamp, volatile uint32_t* _BurstSent_MsTimestamp)
{
	uint32_t TimerTimestamp = TCNT1;
	uint32_t msTimestamp = getGolabal_ms();
	
	if ((*PINx_Echo) & (1<<PINxN_Echo))
	{
		*_BurstSent_TimerTimestamp = TimerTimestamp;
		*_BurstSent_MsTimestamp = msTimestamp;
	}
	else
	{
		if(TimerTimestamp < (*_BurstSent_TimerTimestamp))
		{
			TimerTimestamp = TIMER_1_TOP + TimerTimestamp + 1;
		}
		
		uint32_t TravelTime_us = (TimerTimestamp - (*_BurstSent_TimerTimestamp))*16; // 1 timer tick = 16us
		uint32_t TravelTime_ms = msTimestamp - (*_BurstSent_MsTimestamp);
		if(TravelTime_ms > 8)
		{
			(*_CurrentDistance) = -1;
		}
		else
		{
			(*_CurrentDistance) = ((TravelTime_us*10)/58)/10;
		}
			
	}	
	//(*_CurrentDistance) = ((TravelTime*10)/58)/10;
	
	//(*_CurrentDistance) = (TravelTime * 625) / 58000;
}


ISR(INT2_vect)
{
	Echo_INT_ISR(&PIND, PIND2, &CurrentDistance_INT_2, &BurstSent_Timestamp_INT_2, &BurstSent_Timestamp_Ms_INT_2);
}

ISR(INT3_vect)
{
	Echo_INT_ISR(&PIND, PIND3, &CurrentDistance_INT_3, &BurstSent_Timestamp_INT_3, &BurstSent_Timestamp_Ms_INT_3);
}

ISR(INT5_vect)
{
	Echo_INT_ISR(&PINE, PINE5, &CurrentDistance_INT_5, &BurstSent_Timestamp_INT_5, &BurstSent_Timestamp_Ms_INT_5);
}