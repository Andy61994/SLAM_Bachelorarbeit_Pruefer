/*
 * DC_Motor_Driver.cpp
 *
 * Created: 23.05.2021 21:11:27
 *  Author: User
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define CSN0  0
#define WGMN1 1
#define WGMN3 4
#define COMNA0 6
#define COMNA1 7
#define COMNB0 4
#define COMNB1 5
#define COMNC0 2
#define COMNC1 3

#define PWM_TOP_TICK 7999 // (F_CPU / 1000Hz) / 2 -1

#include "DC_Motor_Driver.h"
#include <avr/interrupt.h>
#include <util/delay.h>

volatile int32_t EncoderTicks_0;
volatile int32_t EncoderTicks_1;

DC_Motor_Driver::DC_Motor_Driver(pwm_num pwmN, uint16_t _PulsesPerRotation)
{
	this->motorRunning = 0;
	this->Encoder.PulsesPerRotation = _PulsesPerRotation;
	
	switch (pwmN)
	{
	case PWM_0: //A
		setPWMRegisters(&DDRE, DDE3, &DDRC, DDC5, DDC7, &PORTC, PC5, PC7, &ICR3, &TCCR3A, &TCCR3B, &OCR3A, COMNA1);
		setEncoderRegisters(INT0, &DDRD, &DDRA, DDD0, DDA0, &PORTD, &PORTA, PD0, PA0, ISC00, ISC01, &EncoderTicks_0);
		break;
	case PWM_1: //B
		setPWMRegisters(&DDRE, DDE4, &DDRC, DDC1, DDC3, &PORTC, PC1, PC3, &ICR3, &TCCR3A, &TCCR3B, &OCR3B, COMNB1);
		setEncoderRegisters(INT1, &DDRD, &DDRA, DDD1, DDA2, &PORTD, &PORTA, PD1, PA2, ISC10, ISC11, &EncoderTicks_1);
		break;
	}
}

void DC_Motor_Driver::setPwmDutyCycle(int16_t OCRnX_Val)
{
	cli();
	*this->OCRnX = (OCRnX_Val < 0) ? (-OCRnX_Val) : OCRnX_Val;
	sei();
	
	if(this->motorRunning)
	{		
		(OCRnX_Val > 0) ? setDirection(COUNTERCLOCKWISE) : setDirection(CLOCKWISE);	
	}
}

void DC_Motor_Driver::setTargetRPM(int32_t _targetRpm)
{
	(this->TargetPpm) = _targetRpm * (this->Encoder.PulsesPerRotation);
}

void DC_Motor_Driver::setTargetPPM(int32_t _targetPpm)
{
	(this->TargetPpm) = _targetPpm;
}

int32_t DC_Motor_Driver::getTargetPPM()
{
	return (this->TargetPpm);
}

void DC_Motor_Driver::setDirection(RotationDirection _direction)
{
	this->CurrentDirection.direction = _direction;
	
	switch (_direction)
	{
		case CLOCKWISE:
		*this->CurrentDirection.PORTx_CTRL |= (1 << this->CurrentDirection.PORTxN_CTRL_1);
		*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_2);
		break;
		case COUNTERCLOCKWISE:
		*this->CurrentDirection.PORTx_CTRL |= (1 << this->CurrentDirection.PORTxN_CTRL_2);
		*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_1);
		break;
		default:
		*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_2);
		*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_1);
		break;
	}
}

void DC_Motor_Driver::startMotor()
{
	this->motorRunning = 1;
}

void DC_Motor_Driver::stopMotor()
{
	this->motorRunning = 0;
	
	*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_1);
	*this->CurrentDirection.PORTx_CTRL &= ~(1 << this->CurrentDirection.PORTxN_CTRL_2);
}

void DC_Motor_Driver::setPWMRegisters(volatile uint8_t* DDRx_PWM,
								uint8_t DDxN_PWM, 
								volatile uint8_t* DDRx_CTRL,
								uint8_t DDxN_CTRL_1,
								uint8_t DDxN_CTRL_2, 
								volatile uint8_t* PORTxN_CTRL,
								uint8_t PORTxN_CTRL_1,
								uint8_t PORTxN_CTRL_2,
								volatile uint16_t* ICRn, 
								volatile uint8_t* TCCRnA, 
								volatile uint8_t* TCCRnB,
								volatile uint16_t* _OCRnX,
								uint8_t COMNx1
								)
{
	*DDRx_PWM |= (1<<DDxN_PWM); // set used PWM pin as out port
	*DDRx_CTRL |= (1<<DDxN_CTRL_1) | (1<<DDxN_CTRL_2); // set used Direction Control pin as out port
	*PORTxN_CTRL &= ~(1<<PORTxN_CTRL_1) | (1<<PORTxN_CTRL_2); // set Control bits to zero so motor is not running
	
	this->CurrentDirection = {
		.direction = CLOCKWISE,
		.PORTx_CTRL = PORTxN_CTRL,
		.PORTxN_CTRL_1 = PORTxN_CTRL_1,
		.PORTxN_CTRL_2 = PORTxN_CTRL_2		
	};
	
	
	*ICRn = PWM_TOP_TICK; // define PWM frequency
	*TCCRnA |= (1<<COMNx1); // clear down counting, set up counting	
	*TCCRnA |= (1<<WGMN1); // Set ICRn as TOP
	*TCCRnB |= (1<<WGMN3); // Set ICRn as TOP
	*TCCRnB |= (1<<CSN0); // Set Timer without Prescaling
	
	this->OCRnX = _OCRnX;
	*this->OCRnX = 0;
}

void DC_Motor_Driver::setEncoderRegisters(uint8_t INTx0,
volatile uint8_t* DDRx_RotaryEncoderPhase_A, //DDR of direction Ctrl Pin1
volatile uint8_t* DDRx_RotaryEncoderPhase_B, //DDR of direction Ctrl Pin2
uint8_t DDxN_RotaryEncoderPhase_A, //Flag to set direction Ctrl Pin1 as input
uint8_t DDxN_RotaryEncoderPhase_B, //Flag to set direction Ctrl Pin2 as input
volatile uint8_t* PORTx_RotaryEncoderPhase_A, //Port of Ctrl Pin1
volatile uint8_t* PORTx_RotaryEncoderPhase_B, //Port of Ctrl Pin1
uint8_t PORTxN_RotaryEncoderPhase_A, //Flag to turn on internal pullUp of  Ctrl Pin1
uint8_t PORTxN_RotaryEncoderPhase_B, //Flag to turn on internal pullUp of  Ctrl Pin2
uint8_t ISCn0,
uint8_t ISCn1,
volatile int32_t* EncoderTicks
)
{
	// make sure ports are set as inports
	*DDRx_RotaryEncoderPhase_A &= ~(1<<DDxN_RotaryEncoderPhase_A);
	*DDRx_RotaryEncoderPhase_B &= ~(1<<DDxN_RotaryEncoderPhase_B);


	this->Encoder.Enocder_Count = EncoderTicks;
	this->Encoder.PORTx_EncoderPhase_A = PORTx_RotaryEncoderPhase_A;
	this->Encoder.PORTx_EncoderPhase_B = PORTx_RotaryEncoderPhase_B;
	this->Encoder.PORTxN_EncoderPhase_A = PORTxN_RotaryEncoderPhase_A;
	this->Encoder.PORTxN_EncoderPhase_B = PORTxN_RotaryEncoderPhase_B;
	
	//Turn on internal PullUp
	*this->Encoder.PORTx_EncoderPhase_A |= (1<<this->Encoder.PORTxN_EncoderPhase_A);
	*this->Encoder.PORTx_EncoderPhase_B |= (1<<this->Encoder.PORTxN_EncoderPhase_B);
	
	EICRA |= (1<<ISCn0) | (1<<ISCn1);
	EIMSK |= (1<<INTx0);
	//sei();
}

int32_t DC_Motor_Driver::getEncoderCount()
{
	cli();
	int32_t currentTicks =  *this->Encoder.Enocder_Count;
	sei();
	return currentTicks;
}

void ENCODER_A_ISR(volatile uint8_t* PORTx_Encoder, uint8_t PORTxN_EncoderPhase_B, volatile int32_t* EncoderTicksX)
{
	if (!(*PORTx_Encoder & (1<<PORTxN_EncoderPhase_B)))
	{
		(*EncoderTicksX)++;
	}
	else if ((*PORTx_Encoder & (1<<PORTxN_EncoderPhase_B)))
	{
		(*EncoderTicksX)--;
	}
}

ISR(INT0_vect)
{
	ENCODER_A_ISR(&PINA, PINA0, &EncoderTicks_0);
}

ISR(INT1_vect)
{
	ENCODER_A_ISR(&PINA, PINA2, &EncoderTicks_1);
}
