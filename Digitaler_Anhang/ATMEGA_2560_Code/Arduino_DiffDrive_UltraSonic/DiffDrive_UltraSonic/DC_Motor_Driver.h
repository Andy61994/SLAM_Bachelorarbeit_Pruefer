/*
 * DC_Motor_Driver.h
 *
 * Created: 23.05.2021 21:06:08
 *  Author: User
 */ 



#ifndef DC_MOTOR_DRIVER_H_
#define DC_MOTOR_DRIVER_H_

#define PWM_0 0
#define PWM_1 1

#include <avr/io.h>
#include "Encoder_Tracker.h"

typedef uint8_t pwm_num;

enum RotationDirection{CLOCKWISE, COUNTERCLOCKWISE};
	
struct MotorDirection
{
	RotationDirection direction;
	volatile uint8_t* PORTx_CTRL;
	uint8_t PORTxN_CTRL_1;
	uint8_t PORTxN_CTRL_2;
};

struct RotaryEncoder
{
	volatile int32_t* Enocder_Count;
	volatile uint8_t* PORTx_EncoderPhase_A;
	volatile uint8_t* PORTx_EncoderPhase_B;
	uint8_t PORTxN_EncoderPhase_A;
	uint8_t PORTxN_EncoderPhase_B;
	uint16_t PulsesPerRotation;
};

struct Wheel
{
	uint16_t wheelDiameter;	
	int16_t distanceFromCenter;
};

class DC_Motor_Driver
{
	public:
	
	DC_Motor_Driver(pwm_num pwmN, uint16_t _PulsesPerRotation);
	void startMotor();
	void stopMotor();
	RotationDirection getDirection();
	void setPwmDutyCycle(int16_t OCRnX_Val);
	void setTargetRPM(int32_t _targetRpm);
	void setTargetPPM(int32_t _targetPpm); // Set Target Pulses Per Minute
	int32_t getTargetPPM();
	int32_t getEncoderCount();
	void setDirection(RotationDirection _direction);
	
	private:	
	
	volatile uint16_t* OCRnX; //determines DutyCycle
	int32_t TargetPpm;
	MotorDirection CurrentDirection;
	RotaryEncoder Encoder;
	uint8_t motorRunning;
	
	void setPWMRegisters(volatile uint8_t* DDRx_PWM, // DDR of PWM Outpt Pin
							uint8_t DDxN_PWM, // Flag of of corresponding PWM output pin
							volatile uint8_t* DDRx_CTRL, //DDR of direction Ctrl Pins
							uint8_t DDxN_CTRL_1, //Flag to set direction Ctrl Pin1 as output
							uint8_t DDxN_CTRL_2, //Flag to set direction Ctrl Pin2 as output
							volatile uint8_t* PORTxN_CTRL, //Port of direction Ctrl Pins
							uint8_t PORTxN_CTRL_1, //Flag to set state Ctrl Pin1 as output
							uint8_t PORTxN_CTRL_2, //Flag to set state Ctrl Pin1 as output
							volatile uint16_t* ICRn, //Corresponding ICR to the PWM output pin. Determines frequency of PWM Signal
							volatile uint8_t* TCCRnA, // Timer Counter CTRL Reg of corresponding pwm output pin
							volatile uint8_t* TCCRnB, // Timer Counter CTRL Reg of corresponding pwm output pin
							volatile uint16_t* _OCRnX, //Output Compare Reg of corresponding pwm output pin. Determines DutyCycle of PWM Signal
							uint8_t COMNx1 // Flag to set the compare output mode
							);

	void setEncoderRegisters(uint8_t INTx0, 
								volatile uint8_t* DDRx_RotaryEncoderPhase_A, //DDR of direction Ctrl Pins
								volatile uint8_t* DDRx_RotaryEncoderPhase_B, //DDR of direction Ctrl Pins
								uint8_t DDxN_RotaryEncoderPhase_A, //Flag to set direction Ctrl Pin1 as output
								uint8_t DDxN_RotaryEncoderPhase_B, //Flag to set direction Ctrl Pin2 as output
								volatile uint8_t* PORTx_RotaryEncoderPhase_A, //Port of direction Ctrl Pins
								volatile uint8_t* PORTx_RotaryEncoderPhase_B, //Port of direction Ctrl Pins
								uint8_t PORTxN_RotaryEncoderPhase_A, //Flag to set state Ctrl Pin1 as output
								uint8_t PORTxN_RotaryEncoderPhase_B, //Flag to set state Ctrl Pin1 as output
								uint8_t ISCn0,
								uint8_t ISCn1,
								volatile int32_t* EncoderTicks
								);
};



#endif /* DC_MOTOR_DRIVER_H_ */