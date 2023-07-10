#include "PID_vision.h"

namespace uav{


PID::PID( float kp , float ki , float kd , float minOut, float maxOut , float maxInt ) 
{
	this->kp		= kp;
	this->ki		= ki; 
	this->kd		= kd;
	this->minOut	= minOut;
	this->maxOut	= maxOut;
	this->maxInt	= maxInt;
	
	this->lastError	= 0.0;
	this->integral	= 0.0;
	this->firstTime	= true;
}

PID::PID( const PID& pid ) 
{
	this->kp		= pid.kp;
	this->ki		= pid.ki; 
	this->kd		= pid.kd;
	this->minOut	= pid.minOut;
	this->maxOut	= pid.maxOut;
	this->maxInt	= pid.maxInt;
	
	this->lastError	= pid.lastError;
	this->integral	= pid.integral;
	this->firstTime	= true;
}

float PID::compensate(float value, float error )
{
	integral += ki * error;

	if(integral > maxInt)
		integral = maxInt;
	else
	if(integral < -maxInt)
		integral = -maxInt;

	float out = 0.0;

	if(firstTime)
	{
		integral = 0;
		out = value + kp * error;
		firstTime = false;
	}
	else
		out = value + (kp * error + kd * (error - lastError) + integral);

	if(out > maxOut )
		out = maxOut;
	
	if(out < minOut)
		out = minOut;

	lastError = error;
	
	return out;
}

void PID::reset()
{
	firstTime = true;
}

void PID::display() 
{
	printf("  p(%.2f) i(%.2f) d(%.2f) maxOut(%.2f) maxInt(%.2f)\n", kp, ki, kd, maxOut, maxInt);	
}
}
