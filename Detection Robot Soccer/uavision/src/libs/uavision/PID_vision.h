#ifndef PIDVISION_H_
#define PIDVISION_H_

#include <stdio.h>
#include <iostream>

namespace uav{

class PID 
{

public:
	PID( float kp = 1.0 , float ki = 0.0 , float kd = 0.0 , float minOut=0.0, float maxOut = 100.0 , float maxInt = 100.0 ); 
	PID( const PID& pid ) ;

	float compensate( float value, float error );

	inline void setP( float p ) { kp = p; std::cout<<"P - "<<p<<std::endl; }
	inline void setI( float i ) { ki = i; }
	inline void setD( float d ) { kd = d; }
	inline void setMinOut( float mo ) { minOut = mo; }
	inline void setMaxOut( float mo ) { maxOut = mo; }
	inline void setMaxInt( float mi ) { maxInt = mi; }
	
	inline float getP() { return kp; }
	inline float getI() { return ki; }
	inline float getD() { return kd; }
	inline float getMaxOut() { return maxOut; }
	inline float getMaxInt() { return maxInt; }
	inline float getMinOut() { return minOut; }
	
	void reset();
	void display(); 
	
private:
	float kp;
	float ki;
	float kd;
	float minOut;
	float maxOut;
	float maxInt;
	float lastError;
	float integral;
	bool  firstTime;
};
}
#endif // _PID_H_

