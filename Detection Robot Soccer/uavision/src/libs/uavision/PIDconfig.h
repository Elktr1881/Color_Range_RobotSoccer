/* Made by Tiago Daniel Oliveira Angelo (tiago.angelo@ua.pt)
 * at Universidade de Aveiro, Portugal
 * 2010
 * */
 
 /** \file AutoCalibration.cpp 
  *  \author Tiago Daniel Oliveira Angelo (tiago.angelo@ua.pt)
  *  \brief Simple file to save all the PID parameters.
  * 
  *  One file where all the PID parameters involved in the Auto Calibration
  *  process are listed to ease the tunning of any PID loop. 
  * */
#ifndef _PIDCONFIG_H_
#define _PIDCONFIG_H_
/*
 * Other variables used in the Auto Calibration Routine 
 */

unsigned int SHist[5] = {0};
unsigned int sum = 0;
unsigned int msv[2] = {0};
unsigned int saturation = 0;


/*
 * wb_red and wb_blue factors
 */
 
static float WbKp=0.4;
static float WbKi=0.2;
static float u_erro=0.0;
//static float u_erro_ant=0.0;
//static float integral_Blue=0.0;
static float v_erro=0.0;
//static float v_erro_ant=0.0;
//static float integral_Red=0.0;

/*
 * Gain factors 
 */

static float GainKp=5;
static float GainKi=1;
static float av_erro=0.0;
//static float av_erro_ant=0.0;
//static float integral_gain=0.0;

/*
 * Brightness factors 
 */
 
static float brightKp=5.0;
static float brightKi=1.0;
static float black_erro=0;
//static float black_erro_ant=0;
//static float integral_black=0.0;

/*
 * Shutter and exposure factors
 */
static float shtKp=10.0;			//Static added
static float shtKi=0.0;			//Same here
//float integral_msv=0.0;
float msv_erro=0.0;
//float msv_erro_ant=0.0;
float expKp = 150.0;
float expKi = 100.0;

/*
 * Saturation factors 
 * Needs tuning!
 */

float msv_satur_erro = 0.0;
//float msv_satur_erro_ant = 0.0;
float msv_satur = 0.0;
float satKp = 10.0;
float satKi = 0.0;
float integral_satur=0.0;

#endif
