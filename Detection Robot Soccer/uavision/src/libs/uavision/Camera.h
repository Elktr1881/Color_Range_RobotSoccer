/* Base class for all the types of cameras that we will be using. */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include "CameraSettings.h"

/**
 * Abstract class working as an interface for the definition
 * of different camera classes.
 */

namespace uav{

class Camera
{

public:

	/**
	 * Virtual destructor for proper clean-up.
	 */
	virtual ~Camera(){}


	/**
	 * Pure virtual method for initializing the camera.
	 */
	virtual int initCameraKinect(CameraSettings *camSettings) = 0;
	virtual int initCameraOmni(CameraSettings *camSettings) = 0;
	virtual int initCameraDepan(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for initializing the camera.
	 */
	virtual int init(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for start sending images.
	 */
	virtual void startCamera(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for stop sending images.
	 */
	virtual void stopCamera(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for stopping the camera and finishing the connection.
	 */
	virtual void shutDownCamera() = 0;

	/**
	 * Pure virtual method for setting camera parameters.
	 * @param *camSettings - camera parameters to be set.
	 */
	virtual void setParameters (CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for setting the value of one camera parameter.
	 * @param parameter - camera parameter to be set.
	 * @param value - new value of the parameter to be set.
	 */
	virtual void setParameter(unsigned int parameter, unsigned value) = 0;

	/**
	 * Pure virtual method for retrieving the value of one camera parameter.
	 * @param parameter - camera parameter to be retrieved.
	 * @param *value - value of the parameter to be retrieved.
	 */
	virtual void getParameter(unsigned int parameter, double *value) = 0;

	/**
	 * Pure virtual method for retrieving the camera settings.
	 * @param *camSettings - camera settings to be retrieved.
	 */
	virtual void getParameters(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for retrieving the limits of the value of a camera parameter.
	 * @param parameter - camera parameter to be retrieved.
	 * @param parameterRange - values of the limits of the parameter to be filled.
	 */
	virtual void getParameterRanges(unsigned int parameter,ParameterRange parameterRange) = 0;

	/**
	 * Pure virtual method for retrieving the limits of the camera parameters.
	 * @param parameterRange[] - value of the limits to be filled.
	 */
	virtual void getParametersRanges( ParameterRange parameterRange[] ) = 0; // get the parameters range from the camera

	/**
	 * Pure virtual method for reading a frame from the camera.
	 * @param &imageOpenCV - frame read.
	 */
	virtual int readFrame( cv::Mat &imageOpenCV ) = 0;
	//virtual Mat& operator>> () = 0;

	/**
	 * Pure virtual method for setting the camera in full auto-mode.
	 * @param *camSettings - camera settings that are to be set in automode.
	 */
	virtual void setAutoMode(CameraSettings *camSettings ) = 0;

	/**
	 * Pure virtual method for setting the camera in manual mode.
	 * @param *camSettings - camera settings to be set in manual mode.
	 */
	virtual void setManualMode(CameraSettings *camSettings) = 0;

	/**
	 * Pure virtual method for printing the camera settings.
	 */
	virtual void printParameters() = 0;

	/**
	 * Pure virtual method for printing just one camera parameter.
	 * @param parameter - camera parameter whose value should be printed.
	 */
	virtual void printParameter(unsigned int parameter) = 0;

	virtual void rewind() = 0;

};
}
#endif
