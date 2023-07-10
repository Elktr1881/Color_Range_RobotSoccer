#ifndef CAMERAFROMFILE_H_
#define CAMERAFROMFILE_H_

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "UAVision.h"
#include "Camera.h"
#include "CameraSettings.h"

/**
 * Class for accessing a file
 */


namespace uav{


class CameraFromFile: public Camera
{
public:

	/** Stream for manipulation of the file.
	 *
	 */
	std::fstream file;

	/**
	 * Name of the file.
	 */
	std::string fileName;


	/**
	 * Constructor
	 * * @param fileName_ - name of the file containning the video.
	 */

	CameraFromFile(const std::string fileName_, CameraSettings *camSettings );
	cv::Mat image;

	void rewind();
	/**
	 * Destructor
	 */
	~CameraFromFile(){file.close();}

	/**
	 * Method for initializing the camera.
	 */
	int initCameraKinect(CameraSettings *camSettings);
	int initCameraOmni(CameraSettings *camSettings);
	int initCameraDepan(CameraSettings *camSettings);

	/**
	 * Method for initializing the camera.
	 */
	int init(CameraSettings *camSettings);

	/**
	 * Method for start sending images.
	 */
	void startCamera(CameraSettings *camSettings);

	/**
	 * Method for stop sending images.
	 */
	void stopCamera(CameraSettings *camSettings);

	/**
	 * Method for stopping the camera and finishing the connection.
	 */
	void shutDownCamera();

	/**
	 * Method for setting camera parameters.
	 * @param *camSettings - camera parameters to be set.
	 */
	void setParameters(CameraSettings *camSettings);

	/**
	 * Pure virtual method for setting the value of one camera parameter.
	 * @param parameter - camera parameter to be set.
	 * @param value - new value of the parameter to be set.
	 */
	void setParameter(unsigned int parameter, unsigned value);

	/**
	 * Method for retrieving the value of one camera parameter.
	 * @param parameter - camera parameter to be retrieved.
	 * @param *value - value of the parameter to be retrieved.
	 */
	void getParameter(unsigned int parameter, double *value);

	/**
	 * Method for retrieving the camera settings.
	 * @param *camSettings - camera settings to be retrieved.
	 */
	void getParameters(CameraSettings *camSettings);

	/**
	 * Method for retrieving the limits of the value of a camera parameter.
	 * @param parameter - camera parameter to be retrieved.
	 * @param parameterRange - values of the limits of the parameter to be filled.
	 */
	void getParameterRanges(unsigned int parameter,ParameterRange parameterRange);

	/**
	 * Method for retrieving the limits of the value of a camera parameter.
	 * @param parameter - camera parameter to be retrieved.
	 * @param parameterRange - values of the limits of the parameter to be filled.
	 */
	void getParametersRanges( ParameterRange parameterRange[] );

	/**
	 * Pure virtual method for reading a frame from the camera.
	 * @param &imageOpenCV - frame read.
	 */
	int readFrame( cv::Mat &imageOpenCV );
	//virtual Mat& operator>> () = 0;

	/**
	 * Method for setting the camera in full auto-mode.
	 * @param *camSettings - camera settings that are to be set in automode.
	 */
	void setAutoMode(CameraSettings *camSettings );

	/**
	 * Method for setting the camera in manual mode.
	 * @param *camSettings - camera settings to be set in manual mode.
	 */
	void setManualMode(CameraSettings *camSettings );

	/**
	 * Method for printing the camera settings.
	 */
	void printParameters();


	/**
	 * Method for printing just one camera parameter.
	 * @param parameter - camera parameter whose value should be printed.
	 */
	void printParameter(unsigned int parameter);

};
}
#endif
