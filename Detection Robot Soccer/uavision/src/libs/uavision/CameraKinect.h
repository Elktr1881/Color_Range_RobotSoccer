#ifndef CAMERAKINECT_H_
#define CAMERAKINECT_H_

/**
 * \file
 * Define CameraKinect class.
 */

#include <opencv2/opencv.hpp>
#include <libfreenect.h>
#include <pthread.h>
#include "Camera.h"

namespace uav{

/**
 * C++ wrapper libfreenect in an OpenCV friendly way.
 *
 * Access to the hardware buffers is made using callback functions. Mechanisms
 * are in place to ensure mutual exclusion in data access.
 */

class CameraKinect:  public Camera
{
	public:
	/** Constructor */
	CameraKinect();

	/** Destructor */
	~CameraKinect();

	/**
	 * Try to grab the latest depth frame. The grabbed frame can be
	 * accessed in the depth attribute.
	 *
	 * \returns true if frame was successfully grabbed, false otherwise.
	 */
	bool readFrameDepth(cv::Mat &depth);

	/**
	 * Set tilt servo angle, in degrees.
	 *
	 * \param[in] angle Angle to set the servo to. 0 corresponds to
	 * horizontal.
	 */
	void setTilt(double angle);

	/**
	 * Get tilt servo angle, in degrees.
	 *
	 * \returns Servo current angle, in degrees. 0 corresponds to horizontal.
	 */
	double getTilt(void);

	/**
	 * Get accelerometers information.
	 *
	 * \param[out] ax X axis acceleration.
	 * \param[out] ay Y axis acceleration.
	 * \param[out] az Z axis acceleration.
	 */
	void getAccelerometers(double *ax, double *ay, double *az);

	/**
	 * Do not call. Internal callback function.
	 */
	void videoCallback(void *video, uint32_t timestamp);

	/**
	 * Do not call. Internal callback function.
	 */
	void depthCallback(void *depth, uint32_t timestamp);

	/**
	 * Do not call. Internal callback function.
	 */
	void processEventsCallback();

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
	int readFrame( cv::Mat &image );
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

	void rewind(){}

	private:
	bool requestFrame(cv::Mat &image);
	// Internal buffers and book keeping variables
	uint8_t depthBuffer[640*480*2];
	cv::Mat depthMat;
	unsigned int depthFrameCount;
	bool newDepthFrame;

	uint8_t videoBuffer[640*480*3];
	cv::Mat videoMat;
	unsigned int videoFrameCount;
	bool newVideoFrame;

	// libfreenect specific stuff
	freenect_context *ctx;
	freenect_device *dev;

	pthread_t thread;
	bool running;

	// Throw exception when libfreenect error occurs
	void throwOnError(int retval);
};
}
#endif
