#include "CameraKinect.h"
#include <iostream>
#include <stdexcept>

namespace uav{

int CameraKinect::init(CameraSettings *camSettings)
{
	//capture = cv::VideoCapture(0);

	return 0;
}

int CameraKinect::initCameraKinect(CameraSettings *camSettings)
{
	int retval = freenect_start_video(dev);
	throwOnError(retval);
	retval = freenect_start_depth(dev);
	throwOnError(retval);

	return 0;
}

int CameraKinect::initCameraOmni(CameraSettings *camSettings)
{

	return 0;
}

int CameraKinect::initCameraDepan(CameraSettings *camSettings)
{

	return 0;
}

void CameraKinect::startCamera(CameraSettings *camSettings)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraKinect::stopCamera(CameraSettings *camSettings)
{
	int retval = freenect_stop_video(dev);
	throwOnError(retval);
	retval = freenect_stop_depth(dev);
	throwOnError(retval);
}

void CameraKinect::shutDownCamera()
{
	std::cout<<"Method not supported" <<std::endl;
}

void CameraKinect::getParameter(unsigned parameter, double *value)
{
	std::cout << "Method not supported\n";
}

void CameraKinect::getParameterRanges(unsigned parameter, ParameterRange parameterRange)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraKinect::getParameters(CameraSettings *camSettings)
{
	std::cout << "Method not supported\n";
}

void CameraKinect::getParametersRanges(ParameterRange* parameterRange)
{
	std::cout<<"Method not supported for this type of camera!" <<std::endl;

	parameterRange[UAV_EXPOSURE].min = 0; // FIXME: only for debug
	parameterRange[UAV_EXPOSURE].max = 100;
	parameterRange[UAV_GAIN].min = 0;
	parameterRange[UAV_GAIN].max = 100;
	parameterRange[UAV_WB].min = 0;
	parameterRange[UAV_WB].max = 100;

}

void CameraKinect::printParameter(unsigned parameter)
{
	std::cout << "Method not supported\n";
}

void CameraKinect::printParameters()
{
	std::cout << "Method not supported\n";
}

void CameraKinect::setAutoMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraKinect::setManualMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraKinect::setParameter(unsigned parameter, unsigned value)
{

	std::cout << "Method not supported\n";

}

void CameraKinect::setParameters(CameraSettings *camSettings)
{
	std::cout << "Method not supported\n";
}


static void freenectDepthCallback(freenect_device *dev, void *depth, uint32_t timestamp)
{
	CameraKinect *kinect = static_cast<CameraKinect *>(freenect_get_user(dev));
	kinect->depthCallback(depth, timestamp);
}

static void freenectVideoCallback(freenect_device *dev, void *video, uint32_t timestamp)
{
	CameraKinect *kinect = static_cast<CameraKinect *>(freenect_get_user(dev));
	kinect->videoCallback(video, timestamp);
}

static void *pthreadCallback(void *kinect)
{
	((CameraKinect *)kinect)->processEventsCallback();
	return NULL;
}

CameraKinect::CameraKinect()
: depthMat(cv::Size(640,480),CV_16UC1, (void *)depthBuffer),
  depthFrameCount(0),
  newDepthFrame(false),
  videoMat(cv::Size(640,480),CV_8UC3, (void *)videoBuffer),
  videoFrameCount(0),
  newVideoFrame(false),
  running(true)
{
	// Device cannot be created more than once.
	int retval;

	retval = freenect_init(&ctx, NULL);
	throwOnError(retval);

	freenect_select_subdevices(ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	retval = freenect_num_devices(ctx);
	if (retval == 0) {
		std::cerr << "No kinect is connected.\n";
		throw "nokinect";
	} else throwOnError(retval);

	retval = freenect_open_device(ctx, &dev, 0);
	throwOnError(retval);

	// Send this pointer to be used in callbacks
	freenect_set_user(dev, this);

	freenect_set_led(dev,LED_RED);

	// Set callbacks
	freenect_set_video_callback(dev, freenectVideoCallback);
	freenect_set_depth_callback(dev, freenectDepthCallback);

	// Set capture formats, hard coded for now.
	freenect_set_video_mode(dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,FREENECT_DEPTH_11BIT));

	// Set buffers
	freenect_set_video_buffer(dev, (void *)videoBuffer);
	freenect_set_depth_buffer(dev, (void *)depthBuffer);

	// Launch thread to process usb events
	pthread_create(&thread, NULL, pthreadCallback, (void *)this);
}

CameraKinect::~CameraKinect()
{
	int retval = freenect_shutdown(ctx);
	throwOnError(retval);

	running = false;

	pthread_join(thread, NULL);

	ctx = NULL;
}

bool CameraKinect::requestFrame(cv::Mat &image)
{
	if(newVideoFrame) {
		// Frame is not new anymore
		newVideoFrame = false;

		unsigned int thisVideoFrameCount = videoFrameCount;

		videoMat.copyTo(image);

		// Check if new frame arrived while collecting
		if (videoFrameCount != thisVideoFrameCount)
			return false;
		else
			return true;
	} else
		return false;
}
int CameraKinect::readFrame(cv::Mat &image)
{
	while(!requestFrame(image));

	return 0;
}

bool CameraKinect::readFrameDepth(cv::Mat &image)
{
	if(newDepthFrame) {
		// Frame is not new anymore
		newDepthFrame = false;

		unsigned int thisDepthFrameCount = depthFrameCount;
		depthMat.copyTo(image);

		// Check if new frame arrived while collecting
		if (depthFrameCount != thisDepthFrameCount)
			return false;
		else
			return true;
	} else
		return false;
}

void CameraKinect::setTilt(double angle)
{
	int retval = freenect_set_tilt_degs(dev, angle);
	throwOnError(retval);
}

double CameraKinect::getTilt(void)
{
	freenect_update_tilt_state(dev);
	return freenect_get_tilt_degs(freenect_get_tilt_state(dev));
}

void CameraKinect::getAccelerometers(double *ax, double *ay, double *az)
{
	freenect_update_tilt_state(dev);
	freenect_get_mks_accel(freenect_get_tilt_state(dev), ax, ay, az);
}

void CameraKinect::depthCallback(void *depth, uint32_t timestamp)
{
	++depthFrameCount;
	newDepthFrame = true;
}

void CameraKinect::videoCallback(void *video, uint32_t timestamp)
{
	++videoFrameCount;
	newVideoFrame = true;
}

void CameraKinect::processEventsCallback()
{
	while(running)
		{
		freenect_process_events(ctx);
		}
}

void CameraKinect::throwOnError(int retval)
{
	if (retval  < 0) {
		throw new std::runtime_error("libfreenect");
	}
}

}
