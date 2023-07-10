#include "CameraOpenCV.h"

namespace uav{


int CameraOpenCV::readFrame(cv::Mat &image)
{
	capture >> image;

	//image = cv::imread("x.png");
	//cv::cvtColor(image, image, CV_BGR2RGB);

	return 0;
}

int CameraOpenCV::init(CameraSettings *camSettings)
{
	//capture = cv::VideoCapture(0);

	return 0;
}

int CameraOpenCV::initCameraKinect(CameraSettings *camSettings)
{
	capture = cv::VideoCapture(0);

	return 0;

}

int CameraOpenCV::initCameraOmni(CameraSettings *camSettings)
{
	capture = cv::VideoCapture(2);

	return 0;

}

int CameraOpenCV::initCameraDepan(CameraSettings *camSettings)
{
	capture = cv::VideoCapture(1);

	return 0;

}

void CameraOpenCV::startCamera(CameraSettings *camSettings)
{

	std::cout<<"Method not supported" << std::endl;

}

void CameraOpenCV::stopCamera(CameraSettings *camSettings)
{

	std::cout<<"Method not supported!" << std::endl;

}

void CameraOpenCV::shutDownCamera()
{

	std::cout<<"Method not supported" <<std::endl;

}

void CameraOpenCV::getParameter(unsigned parameter, double *value)
{

	switch(parameter)
	{

	case UAV_FPS:

		*value = capture.get(CV_CAP_PROP_FRAME_WIDTH);
		break;

	default:
		fprintf(stderr, "Parameter not supported!");
		break;

	}
}

void CameraOpenCV::getParameterRanges(unsigned parameter, ParameterRange parameterRange)
{

	std::cout<<"Method not supported for this type of camera!"<<std::endl;


}

void CameraOpenCV::getParameters(CameraSettings *camSettings)
{
	double value;
	getParameter(UAV_FPS, &value);
}

void CameraOpenCV::getParametersRanges(ParameterRange* parameterRange)
{
	std::cout<<"Method not supported for this type of camera!" <<std::endl;

	parameterRange[UAV_EXPOSURE].min = 0; // FIXME: only for debug
	parameterRange[UAV_EXPOSURE].max = 100;
	parameterRange[UAV_GAIN].min = 0;
	parameterRange[UAV_GAIN].max = 100;
	parameterRange[UAV_WB].min = 0;
	parameterRange[UAV_WB].max = 100;

}

void CameraOpenCV::printParameter(unsigned parameter)
{

	double value;

	switch(parameter)
	{

	case UAV_FPS:

		value = capture.get(CV_CAP_PROP_FRAME_WIDTH);
		std::cout<<"FPS: "<<value << std::endl;
		break;

	default:
		std::cout <<"Parameter not supported!"<<std::endl;
		break;

	}
}

void CameraOpenCV::printParameters()
{
	printParameter(UAV_FPS);
}

void CameraOpenCV::setAutoMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraOpenCV::setManualMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraOpenCV::setParameter(unsigned parameter, unsigned value)
{

	switch(parameter)
	{

	case UAV_FPS:

		capture.set(CV_CAP_PROP_FRAME_WIDTH,value);
		break;

	default:
		std::cout<<"Parameter not supported!" << std::endl;
		break;

	}

}

void CameraOpenCV::setParameters(CameraSettings *camSettings)
{
	setParameter(UAV_FPS, camSettings->getCameraSetting(UAV_FPS));
}

}
