#include "CameraFromFile.h"

namespace uav{

CameraFromFile::CameraFromFile( const std::string fileName_, CameraSettings *camSettings)
{
	fileName = fileName_;

	if(camSettings->getCameraSetting(UAV_VIDMODE) == UAV_GRAY)
		image = cv::Mat(cv::Size(camSettings->getCameraSetting(UAV_NCOLS),
				camSettings->getCameraSetting(UAV_NROWS)),CV_8UC1);
	else
		if(camSettings->getCameraSetting(UAV_VIDMODE) == UAV_RGB8)

			image = cv::Mat(cv::Size(camSettings->getCameraSetting(UAV_NCOLS),
					camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);

	file.open(fileName.c_str(), std::fstream::in |  std::fstream::binary);

	if( file.fail() )
	{
		std::cout<< "Something failed on loading the file" << std::endl;
	}
}

void CameraFromFile::rewind()
{
	file.close();
	file.open(fileName.c_str(), std::fstream::in |  std::fstream::binary);
}

int CameraFromFile::readFrame(cv::Mat &imageToReturn)
{
	file.read((char*)image.ptr(), image.rows * image.cols * image.channels());

	image.copyTo(imageToReturn);

	if(!file)
	{
		//std::cerr << "ERROR: only " << file.gcount() << " bytes could be read from video file\n";
		return -1;
	}
	else
		return 0;
}

int CameraFromFile::init(CameraSettings *camSettings)
{
	return 0;
}

int CameraFromFile::initCameraKinect(CameraSettings *camSettings)
{
	return 0;
}

int CameraFromFile::initCameraOmni(CameraSettings *camSettings)
{
	return 0;
}

int CameraFromFile::initCameraDepan(CameraSettings *camSettings)
{
	return 0;
}

void CameraFromFile::startCamera(CameraSettings *camSettings)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::stopCamera(CameraSettings *camSettings)
{

	std::cout<<"Method not supported!" << std::endl;

}

void CameraFromFile::shutDownCamera()
{

	std::cout<<"Method not supported" <<std::endl;

}

void CameraFromFile::getParameter(unsigned parameter, double *value)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::getParameterRanges(unsigned parameter, ParameterRange parameterRange)
{

	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraFromFile::getParameters(CameraSettings *camSettings)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::getParametersRanges(ParameterRange* parameterRange)
{
	std::cout<<"Method not supported for this type of camera!" <<std::endl;
}

void CameraFromFile::printParameter(unsigned parameter)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::printParameters()
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::setAutoMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraFromFile::setManualMode(CameraSettings *camSettings)
{
	std::cout<<"Method not supported for this type of camera!"<<std::endl;
}

void CameraFromFile::setParameter(unsigned parameter, unsigned value)
{
	std::cout<<"Method not supported" << std::endl;
}

void CameraFromFile::setParameters(CameraSettings *camSettings)
{
	setParameter(UAV_FPS, camSettings->getCameraSetting(UAV_FPS));
}

}
