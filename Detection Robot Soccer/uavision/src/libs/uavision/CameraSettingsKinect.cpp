#include "CameraSettingsKinect.h"
#include "UAVision.h"
#include <string>
#include <fstream>

namespace uav{

CameraSettingsKinect::CameraSettingsKinect(){

	buffer = new unsigned int[14];

	buffer[0] = 640;	// nCols
	buffer[1] = 480;	// nRows
	buffer[2] = 0;	    // ccdCol
	buffer[3] = 0;		// ccdRow
	buffer[4] = 320;	// centerCol
	buffer[5] = 240;	// centerRow
	buffer[6] = 50;		// inRadius
	buffer[7] = 200;	// outRadius
	buffer[8] = 30; 	//fps = 30;
	buffer[9] = 3;		// nChannels
	buffer[10] = UAV_RGB8; // vidMode
	buffer[11] = 1;		// format7
	buffer[12] = 0;		// cameraWasUsed
	buffer[13] = 0;		// cameraRunning
}

CameraSettingsKinect::~CameraSettingsKinect()
{
	delete []buffer;
}

int CameraSettingsKinect::getSize()
{

	int cs_size = 14 * sizeof(unsigned int);

	return cs_size;

}

unsigned int* CameraSettingsKinect::ptr()
{
	return buffer;
}

void CameraSettingsKinect::printCameraSettings(){

	std::cout<< "CAMERA SETTINGS KINECT PRINTED" << std::endl;
	std::cout << "---Resolution info---" << std::endl;
	std::cout << "Width (nCols): "<< buffer[0] <<std::endl;
	std::cout << "Height (nRows):" << buffer[1] <<std::endl;
	std::cout << "CCD column: " << buffer[2] <<std::endl;
	std::cout << "CCD row: " << buffer[3] <<std::endl;
	std::cout << "center Col: " << buffer[4] << std::endl;
	std::cout << "center Row: " << buffer[5] << std::endl;
	std::cout << "inRadius: " << buffer[6] << std::endl;
	std::cout << "outRadius: " << buffer[7] << std::endl;
	std::cout << "Video mode: " << buffer[10] << std::endl;
	std::cout << "Number of channels: " << buffer[9] << std::endl;
	std::cout << "Frames per second: " << buffer[8] << std::endl;
	std::cout << "---Flags---" << std::endl;
	std::cout << "Format7: " << buffer[11] << std::endl;
	std::cout << "Camera was used: " << buffer[12] << std::endl;
	std::cout << "Camera running: " << buffer[13] << std::endl;

}

int CameraSettingsKinect::getCameraSetting(unsigned setting)
{
	int value = -1;

	switch(setting)
		{
		case UAV_FPS:
			value = buffer[8];
			break;
		case UAV_FORMAT7:
			value = buffer[11];
			break;
		case UAV_CAMERARUNNING:
			value = buffer[13];
			break;
		case UAV_CAMERAUSED:
			value = buffer[12];
			break;
		case UAV_NCOLS:
			value = buffer[0];
			break;
		case UAV_NROWS:
			value = buffer[1];
			break;
		case UAV_CCDCOL:
			value = buffer[2];
			break;
		case UAV_CCDROW:
			value = buffer[3];
			break;
		case UAV_CENTERROW:
			value = buffer[4];
			break;
		case UAV_CENTERCOL:
			value = buffer[5];
			break;
		case UAV_INRADIUS:
			value = buffer[6];
			break;
		case UAV_OUTRADIUS:
			value = buffer[7];
			break;
		case UAV_VIDMODE:
			value = buffer[10];
			break;
		case UAV_NCHANNELS:
			value = buffer[9];
			break;
		default:
			std::cout<<"Invalid parameter request to get!"<<std::endl;
			break;
		}

		return value;
	}

void CameraSettingsKinect::setCameraSetting(unsigned setting, unsigned value)
{
	switch(setting)
	{
	case UAV_FPS:
		buffer[8] = value;
		break;
	case UAV_PIXELCLOCK:
		buffer[2] = value;
		break;
	case UAV_FORMAT7:
		buffer[11] = value;
		break;
	case UAV_CAMERARUNNING:
		buffer[13] = value;
		break;
	case UAV_CAMERAUSED:
		buffer[12] = value;
		break;
	case UAV_NCOLS:
		buffer[0] = value;
		break;
	case UAV_NROWS:
		buffer[1] = value;
		break;
	case UAV_CCDCOL:
		buffer[2] = value;
		break;
	case UAV_CCDROW:
		buffer[3] = value;
		break;
	case UAV_CENTERCOL:
		buffer[4] = value;
		break;
	case UAV_CENTERROW:
		buffer[5] = value;
		break;
	case UAV_INRADIUS:
		buffer[6] = value;
		break;
	case UAV_OUTRADIUS:
		buffer[7] = value;
		break;
	case UAV_VIDMODE:
		buffer[10] = value;
		break;
	case UAV_NCHANNELS:
		buffer[9] = value;
		break;
	default:
		std::cout<<"Invalid parameter request to get!"<<std::endl;
		break;
	}
}
}
