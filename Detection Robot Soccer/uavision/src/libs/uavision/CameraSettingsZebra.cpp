#include "CameraSettingsZebra.h"
#include <string>
#include <fstream>
#include <iostream>


namespace uav{

    CameraSettingsZebra::CameraSettingsZebra()
    {
        buffer = new unsigned int[21];

        buffer[0] = 0; 		// exposure
        buffer[1] = 0; 		// gain
        buffer[2] = 71;		// pixelClock
        buffer[3] = 0;		// wb_blue
        buffer[4] = 0;		// wb_red
        buffer[5] = 1600;	// nCols 1224
        buffer[6] = 1200;	// nRows 1624
        buffer[7] = 128;	// ccdCol
        buffer[8] = 0;		// ccdRow
        buffer[9] = 512;	// centerCol
        buffer[10] = 512;	// centerRow
        buffer[11] = 50;	// inRadius
        buffer[12] = 500;	// outRadius
        buffer[13] = 0;		// packetSize
        buffer[14] = 30; 	//fps = 30;
        buffer[15] = UAV_RGB8; // vidMode
        buffer[16] = 0; // colorCoding
        buffer[17] = 3;		// nChannels
        buffer[18] = 1;		// format7
        buffer[19] = 0;		// cameraWasUsed
        buffer[20] = 0;		// cameraRunning
    }

    CameraSettingsZebra::~CameraSettingsZebra()
    {
            delete []buffer;

    }
unsigned int *CameraSettingsZebra::ptr()
{
	return buffer;
}

int CameraSettingsZebra::getSize()
{
	int cs_size = 21 * sizeof(unsigned int);

	return cs_size;
}


void CameraSettingsZebra::printCameraSettings()
{

	std::cout << "CAMERA SETTINGS ETHERNET PRINTED" << std::endl;
	std::cout << "---Colormetric info---"<< std::endl;
	std::cout << "Exposure: "<< buffer[0] << std::endl;
	std::cout << "Gain: "<< buffer[1] << std::endl;
	std::cout << "Pixel clock: "<< buffer[2] << std::endl;
	std::cout << "White balance blue: "<< buffer[3] <<std::endl;
	std::cout << "White balance red: "<< buffer[4] <<std::endl;
	std::cout << "---Resolution info---" << std::endl;
	std::cout << "Width (nCols): "<< buffer[5] <<std::endl;
	std::cout << "Height (nRows):" << buffer[6] <<std::endl;
	std::cout << "CCD column: " << buffer[7] <<std::endl;
	std::cout << "CCD row: " << buffer[8] <<std::endl;
	std::cout << "center Col: " << buffer[9] << std::endl;
	std::cout << "center Row: " << buffer[10] << std::endl;
	std::cout << "inRadius: " << buffer[11] << std::endl;
	std::cout << "outRadius: " << buffer[12] << std::endl;
	std::cout << "Packet size: "<< buffer[13] << std::endl;
	std::cout << "Frames per second: " << buffer[14] << std::endl;
	std::cout << "Video mode: " << buffer[15] << std::endl;
	std::cout << "Color coding: "<<  buffer[16] << std::endl;
	std::cout << "Number of channels: " << buffer[17] << std::endl;
	std::cout << "---Flags---" << std::endl;
	std::cout << "Format7: " << buffer[18] << std::endl;
	std::cout << "Camera was used: " << buffer[19] << std::endl;
	std::cout << "Camera running: " << buffer[20] << std::endl;

}


int CameraSettingsZebra::getCameraSetting(unsigned setting)
{
	int value = 0;

	switch(setting)
	{

	case UAV_EXPOSURE:
		value = buffer[0];
		break;
	case UAV_GAIN:
		value = buffer[1];
		break;
	case UAV_PIXELCLOCK:
		value = buffer[2];
		break;
	case UAV_WBB:
		value = buffer[3];
		break;
	case UAV_WBR:
		value = buffer[4];
		break;
	case UAV_NCOLS:
		value = buffer[5];
		break;
	case UAV_NROWS:
		value = buffer[6];
		break;
	case UAV_CCDCOL:
		value = buffer[7];
		break;
	case UAV_CCDROW:
		value = buffer[8];
		break;
	case UAV_CENTERCOL:
		value = buffer[9];
		break;
	case UAV_CENTERROW:
		value = buffer[10];
		break;
	case UAV_INRADIUS:
		value = buffer[11];
		break;
	case UAV_OUTRADIUS:
		value = buffer[12];
		break;
	case UAV_PACKETSIZE:
		value = buffer[13];
		break;
	case UAV_FPS:
		value = buffer[14];
		break;
	case UAV_VIDMODE:
		value = buffer[15];
		break;
	case UAV_COLORCODING:
		value = buffer[16];
		break;
	case UAV_NCHANNELS:
		value = buffer[17];
		break;
	case UAV_FORMAT7:
		value = buffer[18];
		break;
	case UAV_CAMERAUSED:
		value = buffer[19];
		break;
	case UAV_CAMERARUNNING:
		value = buffer[20];
		break;
	default:
		std::cout<<"Invalid parameter request to get!"<<std::endl;
		break;
	}

	return value;
}

void CameraSettingsZebra::setCameraSetting(unsigned setting, unsigned value)
{
	switch(setting)
	{
	case UAV_EXPOSURE:
		buffer[0] = value;
		break;
	case UAV_GAIN:
		buffer[1] = value;
		break;
	case UAV_PIXELCLOCK:
		buffer[2] = value;
		break;
	case UAV_WBB:
		buffer[3] = value;
		break;
	case UAV_WBR:
		buffer[4] = value;
		break;
	case UAV_NCOLS:
		buffer[5] = value;
		break;
	case UAV_NROWS:
		buffer[6] = value;
		break;
	case UAV_CCDCOL:
		buffer[7] = value;
		break;
	case UAV_CCDROW:
		buffer[8] = value;
		break;
	case UAV_CENTERCOL:
		buffer[9] = value;
		break;
	case UAV_CENTERROW:
		buffer[10] = value;
		break;
	case UAV_INRADIUS:
		buffer[11] = value;
		break;
	case UAV_OUTRADIUS:
		buffer[12] = value;
		break;
	case UAV_PACKETSIZE:
		buffer[13] = value;
		break;
	case UAV_FPS:
		buffer[14] = value;
		break;
	case UAV_VIDMODE:
		buffer[15] = value;
		break;
	case UAV_COLORCODING:
		buffer[16] = value;
		break;
	case UAV_NCHANNELS:
		buffer[17] = value;
		break;
	case UAV_FORMAT7:
		buffer[18] = value;
		break;
	case UAV_CAMERAUSED:
		buffer[19] = value;
		break;
	case UAV_CAMERARUNNING:
		buffer[20] = value;
		break;
	default:
		std::cout<<"Invalid parameter request to get!"<<std::endl;
		break;
	}
}
}
