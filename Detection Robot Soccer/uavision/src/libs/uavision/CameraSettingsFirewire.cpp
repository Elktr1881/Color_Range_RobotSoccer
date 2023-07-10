#include "CameraSettingsFirewire.h"
#include <string>
#include <fstream>
#include <iostream>

namespace uav{

CameraSettingsFirewire::CameraSettingsFirewire()
{
	buffer = new unsigned int[29];

	buffer[0] = 0; //powerState
	buffer[1] = 800; //speed
	buffer[2] = 1; //opMode
	buffer[3] = UAV_RGB8; //vidMode
	buffer[4] = UAV_RGB8; //colorCoding
	buffer[5] = 30; //fps
	buffer[6] = 1.0f; //f7fps

	buffer[7]= 0; //exposure
	buffer[8] = 0; //wbb
	buffer[9] = 0; // wbr
	buffer[10] = 0; //gain
	buffer[11] = 0; //shutter
	buffer[12] = 0; //brightness
	buffer[13] = 0; //saturation
	buffer[14] = 0; //gamma
	buffer[15] = 0; //sharpness


	buffer[16] = 640; //nCols
	buffer[17] = 480; //nRows
	buffer[18] = 0; //ccdCol
	buffer[19] = 0; //ccdRow
	buffer[20] = 320; //centerCol
	buffer[21] = 240; //centerRow
	buffer[22] = 50; //inRadius
	buffer[23] = 100; //outRadius
	buffer[24] = 0; //packetSize
	buffer[25] = 3; //nChannels


	buffer[26] = true; //format7
	buffer[27] = false; //cameraWasUsed
	buffer[28] = false; //cameraRunning
}

CameraSettingsFirewire::~CameraSettingsFirewire()
{
	delete []buffer;
}

unsigned int* CameraSettingsFirewire::ptr()
{
	return buffer;
}

int CameraSettingsFirewire::getSize()
{
	int cs_size = 29 * sizeof(unsigned int);

	return cs_size;

}


void CameraSettingsFirewire::printCameraSettings()
{

	std::cout<<"CAMERA SETTINGS FIREWIRE PRINTED"<<std::endl;
	std::cout <<"Power state: "<< buffer[0]<<std::endl;
	std::cout<<"Speed: "<<buffer[1]<<std::endl;
	std::cout <<"Operation mode: "<< buffer[2]<<std::endl;
	std::cout << "Video mode: " << buffer[3] << std::endl;
	std::cout << "Color coding: "<<  buffer[4] << std::endl;
	std::cout << "Frames per second: " <<buffer[5] << std::endl;
	std::cout << "Format7 fps:" << buffer[6] << std::endl;
	std::cout<<"---Colormetric info---"<<std::endl;
	std::cout<<"Exposure: "<< buffer[7]<<std::endl;
	std::cout<<"White balance blue: "<< buffer[8] <<std::endl;
	std::cout<<"White balance red: "<< buffer[9] <<std::endl;
	std::cout<<"Gain: "<< buffer[10] <<std::endl;
	std::cout<<"Shutter: "<< buffer[11]<<std::endl;
	std::cout<<"Brightness: "<< buffer[12]<<std::endl;
	std::cout<<"Saturation: "<< buffer[13]<<std::endl;
	std::cout<<"Gamma: "<< buffer[14]<<std::endl;
	std::cout<<"Sharpness: "<< buffer[15]<<std::endl;
	std::cout<<"---Resolution info---"<<std::endl;
	std::cout<<"Width (nCols): "<<buffer[16]<<std::endl;
	std::cout<<"Height (nRows):" << buffer[17] <<std::endl;
	std::cout<< "CCD column: " << buffer[18] <<std::endl;
	std::cout<< "CCD row: " << buffer[19] <<std::endl;
	std::cout <<"center Col: " << buffer[20] << std::endl;
	std::cout << "center Row: " << buffer[21] << std::endl;
	std::cout << "inRadius: " << buffer[22] << std::endl;
	std::cout <<"outRadius: " << buffer[23] << std::endl;
	std::cout << " Packet size: "<< buffer[24] << std::endl;
	std::cout << "Number of channels: " << buffer[25] << std::endl;
	std::cout << "---Flags---" << std::endl;
	std::cout << "Format7: " << buffer[26] << std::endl;
	std::cout <<"Camera was used: " << buffer[27] << std::endl;
	std::cout << "Camera running: " << buffer[28] << std::endl;
}


int CameraSettingsFirewire::getCameraSetting(unsigned setting)
{
	int value = -1;

	switch(setting)
	{
	case UAV_POWERSTATE:
		value = buffer[0];
		break;
	case UAV_SPEED:
		value = buffer[1];
		break;
	case UAV_OPMODE:
		value = buffer[2];
		break;
	case UAV_VIDMODE:
		value = buffer[3];
		break;
	case UAV_COLORCODING:
		value = buffer[4];
		break;
	case UAV_FPS:
		value = buffer[5];
		break;
	case UAV_F7FPS:
		value = buffer[6];
		break;
	case UAV_EXPOSURE:
		value = buffer[7];
		break;
	case UAV_WBB:
		value = buffer[8];
		break;
	case UAV_WBR:
		value = buffer[9];
		break;
	case UAV_GAIN:
		value = buffer[10];
		break;
	case UAV_SHUTTER:
		value = buffer[11];
		break;
case UAV_BRIGHTNESS:
	value = buffer[12];
	break;
case UAV_SATURATION:
	value = buffer[13];
	break;
case UAV_GAMMA:
	value = buffer[14];
	break;
case UAV_SHARPNESS:
	value = buffer[15];
	break;
case UAV_NCOLS:
	value = buffer[16];
	break;
case UAV_NROWS:
	value = buffer[17];
	break;
case UAV_CCDCOL:
	value = buffer[18];
	break;
case UAV_CCDROW:
	value = buffer[19];
	break;
case UAV_CENTERCOL:
	value = buffer[20];
	break;
case UAV_CENTERROW:
	value = buffer[21];
	break;
case UAV_INRADIUS:
	value = buffer[22];
	break;
case UAV_OUTRADIUS:
	value = buffer[23];
	break;
case UAV_PACKETSIZE:
	value = buffer[24];
	break;
case UAV_NCHANNELS:
	value = buffer[25];
	break;
case UAV_FORMAT7:
	value = buffer[26];
	break;
case UAV_CAMERAUSED:
	value = buffer[27];
	break;
case UAV_CAMERARUNNING:
	value = buffer[28];
	break;
default:
	std::cout<<"Invalid parameter request to get!"<<std::endl;
	break;
	}

	return value;
}

void CameraSettingsFirewire::setCameraSetting(unsigned setting, unsigned value)
{

	switch(setting)
	{
	case UAV_POWERSTATE:
		buffer[0] = value;
		break;
	case UAV_SPEED:
		buffer[1] = value;
		break;
	case UAV_OPMODE:
		buffer[2] = value;
		break;
	case UAV_VIDMODE:
		buffer[3] = value;
		break;
	case UAV_COLORCODING:
		buffer[4] = value;
		break;
	case UAV_FPS:
		buffer[5] = value;
		break;
	case UAV_F7FPS:
		buffer[6] = value;
		break;
	case UAV_EXPOSURE:
		buffer[7] = value;
		break;
	case UAV_WBB:
		buffer[8] = value;
		break;
	case UAV_WBR:
		buffer[9] = value;
		break;
	case UAV_GAIN:
		buffer[10] = value;
		break;
	case UAV_SHUTTER:
		buffer[11] = value;
		break;
	case UAV_BRIGHTNESS:
		buffer[12] = value;
		break;
	case UAV_SATURATION:
		buffer[13] = value;
		break;
	case UAV_GAMMA:
		buffer[14] = value;
		break;
	case UAV_SHARPNESS:
		buffer[15] = value;
		break;
	case UAV_NCOLS:
		buffer[16] = value;
		break;
	case UAV_NROWS:
		buffer[17] = value;
		break;
	case UAV_CCDCOL:
		buffer[18] = value;
		break;
	case UAV_CCDROW:
		buffer[19] = value;
		break;
	case UAV_CENTERCOL:
		buffer[20] = value;
		break;
	case UAV_CENTERROW:
		buffer[21] = value;
		break;
	case UAV_INRADIUS:
		buffer[22] = value;
		break;
	case UAV_OUTRADIUS:
		buffer[23] = value;
		break;
	case UAV_PACKETSIZE:
		buffer[24] = value;
		break;
	case UAV_NCHANNELS:
		buffer[25] = value;
		break;
	case UAV_FORMAT7:
		buffer[26] = value;
		break;
	case UAV_CAMERAUSED:
		buffer[27] = value;
		break;
	case UAV_CAMERARUNNING:
		buffer[28] = value;
		break;
	default:
		std::cout<<"Invalid parameter request to set!"<<std::endl;
		break;
	}


}
}
