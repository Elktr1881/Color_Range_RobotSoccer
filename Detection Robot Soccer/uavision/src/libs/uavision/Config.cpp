/*
 * Config.cpp
 *
 *  Author: alt & an
 */

#include "Config.h"
#include <dc1394/dc1394.h>
#include "CameraSettings.h"
#include "CameraSettingsOpenCV.h"
#include "CameraSettingsEth.h"
#include "CameraSettingsFirewire.h"


namespace uav{

Config::Config( const std::string fileName_ )
{
	fileName = fileName_;
	colorRange = new ColorRange[UAV_NCOLORS];
	internalData.mapExists = false;
	internalData.maskExists= false;
	internalData.camSettingsExists = false;
	internalData.colorRangesExists = false;
	internalData.cameraType = UAV_ETH;
	camSettings = camSettingsFactory.Create(internalData.cameraType);
	map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));
	int error;

	if( fileName_.size() != 0 )
	{
		error = load( fileName_ );

		switch( error )
		{
		case ERROR_EMPTYCONFIG:
			std::cout<<"ERROR_EMPTYCONFIG"<<std::endl;
			break;
		case ERROR_INCOMPATIBLE:
			std::cout<<"ERROR_INCOMPATIBLE"<<std::endl;
			break;
		case ERROR_NOFILENAME:
			std::cout<<"ERROR_NOFILENAME"<<std::endl;
			break;
		case ERROR_WRONGVERSION:
			std::cout<<"ERROR_WRONGCONVERSION"<<std::endl;
			break;
		}
	}
}

void Config::initializeConfig(unsigned int camType, bool mapFlag, bool maskFlag, bool camSettingsFlag, bool colorRangesFlag)
{

	internalData.mapExists = mapFlag;
	internalData.maskExists= maskFlag;
	internalData.camSettingsExists = camSettingsFlag;
	internalData.colorRangesExists = colorRangesFlag;
	internalData.cameraType = camType;

	if(camSettingsFlag)
		camSettings = camSettingsFactory.Create(internalData.cameraType);

	if(mapFlag)
	{
		if(camSettingsFlag)
			map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));
		else
			std::cout<<"Cannot create map without knowledge of camSettings"<<std::endl;
	}


	if(colorRangesFlag)
		colorRange = new ColorRange[UAV_NCOLORS];


	if(maskFlag)
	{
		if(camSettingsFlag)
		{
			mask = cv::Mat(camSettings->getCameraSetting(UAV_NROWS),camSettings->getCameraSetting(UAV_NCOLS),CV_8UC1);
			mask.setTo(cv::Scalar(255));
		}
		else
		{
			std::cout<<"Cannot create mask without knowledge of camSettings"<<std::endl;
		}
	}


}
void Config::initializeConfigWithoutCamSettings(unsigned int camType, bool mapFlag, bool maskFlag, bool colorRangesFlag)
{

	internalData.mapExists = mapFlag;
	internalData.maskExists= maskFlag;
	internalData.camSettingsExists = true;
	internalData.colorRangesExists = colorRangesFlag;
	internalData.cameraType = camType;

	if(mapFlag)
	{
		map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));
	}


	if(colorRangesFlag)
		colorRange = new ColorRange[UAV_NCOLORS];


	if(maskFlag)
	{
		mask = cv::Mat(camSettings->getCameraSetting(UAV_NROWS),camSettings->getCameraSetting(UAV_NCOLS),CV_8UC1);
		mask.setTo(cv::Scalar(255));
	}
}

Config::~Config()
{
	if(internalData.maskExists) mask.release() ;
	if(internalData.colorRangesExists) delete []colorRange;
	if(internalData.mapExists) map.clear();
	if(file.is_open()) file.close();
}

void Config::createConfig(const std::string name, unsigned int camType, bool mapFlag, bool maskFlag, bool camSettingsFlag, bool colorRangesFlag, bool binary, CameraSettings *_camSettings )
{
	fileName = name;

	if(_camSettings != NULL)
	{
		camSettings = _camSettings;
		initializeConfigWithoutCamSettings(camType, mapFlag, maskFlag, colorRangesFlag);
	}
	else
	{
		initializeConfig(camType,mapFlag,maskFlag,camSettingsFlag,colorRangesFlag);
	}

	if(!binary)
		saveAscii(name);
	else
		save(name);
}

void Config::resizeConfig(int resizeFactor)
{
	if(internalData.mapExists)
	{
		//map.clear();

		std::vector<cv::Point2d> mapTemp;
		mapTemp = map;

		int cols = camSettings->getCameraSetting(UAV_NCOLS)/resizeFactor;
		int rows = camSettings->getCameraSetting(UAV_NROWS)/resizeFactor;

		int idxOriginal, idxResize;

		map.resize((camSettings->getCameraSetting(UAV_NCOLS)/resizeFactor)*(camSettings->getCameraSetting(UAV_NROWS)/resizeFactor));
		for(int i = 0; i < rows; i++)
		{
			for(int j = 0; j < cols; j++)
			{
				idxOriginal = (i*cols*resizeFactor + j)*resizeFactor;
				idxResize = (i*cols + j);
				map[idxResize] = mapTemp[idxOriginal];
			}
		}


	}


	if(internalData.maskExists)
	{
		//mask.release();
		cv::Mat maskTemp;
		mask.copyTo(maskTemp);
		cv::resize(mask, mask,cv::Size(), 1.0/resizeFactor, 1.0/resizeFactor);
		convertResolutionGray(maskTemp, mask, resizeFactor);
	}

	camSettings->setCameraSetting(UAV_CENTERCOL, camSettings->getCameraSetting(UAV_CENTERCOL)/resizeFactor);
	camSettings->setCameraSetting(UAV_CENTERROW, camSettings->getCameraSetting(UAV_CENTERROW)/resizeFactor);

	camSettings->setCameraSetting(UAV_NROWS, camSettings->getCameraSetting(UAV_NROWS)/resizeFactor);
	camSettings->setCameraSetting(UAV_NCOLS, camSettings->getCameraSetting(UAV_NCOLS)/resizeFactor);

	int inRadius = camSettings->getCameraSetting(UAV_INRADIUS);
	camSettings->setCameraSetting(UAV_INRADIUS, inRadius/resizeFactor);

	int outRadius = camSettings->getCameraSetting(UAV_OUTRADIUS);
	camSettings->setCameraSetting(UAV_OUTRADIUS, outRadius/resizeFactor);

}

void Config::imageSizeChanged()
{

	mask.release();
	map.clear();

	if(internalData.maskExists)
		mask = cv::Mat(camSettings->getCameraSetting(UAV_NROWS),camSettings->getCameraSetting(UAV_NCOLS),CV_8UC1);
	if(internalData.mapExists)
		map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));

	camSettings->setCameraSetting(UAV_CENTERCOL, camSettings->getCameraSetting(UAV_NCOLS)/2);
	camSettings->setCameraSetting(UAV_CENTERROW, camSettings->getCameraSetting(UAV_NROWS)/2);
	camSettings->setCameraSetting(UAV_INRADIUS, 50);
	camSettings->setCameraSetting(UAV_OUTRADIUS, 100);

	std::cout << " - ATTENTION image size changed (map and mask reset) - ";
}


int Config::load( const std::string fileName_ )
{
	if( fileName_.size() == 0 )
	{
		std::cout << "Config::load ERROR invalid config file name." << std::endl;
		return ERROR_NOFILENAME;
	}

	fileName = fileName_;

	file.open(fileName.c_str(), std::fstream::in |  std::fstream::binary);

	if( file.fail() )
	{
		std::cout<< "Something failed on loading the file" << std::endl;
	}

	if( !file.is_open() )
	{
		std::cout<< "Config::load  ERROR opening " << fileName_ << " configuration file" << std::endl;
		return ERROR_CANNOTOPENFILE;
	}

	if(file.is_open())
	{

		file.read(reinterpret_cast<char *>(&internalData), sizeof(internalData));

		std::cout << "Config file version " << internalData.version << std::endl;

		if(internalData.camSettingsExists)
		{
			std::cout<<"load camSettings\n";
			camSettings = camSettingsFactory.Create(internalData.cameraType);
			file.read(reinterpret_cast<char *>(camSettings->ptr()), camSettings->getSize());
		}

		if(internalData.mapExists)
		{
			std::cout << "load map" << std::endl;
			map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));
			file.read(reinterpret_cast<char *>(&map[0]), map.size() * sizeof(cv::Point2d));
		}

		if(internalData.maskExists)
		{
			std::cout << "load mask" <<std::endl;
			mask = cv::Mat(camSettings->getCameraSetting(UAV_NROWS),camSettings->getCameraSetting(UAV_NCOLS),CV_8UC1);
			file.read(reinterpret_cast<char *>(mask.ptr()), camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS));
			//file.read(reinterpret_cast<char *>(mask.ptr()), camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS));
		}

		if(internalData.colorRangesExists)
		{
			std::cout << "load color ranges"<<std::endl;
			colorRange = new ColorRange[UAV_NCOLORS];
			file.read(reinterpret_cast<char *>(colorRange), colorRange->getSize()*UAV_NCOLORS);
		}

		std::cout << "Loading done..." << std::endl;
	}

	file.close();

	return ERROR_NOERROR;
}


int Config::save( const std::string fileName_ )
{

	std::string saveFileName;

	if( fileName_.size() != 0 )
		saveFileName = fileName_;
	else if( fileName.size() == 0 )
		return ERROR_NOFILENAME;
	else
		saveFileName = fileName;

	if(file.is_open())
		file.close();

	file.open(saveFileName.c_str(), std::fstream::out | std::fstream::binary);
	if( file.fail() )
	{
		std::cout << "Config::save ERROR opening " << saveFileName << " configuration file" << std::endl;
		return ERROR_CANNOTOPENFILE;
	}

	if( !file.is_open() )
	{
		std::cout<< "createConfig:: save ERROR opening " << fileName_ << " configuration file" << std::endl;
		return ERROR_CANNOTOPENFILE;
	}

	if(file.is_open())
	{
		std::cout<< "Config: saving... camera type"<<std::endl;

		std::string s("");

		if(internalData.cameraType > 4)
		{
			std::cout<< " Invalid camera type! {UAV_ETHERNET = 0, UAV_OPENCV, UAV_FIREWIRE, UAV_I2C, UAV_KINECT}" << std::endl;
		}

		file.write(reinterpret_cast<char *>(&internalData), sizeof(ConfigInternals));

		if(internalData.camSettingsExists)
		{
			std::cout << "save camera settings..." << camSettings->getSize();
			camSettings->setCameraSetting(UAV_CAMERARUNNING, 0);
			camSettings->setCameraSetting(UAV_CAMERAUSED, 0);
			file.write(reinterpret_cast<char *>(camSettings->ptr()), camSettings->getSize());
			std::cout << " done!\n";
		}

		if(internalData.mapExists)
		{
			std::cout << "save map..." << map.size() * sizeof(cv::Point2d) ;
			file.write(reinterpret_cast<char *>(&map[0]), map.size() * sizeof(cv::Point2d));
			std::cout << " done!\n";
		}

		if(internalData.maskExists)
		{
			std::cout << "save mask " << camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS);
			file.write(reinterpret_cast<char *>(mask.ptr()), camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS));
			/*std::vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);
			cv::imwrite("mask.png", mask, compression_params);*/
			std::cout << " (DONE: " << camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS) << " bytes) \n";
		}

		if(internalData.colorRangesExists)
		{
			std::cout << "save color range..." << colorRange->getSize()*UAV_NCOLORS;
			file.write(reinterpret_cast<char *>(colorRange), colorRange->getSize()*UAV_NCOLORS);
			std::cout << " done!\n";

		}
		std::cout << "Save done."<<std::endl;

	}
	fileName = saveFileName;

	file.close();

	return ERROR_NOERROR;
}


void Config::loadAscii( const std::string fileName_)
{
	fileName = fileName_;

	bool cameraSettingsDone = false;
	bool colorRangesDone = false;
	bool maskDone = false;
	bool mapDone = false;

	std::fstream inFile;

	// test if we are printing to file or standard output
	if( fileName_.size() != 0 )
	{
		// open the file
		inFile.open(fileName_.c_str(), std::fstream::in | std::fstream::binary);
	}

	std::string confName;
	std::string modeName;
	short trashFlag;
	int trashCam;

	inFile >> internalData.version;
	inFile >> internalData.cameraType;

	std::cout << "Camera type: " << internalData.cameraType << std::endl;

	camSettings = camSettingsFactory.Create(internalData.cameraType);

	std::cout << "Config: reading camera parameters";
	std::cout.flush();

	while(inFile.good())// == false)
	{
		inFile >> confName;

		if( !confName.compare("camSettingsExist") )
		{
			internalData.camSettingsExists = true;
		}

		if( !confName.compare("maskExists") )
		{
			if(!cameraSettingsDone) std::cout << "... done." << std::endl;
			cameraSettingsDone = true;
			internalData.maskExists= true;
		}

		if( !confName.compare("mapExists") )
		{
			if(!cameraSettingsDone) std::cout << "... done." << std::endl;
			cameraSettingsDone = true;
			internalData.mapExists = true;
		}


		if( !confName.compare("colorRangesExist") )
		{
			if(!cameraSettingsDone) std::cout << "... done." << std::endl;
			cameraSettingsDone = true;
			internalData.colorRangesExists = true;
		}

		if(internalData.camSettingsExists && !cameraSettingsDone){
			switch (internalData.cameraType)
			{
			case UAV_ETH:
				if( !confName.compare("exposure") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_EXPOSURE,trashCam);
				}

				else if( !confName.compare("gain") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_GAIN,trashCam);
				}

				else if( !confName.compare("pixelClock") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_PIXELCLOCK,trashCam);
				}

				else if( !confName.compare("wbb") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_WBB,trashCam);
				}

				else if( !confName.compare("wbr") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_WBR,trashCam);
				}

				else if( !confName.compare("nCols") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCOLS,trashCam);
				}

				else if( !confName.compare("nRows") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NROWS,trashCam);
				}

				else if( !confName.compare("ccdC") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDCOL, trashCam);
				}

				else if( !confName.compare("ccdR") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDROW,trashCam);
				}


				else if( !confName.compare("packetSize") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_PACKETSIZE, trashCam);
				}

				else if( !confName.compare("fps") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_FPS, value);
				}

				else if( !confName.compare("nChannels") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCHANNELS,trashCam);
				}


				else if( !confName.compare("mode") )
				{
					std::string value;
					inFile >> value;
					if( !value.compare("YUV411") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV411);
					else if( !modeName.compare("YUV422") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV422);
					else if( !modeName.compare("RGB8") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_RGB8);
				}


				else if( !confName.compare("colorCoding") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_COLORCODING,trashCam);
				}

				else if( !confName.compare("format7") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_FORMAT7, value);
				}

				else if( !confName.compare("cameraUsed") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERAUSED, value);

				}

				else if( !confName.compare("cameraRunning") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERARUNNING, value);

				}


				else if( !confName.compare("center") )
				{
					unsigned valueCol, valueRow;
					inFile >> valueCol >> valueRow;
					camSettings->setCameraSetting(UAV_CENTERCOL, valueCol);
					camSettings->setCameraSetting(UAV_CENTERROW, valueRow);
				}



				else if( !confName.compare("inRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_INRADIUS, trashCam);
				}

				else if( !confName.compare("outRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_OUTRADIUS, trashCam);
				}
				break;

			case UAV_FIREWIRE:
				if( !confName.compare("powerState") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_POWERSTATE,trashCam);
				}

				if( !confName.compare("speed") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_SPEED,trashCam);
				}

				if( !confName.compare("opMode") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_OPMODE,trashCam);
				}


				if( !confName.compare("f7fps") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_F7FPS,trashCam);
				}

				if( !confName.compare("exposure") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_EXPOSURE,trashCam);
				}


				if( !confName.compare("brightness") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_BRIGHTNESS,trashCam);
				}

				if( !confName.compare("sharpness") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_SHARPNESS,trashCam);
				}

				if( !confName.compare("saturation") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_SATURATION,trashCam);
				}

				if( !confName.compare("gamma") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_GAMMA,trashCam);
				}

				if( !confName.compare("shutter") )
				{
					inFile >>trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_SHUTTER,trashCam);
				}

				else if( !confName.compare("gain") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_GAIN,trashCam);
				}

				else if( !confName.compare("wbb") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_WBB,trashCam);
				}

				else if( !confName.compare("wbr") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_WBR,trashCam);
				}

				else if( !confName.compare("nCols") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCOLS,trashCam);
				}

				else if( !confName.compare("nRows") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NROWS,trashCam);
				}

				else if( !confName.compare("ccdC") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDCOL, trashCam);
				}

				else if( !confName.compare("ccdR") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDROW,trashCam);
				}


				else if( !confName.compare("packetSize") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_PACKETSIZE, trashCam);
				}

				else if( !confName.compare("fps") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_FPS, value);
				}

				else if( !confName.compare("nChannels") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCHANNELS,trashCam);
				}


				else if( !confName.compare("mode") )
				{
					std::string value;
					inFile >> value;
					if( !value.compare("YUV411") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV411);
					else if( !modeName.compare("YUV422") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV422);
					else if( !modeName.compare("RGB8") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_RGB8);
				}


				else if( !confName.compare("colorCoding") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_COLORCODING,trashCam);
				}

				else if( !confName.compare("format7") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_FORMAT7, value);
				}

				else if( !confName.compare("cameraUsed") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERAUSED, value);

				}

				else if( !confName.compare("cameraRunning") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERARUNNING, value);

				}


				else if( !confName.compare("center") )
				{
					unsigned valueCol, valueRow;
					inFile >> valueCol >> valueRow;
					camSettings->setCameraSetting(UAV_CENTERCOL, valueCol);
					camSettings->setCameraSetting(UAV_CENTERROW, valueRow);
				}



				else if( !confName.compare("inRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_INRADIUS, trashCam);
				}

				else if( !confName.compare("outRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_OUTRADIUS, trashCam);
				}
				break;

			case UAV_OPENCV:
			case UAV_KINECT:
				if( !confName.compare("nCols") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCOLS,trashCam);
				}

				else if( !confName.compare("nRows") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NROWS,trashCam);
				}

				else if( !confName.compare("ccdC") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDCOL, trashCam);
				}

				else if( !confName.compare("ccdR") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_CCDROW,trashCam);
				}

				else if( !confName.compare("fps") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_FPS, value);
				}

				else if( !confName.compare("nChannels") )
				{
					inFile >> trashCam;
					if(trashCam != -1)
						camSettings->setCameraSetting(UAV_NCHANNELS,trashCam);
				}


				else if( !confName.compare("mode") )
				{
					std::string value;
					inFile >> value;
					if( !value.compare("YUV411") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV411);
					else if( !modeName.compare("YUV422") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_YUV422);
					else if( !modeName.compare("RGB8") )
						camSettings->setCameraSetting(UAV_VIDMODE, UAV_RGB8);
				}

				else if( !confName.compare("cameraUsed") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERAUSED, value);

				}

				else if( !confName.compare("cameraRunning") )
				{
					unsigned value;
					inFile >> value;
					camSettings->setCameraSetting(UAV_CAMERARUNNING, value);

				}
				else if( !confName.compare("center") )
				{
					unsigned valueCol, valueRow;
					inFile >> valueCol >> valueRow;
					camSettings->setCameraSetting(UAV_CENTERCOL, valueCol);
					camSettings->setCameraSetting(UAV_CENTERROW, valueRow);
				}



				else if( !confName.compare("inRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_INRADIUS, trashCam);
				}

				else if( !confName.compare("outRadius") )
				{
					inFile >> trashCam;
					camSettings->setCameraSetting(UAV_OUTRADIUS, trashCam);
				}
				break;
			default:
				fprintf(stderr, "Config::loadAscii -> unknown camera type!\n");
				break;

			}

		}

		if(internalData.maskExists&& !maskDone)
		{
			std::cout << "Config: reading mask";
			std::cout.flush();

			mask = cv::Mat(camSettings->getCameraSetting(UAV_NROWS),camSettings->getCameraSetting(UAV_NCOLS),CV_8UC1);
			for(unsigned i = 0 ; i < camSettings->getCameraSetting(UAV_NROWS); i++)
				for (unsigned j = 0; j < camSettings->getCameraSetting(UAV_NCOLS); j++)
				{
					int tmp;

					inFile >> tmp;
					mask.at<uchar>(i,j) = (uchar)tmp;
				}

			std::cout << "... done." << std::endl;
			maskDone = true;
		}

		if(internalData.mapExists && !mapDone)
		{

			std::cout << "Config: reading map...";
			std::cout.flush();
			//inFile.precision(12);
			map.resize(camSettings->getCameraSetting(UAV_NCOLS)*camSettings->getCameraSetting(UAV_NROWS));
			for(unsigned i = 0 ; i < camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS); i++)
			{

				inFile >> map[i].x >> map[i].y >> std::ws;
			}

			std::cout << "... done." << std::endl;
			mapDone = true;
		}

		if(internalData.colorRangesExists && !colorRangesDone)
		{

			std::cout << "Config: reading color range settings";
			std::cout.flush();

			unsigned nColors;
			std::string trashStr;
			inFile >> trashStr >> nColors >> std::ws;
			std::cout << " (number of colors " << nColors << ")";

			colorRange = new ColorRange[UAV_NCOLORS];

			for(unsigned c = 0; c < UAV_NCOLORS; c++)
			{
				inFile >> colorRange[c].name >> std::ws;
				colorRange[c].name[COLOR_NAME_LENGTH-1] = '\0';
				inFile >> trashStr >> colorRange[c].paintColor >> std::ws;
				inFile >> trashStr >> colorRange[c].hMax >> std::ws;
				inFile >> trashStr >> colorRange[c].hMin >> std::ws;
				inFile >> trashStr >> colorRange[c].sMax >> std::ws;
				inFile >> trashStr >> colorRange[c].sMin >> std::ws;
				inFile >> trashStr >> colorRange[c].vMax >> std::ws;
				inFile >> trashStr >> colorRange[c].vMin >> std::ws;
			}

			std::cout << "... done." << std::endl;
			colorRangesDone = true;
		}

	}
}

void Config::saveAscii( const std::string fileName_)
{
	std::fstream outFile;
	std::string saveFileName;

	if( fileName_.size() != 0 )
		saveFileName = fileName_;
	else if( fileName.size() == 0 )
		std::cout<<"x"<<std::endl;
	else
		saveFileName = fileName;

	// open the file
	outFile.open(saveFileName.c_str(), std::fstream::out | std::fstream::binary);

	outFile << 201403 << std::endl;
	outFile << internalData.cameraType << std::endl;

	if( internalData.camSettingsExists )
	{
		std::cout << "Config: saving camera parameters\n";
		std::cout.flush();

		outFile << "camSettingsExist"<< std::endl;

		switch(internalData.cameraType)
		{

		case UAV_ETH:
			outFile << "exposure " << camSettings->getCameraSetting(UAV_EXPOSURE) << std::endl;
			outFile << "gain " << camSettings->getCameraSetting(UAV_GAIN) << std::endl;
			outFile << "wbb " << camSettings->getCameraSetting(UAV_WBB) << std::endl;
			outFile << "wbr " << camSettings->getCameraSetting(UAV_WBR) << std::endl;
			outFile << "pixelClock " << camSettings->getCameraSetting(UAV_PIXELCLOCK) << std::endl;
			outFile << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
			outFile << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
			outFile << "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
			outFile << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
			outFile << "packetSize " << camSettings->getCameraSetting(UAV_PACKETSIZE) << std::endl;
			outFile << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
			outFile << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;
			outFile << "inRadius " << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
			outFile << "outRadius " << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
			outFile << "center " << camSettings->getCameraSetting(UAV_CENTERCOL) << " " <<  camSettings->getCameraSetting(UAV_CENTERROW) << std::endl;

			switch( camSettings->getCameraSetting(UAV_VIDMODE) )
			{
			case UAV_YUV411:
				outFile << "mode YUV411" << std::endl;
				break;
			case UAV_YUV422:
				outFile << "mode YUV422" << std::endl;
				break;
			case UAV_RGB8:
				outFile << "mode RGB8" << std::endl;
				break;
			default:
				fprintf(stderr, "Video mode not supported!\n");
				break;
			}

			outFile << "colorCoding " << camSettings->getCameraSetting(UAV_COLORCODING) << std::endl;

			outFile << "format7 " << camSettings->getCameraSetting(UAV_FORMAT7) << std::endl;
			outFile << "camUsed " << camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
			outFile << "camRunning " << camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;

			std::cout << "... done." << std::endl;
			break;

			case UAV_FIREWIRE:
				outFile << "powerState " << camSettings->getCameraSetting(UAV_POWERSTATE) << std::endl;
				outFile << "speed " << camSettings->getCameraSetting(UAV_SPEED) << std::endl;
				outFile << "opMode " << camSettings->getCameraSetting(UAV_OPMODE) << std::endl;
				outFile << "exposure " << camSettings->getCameraSetting(UAV_EXPOSURE) << std::endl;
				outFile << "colorCoding " << camSettings->getCameraSetting(UAV_COLORCODING) << std::endl;
				outFile << "f7fps " << camSettings->getCameraSetting(UAV_F7FPS) << std::endl;
				outFile << "brightness " << camSettings->getCameraSetting(UAV_BRIGHTNESS) << std::endl;
				outFile << "sharpness " << camSettings->getCameraSetting(UAV_SHARPNESS) << std::endl;
				outFile << "saturation " << camSettings->getCameraSetting(UAV_SATURATION) << std::endl;
				outFile << "gamma " << camSettings->getCameraSetting(UAV_GAMMA) << std::endl;
				outFile << "shutter " << camSettings->getCameraSetting(UAV_SHUTTER) << std::endl;
				outFile << "gain " << camSettings->getCameraSetting(UAV_GAIN) << std::endl;
				outFile << "wbb " << camSettings->getCameraSetting(UAV_WBB) << std::endl;
				outFile << "wbr " << camSettings->getCameraSetting(UAV_WBR) << std::endl;
				outFile << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
				outFile << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
				outFile << "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
				outFile << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
				outFile << "packetSize " << camSettings->getCameraSetting(UAV_PACKETSIZE) << std::endl;
				outFile << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
				outFile << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;

				switch( camSettings->getCameraSetting(UAV_VIDMODE) )
				{
				case UAV_YUV411:
					outFile << "mode YUV411" << std::endl;
					break;
				case UAV_YUV422:
					outFile << "mode YUV422" << std::endl;
					break;
				case UAV_RGB8:
					outFile << "mode RGB8" << std::endl;
					break;
				default:
					fprintf(stderr, "Video mode not supported!\n");
					break;
				}

				outFile << "inRadius " << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
				outFile << "outRadius " << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
				outFile << "center " << camSettings->getCameraSetting(UAV_CENTERCOL) << " " <<  camSettings->getCameraSetting(UAV_CENTERROW) << std::endl;
				outFile << "format7 " << camSettings->getCameraSetting(UAV_FORMAT7) << std::endl;
				outFile << "camUsed " << camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
				outFile << "camRunning " << camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;

				std::cout << "... done." << std::endl;
				break;

				case UAV_OPENCV:
				case UAV_KINECT:
					outFile << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
					outFile << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
					outFile << "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
					outFile << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
					outFile << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
					outFile << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;

					switch( camSettings->getCameraSetting(UAV_VIDMODE) )
					{
					case UAV_YUV411:
						outFile << "mode YUV411" << std::endl;
						break;
					case UAV_YUV422:
						outFile << "mode YUV422" << std::endl;
						break;
					case UAV_RGB8:
						outFile << "mode RGB8" << std::endl;
						break;
					default:
						fprintf(stderr, "Video mode not supported!\n");
						break;
					}

					outFile << "inRadius " << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
					outFile << "outRadius " << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
					outFile << "center " << camSettings->getCameraSetting(UAV_CENTERCOL) << " " <<  camSettings->getCameraSetting(UAV_CENTERROW) << std::endl;
					outFile << "camUsed " << camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
					outFile << "camRunning " << camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;
					break;

					default:
						break;

		}
	}

	if(internalData.maskExists)
	{


		std::cout << "Config: saving mask ascii\n";
		std::cout.flush();

		outFile <<"maskExists"<< std::endl;


		for(unsigned i = 0 ; i < camSettings->getCameraSetting(UAV_NROWS); i++)
			for (unsigned j = 0; j < camSettings->getCameraSetting(UAV_NCOLS); j++)
				outFile << (int)mask.at<uchar>(i,j) << std::endl;

		std::cout << "... done." << std::endl;
	}

	if(internalData.mapExists)
	{
		outFile << "mapExists" << std::endl;
		std::cout << "Config: saving map (ATTENTION: may lose map precision)\n";
		std::cout.flush();

		outFile << camSettings->getCameraSetting(UAV_NCOLS) << " " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
		//outFile.precision(12);
		for(unsigned i = 0 ; i < camSettings->getCameraSetting(UAV_NROWS)*camSettings->getCameraSetting(UAV_NCOLS); i++)
			outFile << map[i].x << "\t" << map[i].y << std::endl;

		std::cout << "... done." << std::endl;
	}

	if( internalData.colorRangesExists )
	{
		outFile << "colorRangesExist" << std::endl;
		std::cout << "Config: saving color range settings\n";
		std::cout.flush();

		outFile << "nColors " << UAV_NCOLORS <<std::endl;

		for(int c = 0 ; c < UAV_NCOLORS ; c++)
		{
			colorRange[c].name[COLOR_NAME_LENGTH-1] = '\0';
			outFile << colorRange[c].name << std::endl;
			outFile << "HMax " << colorRange[c].hMax << std::endl;
			outFile << "HMin " << colorRange[c].hMin << std::endl;
			outFile << "SMax " << colorRange[c].sMax << std::endl;
			outFile << "SMin " << colorRange[c].sMin << std::endl;
			outFile << "VMax " << colorRange[c].vMax << std::endl;
			outFile << "VMin " << colorRange[c].vMin << std::endl;
		}

		std::cout << "... done." << std::endl;
	}

	outFile.close();
}

void Config::print()
{
	if( internalData.camSettingsExists )
	{
		std::cout << "Config: printing camera parameters\n";
		std::cout.flush();

		switch(internalData.cameraType)
		{

		case UAV_ETH:
		{
			std::cout << "exposure " << camSettings->getCameraSetting(UAV_EXPOSURE) << std::endl;
			std::cout << "gain " << camSettings->getCameraSetting(UAV_GAIN) << std::endl;
			std::cout << "pixelClock " << camSettings->getCameraSetting(UAV_PIXELCLOCK) << std::endl;
			std::cout << "wbb " << camSettings->getCameraSetting(UAV_WBB)<< std::endl;
			std::cout << "wbr " << camSettings->getCameraSetting(UAV_WBR)<< std::endl;
			std::cout << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
			std::cout << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
			std::cout << "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
			std::cout << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
			std::cout << "centerCol" << camSettings->getCameraSetting(UAV_CENTERCOL) << std::endl;
			std::cout << "centerRow" << camSettings->getCameraSetting(UAV_CENTERROW) << std::endl;
			std::cout << "inRadius" << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
			std::cout << "outRadius" << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
			std::cout << "packetSize " << camSettings->getCameraSetting(UAV_PACKETSIZE)<< std::endl;
			std::cout << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
			std::cout << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;

			switch( camSettings->getCameraSetting(UAV_VIDMODE) )
			{
			case UAV_YUV411:
				std::cout << "mode YUV411" << std::endl;
				break;
			case UAV_YUV422:
				std::cout << "mode YUV422" << std::endl;
				break;
			case UAV_RGB8:
				std::cout << "mode RGB8" << std::endl;
				break;
			}
			std::cout << "colorCoding "<< camSettings->getCameraSetting(UAV_COLORCODING) << std::endl;
			std::cout << "format7 "<< camSettings->getCameraSetting(UAV_FORMAT7) << std::endl;
			std::cout << "camUsed "<< camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
			std::cout << "camRunning "<< camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;
			std::cout << "nColors " << UAV_NCOLORS << std::endl;



			std::cout << "... camera settings done." << std::endl;
		}
		break;

		case UAV_FIREWIRE:
		{

			std::cout << "powerState " << camSettings->getCameraSetting(UAV_POWERSTATE) << std::endl;
			std::cout << "speed " << camSettings->getCameraSetting(UAV_SPEED) << std::endl;
			std::cout << "opMode " << camSettings->getCameraSetting(UAV_OPMODE) << std::endl;
			std::cout << "colorCoding " << camSettings->getCameraSetting(UAV_COLORCODING) << std::endl;
			std::cout << "f7fps " << camSettings->getCameraSetting(UAV_F7FPS) << std::endl;
			std::cout << "exposure " << camSettings->getCameraSetting(UAV_EXPOSURE) << std::endl;
			std::cout << "brightness " << camSettings->getCameraSetting(UAV_BRIGHTNESS) << std::endl;
			std::cout << "sharpness " << camSettings->getCameraSetting(UAV_SHARPNESS) << std::endl;
			std::cout << "shutter " << camSettings->getCameraSetting(UAV_SHUTTER) << std::endl;
			std::cout << "saturation " << camSettings->getCameraSetting(UAV_SATURATION) << std::endl;
			std::cout << "gamma " << camSettings->getCameraSetting(UAV_GAMMA) << std::endl;
			std::cout << "gain " << camSettings->getCameraSetting(UAV_GAIN) << std::endl;
			std::cout << "wbb " << camSettings->getCameraSetting(UAV_WBB)<< std::endl;
			std::cout << "wbr " << camSettings->getCameraSetting(UAV_WBR)<< std::endl;
			std::cout << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
			std::cout << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
			std::cout << "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
			std::cout << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
			std::cout << "centerCol" << camSettings->getCameraSetting(UAV_CENTERCOL) << std::endl;
			std::cout << "centerRow" << camSettings->getCameraSetting(UAV_CENTERROW) << std::endl;
			std::cout << "inRadius" << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
			std::cout << "outRadius" << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
			std::cout << "packetSize " << camSettings->getCameraSetting(UAV_PACKETSIZE)<< std::endl;
			std::cout << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
			std::cout << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;

			switch( camSettings->getCameraSetting(UAV_VIDMODE) )
			{
			case UAV_YUV411:
				std::cout << "mode YUV411" << std::endl;
				break;
			case UAV_YUV422:
				std::cout << "mode YUV422" << std::endl;
				break;
			case UAV_RGB8:
				std::cout << "mode RGB8" << std::endl;
				break;
			}

			std::cout << "colorCoding "<< camSettings->getCameraSetting(UAV_COLORCODING) << std::endl;
			std::cout << "format7 "<< camSettings->getCameraSetting(UAV_FORMAT7) << std::endl;
			std::cout << "camUsed "<< camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
			std::cout << "camRunning "<< camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;
			std::cout << "nColors " << UAV_NCOLORS << std::endl;


			std::cout << "... camera settings done." << std::endl;
		}

		break;

		case UAV_OPENCV:
		case UAV_KINECT:
			std::cout << "nCols " << camSettings->getCameraSetting(UAV_NCOLS) << std::endl;
			std::cout << "nRows " << camSettings->getCameraSetting(UAV_NROWS) << std::endl;
			std::cout<< "ccdC " << camSettings->getCameraSetting(UAV_CCDCOL) << std::endl;
			std::cout << "ccdR " << camSettings->getCameraSetting(UAV_CCDROW) << std::endl;
			std::cout << "centerCol" << camSettings->getCameraSetting(UAV_CENTERCOL) << std::endl;
			std::cout << "centerRow" << camSettings->getCameraSetting(UAV_CENTERROW) <<std::endl;
			std::cout << "inRadius" << camSettings->getCameraSetting(UAV_INRADIUS) << std::endl;
			std::cout << "outRadius" << camSettings->getCameraSetting(UAV_OUTRADIUS) << std::endl;
			std::cout << "fps " << camSettings->getCameraSetting(UAV_FPS) << std::endl;
			std::cout << "nChannels " << camSettings->getCameraSetting(UAV_NCHANNELS) << std::endl;

			switch( camSettings->getCameraSetting(UAV_VIDMODE) )
			{
			case UAV_YUV411:
				std::cout << "mode YUV411" << std::endl;
				break;
			case UAV_YUV422:
				std::cout << "mode YUV422" << std::endl;
				break;
			case UAV_RGB8:
				std::cout << "mode RGB8" << std::endl;
				break;
			}

			std::cout << "camUsed "<< camSettings->getCameraSetting(UAV_CAMERAUSED) << std::endl;
			std::cout << "camRunning "<< camSettings->getCameraSetting(UAV_CAMERARUNNING) << std::endl;
			std::cout << "nColors " << UAV_NCOLORS << std::endl;
			std::cout << "format7 "<< camSettings->getCameraSetting(UAV_FORMAT7) << std::endl;

			std::cout << "...camera settings done." << std::endl;

			break;

			default:
				std::cout<<"Default!"<<std::endl;
				break;

		}
	}

	if( internalData.maskExists)
	{
		std::cout << "Config: printing mask... ";
		std::cout << "just kidding... no mask will be printed out in ASCII art";
		std::cout << "... done." << std::endl;
	}

	if( internalData.mapExists )
	{
		std::cout << "Config: printing map... ";
		std::cout << "just kidding... no map will be printed out in ASCII art";
		std::cout << "... done." << std::endl;
	}

	if( internalData.colorRangesExists )
	{
		std::cout << "Config: printing color range settings\n";
		std::cout.flush();

		for(int c = 0 ; c < UAV_NCOLORS ; c++)
		{
			std::cout << colorRange[c].name << std::endl;
			std::cout << "PaintColor " << colorRange[c].paintColor << std::endl;
			std::cout << "HMax " << colorRange[c].hMax << std::endl;
			std::cout << "HMin " << colorRange[c].hMin << std::endl;
			std::cout << "SMax " << colorRange[c].sMax << std::endl;
			std::cout << "SMin " << colorRange[c].sMin << std::endl;
			std::cout << "VMax " << colorRange[c].vMax << std::endl;
			std::cout << "VMin " << colorRange[c].vMin << std::endl;
		}

		std::cout << "...color ranges done." << std::endl;
	}
}

void Config::addColorRange(ColorRange *nColorRange)
{

	internalData.colorRangesExists = true;

	for(unsigned c = 0; c < UAV_NCOLORS; c++)
	{
		strcpy(nColorRange[c].name,colorRange[c].name);
		colorRange[c].hMax = nColorRange[c].hMax;
		colorRange[c].hMin = nColorRange[c].hMin;
		colorRange[c].sMax = nColorRange[c].sMax;
		colorRange[c].sMin = nColorRange[c].sMin;
		colorRange[c].vMax = nColorRange[c].vMax;
		colorRange[c].vMin = nColorRange[c].vMin;
	}


}

void Config::addMap(std::vector<cv::Point2d> &newMap)
{

	internalData.mapExists = true;
	map.clear();


	for(unsigned int i = 0; i < camSettings->getCameraSetting(UAV_NROWS) * camSettings->getCameraSetting(UAV_NCOLS); i++)
	{
		cv::Point2d aux;
		aux = newMap.at(i);
		map.push_back(aux);
	}



}

void Config::addMask(cv::Mat &newMask)
{

	internalData.maskExists= true;

	newMask.copyTo(mask);

	std::cerr << "New mask added!\n";
}

void Config::deleteColorRange()
{

	internalData.colorRangesExists = false;

}

void Config::deleteMask()
{

	internalData.maskExists= false;

}

void Config::deleteMap()
{

	internalData.mapExists = false;

}

int Config::loadCSfromFile(const std::string fileName_, CameraSettings &camSettings)
{

	if( fileName_.size() == 0 )
	{
		std::cout << "Config: ERROR invalid config file name." << std::endl;
		return ERROR_NOFILENAME;
	}

	file.open(fileName_.c_str(), std::fstream::in | std::fstream::binary);
	if( !file.is_open() )
	{
		std::cout << "Config: ERROR opening " << fileName_ << " configuration file" << std::endl;
		return ERROR_CANNOTOPENFILE;
	}


	file.read(reinterpret_cast<char *>(&camSettings), sizeof(CameraSettingsOpenCV));

	return ERROR_NOERROR;
}
}
