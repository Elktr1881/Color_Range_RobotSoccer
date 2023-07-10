/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                           *
 *  FILE: Config.h                                                         *
 *                                                                           *
 *  Copyright 2008-2013 CAMBADA Team, All Rights Reserved                    *
 *  DETI/IEETA, University of Aveiro                                         *
 *  http://www.ieeta.pt/atri/cambada                                         *
 *                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "Camera.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>
#include "UAVision.h"
#include "CameraSettings.h"
#include "CameraSettingsFactory.h"


namespace uav{

/** Class that allows the creation and manipulation of configuration files
 * for the vision system.
 */

struct ConfigInternals
{
	unsigned int version;
    unsigned int cameraType;
    unsigned int mapExists;
    unsigned int maskExists;
    unsigned int camSettingsExists;
    unsigned int colorRangesExists;
    ConfigInternals()
    {
    	version = 201403;
    	cameraType = 0;
    	mapExists = 0;
    	maskExists = 0;
    	camSettingsExists = 0;
    	colorRangesExists = 0;
    }
};

class Config {

private:


	/** Stream for manipulation of files.
	 *
	 */
	std::fstream file;

protected:

public:
	ConfigInternals internalData;

	/**
	 * Constructor - initializes all the members of the Config class with dummy values.
	 * @param fileName_ - name of the configuration file
	 */
	Config( const std::string fileName_ = std::string() );

	/**
	 * Destructor - deallocation of memory.
	 */
	~Config();

	/**
	 * Name of the file.
	 */
	std::string fileName;

	/**
	 * Camera Settings Factory used for the instantiation of a proper CameraSettings object,
	 * depending on the type of camera in use.
	 */
	CameraSettingsFactory camSettingsFactory;

	/**
	 * Camera Settings
	 */
	CameraSettings* camSettings;

	/**
	 * Mask image.
	 */
	cv::Mat mask;

	/**
	 * Map points.
	 */
	std::vector<cv::Point2d> map;

	/**
	 * Color ranges container.
	 */
	ColorRange *colorRange;

	/**
	 * Memory allocation for the members of a config file, based on the
	 * camera type and the existence flags.
	 * @param camType - type of camera in use. Relevant for the instantiation of a proper ReCamSettings object.
	 * @param mapFlag - if set to true, memory will be allocated for the use of a map structure.
	 * @param maskFlag - if set to true, memory will be allocated for the use of a mask structure.
	 * @param camSettingsFlag - if set to true, memory will be allocated for the use of a CameraSettings object.
	 * @param colorRangesFlag - if set to true, memory will be allocated for the use of a Color Range structure.
	 */
	void initializeConfig(unsigned int camType, bool mapFlag, bool maskFlag, bool camSettingsFlag, bool colorRangesFlag);
	void initializeConfigWithoutCamSettings(unsigned int camType, bool mapFlag, bool maskFlag, bool colorRangesFlag);

	/**
	 * Creation of a configuration file based on flag information.
	 * @param name - name of the configuration file.
	 * @param camType - type of camera in use.
	 * @mapFlag - if set to true, the configuration file will information about a map.
	 * @maskFlag - if set to true, the configuration file will hold information about a mask.
	 * @camSettingsFlag - if set to true, the configuration file will hold information about camera settings.
	 * @colorRangesFlag - if set to true, the configuration file will hold information about the color ranges.
	 * @binary - if set to true, the configuration file will contain binary information; if set to false, the
	 * configuration file will be writen in ASCII.
	 */
	void createConfig(const std::string name, unsigned int camType, bool mapFlag, bool maskFlag, bool camSettingsFlag, bool colorRangesFlag, bool binary, CameraSettings *camSettings = NULL );

	void resizeConfig(int resizeFactor);

	/**
	 * Loading/Reading the information from an existent binary configuration file.
	 * @param fileName_ - name of the configuration file.
	 */
	int load( const std::string fileName_ = std::string() );

	/*
	 * Saving/writing information on a binary configuration file.
	 * @param fileName_ - name of the configuration file.
	 */
	int save( const std::string fileName_ = std::string() );


	/*
	 * Method that allows the addition of information about color ranges to an existing configuration file.
	 * @param *newCr - color range information to be added.
	 */
	void addColorRange(ColorRange *newCr);

	/*
	 * Method that allows the addition of information about mask to an existing configuration file.
	 * @param newMask - mask information to be added.
	 */
	void addMask(cv::Mat &newMask);

	/*
	 * Method that allows the addition of information about map to an existing configuration file.
	 * @newMap - map information to be added.
	 */
	void addMap(std::vector<cv::Point2d> &newMap);

	/*
	 * Method that allows the removal of color range information from an existing configuration file.
	 */
	void deleteColorRange();

	/*
	 * Method that allows the removal of mask information from an existing configuration file.
	 */
	void deleteMask();

	/*
	 * Method that allows the removal of map information from an existing configuration file.
	 */
	void deleteMap();

	/**
	 * Loading/Reading the information from an existent text configuration file.
	 * @param fileName_ - name of the configuration file.
	 */
	void loadAscii( const std::string fileName_ = std::string());

	/*
	 * Saving/Writing information to an existing text configuration file.
	 */
	void saveAscii( const std::string fileName_ = std::string());

	/*
	 * Method for printing the information about the members of a Config object.
	 */
	void print();

	/*
	 * Method for checking if the image was resized. When the image is resized,
	 * the map, mask and camSettings info may change.
	 */
	void imageSizeChanged();

	/*
	 * Method for loading the camera settings information from a file.
	 * @param fileName_ - name of the configuration file
	 * @param &camSettings - container to be filled with the information about the camera settings.
	 */
	int loadCSfromFile(const std::string fileName_, CameraSettings &camSettings);

};
}
#endif /* RECONFIG_H_ */
