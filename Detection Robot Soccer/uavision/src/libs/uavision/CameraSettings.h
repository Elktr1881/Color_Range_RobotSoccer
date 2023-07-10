#ifndef CAMERASETTINGS_H_
#define CAMERASETTINGS_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"

/**
 * Abstract class working as an interface for the definition
 * of different type of camera settings, based on the camera type that
 * is in use.
 */
namespace uav{


class CameraSettings
{
public:

	/**
	 * Virtual destructor for proper clean-up.
	 */
	virtual ~CameraSettings() {}

	/**
	 * Pure virtual method for printing the camera settings.
	 */
	virtual void printCameraSettings() = 0;

	/**
	 * Pure virtual method for retrieving just one camera setting.
	 * @param setting - camera setting to be retrieved.
	 */
	virtual int getCameraSetting(unsigned setting) = 0;

	/**
	 * Pure virtual method for setting the value of one camera setting.
	 * @param setting - camera setting to be set.
	 * @param value - value of the camera setting to be set.
	 */
	virtual void setCameraSetting(unsigned setting, unsigned value) = 0;

	/**
	 * Pure virtual method for retrieving the size of an object of the type ReCamSettings
	 */
	virtual int getSize() = 0;
	/**
	 * Pure virtual method for retrieving the raw buffer containing the parameters
	 */
	virtual unsigned int* ptr() = 0;
};
}
#endif
