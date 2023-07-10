#ifndef CAMERASETTINGSFACTORY_H_
#define CAMERASETTINGSFACTORY_H_

#include "CameraSettings.h"
#include "UAVision.h"
#include "Camera.h"

namespace uav{

/**
 * Class used as a factory for creating objects of the type
 * CameraSettings based on the camera type.
 */

class CameraSettingsFactory {
    public:
	/**
	 * Method for creating objects of the type CameraSettings
	 * based on the camera type.
	 */
      CameraSettings* Create(int camType);
};
}
#endif
