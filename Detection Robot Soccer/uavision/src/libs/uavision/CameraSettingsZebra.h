#ifndef CAMERASETTINGSZEBRA_H
#define CAMERASETTINGSZEBRA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include "Camera.h"
#include "CameraSettings.h"

namespace uav{
class CameraSettingsZebra:public CameraSettings
{
    public:
        CameraSettingsZebra();
        virtual ~CameraSettingsZebra();

        unsigned int *buffer;

        /**
                     * Method for printing the camera settings.
                    */
        void printCameraSettings();

        /**
                     * Method for retrieving just one camera setting.
                     * @param setting - camera setting to be retrieved.
                     */
        int getCameraSetting(unsigned setting);

        /**
                     * Method for setting the value of one camera setting.
                     * @param setting - camera setting to be set.
                     * @param value - value of the camera setting to be set.
                     */
        void setCameraSetting(unsigned setting, unsigned value);

        /**
                     * Method for retrieving the size of an object of the type ReCamSettings.
                     */
        int getSize();

        unsigned int* ptr();
    protected:
    private:
};
}
#endif // CAMERASETTINGSZEBRA_H
