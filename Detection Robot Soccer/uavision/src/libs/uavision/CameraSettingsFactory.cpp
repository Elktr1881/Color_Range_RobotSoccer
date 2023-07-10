#include "CameraSettingsFactory.h"
#include "CameraSettings.h"
#include "CameraSettingsEth.h"
#include "CameraSettingsOpenCV.h"
#include "CameraSettingsFirewire.h"
#include "CameraSettingsKinect.h"
#include "CameraSettingsZebra.h"

namespace uav{

using namespace std;

CameraSettings* CameraSettingsFactory::Create(int camType) {
	if ( camType == UAV_ETH ) return new CameraSettingsEth();
	if ( camType == UAV_OPENCV ) return new CameraSettingsOpenCV();
	if ( camType == UAV_FIREWIRE) return new CameraSettingsFirewire();
	if ( camType == UAV_KINECT) return new CameraSettingsKinect();
    if ( camType == UAV_ZEBRA) return new CameraSettingsZebra();

	return NULL;

}
}
