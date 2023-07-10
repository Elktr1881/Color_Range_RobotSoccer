# Create UAVision

ADD_LIBRARY ( uavision
	Angle.cpp
	Vec.cpp
	CameraSettingsFactory.cpp
	CameraSettingsZebra.cpp
	CameraSettingsOpenCV.cpp
	CameraSettingsEth.cpp
	CameraSettingsKinect.cpp
	CameraSettingsFirewire.cpp
	CameraOpenCV.cpp
	CameraKinect.cpp
	CameraFromFile.cpp
	Config.cpp
	Lut.cpp
	ScanLines.cpp
	RLE.cpp
	Blob.cpp
	UAVision.cpp
	libCalib.cpp	
	CameraCalib.cpp
	PID_vision.cpp
)

TARGET_LINK_LIBRARIES( uavision
  boost_filesystem
  boost_system
	freenect
)

set_target_properties( uavision PROPERTIES COMPILE_FLAGS "-fPIC" )
