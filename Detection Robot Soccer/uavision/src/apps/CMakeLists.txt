#ADD_EXECUTABLE ( kinectVision kinectVision.cpp)
#TARGET_LINK_LIBRARIES( kinectVision
#	uavision
#	commTCP
#	pthread
#	${OpenCV_LIBS}
#	${VTK_LIBRARIES}
#)

ADD_EXECUTABLE ( frontVision frontVision.cpp)
TARGET_LINK_LIBRARIES( frontVision
	uavision
	commTCP
	pthread
	${OpenCV_LIBS}
)

ADD_EXECUTABLE ( omniVision omniVision.cpp)
TARGET_LINK_LIBRARIES( omniVision
	uavision
	commTCP
	pthread
	${OpenCV_LIBS}
)

ADD_EXECUTABLE ( ocalibVision ocalibVision.cpp )
TARGET_LINK_LIBRARIES( ocalibVision
	uavision
	commTCP
	pthread
	${OpenCV_LIBS}
)

#ADD_EXECUTABLE ( kcalibVision kcalibVision.cpp )
#TARGET_LINK_LIBRARIES( kcalibVision
#	uavision
#	commTCP
#	pthread
#	${OpenCV_LIBS}
#)

ADD_EXECUTABLE ( fcalibVision fcalibVision.cpp )
TARGET_LINK_LIBRARIES( fcalibVision
	uavision
	commTCP
	pthread
	${OpenCV_LIBS}
)

ADD_EXECUTABLE ( watershed watershed.cpp )
TARGET_LINK_LIBRARIES( watershed
	${OpenCV_LIBS}
)
