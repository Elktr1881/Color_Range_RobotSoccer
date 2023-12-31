CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( UAVision )

SET( V_MAJOR 1 )
SET( V_MINOR 0 )
SET( V_PATCH 0 )

# ---
MESSAGE (" UAVision library")

# http://www.cmake.org/cmake/help/cmake-2-8-docs.html#section_Policies
IF(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# COMPILER FLAGS
SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W" )
SET( CMAKE_C_FLAGS_RELEASE "-O3" )
SET( CMAKE_C_FLAGS_DEBUG "-g3" )

SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall" )
SET( CMAKE_CXX_FLAGS_RELEASE "-O3" )
SET( CMAKE_CXX_FLAGS_DEBUG "-g3" )

# Set the top directory of the source code (the first CMakeLists of the project, this one actually)
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( SRC_DIR ${BASE_DIR}/src )

# Set output directories for libraries and executables
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

LINK_DIRECTORIES( ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )

# Define where to find modules
SET( CMAKE_MODULE_PATH ${CMAKE_MODULES_PATH} ${BASE_DIR}/CMakeModules )

MESSAGE ("\n==> Finding 3rd Party Packages")
FIND_PACKAGE( OpenCV	REQUIRED )
#FIND_PACKAGE( VTK	REQUIRED )
#INCLUDE (${VTK_USE_FILE})

# Make include directories available for all..
INCLUDE_DIRECTORIES( 
	${OpenCV_INCLUDE_DIR}
)

# Add sources directory
ADD_SUBDIRECTORY( ${SRC_DIR} )

# ---
MESSAGE ("\n==> Building Options")
MESSAGE (STATUS "Building as [${CMAKE_BUILD_TYPE}]")
MESSAGE ("\n==> Building Instructions")
MESSAGE (STATUS "Compile UAVision project using: make")
MESSAGE (STATUS "Generate documentation using: make doc")
MESSAGE ("\n")

# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)



