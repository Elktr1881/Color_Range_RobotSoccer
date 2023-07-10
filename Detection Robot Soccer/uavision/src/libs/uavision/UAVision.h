/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                           *
 *  FILE: UAVision.h                                                            *
 *                                                                           *
 *  Copyright 2006-2013 CAMBADA Team, All Rights served                     *
 *  DET/IEETA, University of Aveiro                                          *
 *  http://www.ieeta.pt/atri/cambada                                         *
 *                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef UAVISION_H
#define UAVISION_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>


namespace uav{
/**
 * Number of colors used in the color classification process.
 */
#define UAV_NCOLORS 9
#define COLOR_NAME_LENGTH 64
#define THRESH_COLOR 2
#define THRESH_GREEN_BEFORE 2
#define THRESH_GREEN_AFTER 2
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#define FIREWIRE_DROP_FRAMES 1
#define FIREWIRE_MAX_CAMERAS 2
#define FIREWIRE_NUM_BUFFERS 2


#define mymin(a, b) ((a)<(b)?(a):(b))
#define min3(a, b, c) mymin(mymin(a, b), c)
#define mymax(a, b) ((a)>(b)?(a):(b))
#define max3(a, b, c) mymax(mymax(a, b), c)

/**
 * Enumeration containing the gain variables used for a Ethernet Camera.
 */
enum GAINETH {GAIN_MASTER = 0, GAIN_RED, GAIN_GREEN, GAIN_BLUE};

/**
 * Enumeration containing all the possible parameters that can be included
 * in an object of a type CameraSettings.
 */
enum UAV_PARAMETERS {UAV_FPS=0, UAV_PIXELCLOCK, UAV_EXPOSURE, UAV_GAIN, UAV_WBB, UAV_WBR, UAV_WB,
UAV_BRIGHTNESS, UAV_SHARPNESS, UAV_SHUTTER, UAV_NCOLS,UAV_NROWS, UAV_CCDCOL, UAV_CCDROW,UAV_CENTERCOL, UAV_CENTERROW,
UAV_INRADIUS, UAV_OUTRADIUS, UAV_PACKETSIZE, UAV_NCHANNELS,UAV_VIDMODE, UAV_COLORCODING,
UAV_FORMAT7, UAV_CAMERAUSED, UAV_CAMERARUNNING, UAV_GAMMA, UAV_SATURATION, UAV_F7FPS, UAV_SPEED,
UAV_OPMODE, UAV_AUTOBRIGHTNESS, UAV_AUTOGAIN, UAV_AUTOWB, UAV_AUTOSHARPNESS,
UAV_AUTOSHUTTER, UAV_AUTOGAMMA, UAV_AUTOSATURATION, UAV_AUTOEXPOSURE, UAV_MODEL,
UAV_POWERSTATE, UAV_COLORCONTROL};

enum UAV_COLORCORRECTION_MODES { UAV_COLORCORRECTION_DISABLED = 0, UAV_COLORCORRECTION_NORMAL, UAV_COLORCORRECTION_HQ};

/**
 * Enumeration containing the possible types of scan lines.
 */

enum UAV_SCANLINES {UAV_HORIZONTAL = 0, UAV_VERTICAL, UAV_RADIAL, UAV_CIRCULAR};


/**
 * Enumeration containing the video modes in which a frame can be acquired.
 */
enum UAV_VIDEOMODE { UAV_GRAY = 0, UAV_RGB8, UAV_YUV411, UAV_YUV422};

/**
 * Enumeration of the possible camera types to be used.
 */
enum UAV_CAMTYPE {UAV_ETH = 0, UAV_OPENCV, UAV_FIREWIRE, UAV_I2C, UAV_KINECT, UAV_ZEBRA};

/**
 * Enumeration of colors used in the color classification process.
 */
enum UAV_COLORS {UAV_BLUE=0, UAV_YELLOW, UAV_ORANGE, UAV_GREEN, UAV_WHITE, UAV_BLACK, UAV_CYAN, UAV_MAGENTA, UAV_NOCOLORS };
enum UAV_COLORS_BIT {UAV_ORANGE_BIT = 32, UAV_BLACK_BIT = 4, UAV_GREEN_BIT = 16,
	UAV_WHITE_BIT = 8, UAV_BLUE_BIT = 128, UAV_YELLOW_BIT = 64, UAV_CYAN_BIT = 2,
	UAV_MAGENTA_BIT = 1, UAV_NOCOLORS_BIT = 0};

enum UAV_SORT {UAV_SORT_BY_SIZE = 0, UAV_SORT_BY_DISTANCE};

/**
 * Enumeration of error types.
 */

enum CONFIG_ERROR {
	ERROR_NOERROR = 0,
	ERROR_NOFILENAME = -1,
	ERROR_CANNOTOPENFILE = -2,
	ERROR_WRONGVERSION = -3,
	ERROR_EMPTYCONFIG = -4,
	ERROR_INCOMPATIBLE = -5
	};

/** Structure for the definition of a range for any parameter of the camera. */

struct ParameterRange
{
	unsigned max;
	unsigned min;
	ParameterRange()
	{
		max = 1000;
		min = 0;
	}
};

/** Structure for the definition of the run-length encoding information. */

struct RLEInfo
{
	int center, start, end, endAfter, startBefore;
	unsigned scIdx;
	unsigned lengthColorBefore;
	unsigned lengthColor;
	unsigned lengthColorAfter;

};

/**
 * Structure for the definition of the color range information.
 */

struct ColorRange
{
	unsigned int hMax;
	unsigned int hMin;
	unsigned int sMax;
	unsigned int sMin;
	unsigned int vMax;
	unsigned int vMin;
	unsigned int paintColor;
	char name[64];

	ColorRange()
	{
		hMax = 0;
		hMin = 0;
		sMax = 0;
		sMin = 0;
		vMax = 255;
		vMin = 255;
		paintColor = 0;
		name[0] = 'a';
		name[1] = '\0';
	}

	int getSize() {

		int result = sizeof(hMax) +
					 sizeof(hMin) +
					 sizeof(sMin) +
					 sizeof(sMax) +
					 sizeof(vMin) +
					 sizeof(vMax) +
					 sizeof(paintColor)+
					 sizeof(name);

		return result;
	}
};


struct ColorCoords
{
	int x1;
	int x2;
	int y1;
	int y2;
};

struct ColorRGB
{
	double RGB[3];
};

struct ColorYUV
{
	float YUV[3];
};

struct Statistics
{
	double RGB_AvColorList [3];	//average RGB
	double RGB_StdColorList[3];	//standard deviation RGB
	double RGB_MaxColorList[3];	//max RGB
	double RGB_MinColorList[3];	//min RGB

	unsigned HSV_AvColorList [3];	//average RGB
	unsigned HSV_StdColorList[3];	//standard deviation RGB
	unsigned HSV_MaxColorList[3];	//max RGB
	unsigned HSV_MinColorList[3];	//min RGB

	double YUV_AvColorList [3];	//average RGB
	double YUV_StdColorList[3];	//standard deviation RGB
	double YUV_MaxColorList[3];	//max RGB
	double YUV_MinColorList[3];	//min RGB
};

struct ImageStatistics
{
	double acm;
	double average;
	double entropy;
	double msvValue;
	double msvSatValue;
};

void Rgb2Hsv2( float r, float g, float b, float *h, float *s, float *v);
void Rgb2Hsv( int r, int g, int b, int *hout, int *sout, int *vout );
void Rgb2Yuv(int r, int g, int b, int *y, int *u, int *v);
void Yuv2Rgb(int y, int u, int v, int *r, int *g, int *b);
void Yuv2Rgb2(float y, float u, float v, float *r, float *g, float *b);

void drawLine(int p1, int p2, cv::Scalar color, cv::Mat &img);
void drawCircle(int center, int radius, cv::Scalar color, cv::Mat &img);

void distRelation(std::vector<float> &data, cv::Mat &image, const std::string fileName, bool full);
void distRelation(std::vector<float> &data, cv::Mat &image, bool full);
void distRelationGoalie (std::vector<float> &data, cv::Mat &image, int value=50);

void convertResolution(cv::Mat &original, cv::Mat &converted, int resolutionFactor);
void convertResolutionGray(cv::Mat &original, cv::Mat &converted, int resolutionFactor);

std::vector<cv::Point> regionGrowingSegmentation(cv::Mat &image, int threshold, int colorOfInterest, int colorOfVisited, std::vector<double> &areaList, std::vector<double> &widthHeightRelation, std::vector<double> &solidity);

std::vector<cv::Point2f> calculateContours(cv::Mat &image, cv::Mat &canny, cv::Mat &idxImage, int colorOfInterst, int threshold, std::vector<double> &areaList, std::vector<double> & widthHeightRelation, std::vector<double> &solidity);

cv::Point3d calcWorldCoords(cv::Point point, cv::Mat &rotationVecs, cv::Mat &transVecs, cv::Mat &intrinsics, cv::Mat &distCoeffs);
}
#endif

