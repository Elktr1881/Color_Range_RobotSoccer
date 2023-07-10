/* Class used for the implementation of scan lines. */

#ifndef SCANLINES_H_
#define SCANLINES_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include <math.h>

namespace uav{
class ScanLines
{

public:

	cv::Point start; /** Starting point of the scan line. */
	cv::Point end; /** Ending point of the scan line. */
	unsigned stepX; /** Horizontal lines sparsity */
	unsigned stepY; /** Vertical lines sparsity */
	unsigned inRadius; /** Inner radius for radial scan lines.*/
	unsigned outRadius; /** Outer radius for radial scan lines.*/
	unsigned type; /** Type of the scanline (Radial, Circular, Horizontal or Vertical */
	cv::Point center; /** The center of the radial and circular scanlines */
	cv::Mat image; /** Reference image used to construct the Scanlines */
	std::vector <std::vector <int> > scanlines; /** Data container */

	ScanLines();

	/**
	 * Constructor for the creation of horizontal/vertical scan lines.
	 */
	ScanLines(cv::Mat &image_, unsigned scanType, cv::Point start_, cv::Point end_, unsigned step_x, unsigned step_y);

	/**
	 * Constructor for the creation of radial and circular scan lines.
	 */
	ScanLines (cv::Mat &image_, unsigned scanType, cv::Point center, unsigned nSensors,
			unsigned inRad, unsigned outRad, int radiusStep = 1);

	ScanLines (cv::Mat &image_, unsigned scanType, cv::Point center, unsigned nSensors,
				unsigned inRad, unsigned outRad, int nLevels, int step);

	ScanLines (cv::Mat &image_, unsigned scanType, cv::Point center, unsigned nSensors,
			unsigned inRad, unsigned outRad, int drajat1, int drajat2, int a);

	/**
	 * Destructor.
	 */
	~ScanLines(){}

	int signum(int d);

	inline std::vector<int>& operator[](int idx) {return scanlines[idx];}

	inline const std::vector<int>& operator[](int idx) const {return scanlines[idx];}

	inline std::vector<int> getLine(int idx) {return scanlines[idx];}

	inline int getPoint(int i, int j) {return scanlines[i][j];}

	inline cv::Point getPointXYFromInteger(int idx)
	{
		cv::Point temp;

		temp.x = idx%image.cols;
		temp.y = idx/image.cols;

		return temp;
	}

	inline cv::Point getPointXY(int i, int j)
	{
		cv::Point temp;
		temp.x = scanlines[i][j]%image.cols;
		temp.y = scanlines[i][j]/image.cols;

		return temp;
	}

	unsigned getStep();

	void draw(cv::Mat &destination, cv::Scalar color);
};
}
#endif
