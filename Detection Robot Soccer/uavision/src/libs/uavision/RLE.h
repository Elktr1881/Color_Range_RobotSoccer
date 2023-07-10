#ifndef RLE_H_
#define RLE_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include "ScanLines.h"

namespace uav{
class RLE
{

public:

	std::vector<RLEInfo> rlData;
	ScanLines s;

	RLE(ScanLines &s_, unsigned colorBefore, unsigned colorOfInterest, unsigned colorAfter,
			unsigned threshColorBefore, unsigned threshColorOfInterest, unsigned threshColorAfter,
			unsigned searchWindow,  bool obstaclesCAMBADA = false);

	inline cv::Point getPointXYFromInteger(int idx)
	{
		cv::Point temp;
		temp.x = idx%s.image.cols;
		temp.y = idx/s.image.cols;

		return temp;
	}
	void draw(cv::Scalar colorBefore, cv::Scalar colorOfInterest, cv::Scalar colorAfter, cv::Mat &destination);
};
}
#endif
