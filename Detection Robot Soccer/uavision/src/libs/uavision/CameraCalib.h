#ifndef CAMERACALIB_H_
#define CAMERACALIB_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include "CameraSettings.h"

namespace uav{

#define MSVTHRESHOLD 0.1
#define UVTHRESHOLD 5

class CameraCalib
{

public:

	int hist_size;
	cv::Mat image;
	std::vector <int> histValue;

	CameraCalib(int size, cv::Mat &imagePassed);

	~CameraCalib(){}

	void calcHistograms(cv::Mat &mask);

	float calcMean(cv::Mat &hist);

	float calcMSV(std::vector<int> &hist);

	float calcACM(cv::Mat &hist);

	cv::Scalar calcUV(cv::Rect &rect);

	cv::Scalar calcRGB(cv::Rect &rect);

	bool CalibrateCamera(CameraSettings *camSettings, ParameterRange *parameterRange,
			cv::Mat &mask, cv::Rect &rWhite, bool firstTimeCalib, bool verbose,
			std::vector<bool> &paramChanged, bool wbCalib = true);

};
}
#endif
