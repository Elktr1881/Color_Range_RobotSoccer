/**
 * Class for implementing a look-up table.
 * Authors: alt & an
 */

#ifndef LUT_H_
#define LUT_H_

#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem/operations.hpp>
#include <opencv2/opencv.hpp>
#include "Config.h"
#include "ScanLines.h"

namespace uav{

#define LUT_MAX_Y		256
#define LUT_MAX_U		256
#define LUT_MAX_V		256
#define LUT_MEMORY_SIZE	( LUT_MAX_Y * LUT_MAX_U * LUT_MAX_V )

class Lut
{
	public:
		unsigned char *info;
		std::vector<int> validPixels;

		Lut();
		Lut(Config& config);
		Lut(Config& config, ScanLines &sc);

		~Lut();

		unsigned char &operator[](const unsigned index)
		{
			return info[index];
		}

		unsigned char &operator[](const unsigned index) const
		{
			return info[index];
		}

		void createValidPixels(cv::Mat &mask, int mode);
		void createValidPixels(ScanLines &sc, cv::Mat &mask, int mode);
		void addValidPixels(ScanLines &sc, cv::Mat &mask, int mode);
		void createLUT(int mode, ColorRange* cr);
		void convertImageToIndex(cv::Mat &original, cv::Mat &index, int imageMode);
		void init(ColorRange* cr);
		void saveToCache(std::string cacheFilename);
		void loadFromCache(std::string cacheFilename);
};
}
#endif
