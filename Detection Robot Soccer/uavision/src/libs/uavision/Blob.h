#ifndef BLOB_H_
#define BLOB_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "UAVision.h"
#include "ScanLines.h"
#include "RLE.h"

namespace uav{
/** Structure for the definition of the information associated to a blob. */

class BlobInfo
{
public:
	int center, upperLeft, downRight;
	double area;
	double widhtHeightRel;
	int nRle;
	int dist;
	int coloredPixels;
	double solidity;

	BlobInfo(RLE rle, int idx)
	{
		center = rle.rlData[idx].center;
		upperLeft = rle.rlData[idx].start;
		downRight = rle.rlData[idx].end;
		nRle = 1;
		area = 1.0 * rle.rlData[idx].lengthColor * rle.s.getStep();
		widhtHeightRel = area;
		dist = 0;
		coloredPixels = rle.rlData[idx].lengthColor;
		solidity = 1.0;
	}
};

class Blob
{

public:

	std::vector<BlobInfo> blobs;
	//RLE rle;

	Blob();

	inline cv::Point getPointXYFromInteger(int idx, RLE &rle)
	{
		cv::Point temp;

		temp.x = idx%rle.s.image.cols;
		temp.y = idx/rle.s.image.cols;

		return temp;
	}

	inline int getIntFromXY(cv::Point pt, RLE &rle)
	{
		int temp;

		temp = pt.y * rle.s.image.cols + pt.x;

		return temp;
	}

	void createBlobs(RLE &rle_, std::vector<float> &threshold, cv::Point &robotCenter);
	void updateBlob(BlobInfo &b, RLE &rle, int idx, cv::Point &robotCenter);
	int findBlob(RLE &rle_, std::vector<float> &threshold, int idx);
	double distance(cv::Point p1, cv::Point p2);
	void sort(unsigned sortMethod, RLE &rle);
	void draw(cv::Scalar color, cv::Mat &destination);

};
}
#endif
