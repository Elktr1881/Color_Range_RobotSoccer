#include "Blob.h"

namespace uav{

Blob::Blob()
{
	blobs.clear();
}

void Blob::createBlobs(RLE &rle_, std::vector<float> &threshold, cv::Point &robotCenter)
{
	if(rle_.rlData.size() == 0)
		return;

	for(unsigned i = 0 ; i < rle_.rlData.size() ; i++)
	{
		int idx = findBlob(rle_, threshold, i);
		if(idx == -1) // there is no blob to add this line
		{
			BlobInfo b(rle_, i);
			blobs.push_back(b); // Add new blob to the list of blobs
		}
		else
		{
			updateBlob(blobs[idx], rle_, i, robotCenter); // otherwise, update an existing one
		}
	}
}

void Blob::updateBlob(BlobInfo &b, RLE &rle, int idx, cv::Point &robotCenter)
{

	cv::Point startPt, endPt, pt3, pt4;

	startPt = rle.getPointXYFromInteger(rle.rlData[idx].start);
	endPt = rle.getPointXYFromInteger(rle.rlData[idx].end);

	pt3 = rle.getPointXYFromInteger(b.upperLeft);
	// Bounding Box
	if(startPt.x < pt3.x)	pt3.x = startPt.x;
	if(endPt.x < pt3.x) 	pt3.x = endPt.x;
	if(startPt.y < pt3.y) 	pt3.y = startPt.y;
	if(endPt.y < pt3.y) 	pt3.y = endPt.y;
	b.upperLeft = getIntFromXY(pt3, rle);

	pt4 = rle.getPointXYFromInteger(b.downRight);
	if(endPt.x > pt4.x) 	pt4.x = endPt.x;
	if(startPt.x > pt4.x)	pt4.x = startPt.x;
	if(endPt.y > pt4.y) 	pt4.y = endPt.y;
	if(startPt.y > pt4.y)	pt4.y = startPt.y;
	b.downRight = getIntFromXY(pt4, rle);

	// Update Center of Mass
	cv::Point center;
	center.x = (pt4.x + pt3.x) / 2;
	center.y = (pt4.y + pt3.y) / 2;
	b.center = getIntFromXY(center, rle);
	b.nRle++;

	if((pt4.x - pt3.x) == 0)
	{
		b.area = pt4.y - pt3.y;
		b.widhtHeightRel = (float)(1.0)/(pt4.y - pt3.y);
	}
	else if((pt4.y - pt3.y) == 0)
	{
		b.area = pt4.x - pt3.x;
		b.widhtHeightRel = (float)(pt4.x - pt3.x);
	}
	else
	{
		b.area = (pt4.x - pt3.x)*(pt4.y - pt3.y);
		b.widhtHeightRel = (float)(pt4.x - pt3.x)/(pt4.y - pt3.y);
	}

	b.coloredPixels += rle.rlData[idx].lengthColor;
	b.solidity = (double)b.coloredPixels / b.area;
	b.dist = (int)distance(robotCenter,getPointXYFromInteger(b.center, rle));
}

int Blob::findBlob(RLE &rle_, std::vector<float> &threshold, int idx)
{

	if(blobs.size() == 0)
		return -1;

	for(unsigned i = 0 ; i < blobs.size() ; i++)
	{
		cv::Point pt1, pt2;
		pt1 = rle_.getPointXYFromInteger(rle_.rlData[idx].center);
		pt2 = rle_.getPointXYFromInteger(blobs[i].center);
		double dist = distance(pt1, pt2);
		if( dist <= (int)threshold[blobs[i].dist] * 3)
		{
			return i;
		}
	}

	return -1;

}

double Blob::distance(cv::Point p1, cv::Point p2)
{
	double distance = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));

	return distance;
}

void Blob::sort(unsigned sortMethod, RLE &rle)
{
	if(blobs.size() <= 1) return;

	BlobInfo tmp(rle, 0);

	switch(sortMethod){
	case UAV_SORT_BY_DISTANCE:
		for(unsigned i = 0 ; i < blobs.size() - 1 ; i++)
		{
			for(unsigned j = i + 1 ; j < blobs.size() ; j++)
			{
				if (blobs[i].dist > blobs[j].dist)
				{
					tmp = blobs[j];
					blobs[j] = blobs[i];
					blobs[i] = tmp;
				}
			}
		}
		break;

	case UAV_SORT_BY_SIZE:

		for(unsigned i = 0 ; i < blobs.size() - 1 ; i++)
		{
			for(unsigned j = i + 1 ; j < blobs.size() ; j++)
			{
				if(blobs[i].area < blobs[j].area)
				{
					tmp = blobs[j];
					blobs[j] = blobs[i];
					blobs[i] = tmp;
				}
			}
		}
		break;
	default:
		std::cout<<" Sort method not supported!"<<std::endl;
		break;
	}

}

void Blob::draw(cv::Scalar color, cv::Mat &destination)
{

	for(unsigned i = 0; i < blobs.size(); i++){
		std::cerr << "Blob " << i << " has " << blobs[i].nRle << std::endl;
		drawCircle(blobs[i].center, 5, color, destination);
		drawCircle(blobs[i].upperLeft, 5, cv::Scalar(0, 0, 0), destination);
		drawCircle(blobs[i].downRight, 5, cv::Scalar(255, 255, 255), destination);
	}
}
}
