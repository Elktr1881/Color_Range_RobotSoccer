#include "RLE.h"

namespace uav{

RLE::RLE(ScanLines &s_, unsigned colorBefore, unsigned colorOfInterest, unsigned colorAfter, unsigned threshColorBefore,
		unsigned threshColorOfInterest, unsigned threshColorAfter, unsigned searchWindow, bool obstaclesCAMBADA)
{
	unsigned pointsColor;
	unsigned pointsColorBefore;
	unsigned pointsColorAfter;
	RLEInfo aux_rle;
	std::vector<int> temp;
	s = s_;

	searchWindow = searchWindow/s.getStep();
	threshColorBefore = threshColorBefore/s.getStep();
	threshColorAfter = threshColorAfter/s.getStep();


	for(unsigned j = 0; j < s.scanlines.size(); j++)
	{
		temp = s.getLine(j);

		for (unsigned i = 0; i < temp.size(); i++)
		{
			//std::cerr << " i " << i << " j " << j << " img " << (unsigned)s.image.ptr()[temp[i]] << std::endl;
			if(!((unsigned)s.image.ptr()[temp[i]] & colorOfInterest))
				continue;

			pointsColor = 0;
			pointsColorAfter = 0;

			aux_rle.start = temp[i];
			aux_rle.startBefore = temp[i];
			while(i < temp.size() - 1 && ((unsigned)s.image.ptr()[temp[i]] & colorOfInterest))
			{
				pointsColor++;
				i++;
			}
			aux_rle.end = temp[i];
			aux_rle.endAfter = temp[i];
			aux_rle.center = temp[i - pointsColor/2];

			if(pointsColor < threshColorOfInterest)
				continue;

			int k;

			pointsColorBefore = 0;
			for(k = i - pointsColor ; k > (i - pointsColor - searchWindow) && k >= 0 ; k--)
			{
				if((unsigned)s.image.ptr()[temp[k]] & colorBefore)
				{
					pointsColorBefore++;
				}
			}
			aux_rle.startBefore = temp[k];

			pointsColorAfter = 0;
			for(k = i ; k < (i + searchWindow) && k < temp.size() - 1 ; k++) // -1 for the end of SC
			{
				if((unsigned)s.image.ptr()[temp[k]] & colorAfter)
				{
					pointsColorAfter++;

				}
			}
			aux_rle.endAfter = temp[k];

			if((pointsColorAfter >= threshColorAfter && pointsColorBefore >= threshColorBefore)
					&& pointsColor >= threshColorOfInterest)
			{

				aux_rle.scIdx = j;
				//aux_rle.start = temp[i - pointsColor];
				//aux_rle.end = temp[i];
				//aux_rle.center = temp[i - pointsColor/2];
				aux_rle.lengthColor = pointsColor * s.getStep();
				aux_rle.lengthColorBefore = pointsColorBefore * s.getStep();
				aux_rle.lengthColorAfter = pointsColorAfter * s.getStep();
				rlData.push_back(aux_rle);
			}

		}


	}
}

void RLE::draw(cv::Scalar colorBefore, cv::Scalar colorOfInterest, cv::Scalar colorAfter, cv::Mat &destination)
{
	for(unsigned k = 0; k < rlData.size(); k++)
	{
		drawLine(rlData[k].startBefore, rlData[k].start, colorBefore, destination);
		drawLine(rlData[k].start, rlData[k].end, colorOfInterest, destination);
		drawLine(rlData[k].end, rlData[k].endAfter, colorAfter, destination);
	}
}
}



