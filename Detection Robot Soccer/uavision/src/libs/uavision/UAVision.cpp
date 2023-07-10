#include "UAVision.h"
#include "stdio.h"

namespace uav{

void drawLine(int pt1, int pt2, cv::Scalar color, cv::Mat &img)
{
	cv::line(img, cv::Point(pt1 % img.cols, pt1 / img.cols), cv::Point(pt2 % img.cols, pt2 / img.cols), color, 2);
}

void drawCircle(int pt, int radius, cv::Scalar color, cv::Mat &img)
{
	cv::circle(img, cv::Point(pt % img.cols, pt / img.cols), radius, color, 4);
}

/*
void InitMask(vector<int> &validPixels, int mode, unsigned char *mask, unsigned width, unsigned height)
{
	int offset, mb, imb;

	for(unsigned i = 0 ; i < width * height ; i++)
		if(mask[i] > 0)
		{
			switch(mode)
			{
				case RGB8:
					validPixels.push_back(i); // index
					validPixels.push_back(i * 3); // R
					validPixels.push_back(i * 3 + 1); // G
					validPixels.push_back(i * 3 + 2); // B
					break;

				case YUV422: // UYVY
					offset = 2 * i + 1;
					validPixels.push_back(i); // index
					validPixels.push_back(offset); // Y
					if(i % 2 == 0)
					{
						validPixels.push_back(offset - 1); // U
						validPixels.push_back(offset + 1); // V
					}
					else
					{
						validPixels.push_back(offset - 3); // U
						validPixels.push_back(offset - 1); // V
					}

					break;

				case YUV411: // UYYVYY
					mb = i / 4;
					imb = i % 4;
					validPixels.push_back(i); // index
					validPixels.push_back(mb * 6 + imb + imb / 2 + 1); // Y
					validPixels.push_back(mb * 6); // U
					validPixels.push_back(mb * 6 + 3); // V

				case GRAY:

					int row = i / width;
					int col = i % width;
					int r = 0, g = 0, b = 0;

					if(row % 2 == 0) // linhas pares
					{
						if(col % 2 == 0) // colunas pares
						{
							r = i;
							g = i + 1;
							b = i + width + 1;
						}
						else
						{
							g = i;
							r = i - 1;
							b = i + width;
						}
					}
					else // linhas impares
					{
						if(col % 2 == 0) // colunas pares
						{
							g = i ;
							r = i - width;
							b = i + 1;
						}
						else // colunas impares
						{
							g = i - 1;
							r = i - width - 1;
							b = i;
						}
					}

					validPixels.push_back(i); // index
					validPixels.push_back(r); // R
					validPixels.push_back(g); // G
					validPixels.push_back(b); // B

			}

		}

}
 */

void Yuv2Rgb2(float y, float u, float v, float *r, float *g, float *b)
{
	int c, d, e;
	int modo = 2;

	switch(modo)
	{
	case 0:
		*b = (y + 1.772  * (u-128));
		*g = (y - 0.34414 * (u-128) - 0.71414 * (v-128));
		*r = (y			 + 1.402  * (v-128));
		break;

	case 1:
		*b = (1.164 * (y - 16)                     + 2.018 * (u - 128));
		*g = (1.164 * (y - 16) - 0.813 * (v - 128) - 0.391 * (u - 128));
		*r = (1.164 * (y - 16) + 1.596 * (v - 128));
		break;

	case 2:
		*b = 1.164*(y - 16) + 2.018*(u-128);
		*g = 1.164*(y - 16) - 0.813*(u-128) - 0.391*(v-128);
		*r = 1.164*(y - 16) + 1.596*(v-128);
		break;

	case 3:
		// Wikipedia
		c = (int)(y - 16);
		d = (int)(u - 128);
		e = (int)(v - 128);
		*r = ( 298 * c           + 409 * e + 128) >> 8;
		*g = ( 298 * c - 100 * d - 208 * e + 128) >> 8;
		*b = ( 298 * c + 516 * d           + 128) >> 8;
		break;

	case 4:
		// Prof. Bernardo
		*r = y + 1.14 * v;
		*g = y - 0.395 * u - 0.581 * v;
		*b = y + 2.032 * u;
		break;

	case 5:
		// yuvtoppm
		u = u - 128;
		v = v - 128;
		y = y - 16;
		//if (y < 0) y = 0;


		*r = 104635 * v;
		*g = -25690 * u + -53294 * v;
		*b = 132278 * u;

		y *= 76310;

		*r = *r + y;
		*g = *g + y;
		*b = *b + y;
		break;
	}

	if(*r < 0) *r = 0;
	if(*g < 0) *g = 0;
	if(*b < 0) *b = 0;
	if(*r > 255) *r = 255;
	if(*g > 255) *g = 255;
	if(*b > 255) *b = 255;
}

void Yuv2Rgb(int y, int u, int v, int *r, int *g, int *b)
{
	*r = ( 298 * (y - 16) + 409 * (v - 128) + 128) >> 8;
	*g = ( 298 * (y - 16) - 100 * (u - 128) - 208 * (v - 128) + 128) >> 8;
	*b = ( 298 * (y - 16) + 516 * (u - 128) + 128) >> 8;

	if(*r < 0) *r = 0;
	if(*g < 0) *g = 0;
	if(*b < 0) *b = 0;
	if(*r > 255) *r = 255;
	if(*g > 255) *g = 255;
	if(*b > 255) *b = 255;
}

void Rgb2Yuv(int r, int g, int b, int *y, int *u, int *v)
{
	*y = 0.299*r + 0.587*g + 0.114*b;
	*u = (r-*y)*0.713 +127;
	*v = (b-*y)*0.564 + 127;


	if(*y < 0) *y = 0;
	if(*u < 0) *u = 0;
	if(*v < 0) *v = 0;
	if(*y > 255) *y = 255;
	if(*u > 255) *u = 255;
	if(*v > 255) *v = 255;


}

void Rgb2Hsv( int r, int g, int b, int *hout, int *sout, int *vout )
{
	float min, max, delta;
	float h, s, v;

	min = min3( r, g, b );
	max = max3( r, g, b );
	v = max;				// v
	delta = max - min;
	if( max != 0 )
		s = delta / max * 100;		// s
	else
	{
		// r = g = b = 0		// s = 0, v is undefined
		s = 0;
		h = -1;
		return;
	}
	if(delta != 0)
	{
		if( r == max )
			h = ( g - b ) / delta;		// between yellow & magenta
		else if( g == max )
			h = 2 + ( b - r ) / delta;	// between cyan & yellow
		else
			h = 4 + ( r - g ) / delta;	// between magenta & cyan
	}
	else{
		h = 0;
	}

	h *= 60;				// degrees
	if( h < 0 )
		h += 360;

	*hout = (int)h;
	*sout = (int)s;
	*vout = (int)v;
}

void Rgb2Hsv2( float r, float g, float b, float *h, float *s, float *v)
{
	float min, max, delta;
	min = min3( r, g, b );
	max = max3( r, g, b );
	*v = max;				// v
	delta = max - min;
	if( max != 0 )
		*s = delta / max * 100;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		*s = 0;
		*h = -1;
		return;
	}
	if( r == max )
		*h = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		*h = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		*h = 4 + ( r - g ) / delta;	// between magenta & cyan
	*h *= 60;				// degrees
	if( *h < 0 )
		*h += 360;

}

void distRelation(std::vector<float> &data, cv::Mat &image, const std::string fileName, bool full = false)
{

	std::fstream inFile;
	double p0, p1, p2, p3;

	if( fileName.size() != 0 )
	{
		inFile.open(fileName.c_str(), std::fstream::in | std::fstream::binary);
		inFile >> p3;
		inFile >> p2;
		inFile >> p1;
		inFile >> p0;
	}

	else
	{
		p3 = -4.6635e-08;
		p2 = 1.1934e-04;
		p1 = -1.0125e-01;
		p0 = 3.2921e+01;
	}

	float tmp;

	for(int d = 0 ; d < 1201 ; d++)
	{
		tmp = p3 * pow(d, 3) + p2 * pow(d, 2) + p1 * d + p0;
		if(tmp < 6) tmp = 6;
		data.push_back(tmp);
	}

	if(full) return;

	for(int d = 0 ; d < 40 && d < data.size(); d++)
	{
		data[d] = (int)(data[d] * 0.8);
	}
}


void distRelation(std::vector<float> &data, cv::Mat &image, bool full = false)
{


	/* full

  	  	double p3 = -2.3922e-07;
        double p2 = 3.3582e-04;
        double p1 = -1.7131e-01;
        double p0 = 3.7821e+01; */

	double p3 = -4.6635e-08;
	double p2 = 1.1934e-04;
	double p1 = -1.0125e-01;
	double p0 = 3.2921e+01;

	/* quarter

	double p3 =  -1.2408e-08;
	double p2 = 3.1264e-05;
	double p1 =  -2.6377e-02;
	double p0 =   8.5702;

	/* half
		double p3 = -2.3710e-08;
		double p2 = 6.0675e-05;
		double p1 = -5.1438e-02;
		double p0 =  1.6800e+01; */


	float tmp;

	for(int d = 0 ; d < 1201 ; d++)
	{
		tmp = p3 * pow(d, 3) + p2 * pow(d, 2) + p1 * d + p0;
		if(tmp < 6) tmp = 6;
		data.push_back(tmp);
	}

	if(full) return;

	for(int d = 0 ; d < 40 && d < data.size(); d++)
	{
		data[d] = (int)(data[d] * 0.8);
	}
}

void distRelationGoalie(std::vector<float> &data, cv::Mat &image, int value)
{
	for(int i = 0 ; i < image.cols * image.rows ; i++)
		data.push_back(value);
}

void convertResolution(cv::Mat &original, cv::Mat &converted, int resolutionFactor)
{
	for(int i = 0; i < converted.cols; i++)
		for(int j = 0; j < converted.rows; j++)
		{
			converted.ptr()[converted.channels() * (i + j*converted.cols)] = original.ptr()[original.channels() * (i*resolutionFactor + (j*resolutionFactor)*original.cols)];
			converted.ptr()[converted.channels() * (i + j*converted.cols) + 1] = original.ptr()[original.channels() * (i*resolutionFactor + (j*resolutionFactor)*original.cols) + 1];
			converted.ptr()[converted.channels() * (i + j*converted.cols) + 2] = original.ptr()[original.channels() * (i*resolutionFactor + (j*resolutionFactor)*original.cols) + 2];
		}

}

void convertResolutionGray(cv::Mat &original, cv::Mat &converted, int resolutionFactor)
{

	for(int i = 0; i < converted.cols; i++)
		for(int j = 0; j < converted.rows; j++)
		{
			converted.ptr()[i + j*converted.cols] = original.ptr()[i*resolutionFactor + (j*resolutionFactor)*original.cols];
		}

}

std::vector<cv::Point> regionGrowingSegmentation(cv::Mat &image, int threshold, int colorOfInterest, int colorOfVisited, std::vector<double> &areaList, std::vector<double> &widthHeightRelation, std::vector<double> &solidity){
	cv::Point seed;
	cv::Rect rect;
	std::vector<cv::Point> list;
	cv::Point center;
	int countPainted = 0;

	for(int i = 0; i < image.cols; i++)
		for(int j = 0; j < image.rows; j++)
		{
			if(image.ptr()[image.channels() * (i + j*image.cols)] == colorOfVisited ||
					image.ptr()[image.channels() * (i + j*image.cols)] == 255)
			{
				continue;
			}

			if(image.ptr()[image.channels() * (i + j*image.cols)] != colorOfInterest)
			{
				image.ptr()[image.channels() * (i + j*image.cols)] = colorOfVisited;
				continue;
			}

			else
			{
				seed.x = i;
				seed.y = j;
				countPainted = 0;

				cv::floodFill(image, seed, cv::Scalar(255, 255, 255) /*cv::Scalar(colorOfVisited, colorOfVisited, colorOfVisited)*/, &rect, cvScalarAll(threshold), cvScalarAll(threshold), 4);

				center.x = rect.x + rect.width/2;
				center.y = rect.y + rect.height/2;

				double area = rect.width * rect.height;
				double whrel =  rect.width/rect.height;

				for(int i = rect.x; i < rect.x + rect.width; i++)
					for(int j = rect.y; j < rect.y + rect.height; j++)
						if(image.ptr()[image.channels() * (i + j*image.cols)] == 255)
							countPainted ++;

				double solid = (double)(countPainted/area);

				areaList.push_back(area);
				widthHeightRelation.push_back(whrel);
				solidity.push_back(solid);
				list.push_back(center);

			}

		}

	//cv::imshow("Index", image);
	//cv::waitKey(0);

	return list;
}

std::vector<cv::Point2f> calculateContours(cv::Mat &image, cv::Mat &canny_output, cv::Mat &idxImage, int colorOfInterest,int threshold, std::vector<double> &areaList, std::vector<double> &widthHeightRelation, std::vector<double> &solidity)
												{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contoursLimited;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<cv::Point2f> contourList;
	int countPainted = 0;

	/// Detect edges using canny
	cv::Canny( image, canny_output, threshold, threshold*2, 3 );
	//cv::imshow("canny", canny_output);
	/// Find contours
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	/// Get the moments
	std::vector<cv::Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{ mu[i] = moments( contours[i], false ); }

	///  Get the mass centers:
	std::vector<cv::Point2f> mc( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{ mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

	/// Calculate the area with the moments 00 and compare with the result of the OpenCV function


	for( int i = 0; i< contours.size(); i++ )
	{
		//printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );

		/* if(contourArea(contours[i]) > 10 &&
			idxImage.ptr()[image.channels() * ((int)mc[i].x + (int)mc[i].y*image.cols)] == colorOfInterest )
			  contourList.push_back(mc[i]); */

		if((int)(mc[i].x)  < 0 || (int)(mc[i].y) < 0) continue;


		if(idxImage.ptr()[idxImage.channels() * ((int)(mc[i].x) + (int)(mc[i].y)*idxImage.cols)] == colorOfInterest )
		{
			contourList.push_back(mc[i]);
			countPainted = 0;

			cv::Rect rect = cv::boundingRect(contours[i]);
			double area = rect.width*rect.height;//contourArea(contours[i]);
			double whrel =  rect.width/rect.height;

			for(int k = rect.x; k < rect.x + rect.width; k++)
				for(int j = rect.y; j < rect.y + rect.height; j++)
					if(idxImage.ptr()[image.channels() * (k + j*image.cols)] == colorOfInterest)
						countPainted ++;

			double solid = (double)(countPainted/area);

			areaList.push_back(area);
			contoursLimited.push_back(contours[i]);
			widthHeightRelation.push_back(whrel);
			solidity.push_back(solid);
		}

	}
	/*
	for(int i = 0; i < contoursLimited.size(); i++)
	{
		drawContours( image, contoursLimited, i, cvScalar(255,255,255), 2, 8, hierarchy, 0, cv::Point());

	}
	//imshow("Contours", image); */
	return contourList;
												}


cv::Point3d calcWorldCoords(cv::Point point, cv::Mat &rotationVecs, cv::Mat &transVecs, cv::Mat &intrinsics, cv::Mat &distCoeffs)
{
	using namespace std;
	using namespace cv;
	cv::Mat invert_rotation_matrix = cv::Mat (3,3,CV_32FC1);
	cv::Mat rotationMatrix = cv::Mat(3,3,CV_32FC1);
	Point3d point3;

	int x=point.x;
	int y=point.y;

	cv::Mat original = cvCreateMat(1,2,CV_32FC2);
	cv::Mat undistorted = cvCreateMat(1,2,CV_32FC2);///Output array from the undistortion process

	//~ std::cout<<"calc World coordinates\n";

	original.at<float>(0,0) = x;
	original.at<float>(0,1) = y;
	//Calculate the new point due the lens distortion
	undistortPoints(original, undistorted, intrinsics, distCoeffs);

	//~ line(image_aux,Point(x-5,y),Point(x+5,y),Scalar(0,0,0));
	//~ line(image_aux,Point(x,y-5),Point(x,y+5),Scalar(0,0,0));

	///Calculate extrinsic parameters
	//~ solvePnP(object_points,image_points,intrinsics,distCoeff,rotationVecs,transVecs);

	//Converts a rotation matrix to a rotation vector, or vice-versa
	Rodrigues(rotationVecs,rotationMatrix);
	invert(rotationMatrix,invert_rotation_matrix);
	invert_rotation_matrix.convertTo(invert_rotation_matrix, CV_64FC1);
	// Camera coordinate in Real World
	cv::Mat CamCoordsInRealWorld = cv::Mat(3,1,CV_64FC1);
	cv::Mat CamCoordsInCam = cv::Mat(3,1,CV_64FC1);

	cv::Mat DirectionVectorInCam = cv::Mat(3,1,CV_64FC1);
	cv::Mat DirectionVectorInRealWorld = cv::Mat(3,1,CV_64FC1);

	cv::Mat DirectionVector = cv::Mat(3,1,CV_64FC1);

	///Inicialize matrix wich 0 values
	CamCoordsInCam.at<double>(0) = 0;
	CamCoordsInCam.at<double>(1) = 0;
	CamCoordsInCam.at<double>(2) = 0;

	//~ cout <<"2************************"<<endl;
	//~ std::cout << CamCoordsInCam << std::endl;

	/// Construction  a vector on origin at the camera, consiring the pixels on undistorted image;
	/// Noted that the vector is a unitary vector.
	DirectionVectorInCam.at<double>(0) = undistorted.at<float>(0,0);
	DirectionVectorInCam.at<double>(1) = undistorted.at<float>(0,1);
	DirectionVectorInCam.at<double>(2) = 1;

	//~ cout <<"3************************"<<endl;
	//~ cout << "DirectionVectorInCam " << DirectionVectorInCam<<endl;
	//~ cout << "Undistorted " << undistorted.at<double_t>(0,0)<<endl;


	//*********************************************************
	//*********************************************************
	CamCoordsInCam.at<double_t>(0) -= transVecs.at<double_t>(0);
	CamCoordsInCam.at<double_t>(1) -= transVecs.at<double_t>(1);
	CamCoordsInCam.at<double_t>(2) -= transVecs.at<double_t>(2);

	//*********************************************************

	//~ cout << "CamCoordsInCam " << CamCoordsInCam <<endl;
	//~ cout << "transVec       " << transVecs <<endl; // --------------------------------> Extra variable to print (Debug)
	///
	DirectionVectorInCam.at<double_t>(0) -= transVecs.at<double_t>(0);
	DirectionVectorInCam.at<double_t>(1) -= transVecs.at<double_t>(1);
	DirectionVectorInCam.at<double_t>(2)  -= transVecs.at<double_t>(2);

	//~ cout <<"4************************"<<endl;
	//~ cout << "DirectionVectorInCam " << DirectionVectorInCam <<endl;


	CamCoordsInRealWorld = invert_rotation_matrix * CamCoordsInCam;
	DirectionVectorInRealWorld = invert_rotation_matrix*DirectionVectorInCam;

	//~ cout <<"5************************"<<endl;
	//~ cout << "CamCoordsInRealWorld " << CamCoordsInRealWorld <<endl;
	//~ cout << "DirectionVectorInRealWorld " << CamCoordsInRealWorld <<endl;

	/// Calculate the amplitude of the vector
	DirectionVector.at<double_t>(0,0) = DirectionVectorInRealWorld.at<double_t>(0,0)-CamCoordsInRealWorld.at<double_t>(0,0);
	DirectionVector.at<double_t>(0,1) = DirectionVectorInRealWorld.at<double_t>(0,1)-CamCoordsInRealWorld.at<double_t>(0,1);
	DirectionVector.at<double_t>(0,2) = DirectionVectorInRealWorld.at<double_t>(0,2)-CamCoordsInRealWorld.at<double_t>(0,2);

	/// Calculate the number of times that the vector must be multiplied to intersects the Z = 0 plane;
	float Lamda = -CamCoordsInRealWorld.at<double_t>(0,2) / DirectionVector.at<double_t>(0,2);

	//~ cout << "\nX:" << CamCoordsInRealWorld.at<double_t>(0,0) << endl;
	//~ cout << "Y:" << CamCoordsInRealWorld.at<double_t>(0,1) << endl;
	//~ cout << "Z:" << CamCoordsInRealWorld.at<double_t>(0,2) << endl;

	/// Calculate the intersection of the direction vector and ground´s plane;
	CamCoordsInRealWorld.at<double_t>(0,0) = CamCoordsInRealWorld.at<double_t>(0,0) + DirectionVector.at<double_t>(0,0) * Lamda;
	CamCoordsInRealWorld.at<double_t>(0,1) = CamCoordsInRealWorld.at<double_t>(0,1) + DirectionVector.at<double_t>(0,1)* Lamda;
	CamCoordsInRealWorld.at<double_t>(0,2) = CamCoordsInRealWorld.at<double_t>(0,2) + DirectionVector.at<double_t>(0,2) * Lamda; // = 0

	/// Print coordinates in real world.
	//~ cout << "\nX:" << CamCoordsInRealWorld.at<double_t>(0,0) << endl;
	//~ cout << "Y:" << CamCoordsInRealWorld.at<double_t>(0,1) << endl;
	//~ cout << "Z:" << CamCoordsInRealWorld.at<double_t>(0,2) << endl;

	point3.x = CamCoordsInRealWorld.at<double_t>(0,0);
	point3.y = CamCoordsInRealWorld.at<double_t>(0,1);
	point3.z = CamCoordsInRealWorld.at<double_t>(0,2);

	return point3;

	//~ CoordsinRealWorld_x = CamCoordsInRealWorld.at<double_t>(0,0);
	//~ CoordsinRealWorld_y = CamCoordsInRealWorld.at<double_t>(0,1);

}

}
