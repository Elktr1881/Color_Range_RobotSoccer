#include <opencv2/opencv.hpp>

#include <cstdio>
#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
	cout << "\nThis program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()\n"
			"Usage:\n"
			"./watershed [image_name -- default is ../data/fruits.jpg]\n" << endl;


	cout << "Hot keys: \n"
			"\tESC - quit the program\n"
			"\tr - restore the original image\n"
			"\tw or SPACE - run watershed segmentation algorithm\n"
			"\t\t(before running it, *roughly* mark the areas to segment on the image)\n"
			"\t  (before that, roughly outline several markers on the image)\n";
}
Mat markerMask, img;
Point prevPt(-1, -1);

static void onMouse( int event, int x, int y, int flags, void* )
{
	if( x < 0 || x >= img.cols || y < 0 || y >= img.rows )
		return;
	if( event == EVENT_LBUTTONUP || !(flags & EVENT_FLAG_LBUTTON) )
		prevPt = Point(-1,-1);
	else if( event == EVENT_LBUTTONDOWN )
		prevPt = Point(x,y);
	else if( event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON) )
	{
		Point pt(x, y);
		if( prevPt.x < 0 )
			prevPt = pt;
		line( markerMask, prevPt, pt, Scalar::all(255), 5, 8, 0 );
		line( img, prevPt, pt, Scalar::all(255), 5, 8, 0 );
		prevPt = pt;
		imshow("image", img);
	}
}

int main( int argc, char** argv )
{
	char* filename = argc >= 2 ? argv[1] : (char*)"../data/fruits.jpg";
	Mat img0 = imread(filename, 1), imgGray;
	Mat edges;

	if( img0.empty() )
	{
		cout << "Couldn'g open image " << filename << ". Usage: watershed <image_name>\n";
		return 0;
	}
	help();
	namedWindow( "image", 1 );

	img0.copyTo(img);
	cvtColor(img, imgGray, COLOR_BGR2GRAY);
	Mat grayBGR;
	cvtColor(imgGray, grayBGR, COLOR_GRAY2BGR);
	//markerMask = Scalar::all(0);
	//imshow( "image", img );
	setMouseCallback( "image", onMouse, 0 );

	for(;;)
	{
		double t;
		int i, j, compCount = 0;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		t = (double)getTickCount();
		Canny(imgGray, edges, 90, 160, 3);

		imwrite("edges.jpg", edges);
	//	imshow( "Canny", edges);
		t = (double)getTickCount() - t;
		printf( "Canny time = %gms\n", t*1000./getTickFrequency() );

		t = (double)getTickCount();
		findContours(edges, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

		t = (double)getTickCount() - t;
		printf( "Find contours time = %gms\n", t*1000./getTickFrequency() );
		if( contours.empty() )
			continue;

		t = (double)getTickCount();
		Mat markers(imgGray.size(), CV_32S);
		markers = Scalar::all(0);
		int idx = 0;
		for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
			drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);

		//imshow( "Contours", markers);
		t = (double)getTickCount() - t;
		printf( "Draw contours time = %gms\n", t*1000./getTickFrequency() );

		if( compCount == 0 )
			continue;

		vector<Vec3b> colorTab;
		for( i = 0; i < compCount; i++ )
		{
			int b = theRNG().uniform(0, 255);
			int g = theRNG().uniform(0, 255);
			int r = theRNG().uniform(0, 255);

			colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
		}

		t = (double)getTickCount();
		watershed( img0, markers );
		t = (double)getTickCount() - t;
		printf( "Watershed time = %gms\n", t*1000./getTickFrequency() );

		Mat wshed(markers.size(), CV_8UC3);

		// paint the watershed image
		for( i = 0; i < markers.rows; i++ )
			for( j = 0; j < markers.cols; j++ )
			{
				int index = markers.at<int>(i,j);
				if( index == -1 )
					wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
				else if( index <= 0 || index > compCount )
					wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
				else
					wshed.at<Vec3b>(i,j) = colorTab[index - 1];
			}

		wshed = wshed*0.5 + grayBGR*0.5;
		imwrite("watershed.jpg", wshed);
		//imshow( "watershed transform", wshed );
		break;
	}

	return 0;
}
