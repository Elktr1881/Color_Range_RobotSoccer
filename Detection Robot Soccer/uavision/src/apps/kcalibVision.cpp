#include <opencv2/opencv.hpp>
#include "UAVision.h"
#include "Config.h"
#include "Camera.h"
#include "CameraOpenCV.h"
#include "CameraSettingsOpenCV.h"
#include "CameraSettingsFirewire.h"
#include "CameraSettingsEth.h"
#include "CameraSettingsFactory.h"
#include "ScanLines.h"
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>
#include "ServerSocket.h"
#include "libCalib.h"

bool end = false;

using namespace uav;

int Hsvtrace=0, HsvRangeMin=0, HsvRangeMax=0;
CvPoint pt1;
CvPoint pt2;
CvPoint pt;
bool pressed = false;
int nColor = 0, lastColor = 0;
ColorRange *colorRange;
std::vector<int> HArray;
std::vector<int> SArray;
std::vector<int> VArray;
int hh, ss, vv;
cv::Mat helpImage(cv::Size(400, 280), CV_8UC3);

void HSV_on_mouse( int event, int x, int y, int flags, void* param);
void on_mouse( int event, int x, int y, int flags, void* param);
void fillHelpImage();

void SignCatch( int sig )
{
	switch ( sig )
	{
	case SIGINT:
		if( !end ) {
			std::cerr << "\n---\nSIGINT: signal received, stoping application.\n---" << std::endl;
			end = true;
		}
		break;
	default:
		std::cerr << "ERROR: Unknow signal received, continuing." << std::endl;
		break;
	}
}

void printHelp()
{
	std::cout << "---- CAMBADA vision ----" << std::endl;
	std::cout << "Command line parameters:" << std::endl;
	std::cout << "  -h (help)" << std::endl;
	std::cout << "  -addr # (address where the server is waiting)" << std::endl;
	std::cout << "  -port # (port number where the server is waiting)" << std::endl;
	std::cout << "  -mask # (filename with a mask)" << std::endl;
}

int main(int argc, char *argv[])
{
	ParameterRange parametersRange[10];
	std::string configFileName = "/tmp/x.conf";
	bool calibColors = false, showHelp = false, manualCalib = false;
	int port = 5000;
	std::string hostname = "127.0.0.1";
	Socket s(hostname, port);
	int rows, cols;
	colorRange = new ColorRange[UAV_NCOLORS];
	int camType;
	unsigned int size;
	int rec;
	std::string x;
	int hMin=0, hMax=0, sMin=0, sMax=0, vMin=0, vMax=0;
	int gain, exposure, wbRed, wbBlue;

	for(int i = 1 ; i < argc ; i++)
	{
		if(strcmp(argv[i] , "-h") == 0 )
		{
			printHelp();
			return 0;
		}

		if(strcmp(argv[i] , "-port") == 0)
			sscanf(argv[++i] , "%d" , &port);

		if(strcmp(argv[i] , "-addr") == 0)
			hostname.assign(argv[i+1]);

		if(strcmp(argv[i] , "1") == 0)
			hostname.assign("172.16.39.1");

		if(strcmp(argv[i] , "2") == 0)
			hostname.assign("172.16.39.2");

		if(strcmp(argv[i] , "3") == 0)
			hostname.assign("172.16.39.3");

		if(strcmp(argv[i] , "4") == 0)
			hostname.assign("172.16.39.4");

		if(strcmp(argv[i] , "5") == 0)
			hostname.assign("172.16.39.5");
	}

	fillHelpImage();

	// Socket -----------------------------------------------------------------
	if(!s.open(hostname, port)){
		std::cerr << "Error opening the socket\n";
		return 0;
	}

	std::string msgRec;
	std::string tmp;
	std::stringstream tmpss;
	s >> tmp; tmpss << tmp; tmpss >> cols  >> rows >> camType;
	std::cerr << "Image size: " << rows << "x" << cols << " camType " << camType << std::endl;
	// ------------------------------------------------------------------------

	cv::Mat srcImage(cv::Size(rows, cols),CV_8UC3);
	cv::Mat segImage(cv::Size(rows, cols),CV_8UC3);
	cv::Mat maskImage(cv::Size(rows, cols),CV_8UC1);
	cv::Mat hsvHist(cv::Size(361, 420),CV_8UC3);

	CameraSettingsFactory camSettingsFactory;
	CameraSettings *camSettings = camSettingsFactory.Create(camType);

	for(int i = 1 ; i < argc ; i++)
	{
		if(strcmp(argv[i] , "-mask") == 0)
			maskImage = cv::imread(argv[i+1], CV_LOAD_IMAGE_GRAYSCALE);
	}

	cv::namedWindow( "Main window", CV_WINDOW_AUTOSIZE );
	cvMoveWindow( "Main window", 0, 0 );

	while(!end)
	{
		if(manualCalib)
		{
			camSettings->setCameraSetting(UAV_GAIN, gain);
			camSettings->setCameraSetting(UAV_EXPOSURE, exposure);
			camSettings->setCameraSetting(UAV_WBR, wbRed);
			camSettings->setCameraSetting(UAV_WBB, wbBlue);
		}

		if(calibColors)
		{
			if(pressed) // Mouse on the segImage create marks on the histogram
			{
				int p = (pt2.y * srcImage.cols + pt2.x)*3;
				int r = srcImage.ptr()[p]; int g = srcImage.ptr()[p + 1]; int b = srcImage.ptr()[p + 2];
				Rgb2Hsv(r, g, b, &hh, &ss, &vv );
				if(hh < 0) hh = 0; if(hh > 359) hh = 359;
				pressed = false;
				HArray.push_back(hh); SArray.push_back(ss); VArray.push_back(vv);

				hMax = hh + 20;
				hMin = hh - 20;
				sMax = ss + 20;
				sMin = ss - 20;
				vMax = vv + 50;
				vMin = vv - 50;

				colorRange[nColor].hMax = hMax; colorRange[nColor].hMin = hMin;
				colorRange[nColor].vMax = vMax; colorRange[nColor].vMin = vMin;
				colorRange[nColor].sMax = sMax; colorRange[nColor].sMin = sMin;

				cv::setTrackbarPos( "Hmax","HSV" , colorRange[nColor].hMax );
				cv::setTrackbarPos( "Hmin","HSV" , colorRange[nColor].hMin );
				cv::setTrackbarPos( "Smax","HSV" , colorRange[nColor].sMax );
				cv::setTrackbarPos( "Smin","HSV" , colorRange[nColor].sMin );
				cv::setTrackbarPos( "Vmax","HSV" , colorRange[nColor].vMax );
				cv::setTrackbarPos( "Vmin","HSV" , colorRange[nColor].vMin );


			}

			segImage.setTo(cv::Scalar(127, 127, 127));
			PaintImage(srcImage, segImage, maskImage, colorRange);
			//cv::cvtColor(segImage, srcImage, CV_RGB2BGR);
			imshow("SEGMENTASI", segImage); // Verify swap colors

			hsvHist.setTo(cv::Scalar(0, 0, 0));
			drawHSVHistogram(srcImage, hsvHist, maskImage, colorRange[nColor].hMin, colorRange[nColor].hMax, nColor,
					HArray, SArray, VArray, colorRange);

			if (lastColor == nColor) // update the color info
			{
				colorRange[nColor].hMax = hMax; colorRange[nColor].hMin = hMin;
				colorRange[nColor].vMax = vMax; colorRange[nColor].vMin = vMin;
				colorRange[nColor].sMax = sMax; colorRange[nColor].sMin = sMin;
			}
			else // update the bars to the new color info...
			{
				cv::setTrackbarPos( "Hmax","HSV" , colorRange[nColor].hMax );
				cv::setTrackbarPos( "Hmin","HSV" , colorRange[nColor].hMin );
				cv::setTrackbarPos( "Smax","HSV" , colorRange[nColor].sMax );
				cv::setTrackbarPos( "Smin","HSV" , colorRange[nColor].sMin );
				cv::setTrackbarPos( "Vmax","HSV" , colorRange[nColor].vMax );
				cv::setTrackbarPos( "Vmin","HSV" , colorRange[nColor].vMin );
				lastColor = nColor;
			}

			cv::cvtColor(hsvHist, hsvHist, CV_RGB2BGR);
			imshow("HSV", hsvHist); // FIXME verify swap colors
		}
		else
		{
			cv::Mat tmpImage;
			srcImage.copyTo(tmpImage);
			//cv::cvtColor(tmpImage, tmpImage, CV_RGB2BGR);
			imshow("Main window", tmpImage);
		}

		if(showHelp)
			imshow("Help window", helpImage);

		int k = cv::waitKey(1);
		switch((char)k)
		{
		case 'q':
			end = true;
			break;

		case 'i':
			x = "i";
			s << x;
			size = rows * cols*3;
			rec = s.recvAll((void*)srcImage.ptr(), &size);
			std::cerr << "Received image " << size << " bytes" << std::endl;
			break;

		case 'x':
			x = "x";
			s << x;
			size = camSettings->getSize();
			rec = s.recvAll((void *)camSettings->ptr(), &size);
			std::cerr << "Received camera settings " << size << " bytes" << std::endl;
			x = "ok";
			s << x;
			size = 10 * sizeof(ParameterRange);
			rec = s.recvAll((void *)parametersRange, &size);
			std::cerr << "Received parameters ranges " << size << " bytes" << std::endl;
			break;

		case 'm':
			x = "m";
			s << x;
			size = rows * cols;
			rec = s.recvAll((void*)maskImage.ptr(), &size);
			std::cerr << "Received mask " << size << " bytes" << std::endl;
			break;

		case 'c':
			x = "c";
			s << x;
			size = colorRange->getSize() * UAV_NCOLORS;
			rec = s.recvAll((void*)colorRange, &size);
			std::cerr << "Received colors " << size << " bytes" << std::endl;
			break;

		case 'C':
			x = "C";
			s << x;
			s >> msgRec;
			std::cerr << "Received " << msgRec << std::endl;
			size = s.sendAll((void*)colorRange, colorRange->getSize()*UAV_NCOLORS);
			std::cerr << "Sent colors with " << size << "bytes\n";
			break;

		case 'M':
			x = "M";
			s << x;
			s >> msgRec;
			std::cerr << "Received " << msgRec << std::endl;
			size = rows * cols;
			size = s.sendAll((void*)maskImage.ptr(), size);
			std::cerr << "Sent mask with " << size << "bytes\n";
			break;

		case 'X':
			x = "X";
			s << x;
			s >> msgRec;
			std::cerr << "Received " << msgRec << std::endl;
			size = s.sendAll((void*)camSettings->ptr(), camSettings->getSize());
			std::cerr << "Sent camSettings with " << size << " bytes\n";
			break;

		case 'a':
			x = "a";
			s << x;
			break;

		case 'y':
			if(calibColors)
			{
				cv::destroyWindow("HSV");
				cv::destroyWindow("SEGMENTASI");
				calibColors = false;
			}
			else
			{
				calibColors = true;
				cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
				cvNamedWindow( "SEGMENTASI", CV_WINDOW_AUTOSIZE );
				cvMoveWindow( "HSV", 800, 0 );
				cv::createTrackbar("Hmax","HSV", &hMax, 359, 0 );
				cv::createTrackbar( "Hmin","HSV", &hMin, 359, 0 );
				cv::createTrackbar( "Smax","HSV", &sMax, 100, 0 );
				cv::createTrackbar( "Smin","HSV", &sMin, 100, 0 );
				cv::createTrackbar( "Vmax","HSV", &vMax, 255, 0 );
				cv::createTrackbar( "Vmin","HSV", &vMin, 255, 0 );
				cv::setTrackbarPos( "Hmax","HSV" , colorRange[nColor].hMax );
				cv::setTrackbarPos( "Hmin","HSV" , colorRange[nColor].hMin );
				cv::setTrackbarPos( "Smax","HSV" , colorRange[nColor].sMax );
				cv::setTrackbarPos( "Smin","HSV" , colorRange[nColor].sMin );
				cv::setTrackbarPos( "Vmax","HSV" , colorRange[nColor].vMax );
				cv::setTrackbarPos( "Vmin","HSV" , colorRange[nColor].vMin );

				cv::setMouseCallback( "HSV", HSV_on_mouse);
				cv::setMouseCallback( "Main window", on_mouse);
			}
			break;

		case 'e':
			if(manualCalib == true)
			{
				manualCalib = false;
				cv::destroyWindow( "Manual Calibration" );
			}
			else
			{
				cv::namedWindow( "Manual Calibration");
				cvMoveWindow( "Manual Calibration", 650, 0 );
				cvResizeWindow("Manual Calibration", 200, 200);
				manualCalib = true;

				cv::createTrackbar( "Gain", "Manual Calibration", &gain,(int)parametersRange[UAV_GAIN].max, 0);
				cv::createTrackbar( "Exposure", "Manual Calibration", &exposure, (int)parametersRange[UAV_EXPOSURE].max, 0);
				cv::createTrackbar( "WB_red", "Manual Calibration", &wbRed,(int)parametersRange[UAV_WB].max, 0);
				cv::createTrackbar( "WB_blue", "Manual Calibration", &wbBlue,(int)parametersRange[UAV_WB].max, 0);

				cv::setTrackbarPos("Gain", "Manual Calibration", camSettings->getCameraSetting(UAV_GAIN));
				cv::setTrackbarPos("Exposure", "Manual Calibration", camSettings->getCameraSetting(UAV_EXPOSURE));
				cv::setTrackbarPos("WB_red", "Manual Calibration", camSettings->getCameraSetting(UAV_WBR));
				cv::setTrackbarPos("WB_blue", "Manual Calibration", camSettings->getCameraSetting(UAV_WBB));
			}
			break;

		case 'h':
			if(showHelp)
			{
				cv::destroyWindow("Help window");
				showHelp = false;
			}
			else
			{
				showHelp = true;
				cvNamedWindow( "Help window", CV_WINDOW_AUTOSIZE );
				cvMoveWindow( "Help window", 640, 0 );
			}
			break;
		}

	}
}

void fillHelpImage()
{
	putText(helpImage, "Keys usage: ", cvPoint(0,20), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'q' -> terminate program", cvPoint(0,40), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'h' -> shows this help", cvPoint(0,60), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'i' -> retrives an RGB image from server", cvPoint(0,80), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'm' -> retrieve mask from server", cvPoint(0,100), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'M' -> send mask to server", cvPoint(0,120), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'c' -> retrieve color ranges from server", cvPoint(0,140), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'C' -> send color ranges to server", cvPoint(0,160), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'x' -> retrieve camera settings from server", cvPoint(0,180), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'X' -> send camera settings to server", cvPoint(0,200), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'y' -> calibrate colors", cvPoint(0,220), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'e' -> camera calib with sliders", cvPoint(0,240), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
	putText(helpImage, "  'a' -> auto-calib camera on server", cvPoint(0,260), cv::FONT_HERSHEY_PLAIN, 1.0, cvScalar(147,20,255), 1);
}

void on_mouse( int event, int x, int y, int flags, void* param)
{
	param = NULL;

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		pt2.x=x;
		pt2.y=y;
		pressed=true;
		return;
		break;

	case CV_EVENT_LBUTTONUP:
		pt2.x=x;
		pt2.y=y;
		return;
		break;
	}
}

void HSV_on_mouse( int event, int x, int y, int flags, void* param)
{
	param = NULL;
	flags = 0;

	switch( event )
	{
	case CV_EVENT_LBUTTONUP:
		pt1.x=x;
		pt1.y=y;
		Hsvtrace=1;
		HsvRangeMin=1;
		break;

	case CV_EVENT_RBUTTONUP:
		pt1.x=x;
		pt1.y=y;
		HsvRangeMax=1;
		break;
	}

	if(HsvRangeMin==1 || HsvRangeMax==1)
	{
		unsigned int x=0;

		if(pt1.y<30)
		{
			if(pt1.x<30)
				nColor=UAV_WHITE;
			if(pt1.x>=30 && pt1.x<60)
				nColor=UAV_BLACK;
			if(pt1.x>=60 && pt1.x<90)
				nColor=UAV_GREEN;
			if(pt1.x>=90 && pt1.x<120)
				nColor=UAV_ORANGE;
			if(pt1.x>=120 && pt1.x<150)
				nColor=UAV_BLUE;
			if(pt1.x>=150 && pt1.x<180)
				nColor=UAV_YELLOW;
			if(pt1.x>=180 && pt1.x<210)
				nColor=UAV_CYAN;
			if(pt1.x>=210 && pt1.x<240)
				nColor=UAV_MAGENTA;
			if(pt1.x>=240 && pt1.x<270)
			{

				HArray.clear(); SArray.clear(); VArray.clear();
				hh=ss=vv=0;

			}
		}else
		{
			if(pt1.y >= 0   && pt1.y < 140)
			{
				x=pt1.x;
				if((HsvRangeMin==1) && (x<colorRange[nColor].hMax)){
					colorRange[nColor].hMin=x;
				}
				else if((HsvRangeMax==1) && (x>colorRange[nColor].hMin)){
					colorRange[nColor].hMax=x;
				}
			}

			if(pt1.y >= 140 && pt1.y < 280)
			{
				x = pt1.x / 3.6;
				if((HsvRangeMin==1) && (x<colorRange[nColor].sMax)){
					colorRange[nColor].sMin=x;
				}
				else if((HsvRangeMax==1) && (x>colorRange[nColor].sMin)){
					colorRange[nColor].sMax=x;
				}
			}

			if(pt1.y >= 280 && pt1.y <= 420)
			{
				x = pt1.x / 1.4;
				if((HsvRangeMin==1) && (x<colorRange[nColor].vMax)){
					colorRange[nColor].vMin=x;
				}
				else if((HsvRangeMax==1) && (x>colorRange[nColor].vMin)){
					colorRange[nColor].vMax=x;
				}
			}

			HsvRangeMin=0;
			HsvRangeMax=0;

		}
	}
	return;
}