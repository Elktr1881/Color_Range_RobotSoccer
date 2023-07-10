/*
 * Copyright (C) 2014 Alina Trifan and Ant\Uffffffff Neves
 * mailto: alina.trifan@ua.pt -- an@ua.pt 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//==================Header Uavision====================//
#include <opencv2/opencv.hpp>
#include "UAVision.h"
#include "Config.h"
#include "Camera.h"
#include "Lut.h"
#include "RLE.h"
#include "Blob.h"
#include "libCalib.h"
#include "CameraOpenCV.h"
#include "CameraFromFile.h"
#include "CameraSettingsOpenCV.h"
#include "ScanLines.h"
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>
#include <vector>
#include "ServerSocket.h"
#include "sys/time.h"
#include "Vec.h"
#include "CameraCalib.h"
#include <cmath>
//==================Header Serial======================//
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

using namespace uav;
using namespace std;

//#define MAX_POINTS 500
#define MAX_BALLS 1
#define MAX_Obs 1
#define MAX_team 1
cv::Mat image, originalImage;
unsigned int sizeToSend;
bool end = false;

Config config;
bool flagSaveFile = false;
bool fullSize = false;
bool autoCalib = false;
bool verbose = false;
bool timeInfo = false;
bool runtimeCalibConverged = false;
//std::vector<bool> paramConverged;
ParameterRange parametersRange[10];
Lut *lut;
int camType = 0, statusCam = 0;

cv::Point ballPos(0,0), centerX(0,0), last_ballPos(0,0), selisih(0,0), titik(0,0);
cv::Point sRGpos(0,0), sLGpos(0,0), sHGpos(0,0);
cv::Point obs0(0,0), obs1(0,0), obs(0,0);
cv::Point team0(0,0), team1(0,0), team(0,0);
//==========================================Fungsi untuk Komunikasi UDP==========================================//
#include "udpclientserver.h"
using namespace udp_client_server;

//inisialisasi UDP
std::string ip_clien = "127.0.0.1";   //172.16.4.257  ip tujuan // ip local
int prt_clien = 1111;
bool udp = false;
char senudp[1024];

//==============================Fungsi unruk Menghitung Jarak==================================//
int euclidean(cv::Point2d awal, cv::Point2d akhir){
    cv::Point2d diff = akhir-awal;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

//==============================Fungsi unruk Help Running Program==================================//
void printHelp()
{
	std::cout << "---- SAKERA vision ----" << std::endl;
	std::cout << "Command line parameters:" << std::endl;
	std::cout << "  -h (help)" << std::endl;
	std::cout << "  -nodisp (do not show image)" << std::endl;
	std::cout << "  -server (accept connections from imageCalib app)" << std::endl;
	std::cout << "  -cf # (name of the configuration file)" << std::endl;
	std::cout << "  -debug (shows visual debug information on the image)" << std::endl;
	std::cout << "  -v (verbose)" << std::endl;
	std::cout << "  -tv (time verbose)" << std::endl;
	std::cout << "  -cam # (camera type 0 - eth; 1 - opencv;) used to create a new config" << std::endl;
	std::cout << "  -port # (port number when in server mode)" << std::endl;
	std::cout << "  -load # (load a video file)" << std::endl;
	std::cout << "  -save # (save a video file)" << std::endl;
	std::cout << "  -loop (play video file in loop mode)" << std::endl;
	std::cout << "  -fs (full size display)" << std::endl;
	std::cout << "  -slave (vision does not do PMAN_tick so that we can use cursor)" << std::endl;
	std::cout << "  -ball # (ball color configured in visionCalib 1 - orange 2 - yellow 3 - magenta 4 -cyan 5-blue; if nothing is specified, the orange ball is used by default)" << std::endl;
	std::cout << "  -bayer (access bayer data from camera)" << std::endl;
	std::cout << "  -rgb (access RGB data from camera)" << std::endl;
	std::cout << "  -fps # (set a new falue for fps)" << std::endl;
	std::cout << "  -colorControl # (set a new falue for colorControl 0 - disabled 1 - normal 2 - HQ)" << std::endl;
	std::cout << "  -p # (name of the file containing the polinomyal coeffs)"<<std::endl;
}

//==============================Fungsi unruk Mengetahui Waktu Proses Kondisi==================================//
unsigned long tmp_tick_tack;

void tick( void )
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	tmp_tick_tack = tv.tv_sec*1000000 + tv.tv_usec;
}

void tack(bool _verbose, const char *msg )
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	if(_verbose)
		std::cerr <<  msg << ((tv.tv_sec*1000000 + tv.tv_usec) - tmp_tick_tack) / 1000 << " ms, " << (tv.tv_sec*1000000 + tv.tv_usec) - tmp_tick_tack << " us" << std::endl;
}

//==========================================Fungsi Kalibrasi Warna==========================================//
bool endServer = false;
bool endClient = false;
bool clientExist = false;
int port = 5000;
Socket *clientSocket;
ServerSocket s(5000);
void SignCatch( int sig )
{
	switch ( sig )
	{
	case SIGINT:
		if( !end ) {
			std::cerr << "\n---\nSIGINT: signal received, stoping application.\n---" << std::endl;
			end = true;
			endServer = true;
			endClient = true;
		}
		break;
	default:
		std::cerr << "ERROR: Unknow signal received, continuing." << std::endl;
		break;
	}
}

void *processClient(void*){

	std::string x;
	std::string feedbackMsg;
	int size;
	unsigned int sizeToReceive;
	while(!endClient){
		if(clientExist){
			std::cerr << "Waiting for message... \n";
			*clientSocket >> x;
			std::cerr << "received : " << x << std::endl;
			if(x == "i")
			{
				cv::Mat scaledImage;

				image.copyTo(scaledImage);

				if(scaledImage.channels() == 1)
					cv::cvtColor(scaledImage, scaledImage, CV_BayerBG2RGB);

				if(!fullSize)
					cv::resize(scaledImage, scaledImage, cv::Size(), 0.5, 0.5);

				std::cerr << "Image Size" << scaledImage.cols << "x" <<scaledImage.rows <<  " continuous " << scaledImage.isContinuous() << " channels " << scaledImage.channels() << std::endl;
				sizeToSend = scaledImage.cols * scaledImage.rows * scaledImage.channels();
				size = clientSocket->sendAll((void*)scaledImage.ptr(), sizeToSend);
				std::cerr << "Send an image with " << size << "bytes\n";
			}
			else if(x == "m")
			{
				cv::Mat scaledImage;
				if(fullSize)
					config.mask.copyTo(scaledImage);
				else
					cv::resize(config.mask, scaledImage, cv::Size(), 0.5, 0.5);
				sizeToSend = scaledImage.cols * scaledImage.rows;
				size = clientSocket->sendAll((void*)scaledImage.ptr(), sizeToSend);
				std::cerr << "Send a mask with " << size << "bytes\n";
			}
			else if(x == "c")
			{
				size = clientSocket->sendAll((void*)config.colorRange, config.colorRange->getSize()*UAV_NCOLORS);
				std::cerr << "Send colors with " << size << "bytes\n";
			}
			else if(x == "x")
			{
				size = clientSocket->sendAll((void*)config.camSettings->ptr(), config.camSettings->getSize());
				std::cerr << "Send camera setting with " << size << "bytes\n";
				*clientSocket >> x;
				std::cerr << "received : " << x << std::endl;
				size = clientSocket->sendAll((void*)parametersRange, 10 * sizeof(ParameterRange));
				std::cerr << "Send parameters range with " << size << "bytes\n";
			}
			else if(x == "X")
			{
				*clientSocket << "ok";
				sizeToReceive = config.camSettings->getSize();
				size = clientSocket->recvAll((void*)config.camSettings->ptr(), &sizeToReceive);
				std::cerr << "Received camera settings " << sizeToReceive << " bytes" << std::endl;
				flagSaveFile = true;
			}
			else if(x == "M")
			{
				*clientSocket << "ok";
				sizeToReceive = image.cols * image.rows;
				size = clientSocket->recvAll((void*)config.mask.ptr(), &sizeToReceive);
				std::cerr << "Received mask " << sizeToReceive << " bytes" << std::endl;
				flagSaveFile = true;
			}
			else if(x == "C")
			{
				*clientSocket << "ok";
				sizeToReceive = config.colorRange->getSize() * UAV_NCOLORS;
				size = clientSocket->recvAll((void*)config.colorRange, &sizeToReceive);
				std::cerr << "Received colors " << sizeToReceive << " bytes" << std::endl;
				flagSaveFile = true;
				lut->createLUT(config.camSettings->getCameraSetting(UAV_VIDMODE), config.colorRange);
				std::cerr<< "LUT recreated" <<std::endl;
			}
			else if(x == "a")
			{
				//autoCalib = true;
				std::cerr << "Received AutoCalib flag\n";
			}
			else
			{
				std::cerr << "Some error, aborting...\n";
				endClient = true;
				clientExist = false;
			}

			if(size == -1)
			{
				endClient = true;
				clientExist = false;
			}
		}
	}

	std::cerr << "End of thread\n";
	return NULL;
}

void *processServer(void*){
	if(!s.open(port)){
		std::cerr << "Error opening the socket\n";
	}

	std::cerr << "Server mode: waiting for connections...";
	while(!endServer){
		clientSocket = s.accept();
		if(clientExist) continue;
		clientExist = true;
		endClient = false;
		std::cerr << "DONE\n";
		std:: string tmp;
		std::stringstream tmpss;
		if(fullSize)
		{
			tmpss << image.rows;
			tmpss << " ";
			tmpss << image.cols;
			tmpss << " ";
		}
		else
		{
			tmpss << image.rows / 2;
			tmpss << " ";
			tmpss << image.cols / 2;
			tmpss << " ";
		}
		tmpss << camType;
		tmp = tmpss.str();
		*clientSocket << tmp;
		std::cerr << "sent image data\n";
		pthread_t thrClient;
		pthread_create( &thrClient , NULL , processClient, NULL );
	}
	return NULL;
}

void *processLast(void*){
	last_ballPos = ballPos;
	usleep(10000);
	return NULL;
	//selisih = ballPos - last_ballPos;
}

/*void *autocalibration(void*){

	cv::Point robotCenter(config.camSettings->getCameraSetting(UAV_CENTERCOL), config.camSettings->getCameraSetting(UAV_CENTERROW));
	cv::Rect whiteRectRunTime(config.camSettings->getCameraSetting(UAV_NCOLS) - 100, 0, 50, 50);

	sleep(5);

	while(!end){
		usleep(20000);
		CameraCalib calib(5, image);
		runtimeCalibConverged = calib.CalibrateCamera(config.camSettings, parametersRange,
				config.mask, whiteRectRunTime, true, verbose, paramConverged, true);
	}

}*/

int main(int argc, char *argv[])
{
	struct timeval tvi, tvf;
	unsigned long timeElapsed;
	int displayMode = 1, ballColor = 2, ballBit = UAV_YELLOW_BIT,obsColor = 2,obsbit = UAV_MAGENTA_BIT;
	bool ncf = false, display = true, server = false, debug = false, loop = false,
			freese = false, slave = false, step = false, printHelpOnImage = false,
			screenshot = false; //runtimeAutocalib = true;
	int colorControl = 2;
	int waitTime;
	std::string loadFileName;
	std::string saveFileName;
	std::fstream saveFileStream;

	//paramConverged.resize(3);

	if( signal( SIGINT, SignCatch) == SIG_ERR ) {
		fprintf(stderr,"ERROR: couldn't install signal SIGINT\n");
		return 0;
	}

	// Default config file
	std::string configFileName, polCoeffFileName;
	char* environmentBody = getenv("AGENT");
	char confFileDefault[250];
	int agent;
	if (environmentBody == NULL) {
		std::cerr << "Vision: Error: The environment variable AGENT is not defined." << std::endl;
		configFileName.assign("f.conf");
	}
	else
	{
		agent = atoi(environmentBody);
		sprintf(confFileDefault,"../config/r%d.conf",agent);
		configFileName.assign(confFileDefault);
	}

	// Parse command line arguments
	for(int i = 1 ; i < argc ; i++)
	{
		if(strcmp(argv[i] , "-port") == 0)
			sscanf(argv[++i] , "%d" , &port);

		if(strcmp( argv[i] , "-cf") == 0)
			configFileName.assign(argv[i+1]);

		if(strcmp(argv[i] , "-ncf") == 0 )
			ncf = true;

		if(strcmp(argv[i] , "-cam") == 0 )
			sscanf(argv[i+1] , "%d" , &camType);

		if(strcmp(argv[i] , "-load") == 0 )
			loadFileName.assign(argv[i+1]);

		if(strcmp(argv[i] , "-save") == 0 )
			saveFileName.assign(argv[i+1]);

		if(strcmp(argv[i] , "-nodisp") == 0 )
			display = false;

		if(strcmp(argv[i] , "-server") == 0 )
			server = true;

		if(strcmp(argv[i] , "-udp") == 0 )
			udp = true;

		if(strcmp(argv[i] , "-slave") == 0 )
			slave = true;

		if(strcmp(argv[i] , "-debug") == 0 )
			debug = true;

		if(strcmp(argv[i] , "-loop") == 0 )
			loop = true;

		if(strcmp(argv[i] , "-fs") == 0 )
			fullSize = true;

		if(strcmp(argv[i] , "-v") == 0 )
			verbose = true;

		if(strcmp(argv[i] , "-tv") == 0 )
			timeInfo = true;

		if(strcmp(argv[i] , "-ball") == 0 )
		{
			sscanf(argv[i+1] , "%d" , &ballColor);
		}
		if(strcmp(argv[i] , "-team") == 0 )
		{
			sscanf(argv[i+1] , "%d" , &obsColor);
		}
		if(strcmp( argv[i] , "-p") == 0)
			polCoeffFileName.assign(argv[i+1]);

		/*if(strcmp( argv[i] , "-nocalib") == 0)
			runtimeAutocalib = false;*/

		if(strcmp(argv[i] , "-h") == 0 )
		{
			printHelp();
			return 0;
		}
	}

	// Config -----------------------------------------------------------------
	if(ncf)
	{
		std::cerr << "Creating a new config file " << configFileName << std::endl;
		config.createConfig(configFileName, camType, true, true, true, true, true);
	}
	else
	{
		std::cerr << "Loading the config file " << configFileName << std::endl;
		config.load(configFileName);
		if(verbose) config.print();
	}

	// Save a video file
	if(saveFileName.size() != 0)
	{
		std::cerr << "A video file will be saved " << saveFileName << std::endl;
		saveFileStream.open(saveFileName.c_str(), std::fstream::out | std::fstream::binary);
	}

	for(int i = 1 ; i < argc ; i++)
	{
		if(strcmp(argv[i] , "-bayer") == 0 )
			config.camSettings->setCameraSetting(UAV_VIDMODE, UAV_GRAY);

		if(strcmp(argv[i] , "-rgb") == 0 )
			config.camSettings->setCameraSetting(UAV_VIDMODE, UAV_RGB8);

		if(strcmp(argv[i] , "-fps") == 0 )
		{
			int fps;
			sscanf(argv[i+1] , "%d" , &fps);
			config.camSettings->setCameraSetting(UAV_FPS, fps);
		}

		if(strcmp(argv[i] , "-colorControl") == 0 )
		{
			sscanf(argv[i+1] , "%d" , &colorControl);
		}
	}

	// Camera -----------------------------------------------------------------
	std::cerr << "Initializing camera (or file) type " << config.internalData.cameraType << "...";
	Camera *cam;
	if(loadFileName.size() != 0)
	{
		cam = new CameraFromFile(loadFileName, config.camSettings);
		//runtimeAutocalib = false;
	}
	else
	{
		switch(config.internalData.cameraType)
		{
		case UAV_ETH:
			std::cerr << "CAMBADA uses a IDS Ethernet Camera with proprietary drivers.\n";
			std::cerr << "The user of the library can implement the own CameraEthernet based on the model used\n";
			return 1;
			break;

		case UAV_OPENCV:
			cam = new CameraOpenCV();
			cam->initCameraDepan(config.camSettings);
			break;
		}
		cam->getParametersRanges(parametersRange);
	}
	std::cerr << " DONE!"<< std::endl;

	// images --------------------------------------------------------------------
	std::cerr << "Creating images (cv::Mat)...";
	if(config.camSettings->getCameraSetting(UAV_VIDMODE) == UAV_GRAY)
	{
		image = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
				config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC1);
		originalImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
				config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC1);
	}
	else
	{
		image = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
				config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);
		originalImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
				config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);
	}

	cv::Mat idxImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC1);

	cv::Mat segImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);

	cv::Mat realImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);
	cv::Point robotCenter(config.camSettings->getCameraSetting(UAV_CENTERCOL), config.camSettings->getCameraSetting(UAV_NROWS));
	cv::Rect whiteRect(config.camSettings->getCameraSetting(UAV_NCOLS) - 100, 0, 50, 50);
	std::cerr << "DONE" << std::endl;

	// LUT --------------------------------------------------------------------
	tick();
	lut = new Lut(config);
	tack(true, "TIME: lut.init ");

	// Server -----------------------------------------------------------------
	if(server)
	{
		pthread_t thrServer;
		pthread_create( &thrServer , NULL , processServer, NULL );
		std::cerr << "Server calibration mode: accept thread created\n";
	}
	
	udp_client udpcl(ip_clien, prt_clien);
	std::cerr << "Client UDP mode: accept created\n";

	/*if(runtimeAutocalib)
	{
		pthread_t thrAutoCalib;
		pthread_create(&thrAutoCalib, NULL, autocalibration, NULL);
		std::cerr << "Runtime calib thread created\n";

	}*/

	// Main loop --------------------------------------------------------------
	tick();
	int fNumber = 1;
	if(display)
		cv::namedWindow( "Display Depan", CV_WINDOW_AUTOSIZE );

	//ScanLines linesRad(idxImage, UAV_RADIAL, robotCenter, 360, 50, config.camSettings->getCameraSetting(UAV_OUTRADIUS));
	//ScanLines linesRadNonBall(idxImage, UAV_RADIAL, robotCenter, 90, 80, config.camSettings->getCameraSetting(UAV_OUTRADIUS));
	ScanLines linesRadObs(idxImage, UAV_RADIAL, robotCenter, 360, 130, 180);
	//ScanLines linesCirc(idxImage, UAV_CIRCULAR, robotCenter, 0, 80, config.camSettings->getCameraSetting(UAV_OUTRADIUS), 4);
	//ScanLines linesCircWhite(idxImage, UAV_CIRCULAR, robotCenter, 0, 90, config.camSettings->getCameraSetting(UAV_OUTRADIUS), 12);
	ScanLines linesVer1Obs(idxImage, UAV_HORIZONTAL, cv::Point(160,230), cv::Point(480, 350), 1, 2);
	ScanLines linesVerGL(idxImage, UAV_VERTICAL, cv::Point(340,0), cv::Point(350,470), 1, 2);
	ScanLines linesVerGR(idxImage, UAV_VERTICAL, cv::Point(410,0), cv::Point(640, 100), 1, 2);
	ScanLines linesHorGH(idxImage, UAV_HORIZONTAL, cv::Point(250,100), cv::Point(390, 200), 1, 2);
	ScanLines linesCyan(idxImage, UAV_HORIZONTAL, cv::Point(0,50), cv::Point(640, 480), 1, 2);
	//ScanLines linesMagenta(idxImage, UAV_VERTICAL, cv::Point(0,0), cv::Point(640, 480), 1, 2);
	//ScanLines linesVer2Obs(idxImage, UAV_VERTICAL, cv::Point(340, 0), cv::Point(640, 480), 1, 2);
	ScanLines linesHor(idxImage, UAV_HORIZONTAL, cv::Point(0, 0), cv::Point(300, 250), 1, 2);
	//ScanLines linesVer(idxImage, UAV_VERTICAL, cv::Point(0, 0), cv::Point(640, 480), 2, 1);
	tack(true, "TIME: Scanlines created ");

	std::vector<float> ballBlobThr;
	distRelation(ballBlobThr,image, polCoeffFileName, true);
	std::cerr << "Ball radius data created\n";

	switch(ballColor) 
	{
	case 1:
		ballBit = UAV_ORANGE_BIT;
		break;
	case 2:
		ballBit = UAV_YELLOW_BIT;
		break;
	case 3:
		ballBit = UAV_MAGENTA_BIT;
		break;
	case 4:
		ballBit = UAV_CYAN_BIT;
		break;
	case 5:
		ballBit = UAV_BLUE_BIT;
		break;
	default:
		std::cerr<<"Ball color not correct!"<<std::endl;
		break;
	}


	switch(obsColor)
	{
	case 1:
		obsbit = UAV_CYAN_BIT;
		break;
	case 2:
		obsbit = UAV_MAGENTA_BIT;
		break;
	default:
		std::cerr<<"Obs color not correct!"<<std::endl;
		break;
	}
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	std::cerr << "Main loop will start...\n\n";
	while(!end)
	{
		gettimeofday(&tvi,NULL);

		tick();
		if(display)
		{
			realImage.setTo(cv::Scalar(0, 255, 0));
		}
		tack(timeInfo, "TIME: SetTo real image ");

		tick();
		if(freese && !step)
		{
			originalImage.copyTo(image);
		}
		else
		{
			if(cam->readFrame(image) != 0)
			{
				if(loadFileName.size() != 0 && loop)
				{
					cam->rewind();
					continue;
				}
				else
				{
					end = true;
					endServer = true;
					break;
				}
			}
		}

		if(loadFileName.size() > 0 && (!freese || step))
		{
			step = false;
			image.copyTo(originalImage);
		}
		tack(timeInfo, "TIME: Acquisition ");

		tick();
		lut->convertImageToIndex(image, idxImage, config.camSettings->getCameraSetting(UAV_VIDMODE));
		tack(timeInfo, "TIME: convertImageToIndex ");

		tick();
		// Save a new frame in a video file
		if(saveFileName.size() != 0)
		{
			if(fNumber % 3 == 0)
			{
				saveFileStream.write((char *)image.ptr(), image.cols * image.rows * image.channels());
				fNumber = 1;
			}

			fNumber++;
		}

		/*if(autoCalib && loadFileName.size() == 0)
		{
			if(image.channels() == 1)
				cv::cvtColor(image, image, CV_BayerBG2RGB);

			CameraCalib calib(5, image);
			autoCalib = !calib.CalibrateCamera(config.camSettings, parametersRange, config.mask,
					whiteRect, true, verbose, paramConverged, true);
			cam->setParameters(config.camSettings);
			flagSaveFile = true;
			std::cerr << "\nSelf calibration...\n";
		}*/

		if(!autoCalib && display && image.channels() == 1)
		{
			cv::cvtColor(image, image, CV_BayerBG2RGB);
		}
		tack(timeInfo, "TIME: Save file and autocalib in the beginning ");

		tick();
		//idxImage.copyTo(linesRad.image); 
		//idxImage.copyTo(linesCirc.image);
		//idxImage.copyTo(linesRadNonBall.image);
		//idxImage.copyTo(linesCircWhite.image);
		idxImage.copyTo(linesRadObs.image);
		idxImage.copyTo(linesVer1Obs.image);
		//idxImage.copyTo(linesVerGL.image);
		//idxImage.copyTo(linesVerGR.image);
		idxImage.copyTo(linesCyan.image);
		//idxImage.copyTo(linesMagenta.image);
		//idxImage.copyTo(linesHorGH.image);
		//idxImage.copyTo(linesVer2Obs.image);
		idxImage.copyTo(linesHor.image);
		//idxImage.copyTo(linesVer.image);
		tack(timeInfo, "TIME: scanlines ");

		tick();
		//RLE rleLinesRad(linesRadNonBall, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 2, 2, 10);
		//RLE rleLinesCirc(linesCircWhite, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 8, 2, 10);
		RLE rleObs(linesRadObs, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 1, 4, 1, 10, true);
		//RLE rleBallRad(linesRad, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 2, 8, 2, 10);
		//RLE rleBallCirc(linesRad, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 0, 0);
		RLE rleObsVer1(linesVer1Obs, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 0, 4, 0, 10);
		//RLE rleGR(linesVerGR, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleGL(linesVerGL, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		
		//RLE rleMagenta(linesMagenta, UAV_MAGENTA_BIT, UAV_MAGENTA_BIT, UAV_MAGENTA_BIT, 0, 4, 0, 20);
		//RLE rleGH(linesHorGH, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleObsVer2(linesVer, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 0, 4, 0, 10);
		RLE rleHor(linesHor, UAV_WHITE_BIT, UAV_WHITE_BIT, UAV_WHITE_BIT, 0, 4, 0, 10);
		//RLE rleBallVer(linesVer, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 0, 10);
		
		RLE rleCyan(linesCyan,obsbit, obsbit, obsbit, 0, 4, 0, 20);
		tack(timeInfo, "TIME: RLE ");

		tick();
		Blob ballDepan, ballOmni, ObsDepan,GRDepan,GLDepan,GHDepan,CYDepan,MGDepan,ball,kiper;
		kiper.createBlobs(rleHor, ballBlobThr, robotCenter);
		//ball.createBlobs(rleBallRad, ballBlobThr, robotCenter);
		//ballDepan.createBlobs(rleBallRad, ballBlobThr, robotCenter);
		ObsDepan.createBlobs(rleObsVer1, ballBlobThr, robotCenter);
		//GRDepan.createBlobs(rleGR, ballBlobThr, robotCenter);
		//GLDepan.createBlobs(rleGL, ballBlobThr, robotCenter);
		CYDepan.createBlobs(rleCyan, ballBlobThr, robotCenter);
		//MGDepan.createBlobs(rleMagenta, ballBlobThr, robotCenter);
		//GHDepan.createBlobs(rleGH, ballBlobThr, robotCenter);
		//ObsDepan.createBlobs(rleObsVer2, ballBlobThr, robotCenter);
		//ballOmni.createBlobs(rleBallRad, ballBlobThr, robotCenter);
		//Obs.createBlobs(rleObs, ballBlobThr, robotCenter);
		//ball.createBlobs(rleBallCirc, ballBlobThr, robotCenter);
		//ballDepan.sort(UAV_SORT_BY_DISTANCE, rleBallVer);
		//ballOmni.sort(UAV_SORT_BY_DISTANCE, rleBallRad);
		//Obs.sort(UAV_SORT_BY_DISTANCE, rleObs);
		tack(timeInfo, "TIME Blobs ");

		tick();
		// DEBUG stuff
		//linesRad.draw(image, cv::Scalar(0, 0, 255));
		//linesRadNonBall.draw(image, cv::Scalar(255, 0, 0));
		//linesRadObs.draw(image, cv::Scalar(0, 255, 0));
		//linesCirc.draw(image, cv::Scalar(0, 255, 0));
		//linesCircWhite.draw(image, cv::Scalar(0, 255, 0));
		//linesVerGR.draw(image, cv::Scalar(255, 255, 0));
		linesVerGL.draw(image, cv::Scalar(0, 255, 255));
		//linesCyan.draw(image, cv::Scalar(0, 255, 255));
		//linesMagenta.draw(image, cv::Scalar(0, 255, 255));
		//linesHorGH.draw(image, cv::Scalar(0, 255, 0));
		//linesVer1Obs.draw(image, cv::Scalar(0, 255, 0));
		//linesVer2Obs.draw(image, cv::Scalar(0, 255, 0));
		//linesHor.draw(image, cv::Scalar(255, 255, 0));
		//linesVer.draw(image, cv::Scalar(0, 255, 255));
		//ball.draw(cv::Scalar(255,255,255), image);
		//line(image, cv::Point(320,0), cv::Point(320, 480), cv::Scalar(255, 0, 0), 4, 2);
//===================================Deteksi Bola Depan=================================================//
		int nBallsDepan = 0, nBallsOmni = 0, nteam =0,nObs = 0,dist, dist1, dist2, dist3, dist4;
		cv::Point2d centerImage(image.cols / 2, image.rows);
		double A, cosA, prediksi, sinB, sudutB;
		if(ballDepan.blobs.size() > 0)
			statusCam = 1;
		else
			statusCam = 0;

		for(unsigned k = 0 ; k < ballDepan.blobs.size() ; k++){
			if(ballDepan.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(ballDepan.blobs[k].center % image.cols, ballDepan.blobs[k].center / image.cols);
			cv::Point2d pt2(image.cols / 2, image.rows);

			if(nBallsDepan >= MAX_BALLS) break;

			nBallsDepan++;

			if(display && debug)
				drawCircle(ballDepan.blobs[k].center, 10, cv::Scalar(255, 0, 255), image);
			dist = euclidean(pt1, pt2);
			ballPos = pt1; //Actual ballPos
			centerX = pt2; //Actual centerX
		}
		//===================================Deteksi Teman====================================================//
		int jarteamDep0, jarteamDep1, jarteamDep2;
		int statusteamdep;
		if(CYDepan.blobs.size() == 0)
			statusteamdep = 0;
		
		for(unsigned k = 0 ; k < CYDepan.blobs.size() ; k++){
			if(CYDepan.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(CYDepan.blobs[k].center % image.cols, CYDepan.blobs[k].center / image.cols);
			cv::Point2d pt2(image.cols / 2, image.rows);

			if(nteam >= MAX_team) break;

			nteam++;

			if(display && debug)
				drawCircle(CYDepan.blobs[k].center, 10, cv::Scalar(255, 0, 255), image);
			jarteamDep0 = euclidean(pt1, pt2);
			team = pt1; //Actual ballPos

			//centerX = pt2; //Actual centerX
		}
		if(nteam == 0)
		{
			team.y = 0;
			team.x = 0;
		}

		if(nteam == 0)
			statusteamdep = 0;
		else
			statusteamdep = 1;

		//=======
		//===================================Deteksi Halangan=================================================//
		int jarObsDep0, jarObsDep1, jarObsDep2;
		int statusObsdep;

		for(unsigned k = 0 ; k < ObsDepan.blobs.size() ; k++)
		{
			if(ObsDepan.blobs[k].nRle == 1)
				continue;
		
			if(nObs >= MAX_Obs) break;

			nObs++;
			//cv::Point2d pt2(Obs.blobs[2].center % image.cols, Obs.blobs[2].center / image.cols);


			if(nObs == 1){
				cv::Point2d pt0(ObsDepan.blobs[0].center % image.cols, ObsDepan.blobs[0].center / image.cols);
				obs0 = pt0;
				//statusObs = 1;
			}
			else if(nObs == 2){
				cv::Point2d pt1(ObsDepan.blobs[1].center % image.cols, ObsDepan.blobs[1].center / image.cols);
				obs1 = pt1;
				//statusObs = 2;
			}

			//std::cerr << "OBS size " << Obs.blobs[k].area << " w/h " << Obs.blobs[k].widhtHeightRel <<std::endl;

			//if(Obs.blobs[k].area  < 5000)
			//	continue;
			//if(nObs == 3)
			//	obs2 = pt2;
			//else
			//	cv::Point2d obs2(0,0);

			if(display && debug)
			{
				drawCircle(ObsDepan.blobs[k].center, 10, cv::Scalar(255, 0, 0), image);
			}

			//obsPos = pt1; //Actual ballPos	

			//obsPos = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
			
		}

		if(nObs == 0)
			statusObsdep = 0;
		else
			statusObsdep = 1;
		
		//==========Menghitung Jarak Obs==========//
		jarObsDep0 = euclidean(centerImage, obs0);;
		jarObsDep1 = euclidean(centerImage, obs1);

		if(jarObsDep0 < jarObsDep1){
			obs = obs0;
		}
		else{
			obs = obs1;
		}


		//jarObs2 = euclidean(centerImage, obs2);
		//std::cout << "nObs : " << nObs <<std::endl;
		//std::cout << "Jarak obs : " << jarObs0 << " " << jarObs1 << " " << jarObs2 <<std::endl;
		//std::cout << "obsPos : " << obs0 << " " << obs1 << " " << nObs <<std::endl;
//===================================Deteksi Garis Untuk Kalibrasi=================================================//
		int nGR=0, nGL=0, nGH=0;

		//==========Scen garis Kanan (R)============//
		for(unsigned k = 0 ; k < kiper.blobs.size() ; k++)
		{
			if(kiper.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(kiper.blobs[k].center % image.cols, kiper.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nGR >= 1) break;

			nGR++;

			if(display && debug)
			{
				drawCircle(kiper.blobs[k].center, 5, cv::Scalar(255,0,0), image);
			}	
			sRGpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
			
		}
		if(nGR == 0)
			sRGpos.y = 0;	
		//std::cout << "sRpos : " << sRpos.y << std::endl;

		//==========Scen garis Kiri (L)============//
		for(unsigned k = 0 ; k < GLDepan.blobs.size() ; k++)
		{
			if(GLDepan.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(GLDepan.blobs[k].center % image.cols, GLDepan.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nGL >= 1) break;

			nGL++;

			if(display && debug)
			{
				drawCircle(GLDepan.blobs[k].center, 5, cv::Scalar(255,0,0), image);
			}	
			sLGpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
		}

		if(nGL == 0)
			sLGpos.y = 0;
		//std::cout << "sLpos : " << sLpos.y << std::endl;

		//==========Scen garis Horizontal (H)============//
		for(unsigned k = 0 ; k < GHDepan.blobs.size() ; k++)
		{
			if(GHDepan.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(GHDepan.blobs[k].center % image.cols, GHDepan.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nGH >= 1) break;

			nGH++;

			if(display && debug)
			{
				drawCircle(GHDepan.blobs[k].center, 5, cv::Scalar(0,0,0), image);
			}	
			sHGpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
				
			//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
		}
		if(nGH == 0)
				sHGpos.x = 0;	
			//std::cout << "sHpos : " << sHpos.x << std::endl;
//=============================================================================================================================================		
		/*if(verbose)
		{

			//std::cerr << "Number of lines RLEs: " << rleLinesRad.rlData.size() << " + " << rleLinesCirc.rlData.size() << std::endl;
			//std::cerr << "Number of ball RLEs: " << rleBallRad.rlData.size() << " + " << rleBallCirc.rlData.size() << std::endl;
			std::cerr << "Number of obs RLEs: " << rleObs.rlData.size() << std::endl;
			std::cerr << "Number of BALLs: " << ball.blobs.size() << std::endl;
			for(unsigned k = 0 ; k < ball.blobs.size() ; k++)
			{
				if(ball.blobs[k].nRle == 1)
					continue;
				cv::Point2d pt = config.map[ball.blobs[k].center];
				Vec ballPos(pt.x, pt.y);
				float d = ballPos.length();
				float u = (0.12 / 0.7) * d; // ball radius and camera height
				Vec normalizedVec = ballPos.normalize();
				ballPos = normalizedVec.setLength(d - u);
				int dist = (int)(ballPos.length() * 100);
				if(dist >= ballBlobThr.size())
					continue;

				int expectedSize = pow(ballRadius.at(dist) * 2, 2);
				std::cout << "BALL " << k << " at dist " << dist << " should have " << ballRadius.at(dist);
				std::cout << " radius and " <<  expectedSize << " but has ";
				std::cout << ball.blobs[k].area << " pixels, " << ball.blobs[k]. widhtHeightRel;
				std::cout << " w/h, " << ball.blobs[k].nRle << " scanlines " << " solidity " << ball.blobs[k].solidity ;

				if(ball.blobs[k].area > 2.5 * expectedSize || ball.blobs[k].area < expectedSize / 4)
					std::cout << " NOTVALID for size\n";
				else if(dist > 150 && (ball.blobs[k].widhtHeightRel < 0.4 || ball.blobs[k].widhtHeightRel > 1.6))
					std::cout << " NOTVALID for widhtHeightRel\n";
				else
				{
					if(ball.blobs[k].solidity < 0.15)
						std::cout << " NOTVALID for solidity\n";
					else
						std::cout << " VALID\n";
				}
			}
		}

		if(display && debug)
		{
			//cv::rectangle(image, whiteRect, cv::Scalar(255,0,0), 1, 8, 0);
			cv::circle(realImage, robotCenter, 20, cv::Scalar(255, 0, 0), 3); // draw robot body
		}*/	
		tack(timeInfo, "TIME Validations, verbose and RtDB ");

		tick();
		if(display)
		{
			cv::Mat scaledImage;
			cv::Mat tmpImage;
			switch(displayMode)
			{
			case 1:
				image.copyTo(tmpImage);
				//cv::cvtColor(tmpImage, tmpImage, CV_RGB2BGR);
				if(screenshot) cv::imwrite("i1.png", tmpImage, compression_params);
				if(printHelpOnImage)
				{
					putText(tmpImage, "Keys usage:", cv::Point(0,30), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "a - camera auto-calib", cv::Point(0,60), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "f - freeze", cv::Point(0,90), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "g - go", cv::Point(0,120), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "p - pause", cv::Point(0,150), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "q - quit", cv::Point(0,180), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "1 - original image", cv::Point(0,210), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "2 - index image", cv::Point(0,240), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "3 - painted image", cv::Point(0,270), 1, 2, cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "4 - real image", cv::Point(0,300), 1 ,2 , cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "d - debug on real image", cv::Point(0,330), 1 ,2 , cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "s - save the current image on a file", cv::Point(0,360), 1 ,2 , cv::Scalar(147,20,255), 2, 8, false );
					putText(tmpImage, "u - update the configuration file", cv::Point(0,390), 1 ,2 , cv::Scalar(147,20,255), 2, 8, false );
				}
				if(fullSize)
					imshow( "Display Depan", tmpImage );
				else
				{
					cv::resize(tmpImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Depan", scaledImage);
				}
				break;
			case 2:
				if(screenshot) cv::imwrite("i2.png", idxImage, compression_params);
				if(fullSize)
					imshow( "Display Depan", idxImage );
				else
				{
					cv::resize(idxImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Depan", scaledImage);
				}
				break;
			case 3:
				segImage.setTo(cv::Scalar(255, 0, 127));
				PaintIndexImage(idxImage, segImage, config.mask);
				if(screenshot) cv::imwrite("i3.png", segImage, compression_params);
				if(fullSize)
					imshow( "Display Depan", segImage );
				else
				{
					cv::resize(segImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Depan", scaledImage);
				}
				break;
			case 4:
				if(screenshot) cv::imwrite("i4.png", realImage, compression_params);
				if(fullSize)
					imshow( "Display Depan", realImage );
				else
				{
					cv::resize(realImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Depan", scaledImage);
				}
				break;
			}
		}
		tack(timeInfo, "TIME: Display ");

		/*tick();
		if(runtimeAutocalib)
		{
			if(!runtimeCalibConverged)
			{
				if(paramConverged[0])
					cam->setParameter(UAV_GAIN, config.camSettings->getCameraSetting(UAV_GAIN));
				if(paramConverged[1])
					cam->setParameter(UAV_WBB, config.camSettings->getCameraSetting(UAV_WBB));
				if(paramConverged[2])
					cam->setParameter(UAV_WBR, config.camSettings->getCameraSetting(UAV_WBR));

			}
		}
		tack(timeInfo, "TIME: Runtime calib ");*/

		gettimeofday(&tvf,NULL);
		timeElapsed = (tvf.tv_sec*1000 + tvf.tv_usec / 1000) - (tvi.tv_sec*1000 + tvi.tv_usec / 1000);

		int fps = (int)(1.0 / config.camSettings->getCameraSetting(UAV_FPS) * 1000);
		if((fps - (int)timeElapsed) <= 0)
		{
			if(display)
				waitTime = 3;
			else
				waitTime = 0;
		}
		else
			waitTime = fps - timeElapsed;

		/*if(verbose || timeInfo)
			std::cerr << "Total time elapsed: " << (int)timeElapsed << " waiting " << waitTime << std::endl;
		else
			fprintf(stderr, "Total time elapsed: %3d  waiting %3d ", (int)timeElapsed, waitTime);*/

		int k = 0;
		if(display)
		{
			k = cv::waitKey(waitTime);
		}
		else
		{
			//k = cv::waitKey(waitTime);
			usleep(waitTime * 1000);
		}

		switch((char)k)
		{
		case 'q':
			end = true;
			endServer = true;
			statusCam = 0;
			break;

		case '1':
			displayMode = 1;
			break;

		case '2':
			displayMode = 2;
			break;

		case '3':
			displayMode = 3;
			break;

		case '4':
			displayMode = 4;
			break;

		case 'f':
			if(display) display = false;
			else display = true;
			break;

		case 'p':
			if(freese) freese = false;
			else freese = true;
			break;

		case 'd':
			if(debug) debug = false;
			else debug = true;
			break;

		case 's':
			if(screenshot) screenshot = false;
			else screenshot = true;
			break;

		case 'g':
			step = true;
			break;

		case 'a':
			autoCalib = true;
			break;

		case 'u':
			flagSaveFile = true;
			break;

		case 'h':
			if(printHelpOnImage)
				printHelpOnImage = false;
			else
				printHelpOnImage = true;
			break;
		}

		/*if (team.x < 317)
		{
			nTM = 1;
		}
		else if (team.x < 323 && team.x > 317)	 
		{
			nTM = 2;
		}
		else if (team.x > 323)
		{
			nTM = 3;
		}*/

		sprintf(senudp,"%i,%i,%i,%i,",team.x,team.y,statusteamdep,statusObsdep);
		std::cout << "senudp : " << senudp << std::endl;	
		if(udp){				
			udpcl.send(senudp,strlen(senudp));		
		}

		tick();
		if(flagSaveFile)
		{
			cam->setParameters(config.camSettings);
			config.save(configFileName);
			std::cerr << "Saved config \n";
			flagSaveFile = false;
		}
		tack(timeInfo, "TIME: save config ");

		/*gettimeofday(&tvf,NULL);
		timeElapsed = (tvf.tv_sec*1000 + tvf.tv_usec / 1000) - (tvi.tv_sec*1000 + tvi.tv_usec / 1000);
		if(verbose)
		{
			fprintf(stderr, "CYCLE time: %3ld\n\n", timeElapsed);
		}
		else
		{
			fprintf(stderr, "CYCLE time: %3ld\r", timeElapsed);
		}*/
	}
	std::cerr << std::endl;

	if(saveFileName.size() > 0)
		saveFileStream.close();

	std::cerr << "vision finished semangat :) \n";
}
