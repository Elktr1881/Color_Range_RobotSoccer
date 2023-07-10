#include <opencv2/opencv.hpp>
#include "UAVision.h"
#include "Config.h"
#include "Camera.h"
#include "Lut.h"
#include "RLE.h"
#include "Blob.h"
#include "libCalib.h"
#include "CameraKinect.h"
#include "CameraFromFile.h"
#include "CameraSettingsKinect.h"
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
#include "Angle.h"
#include "Vec.h"

/////////////
//VTK include
#include "vtkRenderer.h"
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleJoystickCamera.h>
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkRenderWindow.h"
#include "vtkAxesActor.h"

#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkCellArray.h"
#include "vtkPointData.h"
#include "vtkCaptionActor2D.h"
#include "vtkProperty2D.h"
#include "vtkTextProperty.h"

#include "vtkSphereSource.h"
#include "vtkCubeSource.h"
#include "vtkProperty.h"

#include "vtkTransform.h"

#include "vtkMath.h"

#define MAX_BALLS 1

using namespace uav;

cv::Mat image, originalImage, originalImageDepth;
unsigned int sizeToSend;
bool stop = false;
bool endServer = false;
bool endClient = false;
Socket *clientSocket;
ServerSocket s(5000);
Config config;
bool flagSaveFile = false;
bool clientExist = false;
bool fullSize = true;
bool verbose = false;
bool timeInfo = false;
bool runtimeCalibConverged = false;
std::vector<bool> paramConverged;
ParameterRange parametersRange[10];
Lut *lut;
int port = 5000;
int camType = UAV_KINECT;

int Display3D_firstFlag = 1;

using namespace std;

//==========================================Fungsi untuk Komunikasi UDP==========================================//
#include "udpclientserver.h"
using namespace udp_client_server;

//inisialisasi UDP
std::string ip_clien = "127.0.0.1";   //172.16.4.257  ip tujuan
int prt_clien = 1111;
bool udp = false;
char senudp[1024];
int statusCam = 0;
cv::Point ballPos(0,0), centerImg(320,480), last_ballPos(0,0), selisih(0,0), titik(0,0);

//==============================Fungsi unruk Menghitung Jarak==================================//
int euclidean(cv::Point2d awal, cv::Point2d akhir){
    cv::Point2d diff = akhir-awal;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

vtkTransform *RBT = vtkTransform::New();
/***************************************************/
/* Utility functions for reading calibration files*/
/* Should pass to utility library file            */

// Read transform coming from kinect Calibration
int read_RBT(vtkTransform *RBT,char *name)
{
	CvMat *tempsave  = cvCreateMat(4, 4,CV_32F);
	tempsave = (CvMat *) cvLoad(name);

	if (tempsave == 0)
		return 0;

	RBT->GetMatrix()->SetElement(0,0,CV_MAT_ELEM( *tempsave, float, 0, 0 ));
	RBT->GetMatrix()->SetElement(0,1,CV_MAT_ELEM( *tempsave, float, 0, 1 ));
	RBT->GetMatrix()->SetElement(0,2,CV_MAT_ELEM( *tempsave, float, 0, 2 ));
	RBT->GetMatrix()->SetElement(0,3,CV_MAT_ELEM( *tempsave, float, 0, 3 ));

	RBT->GetMatrix()->SetElement(1,0,CV_MAT_ELEM( *tempsave, float, 1, 0 ));
	RBT->GetMatrix()->SetElement(1,1,CV_MAT_ELEM( *tempsave, float, 1, 1 ));
	RBT->GetMatrix()->SetElement(1,2,CV_MAT_ELEM( *tempsave, float, 1, 2 ));
	RBT->GetMatrix()->SetElement(1,3,CV_MAT_ELEM( *tempsave, float, 1, 3 ));
	RBT->GetMatrix()->SetElement(2,0,CV_MAT_ELEM( *tempsave, float, 2, 0 ));
	RBT->GetMatrix()->SetElement(2,1,CV_MAT_ELEM( *tempsave, float, 2, 1 ));
	RBT->GetMatrix()->SetElement(2,2,CV_MAT_ELEM( *tempsave, float, 2, 2 ));
	RBT->GetMatrix()->SetElement(2,3,CV_MAT_ELEM( *tempsave, float, 2, 3 ));

	RBT->GetMatrix()->SetElement(3,0,CV_MAT_ELEM( *tempsave, float, 3, 0 ));
	RBT->GetMatrix()->SetElement(3,1,CV_MAT_ELEM( *tempsave, float, 3, 1 ));
	RBT->GetMatrix()->SetElement(3,2,CV_MAT_ELEM( *tempsave, float, 3, 2 ));
	RBT->GetMatrix()->SetElement(3,3,CV_MAT_ELEM( *tempsave, float, 3, 3 ));

	return 1;
}

/* Should pass to utility library file            */
/* Utility functions for reading calibration files*/
/***************************************************/

vector<int> depth2rgb;
vector<int> rgb2depth;
vector<double>rgb2XYZ[3];

// 3D coord of depth points (0,0,0) indicates no valid 3D coord
double coords[640*480*3];

/**************************************************************************************/
/* Functions and variables to compute 3D Cartesian coordinates form kinect depth image*/

#define MIN_DEPTH_DISTANCE 0.0
#define MAX_DEPTH_DISTANCE 8.0

#define DEPTH_RGB_CORRECTION_X 10.0
#define DEPTH_RGB_CORRECTION_Y 10.0


// conversion using formulas in
// http://tech.groups.yahoo.com/group/OpenCV/message/79866
// Now you have a lookup table for all possible depth values.
// The second step is to reproject the depth data into Euclidean space.
// uint16_t depth_data[640*480]; // depth data from Kinect in a 1-D array
// camera intrinsic parameters, representative values
// see http://nicolas.burrus.name/index.php/Research/KinectCalibration for more info

double cx = 320.0; // center of projection
double cy = 240.0; // center of projection
double fx = 600.0; // focal length in pixels
double fy = 600.0; // focal length in pixels

// Rotation matrix
double R[3][3] = { {0.99984628826577793, 	0.0012635359098409581,  -0.017487233004436643},
		{-0.0014779096108364480, 0.99992385683542895,    -0.012251380107679535},
		{0.017470421412464927,   0.012275341476520762,    0.99977202419716948} };

// Translation matrix
double T[3][1] = { {0.019985242312092553}, {-0.00074423738761617583}, {-0.010916736334336222} };

// Intrinsic parameters of RGB camera

double fx_rgb = 529.21508098293293;
double fy_rgb = 525.56393630057437;
double cx_rgb = 328.94272028759258;
double cy_rgb = 267.48068171871557;

// Intrinsic parameters of Depth camera
double fx_d = 594.21434211923247;
double fy_d = 591.04053696870778;
double cx_d = 339.30780975300314;
double cy_d = 242.73913761751615;

double k_gamma[2048];

//////////////////////////////////////////////////////
// return 1: Measurement OK
// return 0: No kinect measurement (value ==2047)

int raw_depth_to_XYZ(int r, int c, uint16_t raw_depth, double *x,double *y,double *z)
{
	double x3D,y3D,z3D;

	z3D = k_gamma[raw_depth];
	//z3D = raw_depth;
	x3D = (c - cx) * z3D / fx;
	y3D = (r - cy) * z3D / fy;

	if ( raw_depth < 1060)
	{
		*x = (x3D*R[0][0])+(y3D*R[0][1])+(z3D*R[0][2])+T[0][0];
		*y = (x3D*R[1][0])+(y3D*R[1][1])+(z3D*R[1][2])+T[1][0];
		*z = (x3D*R[2][0])+(y3D*R[2][1])+(z3D*R[2][2])+T[2][0];
		return 1;
	}

	return 0;
}

//////////////////////////////////////////////////////
void get_3D_coords(cv::Mat &depth,double *coords)
{
	int r,c,n;
	double nx,ny,nz;
	double original[4],transformed[4];
	n = 0;
	for (r = 0, n = 0; r< depth.rows;r++)
	{
		for(c = 0; c < depth.cols; c++, n++)
		{
			// depth to 3D
			if (raw_depth_to_XYZ(r,c,depth.ptr<uint16_t>()[n],&nx,&ny,&nz))
			{
				// Depth value from kinect OK
				original[0]=nx; original[1]=ny; original[2]=nz; original[3]=1;
				RBT->MultiplyPoint(original,transformed);

				coords[n*3]= transformed[0];
				coords[n*3+1]= transformed[1];
				coords[n*3+2]= transformed[2];
			}
			else
			{
				// Depth value from kinect not OK, coords = 0
				coords[n*3]= 0.0;
				coords[n*3+1]= 0.0;
				coords[n*3+2]= 0.0;
				//printf("\r%d",n);
			}
		}
	}
}

//////////////////////////////////////////////////////
void initConversions(cv::Mat &depth, cv::Mat &idxImage)
{
	// Use assign to ensure not used values to -1.0
	depth2rgb.assign(depth.cols * depth.rows,0.0);
	rgb2depth.assign(depth.cols * depth.rows,0.0);
	rgb2XYZ[0].assign(depth.cols * depth.rows,0.0);
	rgb2XYZ[1].assign(depth.cols * depth.rows,0.0);
	rgb2XYZ[2].assign(depth.cols * depth.rows,0.0);

	// Reset

	int r = 0;
	int c = 0;
	int n;

	double nx,ny,nz;
	double rgb_x,rgb_y;
	double original[4],transformed[4];

	for (r = 0, n = 0; r< depth.rows;r++)
	{
		for(c = 0; c < depth.cols; c++, n++)
		{
			// depth to 3D
			if (raw_depth_to_XYZ(r,c,depth.ptr<uint16_t>()[n],&nx,&ny,&nz))
			{
				// coordinate in RGB
				// DEPTH_RGB to compensate misalignment between RGB and depth
				rgb_x = (nx*fx_rgb/nz) + cx_rgb + DEPTH_RGB_CORRECTION_X;
				rgb_y = (ny*fy_rgb/nz) + cy_rgb + DEPTH_RGB_CORRECTION_Y;

				int x = (int)round(rgb_x);
				int y = (int)round(rgb_y);

				// conversion array depth3RGB and RGB2Depth + Transform to robot coordinates
				if(x >= 0 && x < idxImage.cols && y >= 0 && y < idxImage.rows)
				{
					depth2rgb[n] = y * idxImage.cols + x;
					rgb2depth[y * idxImage.cols + x] = r * depth.cols + c;

					// Transform to robot coord
					original[0]=nx; original[1]=ny; original[2]=nz; original[3]=1;
					RBT->MultiplyPoint(original,transformed);

					rgb2XYZ[0][y * idxImage.cols + x] = transformed[0];
					rgb2XYZ[1][y * idxImage.cols + x] = transformed[1];
					rgb2XYZ[2][y * idxImage.cols + x] = transformed[2];
				}
			}

			// filtering
			if (rgb2depth[n] == 0 )
				idxImage.ptr()[n] = UAV_BLUE_BIT;

			else if (rgb2XYZ[2][n] > MAX_DEPTH_DISTANCE)
				idxImage.ptr()[depth2rgb[n]] = UAV_BLUE_BIT;
		}
	}
}

/////////////////////////////////////////////////
void filtering(cv::Mat &depth, cv::Mat &idxImage)
{
	//double z3D;
	for (int p = 0; p < depth.rows * depth.cols ;p++)
	{
		//z3D = k_gamma[depth.ptr<uint16_t>()[p]];
		//z3D=coords[p];

		//if(z3D > MAX_DEPTH_DISTANCE)
		//if(depth.ptr<uint16_t>()[p] >= 2047 ||  coords[p*3+2]> MAX_DEPTH_DISTANCE)
		if (rgb2depth[p] == 0 )
			idxImage.ptr()[p] = UAV_BLUE_BIT;

		else if (coords[p*3+2]> MAX_DEPTH_DISTANCE)
			idxImage.ptr()[depth2rgb[p]] = UAV_BLUE_BIT;
	}
}


float raw_depth_to_meters(int raw_depth)
{
	if (raw_depth < 1050)
	{
		return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
	}
	return 1000;
}

void printHelp()
{
	std::cout << "---- CAMBADA vision ----" << std::endl;
	std::cout << "Command line parameters:" << std::endl;
	std::cout << "  -h (help)" << std::endl;
	std::cout << "  -nodisp (do not show image)" << std::endl;
	std::cout << "  -server (accept connections from imageCalib app)" << std::endl;
	std::cout << "  -cf # (name of the configuration file)" << std::endl;
	std::cout << "  -debug (shows visual debug information on the image)" << std::endl;
	std::cout << "  -v (verbose)" << std::endl;
	std::cout << "  -tv (time verbose)" << std::endl;
	std::cout << "  -cam # (camera type 0 - eth; 1 - opencv; 2 - firewire;) used to create a new config" << std::endl;
	std::cout << "  -port # (port number when in server mode)" << std::endl;
	std::cout << "  -load # (load a video file)" << std::endl;
	std::cout << "  -save # (save a video file)" << std::endl;
	std::cout << "  -loop (play video file in loop mode)" << std::endl;
	std::cout << "  -fs (full size display)" << std::endl;
	std::cout << "  -slave (vision does not do PMAN_tick so that we can use cursor)" << std::endl;
	std::cout << "  -ball # (ball color configured in visionCalib 1 - orange 2 - yellow 3 - magenta 4 -cyan 5-blue; if nothing is specified, the orange ball is used by default)" << std::endl;
	std::cout << "  -bayer (access bayer data from camera)" << std::endl;
	std::cout << "  -rgb (access RGB data from camera)" << std::endl;
	std::cout << "  -mbc (force saving in RGB mode)" << std::endl;
	std::cout << "  -fps # (set a new falue for fps)" << std::endl;
	std::cout << "  -colorControl # (set a new falue for colorControl 0 - disabled 1 - normal 2 - HQ)" << std::endl;
}

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

void SignCatch( int sig )
{
	switch ( sig )
	{
	case SIGINT:
		if( !stop ) {
			std::cerr << "\n---\nSIGINT: signal received, stoping application.\n---" << std::endl;
			stop = true;
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
}

int main(int argc, char *argv[])
{
	system("sudo chrt -f -p 99 $(pidof kinectVision)");

	std::vector<unsigned long> timeVector;
	struct timeval tvi, tvf, tvp;
	unsigned long timeElapsed;
	int displayMode = 1, ballColor = 2, ballBit = UAV_YELLOW_BIT;
	bool ncf = false, display = true, server = false, debug = false, loop = false,
			freese = false, slave = false, step = false, printHelpOnImage = false,
			screenshot = false, mbcFlag =  false;
	int waitTime;
	std::string loadFileName;
	std::string saveFileName;
	std::fstream fileStreamDepth, fileStreamRGB;

	//////////////////////////////////
	// Kinect Calibration File loading
	char kinectCalibFileName[128] = "../config/KinectOnBoard.xml";
	//char *kinectCalibFileName = "KinectOnKinect.xml";
	// Read Transform kinect to robot if not assume identity
	if (!read_RBT(RBT,kinectCalibFileName))
	{
		fprintf(stderr,"\nCannot read %s file, assuming Identity transform",kinectCalibFileName);
		RBT->Identity();
	}
	else
		printf("\nRead %s file for kinect2robot transform",kinectCalibFileName);

	const double k1 = 1.1863;
	const double k2 = 2842.5;
	const double k3 = 0.1236;

	// need to be done only once
	for (size_t i=0; i<2048; i++)
		k_gamma[i] = k3 * tan(i/k2 + k1);
	// LookUp Table creation from depth conversion
	//////////////////////////////////////////////

	paramConverged.resize(3);

	if( signal( SIGINT, SignCatch) == SIG_ERR ) {
		fprintf(stderr,"ERROR: couldn't install signal SIGINT\n");
		return 0;
	}

	// Default config file
	std::string configFileName = "front.conf";

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

		if(strcmp(argv[i] , "-mbc") == 0 )
			mbcFlag =true;

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

		if(strcmp(argv[i] , "-v") == 0 )
			verbose = true;

		if(strcmp(argv[i] , "-tv") == 0 )
			timeInfo = true;

		if(strcmp(argv[i] , "-ball") == 0 )
		{
			sscanf(argv[i+1] , "%d" , &ballColor);
		}

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
		fileStreamRGB.open((saveFileName+".rgb").c_str(), std::fstream::out | std::fstream::binary);
		fileStreamDepth.open((saveFileName+".depth").c_str(), std::fstream::out | std::fstream::binary);
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
	}

	// Camera -----------------------------------------------------------------
	std::cerr << "Initializing camera (org file) type " << config.internalData.cameraType << "...";
	CameraKinect *cam;
	if(loadFileName.size() != 0)
	{
		fileStreamRGB.open((loadFileName+".rgb").c_str(), std::fstream::in | std::fstream::binary);
		fileStreamDepth.open((loadFileName+".depth").c_str(), std::fstream::in | std::fstream::binary);
	}
	else
	{
		cam = new CameraKinect();
		cam->initCameraKinect(config.camSettings);
		cam->getParametersRanges(parametersRange);
	}
	std::cerr << " DONE!"<< std::endl;

	// images --------------------------------------------------------------------
	std::cerr << "Creating images (cv::Mat)...";
	image = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);
	originalImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);
	originalImageDepth = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_16UC1);

	cv::Mat idxImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC1);

	cv::Mat segImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_8UC3);

	cv::Mat depthImage = cv::Mat(cv::Size(config.camSettings->getCameraSetting(UAV_NCOLS),
			config.camSettings->getCameraSetting(UAV_NROWS)),CV_16UC1);

	cv::Point robotCenter(config.camSettings->getCameraSetting(UAV_CENTERCOL), config.camSettings->getCameraSetting(UAV_CENTERROW));
	std::cerr << "DONE" << std::endl;

	// Server -----------------------------------------------------------------
	if(server)
	{
		pthread_t thrServer;
		pthread_create( &thrServer , NULL , processServer, NULL );
		std::cerr << "Server mode: accept thread created\n";
	}

	udp_client udpcl(ip_clien, prt_clien);
	std::cerr << "Client UDP mode: accept created\n";

	int fNumber = 1;
	if(display)
	{
		cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
	}

	tick();
	//ScanLines linesHor(idxImage, UAV_HORIZONTAL, cv::Point(50, 40), cv::Point(610, 370), 2, 2);
	ScanLines linesHor(idxImage, UAV_HORIZONTAL, cv::Point(0, 0), cv::Point(640, 480), 2, 2);
	ScanLines linesVer(idxImage, UAV_VERTICAL, cv::Point(0, 0), cv::Point(640, 480), 2, 2);
	tack(true, "TIME: Scanlines created ");

	// LUT --------------------------------------------------------------------
	tick();
	lut = new Lut(config, linesHor);
	tack(true, "TIME: lut.init ");

	std::vector<float> ballBlobThr;
	distRelationGoalie(ballBlobThr,image, 100);
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

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	std::cerr << "Main loop will start...\n\n";
	while(!stop)
	{
		gettimeofday(&tvi,NULL);

		tick();
		if(freese && !step)
		{
			originalImage.copyTo(image);
		}
		else
		{
			int retVal = 0;

			if(loadFileName.size() != 0)
			{
				fileStreamRGB.read((char *)image.ptr(), image.cols * image.rows * image.channels());
				fileStreamDepth.read((char *)depthImage.ptr(), depthImage.cols * depthImage.rows * 2);
				if(!fileStreamRGB || !fileStreamDepth) retVal = 1;
			}
			else
			{
				retVal = cam->readFrame(image);
				while(!cam->readFrameDepth(depthImage));
			}

			if( retVal != 0) // an error ocurred
			{
				if(loadFileName.size() != 0 && loop)
				{
					fileStreamRGB.close();
					fileStreamDepth.close();
					fileStreamRGB.open((loadFileName+".rgb").c_str(), std::fstream::in | std::fstream::binary);
					fileStreamDepth.open((loadFileName+".depth").c_str(), std::fstream::in | std::fstream::binary);
					continue;
				}
				else
				{
					stop = true;
					endServer = true;
					break;
				}
			}
		}

		if(loadFileName.size() > 0 && (!freese || step))
		{
			step = false;
			image.copyTo(originalImage);
			depthImage.copyTo(originalImageDepth);
		}
		tack(timeInfo, "TIME: Acquisition ");

		gettimeofday(&tvp,NULL); // Start measuring processing time

		tick();
		lut->convertImageToIndex(image, idxImage, config.camSettings->getCameraSetting(UAV_VIDMODE));
		tack(timeInfo, "TIME: convertImageToIndex ");

		tick();
		initConversions(depthImage, idxImage);
		tack(timeInfo, "TIME: Filtering ");
		tick();
		// Save a new frame in a video file
		if(saveFileName.size() != 0)
		{
			fileStreamRGB.write((char *)image.ptr(), image.cols * image.rows * image.channels());
			fileStreamDepth.write((char *)depthImage.ptr(), depthImage.cols * depthImage.rows * 2);
		}

		cv::Mat I;
		depthImage.convertTo(I, CV_8UC1, 255.0 / 4096.0);

		tick();
		RLE rleBallHor(linesHor, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 2, 10);
		RLE rleBallVer(linesVer, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 2, 10);
		tack(timeInfo, "TIME: RLE ");

		tick();
		Blob ball;
		ball.createBlobs(rleBallHor, ballBlobThr, robotCenter);
		ball.createBlobs(rleBallVer, ballBlobThr, robotCenter);
		//ball.sort(UAV_SORT_BY_SIZE, rleBallHor);
		tack(timeInfo, "TIME: Blobs ");

		tick();

		//linesHor.draw(image, cv::Scalar(255, 0, 0));
		//linesVer.draw(image, cv::Scalar(0, 255, 0));

		//===================================Deteksi Bola Depan=================================================//
		int nBallsDepan = 0, nBallsOmni = 0, nObs = 0,dist, dist1, dist2, dist3, dist4, lamaPre, last;		
		double A, cosA, prediksi, sinB, sudutB, height;
		lamaPre++; last++;

		if(ball.blobs.size() > 0)
			statusCam = 1;
		else
			statusCam = 0;
		// PMD to view depth and compute 3D coords
		//get_3D_coords(depthImage,coords);
		// PMD
		int centerx=0,centery=0;
		for(unsigned k = 0 ; k < ball.blobs.size() ; k++)
		{
			if(ball.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt = rleBallHor.getPointXYFromInteger(ball.blobs[k].center);
			ballPos = pt;
			//std::cerr << "pt : " << pt << std::endl;

			if(nBallsDepan >= MAX_BALLS) break;

			nBallsDepan++;

			if(display && debug)
			{
				drawCircle(ball.blobs[k].center, 20, cv::Scalar(255, 255, 0), image);
			}

			if(verbose)
				std::cerr << "BALL size " << ball.blobs[k].area << " w/h " << ball.blobs[k].widhtHeightRel;

			if(ball.blobs[k].area  < 100)
				continue;

			if(ball.blobs[k].widhtHeightRel < 0.3 || ball.blobs[k].widhtHeightRel > 1.9)
				continue;

			if(display && debug)
			{
				drawCircle(ball.blobs[k].center, 10, cv::Scalar(255, 0, 255), image);
				cv::circle(I, cv::Point(rgb2depth[ball.blobs[k].center]%depthImage.cols,
						rgb2depth[ball.blobs[k].center]/depthImage.cols), 20, cv::Scalar(32000, 32000, 32000), 3, 8, 0);

				cv::circle(depthImage, cv::Point(rgb2depth[ball.blobs[k].center]%depthImage.cols,
						rgb2depth[ball.blobs[k].center]/depthImage.cols), 20, cv::Scalar(32000, 32000, 32000), 3, 8, 0);
			}

			centerx = ball.blobs[k].center%idxImage.cols;
			centery = ball.blobs[k].center/idxImage.cols;

			////////////////////////////////
			// Processing for detected ball PMD
			// PMD test larger distance aroung point
			// ball_centre_coord3D_robot !=0 mean valid 3D coord
			double ball_centre_coord3D_robot[3] = {0,0,0};

			if (centerx != 0 && rgb2XYZ[2][centery*idxImage.cols+centerx]!=0 && rgb2XYZ[2][centery*idxImage.cols+centerx]<MAX_DEPTH_DISTANCE)
			{
				ball_centre_coord3D_robot[0] = rgb2XYZ[0][centery*idxImage.cols+centerx];
				ball_centre_coord3D_robot[1] = rgb2XYZ[1][centery*idxImage.cols+centerx];
				ball_centre_coord3D_robot[2] = rgb2XYZ[2][centery*idxImage.cols+centerx];
			}

			//position = Vec(ball_centre_coord3D_robot[0], ball_centre_coord3D_robot[1]);
			//height = ball_centre_coord3D_robot[2];

			//std::cerr << "X : " << ball_centre_coord3D_robot[0] << "\tY : " << ball_centre_coord3D_robot[1] << "\tZ : " << ball_centre_coord3D_robot[2] << std::endl;
			//std::cerr << "position : " << position << std::endl;

			if(verbose)
			{
				std::cerr << " (x3, y3, z3) " << ball_centre_coord3D_robot[0] << " , ";
				std::cerr << ball_centre_coord3D_robot[1] << " , " << ball_centre_coord3D_robot[2];
				std::cerr << " VALID\n";
			}
		}
		tack(timeInfo, "TIME: Validation ");

		//std::cout << "ballPos : " << ballPos << "\tHeight : " << height << "\tprediksi : " << prediksi << std::endl;
		//std::cout << "ballPos : " << ballPos << std::endl;
		sprintf(senudp,"%i,%i,0,%i,", (int)ballPos.x, (int)ballPos.y, (int)statusCam);
		std::cout << "senudp : " << senudp << std::endl;	
		if(udp){				
			udpcl.send(senudp, strlen(senudp));		
		}

		tick();
		if(display)
		{
			cv::Mat scaledImage;
			cv::Mat tmpImage;
			switch(displayMode)
			{
			case 1:
				image.copyTo(tmpImage);
				cv::cvtColor(tmpImage, tmpImage, CV_RGB2BGR);
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
					imshow( "Display window", tmpImage );
				else
				{
					cv::resize(tmpImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display window", scaledImage);
				}
				break;
			case 2:
				if(screenshot) cv::imwrite("i2.png", idxImage, compression_params);
				if(fullSize)
					imshow( "Display window", idxImage );
				else
				{
					cv::resize(idxImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display window", scaledImage);
				}
				break;
			case 3:
				segImage.setTo(cv::Scalar(0, 255, 255));
				PaintIndexImage(idxImage, segImage, config.mask);
				if(screenshot) cv::imwrite("i3.png", segImage, compression_params);
				if(fullSize)
					imshow( "Display window", segImage );
				else
				{
					cv::resize(segImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display window", scaledImage);
				}
				break;
			case 4:
				if(screenshot) cv::imwrite("i4.png", depthImage, compression_params);
				if(fullSize)
					imshow( "Display window", depthImage );
				else
				{
					cv::resize(depthImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display window", scaledImage);
				}
				break;
			}
		}
		tack(timeInfo, "TIME: Display ");

		if(display)
			waitTime = 3;

		int k = 0;
		if(display)
		{
			k = cv::waitKey(3);
		}

		switch((char)k)
		{
		case 'q':
			stop = true;
			endServer = true;
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

		tick();
		if(flagSaveFile)
		{
			config.save(configFileName);
			std::cerr << "Saved config \n";
			flagSaveFile = false;
		}
		tack(timeInfo, "TIME: save config ");


		gettimeofday(&tvf,NULL);
		unsigned long timeProcessing;
		timeProcessing = (tvf.tv_sec*1000000 + tvf.tv_usec) - (tvp.tv_sec*1000000 + tvp.tv_usec);
		timeElapsed = (tvf.tv_sec*1000 + tvf.tv_usec / 1000) - (tvi.tv_sec*1000 + tvi.tv_usec / 1000);

		/*if(verbose || timeInfo)
			std::cerr << "Total time proc. " << (int)timeProcessing << " elapsed: " << (int)timeElapsed << std::endl;
		else
			timeVector.push_back(timeProcessing);*/

	}
	std::cerr << std::endl;

	if(saveFileName.size() > 0 || loadFileName.size() > 0)
	{
		fileStreamRGB.close();
		fileStreamDepth.close();
	}

	if(!verbose && !timeInfo)
	{
		for(int i = 0 ; i < timeVector.size() ; i++)
			std::cerr << "Total time proc. " << timeVector[i] << std::endl;
	}
	std::cerr << "vision finished\n";
}
