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
//==================battery status======================//
#include <iomanip>
#include <fstream>
int bat = 0, x;
#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
using namespace uav;
using namespace std;

//#define MAX_POINTS 500
#define MAX_BALLS 1
#define MAX_Obs 2
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
bool camOmni = false;
bool lastCamOmni = false;
bool camDepan = false;

cv::Point ballPos(0,0), ballPosOmni(0,0), obsPos(0,0) , CENTERR1(0,0);// centerImage(0,0); //last_ballPos(0,0), selisih(0,0), titik(0,0);
cv::Point obs0(0,0), obs1(0,0), obs(0,0);
cv::Point sRpos(0,0), sLpos(0,0), sHRpos(0,0), sHLpos(0,0), sGLpos(0,0), sRDXpos(0,0);
cv::Point team0(0,0), team1(0,0), team(0,0);
//==========================================Fungsi untuk Komunikasi UDP==========================================//
#include "udpclientserver.h"
using namespace udp_client_server;

//inisialisasi UDP
std::string ip_clien = "192.168.0.101";   //172.16.4.257  ip tujuan basetations
std::string ip_server = "192.168.0.103";		//"192.0.0.254";  //172.16.5.135  ip sendiri
std::string ip_serverCam = "127.0.0.1";  	//172.16.5.135  ip sendiri / ip buat penghubung kamera 

int prt_clien = 1122; // R3
int prt_serverCam = 1111;
int prt_serverBase = 1133;
int prt_serverDataRobot = 1142;
//udp_server udpser(ip_server, prt_server);
//udp_client udpcl(ip_clien, prt_clien);
bool udp = false;
char dataCamDepan[50];
char dataBase[50];
char dataRobot[50];
//char triudp[50]={'0',',','0'};
//char triudp2[20]={'0',',','0'};
//char triudpa[20];
char senudp[1024];
bool START = false;
int STATUS = 0;
int STATUS2 = 0;
char dataBaseK[50];
char dataBaseP[50];
char sendBasep[50];
char sendBaseK[50];
char sendKiperK[50];
char sendPenyerang[50];
bool tunda = false;

int x_poslap = 0;
int y_poslap = 0;
int sudutlap = 0;
char karakter[2] = {'o','o'}; 
char kar2; 
char d_rbt3[2] = {'x','x'};
int flagst = 0;

/*char** splitCam(char* a_str, const char a_delim)  // fungsi sub program untuk spilt data dari usart berupa char
{
	char** result    = 0;
	size_t count     = 0;
	char* tmp        = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;
	while(*tmp)
	{
		if (a_delim == *tmp)
		{
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	count += last_comma < (a_str + strlen(a_str) - 1);
	count++;

	result = (char**)malloc(sizeof(char*) * count);

	if (result)
	{
		size_t idx  = 0;
		char* token = strtok(a_str, delim);

		while (token)
		{
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		 *(result + idx) = 0;
	}

	return result;
}

char** splitBase(char* a_str, const char a_delim)  // fungsi sub program untuk spilt data dari usart berupa char
{
	char** result    = 0;
	size_t count     = 0;
	char* tmp        = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = a_delim;
	delim[1] = 0;
	while(*tmp)
	{
		if (a_delim == *tmp)
		{
			count++;
			last_comma = tmp;
		}
		tmp++;
	}

	count += last_comma < (a_str + strlen(a_str) - 1);
	count++;

	result = (char**)malloc(sizeof(char*) * count);

	if (result)
	{
		size_t idx  = 0;
		char* token = strtok(a_str, delim);

		while (token)
		{
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count - 1);
		 *(result + idx) = 0;
	}

	return result;
}
*/
/*void *terima_udp(void*){
	while (true)
	{
		for(int i=0; i<50; i++)
		{
			triudp[i]=NULL;
		}
		//udpser.recv(triudp, 50);		

		char** tokens;
		tokens = str_split(triudp,',');
		if (tokens)
		    {
		        int i;
		        for (i = 0; *(tokens + i); i++)
		        {
		        	
		        	if(i==0)
		            {
		            	x_poslap		= atoi(*(tokens + i));
		            }
		            else if(i==1)
		            {
		            	y_poslap		= atoi(*(tokens + i));
		            }
		            else if(i==2)
		            {
		            	sudutlap		= atoi(*(tokens + i));
		            }
		            else if(i==3)
		            {
		            	sprintf(karakter,"%s",*(tokens + i));
		            }
		            

		            free(*(tokens + i));
		        }
		        free(tokens);
		    }
		    kar2 = karakter[0];

			std::cerr <<"terima udp: "<< triudp<<"    "<<x_poslap<<"    "<<y_poslap<<"   "<<sudutlap<<"   "<<karakter[0]<<"   "<<std::endl;

		if (kar2 == '1') set_udp = 1; //maju
		else if (kar2 == '2') set_udp = 2; //mundur
		else if (kar2 == '3') set_udp = 3; //kiri
		else if (kar2 == '4') set_udp = 4; //kanan
		else if (kar2 == '5') set_udp = 5; //pivot kanan
		else if (kar2 == '6') set_udp = 6; //pivot kiri
		else if (kar2 == '7') set_udp = 7; //reset gyro
		else if (kar2 == '8') set_udp = 8; //tendang
		else if (kar2 == 'x' && STATUS == 0) {MODE = 2; flagst = 1;}

		if (CYAN)
		{
			if (kar2 == 's' && !tunda && flagst == 0) START = true;
			else if (kar2 == 's' && tunda && flagst == 0)
			{
				//for(int i=0;i<1000000;i++){ }
				sleep(5);
				START = true;
			}
			else if (kar2 == 'k') {STATUS = 2; tunda = true; flagst = 0;}
			else if (kar2 == 'f') {STATUS = 2; tunda = true; flagst = 0;}
			else if (kar2 == 't') {STATUS = 4; tunda = true; flagst = 0;}
			else if (kar2 == 'c') {STATUS = 1; tunda = true; flagst = 0;}
			else if (kar2 == 'p') {STATUS = 6; tunda = true; flagst = 0;}
			else if (kar2 == 'g') {STATUS = 6; tunda = true; flagst = 0;}
			else if (kar2 == 'K') {STATUS = 1; flagst = 0;}
			else if (kar2 == 'F') {STATUS = 2; flagst = 0;}
			else if (kar2 == 'T') {STATUS = 4; flagst = 0;}
			else if (kar2 == 'C') {STATUS = 1; flagst = 0;}
			else if (kar2 == 'P') {STATUS = 6; flagst = 0;}
			else if (kar2 == 'G') {STATUS = 6; flagst = 0;}
		}
		else if (MAGENTA)
		{
			if ((kar2 == 's' ) && tunda == false && flagst == 0) START = true;
			else if ((kar2 == 's') && tunda == true && flagst == 0)
			{
				//for(unsigned int i=0;i<10000;i++){sleep(1000);}
				sleep(5);
				START = true;
			}
			else if (kar2 == 'K') {STATUS = 2; tunda = true; flagst = 0;}
			else if (kar2 == 'F') {STATUS = 2; tunda = true; flagst = 0;}
			else if (kar2 == 'T') {STATUS = 4; tunda = true; flagst = 0;}
			else if (kar2 == 'C') {STATUS = 1; tunda = true; flagst = 0;}
			else if (kar2 == 'P') {STATUS = 6; tunda = true; flagst = 0;}
			else if (kar2 == 'G') {STATUS = 6; tunda = true; flagst = 0;}
			else if (kar2 == 'k') {STATUS = 1; flagst = 0;}
			else if (kar2 == 'f') {STATUS = 2; flagst = 0;}
			else if (kar2 == 't') {STATUS = 4; flagst = 0;}
			else if (kar2 == 'c') {STATUS = 1; flagst = 0;}
			else if (kar2 == 'p') {STATUS = 6; flagst = 0;}
			else if (kar2 == 'g') {STATUS = 6; flagst = 0;}
		}


		if (kar2 == 'N') STATUS = 2;
		else if (kar2 == 'o' || karakter[0] == 'a' || karakter[0] == 'A') STATUS = 8;
		

		//if(START && STATUS == 1) MODE = 1;
		if(START && STATUS !=8) MODE = 1;
		else if(!START && STATUS == 1) MODE = 3;
		else if(!START && STATUS == 2) MODE = 2;
		else if(!START && STATUS == 3) MODE = 2;
		else if(!START && STATUS == 4) MODE = 2;
		else if(!START && STATUS == 5) MODE = 2;
		else if(!START && STATUS == 6) MODE = 2;
		else if(!START && STATUS == 7) MODE = 2;
		else if(STATUS == 8) {MODE = 0; START =false; tunda =false; STATUS = 0;}


		for(int i=0; i<20; i++)
		{
			triudp2[i]=NULL;
		}
	}
}*/

//===========Komunikasi UDP Untuk Kamera Depan===========//
int  teamx, teamy, ballPosDepanX, ballPosDepanY, prediksi, statustemandepn,statusobsdep;
void *udp_Cam(void*){	
	udp_server udpCamDepan(ip_serverCam, prt_serverCam);
while(true){
		udpCamDepan.recv(dataBaseK, 1024);
		//std::cout << "Data Base = " << dataBaseK << std::endl;
		std::vector<string> result;
	    boost::split(result, dataBaseK, boost::is_any_of(",")); 
	  
	    for (int i = 0; i < result.size(); i++){
	    	teamx		        = atoi(result[0].c_str());
	    	teamy		        = atoi(result[1].c_str());
	    	statustemandepn		= atoi(result[2].c_str());
	    	statusobsdep		= atoi(result[3].c_str());
	    	//ditkipx		    = atoi(result[4].c_str());
	    	//ditkipy		    = atoi(result[5].c_str());

	    }
		 //std::cout << "Data Base : " << teamx << " " << teamy << " " <<statustemandepn << std::endl;
	}
}

//===========Komunikasi UDP Untuk Base===========//
int botX, botY, angel, kondisi, kr, dumX, dumY, sudut, potA, potB;
//char* kondisi[2];
//char kar;
void *udp_Base(void*){	
	udp_server udpBase(ip_server, prt_serverBase);

	while(true){
		udpBase.recv(dataBaseP, 50);
		std::cout << "Data Base = " << dataBase << std::endl;
		std::vector<string> result;
	    boost::split(result, dataBaseP, boost::is_any_of(",")); 
	    for (int i = 0; i < result.size(); i++){
	    	botX		= atoi(result[0].c_str());
	    	botY		= atoi(result[1].c_str());
	    	angel   	= atoi(result[2].c_str());
	    	kondisi		= atoi(result[3].c_str());
	    	kr			= atoi(result[4].c_str());
	    	dumX		= atoi(result[5].c_str());
	    	dumY		= atoi(result[6].c_str());
	    	/*sudut		= atoi(result[0].c_str());
	    	potA		= atoi(result[1].c_str());
	    	potB		= atoi(result[2].c_str());*/
	    }
	}
}

void *udp_DataRobot(void*){
	udp_server udpDataRobot(ip_server, prt_serverDataRobot);

	while(true){

	}
}

//==========================================Fungsi untuk Komunikasi Serial==========================================//
int USB;
char trimser[1024];	//buffer terima serial
char kirser[1024]; 	//buffer kirim serial
bool komser = false;
int data1,data2,data3;
int Radius,robotX,robotY,Sudut,statusbola,ballX,ballY,st_IR,st_kal,to_robot;
void serial(){
	USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( USB, &tty ) != 0 ) {
		std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */
	cfsetospeed (&tty, (speed_t)B115200);
	cfsetispeed (&tty, (speed_t)B115200);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( USB, TCIFLUSH );
	if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	}
}

void kirim_serial(char num[]){
	write(USB, num, strlen(num));
	//cout<<num<<endl;

}

void *terima_serial(void*){
	
	udp_client udpcl(ip_clien, prt_clien);
	
	while(true)
	{
		int n = 0,
		spot = 0;
		char buf = '\0';

		// Whole response
		//char response[1024];
		memset(trimser, '\0', sizeof trimser);
		//memset(terima,'\0', sizeof terima);
		do 
		{
			n = read( USB, &buf, 1 );
			sprintf( &trimser[spot], "%c", buf );
			spot += n;
			// std::cout << "Terima Serial : " << trimser << std::endl;
			//std::cout << "terima serial: " << buf <<"    "<<n<<"    "<< std::endl;
		} while( buf != '!' && n > 0);

		if (n < 0) {
			std::cout << "Error reading: " << strerror(errno) << std::endl;
		}
		else if (n == 0) {
			std::cout << "Read nothing!" << std::endl;
		}
		else {
			
			//sprintf(sendser,"%s",trimser);
			//std::cout << "Terima Serial : " << trimser << std::endl;
			sprintf(senudp,"%i,%s,", bat, trimser);
			udpcl.send(trimser, strlen(trimser));
			
		}
		std::vector<string> result;
	    boost::split(result, trimser, boost::is_any_of(",")); 
	    for (int i = 0; i < result.size(); i++){
	    	Radius		= atoi(result[0].c_str());
	    	robotX   	= atoi(result[1].c_str());
	    	robotY      = atoi(result[2].c_str());
	    	Sudut       = atoi(result[3].c_str());
	    	statusbola  = atoi(result[4].c_str());
	    	ballX       = atoi(result[5].c_str());
	    	ballY       = atoi(result[6].c_str());
	    	st_IR       = atoi(result[7].c_str());
	    	st_kal      = atoi(result[8].c_str());
	    	to_robot    = atoi(result[9].c_str());
	    	data1		= atoi(result[10].c_str());
	    	data2		= atoi(result[11].c_str());
	    	data3		= atoi(result[12].c_str());
	    }	
		//std::cout << "senudp : " << senudp << std::endl;
		//sprintf(sendser,"%i,%i,%i,%s",azimut,x_bola,y_bola,trimser);
		//sprintf(sendser,"%i,%i,%i,%i,%i,%s",azimut,x_bola,y_bola,jarak,status,trimser);
		/*if(udp)
		{
			udpcl.send(senudp, strlen(senudp));
			//sprintf(senudp,"%i,%i,%i,%i,%i,%i,%i,%i,%i,%i!",Radius,robotX,robotY,Sudut,statusbola,ballX,ballY,st_IR,st_kal,to_robot);
			sprintf(senudp,"%i,%i,%i,%i,%i,%i,%i,%i,%i,%i!",(int)Radius, (int)robotX, (int)robotY, (int)Sudut, (int)statusbola, (int)ballX, (int)ballY, (int)st_IR, (int)st_kal, (int)to_robot);
			std::cout << "senudp : " << senudp << std::endl;		
		}*/

	}
}

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
int port = 6000;
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

/*void *processLast(void*){
	last_ballPos = ballPos;
	usleep(10000);
	return NULL;
	//selisih = ballPos - last_ballPos;
}*/

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
		configFileName.assign("o.conf");
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

		if(strcmp(argv[i] , "-komser") == 0 )
			komser = true;

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
			cam->initCameraOmni(config.camSettings);
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
	cv::Point robotCenter(315,215);
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

	if(udp)
	{
		pthread_t UDPcam;
		pthread_create( &UDPcam , NULL , udp_Cam, NULL );

		pthread_t UDPbase;
		pthread_create( &UDPbase , NULL , udp_Base, NULL );

		//pthread_t UDPdatarobot;
		//pthread_create( &UDPcam , NULL , udp_DataRobot, NULL );
		std::cerr << "Server UDP mode: accept thread created\n";
	}
	udp_client udpcl(ip_clien, prt_clien);
	std::cerr << "Client UDP mode: accept created\n";

	if(komser)
	{
		serial();
		pthread_t TS;
		pthread_create( &TS , NULL , terima_serial, NULL );
		std::cerr << "Komunikasi Serial mode: accept thread created\n";
	}
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
		cv::namedWindow( "Display Omni", CV_WINDOW_AUTOSIZE );

	//ScanLines linesRad(idxImage, UAV_RADIAL, cv::Point(315,223), 360, 95, 180, config.camSettings->getCameraSetting(UAV_OUTRADIUS));
	ScanLines linesCirc(idxImage, UAV_CIRCULAR, cv::Point(305,245), 0, 40, config.camSettings->getCameraSetting(UAV_OUTRADIUS), 2);
	//ScanLines linesRadNonBall(idxImage, UAV_RADIAL, robotCenter, 90, 80, config.camSettings->getCameraSetting(UAV_OUTRADIUS));
	//ScanLines linesRadObs(idxImage, UAV_RADIAL, cv::Point(305,245), 360, 70, 130);
	ScanLines linesCircObs(idxImage, UAV_CIRCULAR, cv::Point(305,245), 0, 70, 130, 2);
	//ScanLines linesCircWhite(idxImage, UAV_CIRCULAR, robotCenter, 0, 90, config.camSettings->getCameraSetting(UAV_OUTRADIUS), 12);
	
	ScanLines linescyanCirc(idxImage, UAV_CIRCULAR,robotCenter, 0, 40, config.camSettings->getCameraSetting(UAV_OUTRADIUS), 2);
	ScanLines linesHorR(idxImage, UAV_HORIZONTAL, cv::Point(240, robotCenter.y-55), cv::Point(255, robotCenter.y+65), 1, 2);
	ScanLines linesHorL(idxImage, UAV_HORIZONTAL, cv::Point(390, robotCenter.y-65), cv::Point(405, robotCenter.y+55), 1, 2);
	ScanLines linesVerR(idxImage, UAV_VERTICAL, cv::Point(250, robotCenter.y+45), cv::Point(395, robotCenter.y+65), 2, 1);
	ScanLines linesVerL(idxImage, UAV_VERTICAL, cv::Point(250, robotCenter.y-65), cv::Point(395, robotCenter.y-45), 2, 1);
	
	//ScanLines linesHorGL(idxImage, UAV_HORIZONTAL, cv::Point(400, robotCenter.y-95), cv::Point(450, robotCenter.y-75), 1, 2);
	//ScanLines linesHorGL(idxImage, UAV_HORIZONTAL, cv::Point(400, robotCenter.y-95), cv::Point(450, robotCenter.y-75), 1, 2);
	//ScanLines linesHor(idxImage, UAV_HORIZONTAL, cv::Point(310, robotCenter.y-200), cv::Point(330, robotCenter.y+150), 1, 2);
	//ScanLines linesVerL(idxImage, UAV_VERTICAL, cv::Point(200, robotCenter.y-55), cv::Point(450, robotCenter.y-35), 2, 1);
	ScanLines linesRadX(idxImage, UAV_HORIZONTAL, cv::Point(315, robotCenter.y-55), cv::Point(450, robotCenter.y-35), 2, 1);
	//ScanLines linesRadBALLs(idxImage, UAV_RADIAL, cv::Point(315,223), 360, 60, 90, config.camSettings->getCameraSetting(UAV_OUTRADIUS));
	//ScanLines linesRadball(idxImage, UAV_RADIAL, cv::Point(315,223), 360, 60, 90, config.camSettings->getCameraSetting(UAV_OUTRADIUS));

	tack(true, "TIME: Scanlines created");

	std::vector<float> ballRadius, ballBlobThr;
	//distRelation(ballRadius,image);
	distRelation(ballBlobThr,image, polCoeffFileName, true);
	std::cerr << "Ball radius data created\n";

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
	compression_params.push_back(10);

	std::cerr << "Main loop will start...\n\n";
	while(!end)
	{
		//ScanLines linesVerR(idxImage, UAV_VERTICAL, cv::Point(data2, robotCenter.y+45), cv::Point(395, robotCenter.y+65), 2, 1);
		//ScanLines linesVerL(idxImage, UAV_VERTICAL, cv::Point(data3, robotCenter.y-65), cv::Point(395, robotCenter.y-45), 2, 1);
		//ScanLines linesCyanrad(idxImage, UAV_RADIAL, robotCenter, 360, 90, 180, data2, data3, 0);
		ScanLines linesRad(idxImage, UAV_RADIAL,  robotCenter, 360, 90, data1, data2, data3, 0);
		ScanLines linesRadObs(idxImage, UAV_RADIAL, robotCenter, 360, 90, data1, data2, data3, 0);
		gettimeofday(&tvi,NULL);

		//===============================battery status=================================
		//ifstream inFile;
	    //inFile.open("/sys/class/power_supply/BAT1/capacity");
	    //if (!inFile) {
	    //    cout << "Unable to open battery";
	    //    exit(1); // terminate with error
	    //}

	    //bat = 0;  	

		//while (inFile >> x) {
	        //bat = bat + x;
	    //}	    
	    //inFile.close();

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
		//===========Blok kabel kompas
		//line(image, cv::Point(315, 300), cv::Point(307, 480), cv::Scalar(255, 0, 0), 8, 8);


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
		idxImage.copyTo(linesRad.image); 
		idxImage.copyTo(linesCirc.image);
		//idxImage.copyTo(linesRadNonBall.image);
		//idxImage.copyTo(linesCircWhite.image);
		idxImage.copyTo(linesRadObs.image);
		idxImage.copyTo(linesCircObs.image);
		idxImage.copyTo(linesHorR.image);
		idxImage.copyTo(linesHorL.image);
		idxImage.copyTo(linesVerR.image);
		idxImage.copyTo(linesVerL.image);
		//idxImage.copyTo(linesHorB.image);
		//idxImage.copyTo(linesHorGL.image);
		idxImage.copyTo(linesRadX.image);
		//idxImage.copyTo(linesCyanrad.image);
		idxImage.copyTo(linescyanCirc.image);
		//idxImage.copyTo(linesRadBALLs.image);
		//idxImage.copyTo(linesRadball.image);

		tack(timeInfo, "TIME: scanlines ");

		tick();
		//RLE rleLinesRad(linesRadNonBall, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 2, 2, 10);
		//RLE rleLinesCirc(linesCircWhite, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 8, 2, 10);
		RLE rleBallRad(linesRad, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 0, 10);
		RLE rleBallCirc(linesCirc, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 0, 10);
		RLE rleObs(linesRadObs, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 2, 4, 0, 20);
		RLE rleBallCircObs(linesCircObs, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 2, 4, 0, 20);
		RLE rleHorR(linesHorR, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		RLE rleHorL(linesHorL, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		RLE rleVerR(linesVerR, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		RLE rleVerL(linesVerL, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleHorB(linesHorB, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleRadX(linesRadX, UAV_GREEN_BIT, UAV_ORANGE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleHorGL(linesHorGL, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 0, 4, 0, 20);
		//RLE rleCyanrad(linesCyanrad,obsbit, obsbit, obsbit, 3, 0, 4, 0, 20);
		//RLE rleCyancirc(linescyanCirc,obsbit, obsbit, obsbit, 0, 4, 0, 20);
		//RLE rleBALLsrad(linesCyanrad,obsbit, obsbit, obsbit, 0, 4, 0, 20);
		RLE rleballsrad(linesRad,obsbit, obsbit, obsbit, 0, 4, 0, 20);
		//RLE rleObs(linesRadObs, obsbit, obsbit, obsbit, 0, 4, 0, 20);
		//RLE rleballRad(linesRadball, UAV_GREEN_BIT, ballBit, UAV_GREEN_BIT, 0, 4, 0, 10);
		tack(timeInfo, "TIME: RLE ");

		tick();
		Blob ballOmni, Obs, sR, sL, sHR, tmn, sHL, sGL, sRDX, sBL, BALLs, ball;
		//ball.createBlobs(rleBallHor, ballBlobThr, robotCenter);
		ball.createBlobs(rleBallRad, ballBlobThr, robotCenter);
		//ballDepan.createBlobs(rleBallVer, ballBlobThr, robotCenter);
		ballOmni.createBlobs(rleBallCirc, ballBlobThr, robotCenter);
		ballOmni.createBlobs(rleBallRad, ballBlobThr, robotCenter);
		//ballDepan.sort(UAV_SORT_BY_DISTANCE, rleBallVer);
		//Obs.createBlobs(rleBallCircObs, ballBlobThr, robotCenter);
		Obs.createBlobs(rleObs, ballBlobThr, robotCenter);
		//tmn.createBlobs(rleCyanrad, ballBlobThr, robotCenter);
		//tmn.createBlobs(rleCyancirc, ballBlobThr, robotCenter);
		sR.createBlobs(rleVerR, ballBlobThr, robotCenter);
		sL.createBlobs(rleVerL, ballBlobThr, robotCenter);
		sHR.createBlobs(rleHorR, ballBlobThr, robotCenter);
		sHL.createBlobs(rleHorL, ballBlobThr, robotCenter);
		//sBL.createBlobs(rleHorB, ballBlobThr, robotCenter);
		//sGL.createBlobs(rleHorGL, ballBlobThr, robotCenter);
		//sRDX.createBlobs(rleRadX, ballBlobThr, robotCenter);

		//ballOmni.sort(UAV_SORT_BY_DISTANCE, rleBallCirc);
		ballOmni.sort(UAV_SORT_BY_DISTANCE, rleBallRad);
		BALLs.sort(UAV_SORT_BY_DISTANCE, rleBallRad);
		//ball.createBlobs(rleballRad, ballBlobThr, robotCenter);
		//Obs.sort(UAV_SORT_BY_DISTANCE, rleBallCircObs);
		tack(timeInfo, "TIME Blobs ");
		tick();
		// DEBUG stuff
		//linesRad.draw(image, cv::Scalar(255, 255, 0));
		//linesCyanrad.draw(image, cv::Scalar(255, 255, 0));
		//linesRadball.draw(image, cv::Scalar(0, 255, 255));
		//linesRadNonBall.draw(image, cv::Scalar(255, 0, 0));
		linesRadObs.draw(image, cv::Scalar(255, 255, 0));
		//linesCirc.draw(image, cv::Scalar(0, 255, 0));
		//linesCircObs.draw(image, cv::Scalar(0, 255, 0));
		//linesCircWhite.draw(image, cv::Scalar(0, 255, 0));
		//linesRadBALLs.draw(image, cv::Scalar(0, 230, 255));
		//linesRadBall.draw(image,cv::Scalar(255, 255, 0));

		linesHorR.draw(image, cv::Scalar(255, 0, 0));
		linesHorL.draw(image, cv::Scalar(255, 0, 0));
		linesVerR.draw(image, cv::Scalar(0, 0, 255));
		linesVerL.draw(image, cv::Scalar(0, 255, 0));
		//linesHorB.draw(image, cv::Scalar(255, 255, 0));
		//linesHorGL.draw(image, cv::Scalar(255, 255, 0));
		//linesRadX.draw(image, cv::Scalar(0, 230, 255));
		ball.draw(cv::Scalar(255, 255, 0), image);
//===================================Deteksi Bola Depan=================================================//
		/*int nBallsDepan = 0, nBallsOmni = 0, nObs = 0, dist, dist1, dist2, dist3, dist4;
		double A, cosA, prediksi, sinB, sudutB;
		for(unsigned k = 0 ; k < ballDepan.blobs.size() ; k++){
			if(ballDepan.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(ballDepan.blobs[k].center % image.cols, ballDepan.blobs[k].center / image.cols);
			cv::Point2d pt2(image.cols / 2, image.rows);

			if(nBallsDepan >= MAX_BALLS) break;

			nBallsDepan++;

			if(display && debug)
			{
				drawCircle(ballDepan.blobs[k].center, 10, cv::Scalar(255, 0, 255), image);
			}			
			dist = euclidean(pt1, pt2);
			ballPos = pt1; //Actual ballPos
			centerX = pt2; //Actual centerX

			pthread_t last;
			pthread_create( &last , NULL , processLast, NULL );

			dist1 = euclidean(centerX, last_ballPos);			
			dist2 = euclidean(last_ballPos, ballPos);
			selisih = ballPos - last_ballPos;

			titik = last_ballPos + centerX;
			titik.x = titik.x / 2;
			titik.y = titik.y / 2;

			dist3 = euclidean(ballPos, titik);
			dist4 = euclidean(titik, last_ballPos);

			//Aturan COS
			A = (pow(dist2, 2) + pow(dist4, 2) - pow(dist3, 2)) / (2*dist2*dist4);
			cosA = acos(A) * 180 / 3.1415;

			if(dist2 >= 10 && cosA <=80){
				//Aturan SIN
				sinB = cosA * 3.14159 / 180;
				sudutB = 180 - (90 + cosA);
				prediksi = (dist1 * sin(sinB)) / sudutB;
				//std::cout << "dist2 : " << dist2 << "\tcosA : " << cosA << "\tprediksi : " << prediksi << std::endl;
			}

			line(image, last_ballPos, ballPos, cv::Scalar( 0, 255, 0 ),  2, 8 );
			line(image, last_ballPos, centerX, cv::Scalar( 255, 255, 0 ),  2, 8 );
			circle(image, titik, 2, cv::Scalar( 0, 0, 255 ) ,2);
			line(image, last_ballPos, titik, cv::Scalar( 0, 0, 255 ),  2, 8 );

				
			//std::cout << "dist1 : " << dist1 << "\tdist2 : " << dist2 << "\tdist3 : " << dist3 << "\tdist4 : " << dist4 << "\tcosA : " << cosA << std::endl;
			//std::cout << "ballPos : " << ballPos << "last_ballPos : " << last_ballPos << "selisih : " << selisih << std::endl;
			std::cout << "ballPosDepan : " << ballPos << std::endl;
		}*/
		//line(image, cv::Point2d(0, 80), cv::Point2d(640, 80), cv::Scalar( 0, 255, 0 ),  2, 8 );
		//line(image, cv::Point2d(0, 400), cv::Point2d(640, 400), cv::Scalar( 0, 255, 0 ),  2, 8 );
		//line(image, cv::Point2d(80, 0), cv::Point2d(80, 480), cv::Scalar( 0, 255, 0 ),  2, 8 );
		//line(image, cv::Point2d(560, 0), cv::Point2d(560, 480), cv::Scalar( 0, 255, 0 ),  2, 8 );
//===================================Deteksi Bola Omni=================================================//
		int nBallsOmni = 0, nObs = 0, jarakBola, nCamOmni,nteam;
		cv::Point2d centerImage(315,215);
		//cv::Point2d CENTERR1(310,250);
		circle(image, centerImage, 2, cv::Scalar( 0, 0, 255 ) ,1);
		if(ballOmni.blobs.size() > 0){
			camOmni = true;
			for(unsigned k = 0 ; k < ballOmni.blobs.size() ; k++)
			{
				if(ballOmni.blobs[k].nRle == 1)
					continue;

				cv::Point2d pt1(ballOmni.blobs[k].center % image.cols, ballOmni.blobs[k].center / image.cols);
				//cv::Point2d pt2(image.cols / 2, image.rows / 2);

				if(nBallsOmni >= MAX_BALLS) break;

				nBallsOmni++;				
				{
					drawCircle(ballOmni.blobs[k].center, 10, cv::Scalar(0,69,255), image);
				}	
				ballPosOmni = pt1; //Actual ballPos		
				//ballPosOmni = pt2 - pt1; //Actual ballPos
				//centerX = pt2; //Actual centerX
				jarakBola = euclidean(centerImage, ballPosOmni);
				//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
			}
		}
		else{
			camOmni = false;
			jarakBola = euclidean(centerImage, centerImage);
		}

		nCamOmni++;

		if(nBallsOmni == 0)
		{
			statusCam = 0;
		}
		else
		{
			statusCam = 1;
		}
		
		if(nCamOmni >=2){
			nCamOmni = 0;
			lastCamOmni = camOmni;
		}
		lastCamOmni = camOmni;

		//std::cout << "lastCamOmni : " << lastCamOmni << std::endl;
	//===================================Deteksi Teman====================================================//
		/*int jarteamDep0, jarteamDep1, jarteamDep2;
		int statusteamdep;
		if(tmn.blobs.size() > 0)
			statusteamdep = 1;
		else
			statusteamdep = 0;
		for(unsigned k = 0 ; k < tmn.blobs.size() ; k++){
			if(tmn.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(tmn.blobs[k].center % image.cols, tmn.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows);

			if(nteam >= MAX_team) break;

			nteam++;

			if(display && debug)
				drawCircle(tmn.blobs[k].center, 10, cv::Scalar(255, 0, 255), image);
			jarteamDep0 = euclidean(pt1, robotCenter);
			team = pt1; //Actual ballPos

			//centerX = pt2; //Actual centerX
		}
		if(nteam == 0)
		{
			team.y = 0;
			team.x = 0;
		}
		//=======*/

//===================================Deteksi Halangan=================================================//
		int jarObs0, jarObs1, jarObs2;
		int statusObs;

		for(unsigned k = 0 ; k < Obs.blobs.size() ; k++)
		{
			if(Obs.blobs[k].nRle == 1)
				continue;
			
			if(nObs >= MAX_Obs) break;

			nObs++;
			//cv::Point2d pt2(Obs.blobs[2].center % image.cols, Obs.blobs[2].center / image.cols);


			if(nObs == 1){
				cv::Point2d pt0(Obs.blobs[0].center % image.cols, Obs.blobs[0].center / image.cols);
				obs0 = pt0;
				//statusObs = 1;
			}
			else if(nObs == 2){
				cv::Point2d pt1(Obs.blobs[1].center % image.cols, Obs.blobs[1].center / image.cols);
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
				drawCircle(Obs.blobs[k].center, 5, cv::Scalar(255, 0, 0), image);
			}
			//obsPos = pt1; //Actual ballPos	

			//obsPos = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
			
		}

		if(nObs == 0)
			statusObs = 0;
		else if(nObs == 1)
			statusObs = 1;
		else
			statusObs = 2;

		/*if(obs0.x > 240 && obs0.y > 140 && obs0.y < 170 || obs1.x > 240 && obs1.y > 140 && obs1.y < 170)
		{
			statusObs =1;
		} 
		else if (obs0.x > 240 && obs0.y > 220 && obs0.y < 240 || obs1.x > 240 && obs1.y > 220 && obs1.y < 240)
		{
			statusObs =2;
		}
		else if (obs0.x > 240 && obs0.y > 300 && obs0.y < 330 || obs1.x > 240 && obs1.y > 300 && obs1.y < 330)
		{
			statusObs =3;
		}
		else
			statusObs =0;*/
		//==========Menghitung Jarak Obs==========//
		jarObs0 = euclidean(centerImage, obs0);
		jarObs1 = euclidean(centerImage, obs1);



		//jarObs2 = euclidean(centerImage, obs2);
		//std::cout << "nObs : " << nObs <<std::endl;
		//std::cout << "Jarak obs : " << jarObs0 << " " << jarObs1 << " " << jarObs2 <<std::endl;
		//std::cout << "obsPos : " << obs0 << " " << obs1 << " " << nObs <<std::endl;

//===================================Deteksi Garis Untuk Kalibrasi=================================================//
		int nR=0, nL=0, nHR=0, nHL=0, nGL=0, nRDX=0, nBL=0;

		//==========Scen garis Kanan (R)============//
		for(unsigned k = 0 ; k < sR.blobs.size() ; k++)
		{
			if(sR.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(sR.blobs[k].center % image.cols, sR.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nR >= 1) break;

			nR++;

			if(display && debug)
			{
				drawCircle(sR.blobs[k].center, 5, cv::Scalar(255,0,255), image);
			}	
			sRpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
			
		}
		if(nR == 0)
			sRpos.x = 0;	
		//std::cout << "sRpos : " << sRpos.y << std::endl;
		//==========Scen garis Kiri (L)============//
		for(unsigned k = 0 ; k < sL.blobs.size() ; k++)
		{
			if(sL.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(sL.blobs[k].center % image.cols, sL.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nL >= 1) break;

			nL++;

			if(display && debug)
			{
				drawCircle(sL.blobs[k].center, 5, cv::Scalar(255,0,255), image);
			}	
			sLpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
		}

		if(nL == 0)
			sLpos.x = 0;
		//std::cout << "sLpos : " << sLpos.y << std::endl;

		//==========Scen garis Horizontal (HR)============//
		for(unsigned k = 0 ; k < sHR.blobs.size() ; k++)
		{
			if(sHR.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(sHR.blobs[k].center % image.cols, sHR.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nHR >= 1) break;

			nHR++;

			if(display && debug)
			{
				drawCircle(sHR.blobs[k].center, 5, cv::Scalar(0,255,255), image);
			}	
			sHRpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
				
			//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
		}
		if(nHR == 0)
				sHRpos.y = 0;	
			//std::cout << "sHpos : " << sHpos.x << std::endl;
		//==========Scan garis Horizontal (HL)============//
		for(unsigned k = 0 ; k < sHL.blobs.size() ; k++)
		{
			if(sHL.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(sHL.blobs[k].center % image.cols, sHL.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nHL >= 1) break;

			nHL++;

			if(display && debug)
			{
				drawCircle(sHL.blobs[k].center, 5, cv::Scalar(0,255,255), image);
			}	
			sHLpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
				
			//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
		}
		if(nHL == 0)
				sHLpos.y = 0;	
			//std::cout << "sHpos : " << sHpos.x << std::endl;

			
       //==========Scan garis Horizontal (GL)============//
		 /*for(unsigned k = 0 ; k < sGL.blobs.size() ; k++)
		{
			if(sGL.blobs[k].nRle == 1)
				continue; 

			//cv::Point2d pt1(sGL.blobs[k].center % image.cols, sGL.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nGL >= 1) break;

			nGL++;

			if(display && debug)
			{
				drawCircle(sGL.blobs[k].center, 5, cv::Scalar(0,255,255), image);
			}	
			sGLpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
				
			//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
		}
		if(nGL == 0)
				sGLpos.y = 0;	*/
			//std::cout << "sHpos : " << sHpos.x << std::endl; 

//==================== Scan Radian (RDX) =====================================//
		/*for(unsigned k = 0 ; k < sRDX.blobs.size() ; k++)
		{
			if(sRDX.blobs[k].nRle == 1)
				continue;

			cv::Point2d pt1(sRDX.blobs[k].center % image.cols, sRDX.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nRDX >= 1) break;

			nRDX++;

			if(display && debug)
			{
				drawCircle(sRDX.blobs[k].center, 10, cv::Scalar(0,69,255), image);
			}	
			sRDXpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
			
		}
		if(nRDX == 0)
			sRDXpos.x = 0;	*/
		//std::cout << "sRpos : " << sRpos.y << std::endl;

//=================== Scan Garis BL ============================================//
		/*for(unsigned k = 0 ; k < sBL.blobs.size() ; k++)
		{
			if(sBL.blobs[k].nRle == 1)
				continue; 

			//cv::Point2d pt1(sGL.blobs[k].center % image.cols, sGL.blobs[k].center / image.cols);
			//cv::Point2d pt2(image.cols / 2, image.rows / 2);

			if(nBL >= 1) break;

			nBL++;

			if(display && debug)
			{
				drawCircle(sBL.blobs[k].center, 5, cv::Scalar(0,255,255), image);
			}	
			sBLpos = pt1; //Actual ballPos		
			//ballPosOmni = pt2 - pt1; //Actual ballPos
			//centerX = pt2; //Actual centerX
				
			//std::cout << "ballPosOmni : " << ballPosOmni << std::endl;
		}
		if(nBL == 0)
		//		sBLpos.y = 0;	
			//std::cout << "sHpos : " << sHpos.x << std::endl; 	*/


//===================================Data yg dikirim=================================================//
		//if(statusCamDepan == 1)
			//camDepan = true;
		//else
		//	camDepan = false;
		/*
		if(lastCamOmni && camDepan){
			statusCam = 1;
			ballPos = ballPosOmni;
		}
		else if(lastCamOmni && !camDepan){
			statusCam = 1;
			ballPos = ballPosOmni;
		}
		else if(!lastCamOmni && camDepan){
			statusCam = 2;
			ballPos.x = ballPosDepanX;
			ballPos.y = ballPosDepanY;
		}
		else{
			statusCam = 0;
			jarakBola = 0;
		}*/

		//sprintf(kirser,"%i,%i,%i,%i,%i,%i,%i,1,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,!",(int)botX, (int)botY, (int)angel, (int)kondisi, (int)kr, (int)dumX, (int)dumY, (int)ballPosOmni.x, (int)ballPosOmni.y, (int)statusCam, (int)sRpos.x ,(int)sLpos.x, (int)sHRpos.y, (int)sHLpos.y, (int)jarakBola, (int)teamx, (int)teamy, (int)statustemandepn);
		sprintf(senudp,"%i,%i,%i,%i,%i,%i,%i,%i,%i,%i!",(int)Radius, (int)robotX, (int)robotY, (int)Sudut, (int)statusbola, (int)ballX, (int)ballY, (int)st_IR, (int)st_kal, (int)to_robot);
		//sprintf(kirser,"%i,%i,%i,%i,1,%i,%i,%i,!",botX, botY, angel, kondisi, ballPos.x, ballPos.y, statusCam);
		//sprintf(kirser,"0,0,0,0,1,1,3,4,4,0,0,0,0,0,0,!");
		//std::cout << "kirser : " << data2  << data1 << data3 << std::endl;
		//std::cout << "kirser : " << kirser << std::endl;
		std::cout << "senudp : " << senudp << std::endl;
		std::cout << "Data Cam Depan : " << teamx << teamy << std::endl;
		if(komser){
			kirim_serial(kirser);
		}

		//sprintf(senudp,"%i,%i,40,6,78,90,", ballPosOmni.x, ballPosOmni.y);
		//std::cout << "senudp : " << senudp << std::endl;
		if(udp){				
			udpcl.send(senudp, strlen(senudp));//data yang dikirim ke basetasion
			udpcl.send("1,2,3,4,5,6,7,8,9,0,9",21);//cek mengirin bs		
			//std::cout << "senudp : " << senudp << std::endl;//cek data yang dikirim dari mikro
		}	

		/*if(verbose)
		{

			//std::cerr << "Number of lines RLEs: " << rleLinesRad.rlData.size() << " + " << rleLinesCirc.rlData.size() << std::endl;
			//std::cerr << "Number of ball RLEs: " << rleBallRad.rlData.size() << " + " << rleBallCirc.rlData.size() << std::endl;
			//std::cerr << "Number of obs RLEs: " << rleObs.rlData.size() << std::endl;
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
*/
		if(display && debug)
		{
			//cv::rectangle(image, whiteRect, cv::Scalar(255,0,0), 1, 8, 0);
			cv::circle(realImage, robotCenter, 20, cv::Scalar(255, 255, 0), 3); // draw robot body
		}
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
					imshow( "Display Omni", tmpImage );
				else
				{
					cv::resize(tmpImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Omni", scaledImage);
				}
				break;
			case 2:
				if(screenshot) cv::imwrite("i2.png", idxImage, compression_params);
				if(fullSize)
					imshow( "Display Omni", idxImage );
				else
				{
					cv::resize(idxImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Omni", scaledImage);
				}
				break;
			case 3:
				segImage.setTo(cv::Scalar(255, 0, 127));
				PaintIndexImage(idxImage, segImage, config.mask);
				if(screenshot) cv::imwrite("i3.png", segImage, compression_params);
				if(fullSize)
					imshow( "Display Omni", segImage );
				else
				{
					cv::resize(segImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Omni", scaledImage);
				}
				break;
			case 4:
				if(screenshot) cv::imwrite("i4.png", realImage, compression_params);
				if(fullSize)
					imshow( "Display Omni", realImage );
				else
				{
					cv::resize(realImage, scaledImage,cv::Size(), 0.5, 0.5);
					imshow( "Display Omni", scaledImage);
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
