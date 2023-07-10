#include "CameraCalib.h"
#include "PID_vision.h"

namespace uav{

CameraCalib::CameraCalib(int size, cv::Mat &imagePassed)
{
	hist_size = size;
	imagePassed.copyTo(image);
}

void CameraCalib::calcHistograms(cv::Mat &mask)
{
	cv::Mat gray_image;
	histValue.clear();
	histValue.resize(hist_size);

	cv::cvtColor(image, gray_image, CV_RGB2GRAY);

	for(int i = 0 ; i < gray_image.cols * gray_image.rows ; i++)
	{
		if((int)mask.ptr()[i] > 0)
		{
			int x = (gray_image.ptr()[i] / (255 / hist_size + 1)); // [0 4]
			histValue[x]++;
		}
	}

}

float CameraCalib::calcMean(cv::Mat &hist)
{
	float sum = 0, mean = 0;

	for(int i = 0; i < hist_size; i++ )
	{
		mean += i * hist.ptr()[i];
		sum += hist.ptr()[i];
	}
	mean /= sum;
	return mean;
}


float CameraCalib::calcMSV(std::vector<int> &hist)
{
	int areas = (int) hist_size/5;
	float msv[areas], MSV = 0;
	int sum = 0;

	for( int i = 0; i < hist_size; i++ )
	{
		sum+= hist[i];
	}
	/*
	for( int i = 0; i < hist_size; i++ )
	{
		msv[i / areas] += hist[i];
	}
	 */
	for(int i = 0 ; i < 5 ; i++)
	{
		MSV += (i+1) * hist[i];
	}

	MSV /= sum;

	return MSV;


}


float CameraCalib::calcACM(cv::Mat &hist)
{
	float prob[hist_size], ACM = 0;
	int sum = 0;

	for( int i = 0; i < hist_size; i++ )
	{
		sum+= hist.ptr()[i];
	}

	for( int i = 0; i < hist_size; i++ )
	{
		prob[i] = (float) hist.ptr()[i]/sum;
		ACM += abs(i - 127) * prob[i];
	}
	return ACM;
}

cv::Scalar CameraCalib::calcUV(cv::Rect &rect)
{

	cv::Mat tmp = image.operator()(rect);
	unsigned resultCr = 0, resultCb = 0, pixelU = 0, pixelV = 0, count = 0;
	cv::cvtColor(tmp, tmp, CV_RGB2YUV);

	for(int i = 0; i < tmp.rows; i++)
	{
		for(int j = 0; j < tmp.cols; j++)

		{
			pixelU = pixelU + tmp.ptr()[(i*tmp.cols + j)*3 +1] ;
			pixelV = pixelV + tmp.ptr()[(i*tmp.cols + j)*3 +2];
			count ++;
		}
	}

	resultCr = pixelU/count;
	resultCb = pixelV/count;

	cv::Scalar s = cv::Scalar(resultCr, resultCb);

	return s;

}

cv::Scalar CameraCalib::calcRGB(cv::Rect &rect)
{
	unsigned resultR = 0, resultG = 0, resultB = 0, pixelR = 0, pixelG = 0, pixelB = 0, count = 0;
	cv::Mat tmp = image.operator()(rect);

	for(int i = 0; i < tmp.rows; i++)
	{
		for(int j = 0; j < tmp.cols; j++)

		{
			pixelR = pixelR + tmp.ptr()[(i*tmp.cols + j)*3] ;
			pixelG = pixelG + tmp.ptr()[(i*tmp.cols +j)*3 +1];
			pixelB = pixelB + tmp.ptr()[(i*tmp.cols + j)*3 +2];

			count ++;
		}
	}
	resultR = pixelR/count;
	resultG = pixelG/count;
	resultB = pixelB/count;

	cv::Scalar s = cv::Scalar(resultR, resultG, resultB);

	return s;
}


bool CameraCalib::CalibrateCamera(CameraSettings *camSettings, ParameterRange *parameterRange,
		cv::Mat &mask, cv::Rect &rWhite, bool firstTimeCalib, bool verbose, std::vector<bool> &paramChanged,
		bool wbCalib)
{
	//*******************************PID values start here******************************//
	static float WbKp=0.1;
	static float WbKi=0.0;
	static float cbError=0.0;
	static float crError=0.0;

	static float GainKp=5.2;
	static float GainKi=0.0;
	static float msvError = 0.0;

	//*******************************PID values end here******************************//
	int wBlue = camSettings->getCameraSetting(UAV_WBB),
			wRed = camSettings->getCameraSetting(UAV_WBR),
			gain = camSettings->getCameraSetting(UAV_GAIN);

	// Initilizations of the PIDs
	PID wbBluePI(WbKp, WbKi, 0, parameterRange[UAV_WB].min, parameterRange[UAV_WB].max);
	PID wbRedPI(WbKp, WbKi, 0, parameterRange[UAV_WB].min, parameterRange[UAV_WB].max);
	PID gainPI(GainKp, GainKi, 0, parameterRange[UAV_GAIN].min, parameterRange[UAV_GAIN].max);

	//cv::Scalar yuvAverage;

	int y = 0, u = 0, v = 0;
	cv::Scalar rgbAverage = calcRGB(rWhite);
	Rgb2Yuv(rgbAverage[0],rgbAverage[1],rgbAverage[2], &y, &u, &v);

	if(verbose)
	{
		std::cerr<<"Average U "<<u<<" Average V "<<v<<std::endl;
		std::cerr<<"Average R "<<rgbAverage[0]<<" Average G "<<rgbAverage[1]<<"Average B"<<rgbAverage[2]<<std::endl;
	}

	if(firstTimeCalib)
	{
		wbBluePI.reset();
		wbRedPI.reset();
		gainPI.reset();
		firstTimeCalib = false;
	}

	calcHistograms(mask);
	double msv = calcMSV(histValue);

	if(verbose)
	{
		std::cerr<<"MSV: " << msv << " should be " <<hist_size / 2.0 <<  std::endl;
		std::cerr<<"CALIB (1) WB_RED: "<<wRed <<" WB_BLUE: "<<wBlue <<" GAIN: "<<gain<<std::endl;
	}

	msvError = (hist_size / 2.0) - msv;
	if(msvError > MSVTHRESHOLD || msvError < -MSVTHRESHOLD) {
		gain = (int)round(gainPI.compensate(gain, msvError));
		paramChanged[0] = true;
	}

	if(wbCalib && rgbAverage[0] < 255 && rgbAverage[1] < 255 && rgbAverage[2] < 255 && msv> 2 && msv < 3)
	{
		cbError=127.0 - v;
		if(cbError > UVTHRESHOLD || cbError < -UVTHRESHOLD) {
			wBlue = (int)round(wbBluePI.compensate(wBlue, cbError));
			paramChanged[1] = true;
		}

		crError= 110.0 - u;
		if(crError > UVTHRESHOLD || crError < -UVTHRESHOLD) {
			wRed = (int)round(wbRedPI.compensate(wRed, crError));
			paramChanged[2] = true;
		}

		std::cerr << "CALIB WB calib (u, v, r, g, b)" << u << ", " << v << ", ";
		std::cerr << rgbAverage[0] << ", " << rgbAverage[1] << ", " << rgbAverage[2] << std::endl;
	}

	if(verbose)
	{
		std::cerr<< "CALIB  msvError " << msvError << " cB error: "<<cbError<<" cR error "<<crError<<std::endl;
		std::cerr<<"CALIB (2) WB_RED: "<<wRed <<" WB_BLUE: "<<wBlue <<" GAIN: "<<gain<<std::endl;
	}

	camSettings->setCameraSetting(UAV_GAIN,gain);
	camSettings->setCameraSetting(UAV_WBR, wRed);
	camSettings->setCameraSetting(UAV_WBB, wBlue);

	if(msvError < MSVTHRESHOLD && msvError > -MSVTHRESHOLD &&
			cbError < UVTHRESHOLD && cbError > -UVTHRESHOLD &&
			crError < UVTHRESHOLD && crError > -UVTHRESHOLD)
		return true;
	else
		return false;
}

}
