
#include "libCalib.h"
#include "Angle.h"

using namespace uav;

void PaintIndexImage (cv::Mat &src, cv::Mat &dst, cv:: Mat &mask)
{
	Angle hue, hueMin, hueMax;

	for(unsigned p = 0 ; p < src.cols * src.rows ; p++)
	{
		if(mask.ptr()[p] > 0)
		{
			switch((int)src.ptr()[p])
			{
			case UAV_BLUE_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_BLUE);
				break;
			case UAV_YELLOW_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_YELLOW);
				break;
			case UAV_ORANGE_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_ORANGE);
				break;
			case UAV_GREEN_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_GREEN);
				break;
			case UAV_WHITE_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_WHITE);
				break;
			case UAV_BLACK_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_BLACK);
				break;
			case UAV_CYAN_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_CYAN);
				break;
			case UAV_MAGENTA_BIT:
				PaintPixelWithColor(dst, p * 3, UAV_MAGENTA);
				break;
			default:
				PaintPixelWithColor(dst, p * 3, 999);
				break;
			}

		}
	}
}

void PaintImage (cv::Mat &src, cv::Mat &dst, cv:: Mat &mask, ColorRange *cr)
{
	Angle hue, hueMin, hueMax;
	/*std::cerr<<"--------------------\n";
	for(unsigned c = 0; c < UAV_NCOLORS; c++)
	{
		std::cerr<<cr[c].name<<std::endl;
		std::cerr<<cr[c].hMax<<cr[c].hMin<<std::endl;
		std::cerr<<cr[c].sMax<<cr[c].sMin<<std::endl;
		std::cerr<<cr[c].vMax<<cr[c].vMin<<std::endl;

	}*/

	for(unsigned p = 0 ; p < src.cols * src.rows * 3 ; p += 3)
	{
		if(mask.ptr()[p/3] > 0)
		{
			int h = 0, s = 0, v = 0;
			int r = src.ptr()[p];
			int g = src.ptr()[p + 1];
			int b = src.ptr()[p + 2];
			Rgb2Hsv(r, g, b, &h, &s, &v);

			hue.set_deg(h);
			for(int c = 0 ; c <= 7 ; c++)
			{
				hueMin.set_deg(cr[c].hMin);
				hueMax.set_deg(cr[c].hMax);

				if(hue.in_between(hueMin, hueMax))
				{
					if(s>=cr[c].sMin && s<=cr[c].sMax)
						if(v>=cr[c].vMin && v<=cr[c].vMax)
						{
							PaintPixelWithColor(dst, p, c);
						}
				}
			}
		}
	}
}

void PaintPixelWithColor(cv::Mat &img, int pos, int color)
{
	switch(color)
	{
	case UAV_BLUE:
		img.ptr()[pos] = 255;
		img.ptr()[pos+1] = 0;
		img.ptr()[pos+2] = 0;
		break;
	case UAV_YELLOW:
		img.ptr()[pos] = 0;
		img.ptr()[pos+1] = 255;
		img.ptr()[pos+2] = 255;
		break;
	case UAV_ORANGE:
		img.ptr()[pos] = 0;
		img.ptr()[pos +1] = 0;
		img.ptr()[pos +2] = 255;
		break;
	case UAV_GREEN:
		img.ptr()[pos] = 0;
		img.ptr()[pos +1] = 255;
		img.ptr()[pos +2] = 0;
		break;
	case UAV_WHITE:
		img.ptr()[pos] = 255;
		img.ptr()[pos +1] = 255;
		img.ptr()[pos +2] = 255;
		break;
	case UAV_BLACK:
		img.ptr()[pos] = 0;
		img.ptr()[pos +1] = 0;
		img.ptr()[pos +2] = 0;
		break;
	case UAV_CYAN:
		img.ptr()[pos] = 255;
		img.ptr()[pos +1] = 0;
		img.ptr()[pos +2] = 255;
		break;
	case UAV_MAGENTA:
		img.ptr()[pos] = 20;
		img.ptr()[pos +1] = 0;
		img.ptr()[pos +2] = 200;
		break;
	default:
		img.ptr()[pos] = 127;
		img.ptr()[pos +1] = 127;
		img.ptr()[pos +2] = 127;
		break;

	}
}

void Hsv2Rgb( float *r, float *g, float *b, float h, float s, float v )
{
	int i;
	float f, p, q, t;

	s/=100;
	v/=255;

	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}

	h /= 60;			// sector 0 to 5
	i = (int)floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch( i ) {
	case 0:
		*r = v;
		*g = t;
		*b = p;
		break;
	case 1:
		*r = q;
		*g = v;
		*b = p;
		break;
	case 2:
		*r = p;
		*g = v;
		*b = t;
		break;
	case 3:
		*r = p;
		*g = q;
		*b = v;
		break;
	case 4:
		*r = t;
		*g = p;
		*b = v;
		break;
	default:		// case 5:
		*r = v;
		*g = p;
		*b = q;
		break;
	}
	*r=(*r)*255.0;
	*g=(*g)*255.0;
	*b=(*b)*255.0;
}

void drawHSVHistogram(cv::Mat &src, cv::Mat &hsvHist, cv::Mat &mask, unsigned int hMin,
		unsigned int hMax, int nColor, std::vector<int> HArray, std::vector<int> SArray,
		std::vector<int> VArray, ColorRange *colorRange)
{
	unsigned int H[361]={0};
	unsigned int S[101]={0};
	unsigned int V[256]={0};

	unsigned divisorh, hmax=0;
	unsigned divisors, smax=0;
	unsigned divisorv, vmax=0;

	Angle hue, hueMin, hueMax;

	for(unsigned p = 0 ; p < src.cols * src.rows * 3 ; p += 3)
	{
		if(mask.ptr()[p/3] > 0)
		{
			int h = 0, s = 0, v = 0;
			int r = src.ptr()[p];
			int g = src.ptr()[p + 1];
			int b = src.ptr()[p + 2];
			Rgb2Hsv(r, g, b, &h, &s, &v);

			hue.set_deg(h);
			H[h]++;
			for(int c = 0 ; c < 7 ; c++)
			{
				hueMin.set_deg(hMin);
				hueMax.set_deg(hMax);

				if(hue.in_between(hueMin, hueMax))
				{
					S[s]++;
					V[v]++;
				}
			}
		}
	}

	for(int i=0; i<=360; i++)
	{
		if(H[i]>hmax)
			hmax=H[i];
	}

	divisorh=(int)ceil(hmax/140);

	for(int i=0; i<=100; i++)
	{
		if(S[i]>smax)
			smax=S[i];
	}

	divisors=(int)ceil(smax/140);

	for(int i=0; i<=255; i++)
	{
		if(V[i]>vmax)
			vmax=V[i];
	}

	divisorv=(int)ceil(vmax/140);

	if(divisorh==0) divisorh=1;
	if(divisors==0) divisors=1;
	if(divisorv==0) divisorv=1;


	cv::rectangle(hsvHist, cv::Point(361,420),
			cv::Point(0, 0),
			cv::Scalar( 0,0,0 ), -1, 8, 0 );


	for(int i=0; i<=360; i++)
	{
		float r, g, b;
		Hsv2Rgb( &r, &g, &b, (float) i, 100.0, 200.0 );

		cv::rectangle(hsvHist, cv::Point(i,140), cv::Point(i+1, 140-H[i]/divisorh),
				cv::Scalar((unsigned char)r, (unsigned char)g, (unsigned char)b ), -1, 8, 0 );
	}
	for(int i=0; i<=100; i++)
	{
		cv::rectangle(hsvHist, cv::Point(i*3.6, 280), cv::Point(i*3.6+3, 280-S[i]/divisors),
				cv::Scalar( 255, 255-i*2, i*2 ), -1, 8, 0 );
	}

	for(int i=0; i<=255; i++)
	{
		cv::rectangle( hsvHist, cv::Point(i*1.4,420), cv::Point(i*1.4+1, 420-V[i]/divisorv),
				cv::Scalar( 0, 255, 255-i ), -1, 8, 0 );
	}


	cv::rectangle(hsvHist, cv::Point(0,0), cv::Point(30, 30),
			cv::Scalar( 255,255,255 ), -1, 8, 0 );
	cv::rectangle(hsvHist, cv::Point(30,0), cv::Point(60, 30),
			cv::Scalar( 0,0,0 ), -1, 8, 0 );
	cv::rectangle(hsvHist, cv::Point(60,0), cv::Point(90, 30),
			cv::Scalar( 0, 255, 0 ), -1, 8, 0 ); // green
	cv::rectangle(hsvHist, cv::Point(90,0), cv::Point(120, 30),
			cv::Scalar( 255, 127, 0 ), -1, 8, 0 ); // orange
	cv::rectangle(hsvHist, cv::Point(120,0), cv::Point(150, 30),
			cv::Scalar( 0, 0, 255 ), -1, 8, 0 ); //blue
	cv::rectangle(hsvHist, cv::Point(150,0), cv::Point(180, 30),
			cv::Scalar( 255, 255, 0 ), -1, 8, 0 ); // yellow
	cv::rectangle(hsvHist, cv::Point(180,0), cv::Point(210, 30),
			cv::Scalar( 0,255,255 ), -1, 8, 0 ); // Cyan
	cv::rectangle(hsvHist, cv::Point(210,0), cv::Point(240, 30),
			cv::Scalar( 255,0,255 ), -1, 8, 0 ); // Magenta
	cv::rectangle(hsvHist, cv::Point(240,0), cv::Point(270, 30),
			cv::Scalar( 127,127,127 ), -1, 8, 0 );

	switch(nColor)
	{
	case UAV_WHITE:
		cv::rectangle(hsvHist, cv::Point(10,10), cv::Point(20, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_BLACK:
		cv::rectangle(hsvHist, cv::Point(40,10), cv::Point(50, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_GREEN:
		cv::rectangle(hsvHist, cv::Point(70,10), cv::Point(80, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_ORANGE:
		cv::rectangle(hsvHist, cv::Point(100,10), cv::Point(110, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_BLUE:
		cv::rectangle(hsvHist, cv::Point(130,10), cv::Point(140, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_YELLOW:
		cv::rectangle(hsvHist, cv::Point(160,10), cv::Point(170, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_CYAN:
		cv::rectangle(hsvHist, cv::Point(190,10), cv::Point(200, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;

	case UAV_MAGENTA:
		cv::rectangle(hsvHist, cv::Point(220,10), cv::Point(230, 20),
				cv::Scalar( 255, 0, 0 ), -1, 8, 0 );
		break;
	}

	for(int i = 0 ; i < HArray.size() ; i++)
	{
		cv::line( hsvHist, cv::Point((HArray[i]),0),
				cv::Point((HArray[i]),140), cv::Scalar( 255, 255, 255 ), 1, 8, 0 );

		cv::line( hsvHist, cv::Point(SArray[i] * 3.6 + 2, 140),
				cv::Point(SArray[i] * 3.6 + 2, 280), cv::Scalar( 255, 255, 255 ), 1, 8, 0 );

		cv::line( hsvHist, cv::Point(VArray[i] * 1.4 + 1, 280),
				cv::Point(VArray[i] * 1.4 +1, 420), cv::Scalar( 255, 255, 255 ), 1, 8, 0 );
	}


	cv::line( hsvHist, cv::Point(colorRange[nColor].hMin,0),
			cv::Point(colorRange[nColor].hMin, 140), cv::Scalar( 0, 0, 255 ), 1, 8, 0 );

	cv::line( hsvHist, cv::Point(colorRange[nColor].hMax,0),
			cv::Point(colorRange[nColor].hMax, 140), cv::Scalar( 255, 0, 0 ), 1, 8, 0 );

	cv::line( hsvHist, cv::Point(colorRange[nColor].sMin*3.6+2,140),
			cv::Point(colorRange[nColor].sMin*3.6+2,280), cv::Scalar( 0, 0, 255 ), 1, 8, 0 );

	cv::line( hsvHist, cv::Point(colorRange[nColor].sMax*3.6+2,140),
			cv::Point(colorRange[nColor].sMax*3.6+2,280), cv::Scalar( 255, 0, 0 ), 1, 8, 0 );

	cv::line( hsvHist, cv::Point(colorRange[nColor].vMin * 1.4 + 1, 280),
			cv::Point(colorRange[nColor].vMin * 1.4 + 1, 420), cv::Scalar( 0, 0, 255 ), 1, 8, 0 );

	cv::line( hsvHist, cv::Point(colorRange[nColor].vMax * 1.4 + 1, 280),
			cv::Point(colorRange[nColor].vMax * 1.4 + 1, 420), cv::Scalar( 255, 0, 0 ), 1, 8, 0 );

}

