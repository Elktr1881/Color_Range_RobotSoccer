#include <ctime>
#include <iostream>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include "Lut.h"
#include "Angle.h"

namespace uav{

extern void yuv_to_hsv(int y, int u, int v, int* h, int* s, int* vv);

Lut::Lut()
{
	info = new unsigned char[LUT_MEMORY_SIZE]();
}

Lut::Lut(Config& config)
{
	info = new unsigned char[LUT_MEMORY_SIZE];
	ColorRange* cr = config.colorRange;

	createValidPixels(config.mask, config.camSettings->getCameraSetting(UAV_VIDMODE));

	boost::filesystem::path configPath(config.fileName);
	boost::filesystem::path cachePath = boost::filesystem::path(config.fileName).replace_extension("cache");

	std::cout<<"LUT: checking for cache " << cachePath.string().c_str() << std::endl;

	if (!boost::filesystem::exists(cachePath))
	{
		std::cout << "LUT: cache does not exists" << std::endl;
		// Does not exist.
		createLUT(config.camSettings->getCameraSetting(UAV_VIDMODE), cr);
		// Save to file for speedup.
		saveToCache(cachePath.string());
		return;
	}

	std::time_t configModifiedOn = boost::filesystem::last_write_time(configPath);
	std::time_t cacheModifiedOn = boost::filesystem::last_write_time(cachePath);

	if (configModifiedOn > cacheModifiedOn || boost::filesystem::file_size(cachePath) != LUT_MEMORY_SIZE)
	{
		// Does exist but is old or wrong size.
		createLUT(config.camSettings->getCameraSetting(UAV_VIDMODE), cr);
		// Save to file for speedup.
		saveToCache(cachePath.string());
		return;
	}

	// Cache is newer than config file so load it.
	loadFromCache(cachePath.string());
}

Lut::Lut(Config& config, ScanLines &sc)
{
	info = new unsigned char[LUT_MEMORY_SIZE];
	ColorRange* cr = config.colorRange;

	createValidPixels(sc, config.mask, config.camSettings->getCameraSetting(UAV_VIDMODE));

	boost::filesystem::path configPath(config.fileName);
	boost::filesystem::path cachePath = boost::filesystem::path(config.fileName).replace_extension("cache");

	std::cout<<"LUT: checking for cache " << cachePath.string().c_str() << std::endl;

	if (!boost::filesystem::exists(cachePath))
	{
		std::cout << "LUT: cache does not exists" << std::endl;
		// Does not exist.
		createLUT(config.camSettings->getCameraSetting(UAV_VIDMODE), cr);
		// Save to file for speedup.
		saveToCache(cachePath.string());
		return;
	}

	std::time_t configModifiedOn = boost::filesystem::last_write_time(configPath);
	std::time_t cacheModifiedOn = boost::filesystem::last_write_time(cachePath);

	if (configModifiedOn > cacheModifiedOn || boost::filesystem::file_size(cachePath) != LUT_MEMORY_SIZE)
	{
		// Does exist but is old or wrong size.
		createLUT(config.camSettings->getCameraSetting(UAV_VIDMODE), cr);
		// Save to file for speedup.
		saveToCache(cachePath.string());
		return;
	}

	// Cache is newer than config file so load it.
	loadFromCache(cachePath.string());
}

void Lut::createValidPixels(ScanLines &sc, cv::Mat &mask, int mode)
{
	std::vector<int> temp;
	int p;

	for(unsigned j = 0; j < sc.scanlines.size(); j++)
	{
		temp = sc.getLine(j);

		for (unsigned i = 0; i < temp.size(); i++)
		{
			p = temp[i];
			if(mask.ptr()[p] > 0)
			{
				switch(mode)
				{
				case UAV_RGB8:
					validPixels.push_back(p); // index
					validPixels.push_back(p * 3); // R
					validPixels.push_back(p * 3 + 1); // G
					validPixels.push_back(p * 3 + 2); // B
					break;

				case UAV_GRAY:
					int row = p / mask.cols;
					int col = p % mask.cols;
					int r = 0, g = 0, b = 0;

					if(row % 2 == 0) // linhas pares
					{
						if(col % 2 == 0) // colunas pares
						{
							r = p;
							g = p + 1;
							b = p + mask.cols + 1;
						}
						else
						{
							g = p;
							r = p - 1;
							b = p + mask.cols;
						}
					}
					else // linhas impares
					{
						if(col % 2 == 0) // colunas pares
						{
							g = p ;
							r = p - mask.cols;
							b = p + 1;
						}
						else // colunas impares
						{
							g = p - 1;
							r = p - mask.cols - 1;
							b = p;
						}
					}

					validPixels.push_back(p); // index
					validPixels.push_back(r); // R
					validPixels.push_back(g); // G
					validPixels.push_back(b); // B
					break;
				}// mode
			}// mask
		}// points in a scan
	}// all scans
}

void Lut::addValidPixels(ScanLines &sc, cv::Mat &mask, int mode)
{
	std::vector<int> temp;
	int p;

	for(unsigned j = 0; j < sc.scanlines.size(); j++)
	{
		temp = sc.getLine(j);

		for (unsigned i = 0; i < temp.size(); i++)
		{
			bool include = true;
			p = temp[i];
			if(mask.ptr()[p] > 0)
			{
				for(int k = 0 ; k < validPixels.size() ; k += 4)
				{
					if(validPixels[k] == p)
					{
						include = false;
						break;
					}
				}
			}
			else
			{
				include = false;
			}

			if(!include) continue;

			switch(mode)
			{
			case UAV_RGB8:
				validPixels.push_back(p); // index
				validPixels.push_back(p * 3); // R
				validPixels.push_back(p * 3 + 1); // G
				validPixels.push_back(p * 3 + 2); // B
				break;

			case UAV_GRAY:
				int row = p / mask.cols;
				int col = p % mask.cols;
				int r = 0, g = 0, b = 0;

				if(row % 2 == 0) // linhas pares
				{
					if(col % 2 == 0) // colunas pares
					{
						r = p;
						g = p + 1;
						b = p + mask.cols + 1;
					}
					else
					{
						g = p;
						r = p - 1;
						b = p + mask.cols;
					}
				}
				else // linhas impares
				{
					if(col % 2 == 0) // colunas pares
					{
						g = p ;
						r = p - mask.cols;
						b = p + 1;
					}
					else // colunas impares
					{
						g = p - 1;
						r = p - mask.cols - 1;
						b = p;
					}
				}

				validPixels.push_back(p); // index
				validPixels.push_back(r); // R
				validPixels.push_back(g); // G
				validPixels.push_back(b); // B
				break;
			}// mode
		}// points in a scan
	}// all scans
}

void Lut::createValidPixels(cv::Mat &mask, int mode)
{
	std::cerr << mask.cols  << ", " << mask.rows << std::endl;

	for(unsigned p = 0 ; p < mask.cols * mask.rows ; p++)
	{
		if(mask.ptr()[p] > 0)
		{
			switch(mode)
			{
			case UAV_RGB8:
				validPixels.push_back(p); // index
				validPixels.push_back(p * 3); // R
				validPixels.push_back(p * 3 + 1); // G
				validPixels.push_back(p * 3 + 2); // B
				break;

			case UAV_GRAY:
				int row = p / mask.cols;
				int col = p % mask.cols;
				int r = 0, g = 0, b = 0;

				if(row % 2 == 0) // linhas pares
				{
					if(col % 2 == 0) // colunas pares
					{
						r = p;
						g = p + 1;
						b = p + mask.cols + 1;
					}
					else
					{
						g = p;
						r = p - 1;
						b = p + mask.cols;
					}
				}
				else // linhas impares
				{
					if(col % 2 == 0) // colunas pares
					{
						g = p ;
						r = p - mask.cols;
						b = p + 1;
					}
					else // colunas impares
					{
						g = p - 1;
						r = p - mask.cols - 1;
						b = p;
					}
				}

				validPixels.push_back(p); // index
				validPixels.push_back(r); // R
				validPixels.push_back(g); // G
				validPixels.push_back(b); // B
				break;
			}
		}
	}
}

void Lut::createLUT(int mode, ColorRange *cr)
{
	std::cout<< "LUT: starting initialization... " << std::endl;
	Angle hue, hueMin, hueMax;
	int h = 0, s = 0, v = 0;
	int r = 0, g = 0, b = 0;

	for(int x = 0 ; x < 256 ; x++)
		for(int y = 0 ; y < 256 ; y++)
			for(int z = 0 ; z < 256 ; z++)
			{
				info[x << 16 | y << 8 | z] = 0;
				switch(mode)
				{
				case UAV_RGB8:
				case UAV_GRAY:
					Rgb2Hsv(x, y, z, &h, &s, &v);
					break;

				case UAV_YUV422:
				case UAV_YUV411:
					Yuv2Rgb(x, y, z, &r, &g, &b);
					Rgb2Hsv(r, g, b, &h, &s, &v);
					break;

				default:
					std::cerr << "InitLUT: error, switch break by default" << std::endl;
					break;
				}

				hue.set_deg(h);
				for(int c = 0 ; c < 8 ; c++)
				{
					hueMin.set_deg(cr[c].hMin);
					hueMax.set_deg(cr[c].hMax);

					if(hue.in_between(hueMin, hueMax) &&
							s >= cr[c].sMin && s <= cr[c].sMax &&
							v >= cr[c].vMin && v <= cr[c].vMax)
						info[x << 16 | y << 8 | z] |= 0x01 << (7 - c);
				}
			}
	std::cout<<"DONE!" <<std::endl;
}

void Lut::init(ColorRange* cr)
{
	std::cout<< "LUT: starting initialization... " << std::endl;

	int h = 0, s = 0, vv = 0;

	for (int y = 0; y < LUT_MAX_Y; y++)
		for (int u = 0; u < LUT_MAX_U; u++)
			for (int v = 0; v < LUT_MAX_V; v++)
			{
				//	yuv_to_hsv(y, u, v, &h, &s, &vv);

				for (int c = 0; c < 8; c++)
				{
					if (((h >= cr[c].hMin && h <= cr[c].hMax) || (cr[c].hMin == cr[c].hMax)) && s >= cr[c].sMin && s
							<= cr[c].sMax && vv >= cr[c].vMin && vv <= cr[c].vMax)
					{
						info[v << 16 | y << 8 | u] |= (0x01 << (7 - c));
					}
				}
			}
	std::cout<<"DONE!" <<std::endl;
}

void Lut::saveToCache(std::string cacheFilename)
{
	std::cout<<"LUT: cache flushed to disk" <<std::endl;

	std::ofstream cache(cacheFilename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
	if (!cache.is_open())
	{
		std::cout<<"Error Opening"<<std::endl;
		return;
	}

	cache.seekp(0, std::ios::beg);
	cache.write((char*) info, LUT_MEMORY_SIZE);
	cache.close();
	std::cout<<"DONE"<<std::endl;
}

void Lut::loadFromCache(std::string cacheFilename)
{
	std::cout<< "LUT: cache loaded from disk" << std::endl;

	std:: ifstream cache(cacheFilename.c_str(), std::ios::in | std::ios::binary);
	if (!cache.is_open())
	{
		std::cout << "Error Opening" << std::endl;
		return;
	}

	cache.seekg(0, std::ios::beg);
	cache.read((char *) info, LUT_MEMORY_SIZE);
	cache.close();
}

void Lut::convertImageToIndex(cv::Mat &original, cv::Mat &index, int imageMode)
{
	unsigned i = 0, r = 0, c = 0;
	unsigned lutPos;

	switch(imageMode)
	{

	case UAV_YUV422:
		unsigned char y1, u, v, y2;
		for (i = 0; i <original.cols*original.rows*2; i+=4)
		{
			y1 = original.ptr(0)[i];
			u  = original.ptr(0)[i+1];
			y2 = original.ptr(0)[i+2];
			v  = original.ptr(0)[i+3];

			lutPos = v << 16 | y1 << 8 | u;

			index.ptr(r)[c] = info[lutPos];
			c++;
			lutPos = v << 16 | y2 << 8 | u;
			index.ptr(r)[c] = info[lutPos];
			c++;
			if( c >= original.cols )
			{
				c = 0;
				r++;
			}

		}
		break;

	case UAV_RGB8:
	case UAV_GRAY:

		unsigned char rr, gg, bb;
		for (i = 0; i < validPixels.size() ; i += 4)
		{
			rr = original.ptr()[validPixels[i+1]];
			gg  = original.ptr()[validPixels[i+2]];
			bb = original.ptr()[validPixels[i+3]];

			lutPos = rr << 16 | gg << 8 | bb;

			index.ptr()[validPixels[i]] = info[lutPos];
		}
		break;

	default:
		std::cout<< "Image mode not supported!\n" << std::endl;
		break;

	}

}



Lut::~Lut()
{
	delete[] info;
}
}
