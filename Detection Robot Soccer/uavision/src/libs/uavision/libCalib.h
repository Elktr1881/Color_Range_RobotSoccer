/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                           *
 *  FILE: libAutoCalib.h                                                     *
 *                                                                           *
 *  Copyright 2008-212 CAMBADA Team, All Rights Reserved                     *
 *  DET/IEETA, University of Aveiro                                          *
 *  http://www.ieeta.pt/atri/cambada                                         *
 *                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef LIBAUTOCALIB_H
#define LIBAUTOCALIB_H

#include "Config.h"
#include "UAVision.h"

using namespace uav;

void PaintIndexImage (cv::Mat &src, cv::Mat &dst, cv:: Mat &mask);

void PaintImage (cv::Mat &src, cv::Mat &dst, cv:: Mat &mask, ColorRange *cr);

void PaintPixelWithColor(cv::Mat &img, int pos, int color);

void drawHSVHistogram(cv::Mat &src, cv::Mat &hsvHist, cv::Mat &mask, unsigned int hMin,
  unsigned int hMax, int ncolor, std::vector<int> HArray, std::vector<int> SArray,
  std::vector<int> VArray, ColorRange *colorRange);

void Hsv2Rgb( float *r, float *g, float *b, float h, float s, float v );

#endif

