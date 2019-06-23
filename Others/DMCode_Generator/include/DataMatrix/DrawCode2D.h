#ifndef DRAW_CODE_2D_H_
#define DRAW_CODE_2D_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include <string>
#include "dataMatrix.h"

using namespace cv;

void DrawCode2D(const unsigned char* matrix, int dimension);
void DrawSqure(Mat img, unsigned char x, unsigned char y, int size, unsigned char color);
void DrawCode2D_5X2(const unsigned char* matrix, int dimension, int page);
void DrawCode2D_5X4(const unsigned char* matrix, int dimension, int page);
#endif