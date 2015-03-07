#ifndef _SFM_RECOTNRUCT
#define _SFM_RECOTNRUCT

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include "MultiViewPnP.h"
#include "5point.h"
#include "essentialMatrixCalculation.h"
#include "Polynomial.h"
#include "Rpoly.h"

using namespace std;
using namespace cv;

#ifndef MIN_VIEWS_FOR_PRUNING
#define MIN_VIEWS_FOR_PRUNING 3
#endif
int extractInt(string s);
void readImgInSequence(char *picFolder, vector<cv::Mat> *imgs,
		vector<std::string> *img_names);
void writePLY(cv::Ptr<MultiViewPnP> distance,string outfile);
int SFMReconstruct(string picFolder,string outfile);
#endif
