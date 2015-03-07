

#ifndef ___D_Reconstruction__essentialMatrixCalculation__
#define ___D_Reconstruction__essentialMatrixCalculation__

#include <iostream>
#include <vector>
#include <assert.h>
#include <opencv2/core/core.hpp>
#include "5point.h"

cv::Mat essentialMatrixCalculation(double *pts1, double *pts2);
cv::Mat bestMatrixCalculation(cv::vector<cv::Mat> E, cv::vector<cv::Mat> P, cv::vector<int> inliers);

int ransac(cv::Mat E, double pts1[],double pts2[], int size);


#endif
