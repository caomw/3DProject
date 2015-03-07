

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "CommonFunctions.h"

/**
 Feature Matching Interface
 */
class IFeatureMatcher {
public:
	virtual void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches) = 0;
	virtual std::vector<cv::KeyPoint> GetImagePoints(int idx) = 0;
};
