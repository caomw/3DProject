

#include "CommonFunctions.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#ifndef WIN32
#include <dirent.h>
#endif

using namespace std;
using namespace cv;

std::vector<cv::DMatch> FlipMatches(const std::vector<cv::DMatch>& matches) {
	std::vector<cv::DMatch> flip;
	for(int i=0;i<matches.size();i++) {
		flip.push_back(matches[i]);
		swap(flip.back().queryIdx,flip.back().trainIdx);
	}
	return flip;
}

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts) {
	std::vector<cv::Point3d> out;
	for (unsigned int i=0; i<cpts.size(); i++) {
		out.push_back(cpts[i].pt);
	}
	return out;
}

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
							   const std::vector<cv::KeyPoint>& imgpts2,
							   const std::vector<cv::DMatch>& matches,
							   std::vector<cv::KeyPoint>& pt_set1,
							   std::vector<cv::KeyPoint>& pt_set2) 
{
	for (unsigned int i=0; i<matches.size(); i++) {
		assert(matches[i].queryIdx < imgpts1.size());
		pt_set1.push_back(imgpts1[matches[i].queryIdx]);
		assert(matches[i].trainIdx < imgpts2.size());
		pt_set2.push_back(imgpts2[matches[i].trainIdx]);
	}	
}

void KeyPointsToPoints(const vector<KeyPoint>& kps, vector<Point2f>& ps) {
	ps.clear();
	for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}

void PointsToKeyPoints(const vector<Point2f>& ps, vector<KeyPoint>& kps) {
	kps.clear();
	for (unsigned int i=0; i<ps.size(); i++) kps.push_back(KeyPoint(ps[i],1.0f));
}
