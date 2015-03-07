


#include "FeatureMatcher.h"

#include "CameraMatricesCalculation.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <set>

using namespace std;
using namespace cv;

FeatureMatcher::FeatureMatcher(std::vector<cv::Mat>& imgs_,
									   std::vector<std::vector<cv::KeyPoint> >& imgpts_) :
	imgpts(imgpts_), imgs(imgs_)
{
    
    // Feature Dectector and extractor has multiple solutions here such as "SIFT", "SURF", etc
    
	detector = FeatureDetector::create("PyramidFAST");
	extractor = DescriptorExtractor::create("ORB");
	
    
	std::cout << " -------------------- extract feature points for all images -------------------\n";
	detector->detect(imgs, imgpts);
	extractor->compute(imgs, imgpts, descriptors);
	std::cout << " ------------------------------------- done -----------------------------------\n";
}	

void FeatureMatcher::MatchFeatures(int idx_i, int idx_j, vector<DMatch>* matches) {
    
    const vector<KeyPoint>& imgpts1 = imgpts[idx_i];
    const vector<KeyPoint>& imgpts2 = imgpts[idx_j];
    const Mat& descriptors_1 = descriptors[idx_i];
    const Mat& descriptors_2 = descriptors[idx_j];
    
    std::vector< DMatch > good_matches_,very_good_matches_;
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    
	stringstream ss; ss << "imgpts1 has " << imgpts1.size() << " points (descriptors " << descriptors_1.rows << ")" << endl;
    cout << ss.str();
	stringstream ss1; ss1 << "imgpts2 has " << imgpts2.size() << " points (descriptors " << descriptors_2.rows << ")" << endl;
    cout << ss1.str();
    
    keypoints_1 = imgpts1;
    keypoints_2 = imgpts2;
    
    if(descriptors_1.empty()) {
        CV_Error(0,"descriptors_1 is empty");
    }
    if(descriptors_2.empty()) {
        CV_Error(0,"descriptors_2 is empty");
    }
    
    // Matching descriptor vectors using fast Brute Force matcher
    BFMatcher matcher(NORM_HAMMING,true);
    std::vector< DMatch > matches_;
    if (matches == NULL) {
        matches = &matches_;
    }
    if (matches->size() == 0) {
        matcher.match( descriptors_1, descriptors_2, *matches );
    }
    
	assert(matches->size() > 0);
    
    vector<KeyPoint> imgpts1_good,imgpts2_good;
    
    std::set<int> existing_trainIdx;
    for(unsigned int i = 0; i < matches->size(); i++ )
    { 
        //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
        if ((*matches)[i].trainIdx <= 0) {
            (*matches)[i].trainIdx = (*matches)[i].imgIdx;
        }
        
        if( existing_trainIdx.find((*matches)[i].trainIdx) == existing_trainIdx.end() && 
           (*matches)[i].trainIdx >= 0 && (*matches)[i].trainIdx < (int)(keypoints_2.size()) /*&&
           (*matches)[i].distance > 0.0 && (*matches)[i].distance < cutoff*/ ) 
        {
            good_matches_.push_back( (*matches)[i]);
            imgpts1_good.push_back(keypoints_1[(*matches)[i].queryIdx]);
            imgpts2_good.push_back(keypoints_2[(*matches)[i].trainIdx]);
            existing_trainIdx.insert((*matches)[i].trainIdx);
        }
    }
    
    vector<uchar> status;
    vector<KeyPoint> imgpts2_very_good,imgpts1_very_good;
    
    assert(imgpts1_good.size() > 0);
	assert(imgpts2_good.size() > 0);
	assert(good_matches_.size() > 0);
	assert(imgpts1_good.size() == imgpts2_good.size() && imgpts1_good.size() == good_matches_.size());
	
    //Select features that make epipolar sense
    GetFundamentalMat(keypoints_1,keypoints_2,imgpts1_very_good,imgpts2_very_good,good_matches_);
}
