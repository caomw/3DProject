

#include "IFeatureMatcher.h"

class FeatureMatcher : public IFeatureMatcher {
private:
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	
	std::vector<cv::Mat> descriptors;
	
	std::vector<cv::Mat>& imgs; 
	std::vector<std::vector<cv::KeyPoint> >& imgpts;
public:
	FeatureMatcher(std::vector<cv::Mat>& imgs, 
					   std::vector<std::vector<cv::KeyPoint> >& imgpts);
	
	void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches = NULL);
	
	std::vector<cv::KeyPoint> GetImagePoints(int idx) { return imgpts[idx]; }
};
