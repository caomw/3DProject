
#include <vector>
#include <opencv2/core/core.hpp>
#include "CommonFunctions.h"

class BAHandler {
public:
	void adjustBundle(std::vector<CloudPoint>& pointcloud, 
					  cv::Mat& cam_matrix,
					  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
					  std::map<int ,cv::Matx34d>& Pmats);
    
    // ntk
    void adjustBundle_vid(std::vector<CloudPoint>& pointcloud,
					  cv::Mat& cam_matrix,
					  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
					  std::map<int ,cv::Matx34d>& Pmats, std::vector<std::vector <int> >& mapViews, int nCams);
    
    // ntk
    void adjustBundle_pruned(std::vector<CloudPoint>& pointcloud,
					  cv::Mat& cam_matrix,
					  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
					  std::map<int ,cv::Matx34d>& Pmats);
    
    
private:
	int Count2DMeasurements(const std::vector<CloudPoint>& pointcloud);
    int Count2DMeasurements_pruned(const std::vector<CloudPoint>& pointcloud);
    
};
