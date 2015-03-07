
#include "MultiViewHandler.h"
#include "CommonFunctions.h"

class MultiViewPnP:public MultiViewHandler{
	std::vector<CloudPoint> pointcloud_beforeBA;
	std::vector<cv::Vec3b> pointCloudRGB_beforeBA;

public:
	MultiViewPnP(const std::vector<cv::Mat>& imgs_,	const std::vector<std::string>& imgs_names_):MultiViewHandler(imgs_,imgs_names_)
	{
	};

	void RecoverDepthFromImages();
    void RecoverDepthFromImagesVid();  // ntk

    bool IsGoodPair(int v1, int v2, bool forTriangulation);           // ntk
    bool IsGoodView(int view);    // ntk
    
	std::vector<cv::Point3d> getPointCloudBeforeBA() { return CloudPointsToPoints(pointcloud_beforeBA); }
	const std::vector<cv::Vec3b>& getPointCloudRGBBeforeBA() { return pointCloudRGB_beforeBA; }
	void CountPointViews();   // ntk
    std::vector<int> ptNViews; // ntk
    
    std::vector<std::vector <int> > mapViews;   // ntk

    
private:
	void PruneMatchesBasedOnF();
	void AdjustCurrentBundle();
    void AdjustCurrentBundle_vid();  // ntk
    void AdjustBundle_pruned();     // ntk
	void GetBaseLineTriangulation();
    
    bool GetBaseLineTriangulation_vid(int v1, int v2);  // ntk
        
	void Find2D3DCorrespondences(int working_view, 
		std::vector<cv::Point3f>& ppcloud, 
		std::vector<cv::Point2f>& imgPoints);
    
    void Find2D3DCorrespondences_vid(int working_view, int anchor_view,
                                 std::vector<cv::Point3f>& ppcloud,
                                 std::vector<cv::Point2f>& imgPoints);
    
	bool FindPoseEstimation(
		int working_view,
		cv::Mat_<double>& rvec,
		cv::Mat_<double>& t,
		cv::Mat_<double>& R,
		std::vector<cv::Point3f> ppcloud,
		std::vector<cv::Point2f> imgPoints);
	bool TriangulatePointsBetweenViews(
		int working_view, 
		int second_view,
		std::vector<struct CloudPoint>& new_triangulated,
		std::vector<int>& add_to_cloud
		);
    
    bool TriangulatePointsBetweenViews_vid(cv::Matx34d& P,
                                           cv::Matx34d& P1,
                                           int working_view,
                                           int older_view,
                                       std::vector<struct CloudPoint>& new_triangulated,
                                       std::vector<int>& add_to_cloud
                                       );
    
	//////void MultiViewPnP::writePLY();
	//////void MultiViewPnP::writePLYPruned();
    void writePLY();
	void writePLYPruned();
    
	
	int FindHomographyInliers2Views(int vi, int vj);
	int m_first_view;
	int m_second_view; //baseline's second view other to 0
	
    std::set<int> done_views;
	std::set<int> good_views;
};
