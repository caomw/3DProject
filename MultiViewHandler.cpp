

#include "MultiViewHandler.h"
#include "FeatureMatcher.h"

/****** to be restored

//ly processing acceleration flag assume ordered pics
#ifndef ORDERED_INPUT
#define ORDERED_INPUT 2			//0: default, 1: moving window algo, 2: fast algo for video
//#define ORDERED_INPUT 1
//#define ORDERED_INPUT 0
#define WINDOW_SIZE 3			// number of images in processing window, NOT used if ORDERED_INPUT==0
#define WINDOW_STEP 1			//1: default, range=[1,WINDOW_SIZE) each time, scan window is moved by this amount of pictures, NOT used if ORDERED_INPUT==0
#endif
*************/


MultiViewHandler::MultiViewHandler(
	const std::vector<cv::Mat>& imgs_,
	const std::vector<std::string>& imgs_names_):
imgs_names(imgs_names_),features_matched(false),use_rich_features(true)
{
	std::cout << "=========================== Load Images ===========================\n";
	for (unsigned int i=0; i<imgs_.size(); i++) {
		imgs_orig.push_back(cv::Mat_<cv::Vec3b>());
		if (!imgs_[i].empty()) {
				imgs_[i].copyTo(imgs_orig[i]);
        }

		imgs.push_back(cv::Mat());
		cvtColor(imgs_orig[i],imgs[i], CV_BGR2GRAY);

		imgpts.push_back(std::vector<cv::KeyPoint>());
		imgpts_good.push_back(std::vector<cv::KeyPoint>());
		std::cout << ".";
	}
	std::cout << std::endl;

	//load calibration matrix
    cv::Size imgs_size = imgs_[0].size();
    double max_w_h = MAX(imgs_size.height,imgs_size.width);
    cam_matrix = (cv::Mat_<double>(3,3) <<	max_w_h , 0	, imgs_size.width/2.0, 0, max_w_h, imgs_size.height/2.0, 0,	0, 1);
    distortion_coeff = cv::Mat_<double>::zeros(1,4);

	K = cam_matrix;
	invert(K, Kinv); //get inverse of camera matrix

	distortion_coeff.convertTo(distcoeff_32f,CV_32FC1);
	K.convertTo(K_32f,CV_32FC1);
}



void MultiViewHandler::GetRGBForPointCloud(
	const std::vector<struct CloudPoint>& _pcloud,
	std::vector<cv::Vec3b>& RGBforCloud
	) 
{
	RGBforCloud.resize(_pcloud.size());
	for (unsigned int i=0; i<_pcloud.size(); i++) {
		unsigned int good_view = 0;
		std::vector<cv::Vec3b> point_colors;
		for(; good_view < imgs_orig.size(); good_view++) {
			if(_pcloud[i].imgpt_for_img[good_view] != -1) {
				int pt_idx = _pcloud[i].imgpt_for_img[good_view];
				if(pt_idx >= imgpts[good_view].size()) {
					std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << imgpts[good_view].size() << std::endl;
					continue;
				}
				cv::Point _pt = imgpts[good_view][pt_idx].pt;
				assert(good_view < imgs_orig.size() && _pt.x < imgs_orig[good_view].cols && _pt.y < imgs_orig[good_view].rows);
				
				point_colors.push_back(imgs_orig[good_view].at<cv::Vec3b>(_pt));
				
			}
		}
		cv::Scalar res_color = cv::mean(point_colors);
		RGBforCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
		if(good_view == imgs.size()) //nothing found
			RGBforCloud.push_back(cv::Vec3b(255,0,0));
	}
}
