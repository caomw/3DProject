

#ifndef __5POINT_H__
#define __5POINT_H__

#include <opencv2/core/core.hpp>

// Call this function
bool Solve5PointEssential(double *pts1, double *pts2, int num_pts, std::vector<cv::Mat> &ret_E, std::vector<cv::Mat> &ret_P, std::vector<int> &ret_inliers);
cv::Mat TriangulatePoint(const cv::Point2d &pt1, const cv::Point2d &pt2, const cv::Mat &P1, const cv::Mat &P2);

#endif
