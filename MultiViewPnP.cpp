#include <cstdio>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include "MultiViewPnP.h"
#include "BAHandler.h"
#include "FeatureMatcher.h"  // ntk

using namespace std;

#include <opencv2/calib3d/calib3d.hpp>

#define _ZL_DEBUG

/********* to be restored 
 
 //ly processing acceleration flag assume ordered pics
 #ifndef ORDERED_INPUT
 //#define ORDERED_INPUT 0
 //#define ORDERED_INPUT 1			//0: default, 1: moving window algo
 #define ORDERED_INPUT 2             //2: fast version for ordered image sequence


 #define WINDOW_SIZE 3			// number of images in processing window, NOT used if ORDERED_INPUT==0
 #define WINDOW_STEP 1			//1: default, range=[1,WINDOW_SIZE) each time, scan window is moved by this amount of pictures, NOT used if ORDERED_INPUT==0
 #endif

 ************/

#define WINDOW_SIZE 3     // to be deleted

//ntk
#ifndef MIN_VIEWS_FOR_PRUNING
#define MIN_VIEWS_FOR_PRUNING 3
#endif

//ntk
#ifndef THRESH_GOOD_VIEW 
//#define THRESH_GOOD_VIEW_TRI 90
//#define THRESH_GOOD_VIEW 90
//#define THRESH_GOOD_VIEW_TRI 40
//#define THRESH_GOOD_VIEW 40

#define THRESH_GOOD_VIEW_TRI 70
#define THRESH_GOOD_VIEW 70

#define LOOK_AHEAD_NVIEWS 10
#endif

bool sort_by_first(pair<int, pair<int, int> > a, pair<int, pair<int, int> > b) {
	return a.first < b.first;
}

void MultiViewPnP::CountPointViews() {  // ntk
	ptNViews.clear();

	for (unsigned int pcldp = 0; pcldp < pcloud.size(); pcldp++) {
		int count = 0;
		for (int v = 0; v < pcloud[pcldp].imgpt_for_img.size(); v++) {
			if (pcloud[pcldp].imgpt_for_img[v] != -1) {
				count++;
			}
		}
		ptNViews.push_back(count);
	}
}
void MultiViewPnP::writePLY() {
	ofstream myfile;
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, 80, "%d-%m-%Y-%I-%M-%S", timeinfo);
	std::string str(buffer);

	int a = WINDOW_SIZE;
	int b = MIN_VIEWS_FOR_PRUNING;
	stringstream ss1;
	ss1 << a;
	string winsize = ss1.str();
	stringstream ss2;
	ss2 << b;
	string minview = ss2.str();
	std::string filename = "/Users/ngtk/pics/temp/" + str + "-w" + winsize
			+ "-v" + minview + ".ply";

	cout << filename << endl;
	myfile.open(filename.c_str());

	vector<cv::Point3d> cloud = getPointCloud();
	vector<cv::Vec3b> colors = getPointCloudRGB();

	//write point cloud to .ply file
	myfile << "ply" << endl;
	myfile << "format ascii 1.0" << endl;

	myfile << "element vertex " << cloud.size() << endl;    // to be deleted

	myfile << "property float x" << endl;
	myfile << "property float y" << endl;
	myfile << "property float z" << endl;
	myfile << "property uchar red" << endl;
	myfile << "property uchar green" << endl;
	myfile << "property uchar blue" << endl;

	myfile << "end_header" << endl;

	for (unsigned int n = 0; n < cloud.size(); n++) {
		float x = cloud.at(n).x;
		float y = cloud.at(n).y;
		float z = cloud.at(n).z;
		int r = colors.at(n)[0];
		int g = colors.at(n)[1];
		int b = colors.at(n)[2];
		myfile << x << " " << y << " " << z << " " << b << " " << g << " " << r
				<< endl;
	}
}

void MultiViewPnP::writePLYPruned() {
	ofstream myfile;
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, 80, "%d-%m-%Y-%I-%M-%S", timeinfo);
	std::string str(buffer);

	int a = WINDOW_SIZE;
	int b = MIN_VIEWS_FOR_PRUNING;
	stringstream ss1;
	ss1 << a;
	string winsize = ss1.str();
	stringstream ss2;
	ss2 << b;
	string minview = ss2.str();

	//std::string filename = "D:/MyFolder/y4/FYP/3Dvs/outputs/"+str+"-w"+winsize+"-v"+minview+"_pruned.ply";
	std::string filename = "/Users/ngtk/pics/temp/" + str + "-w" + winsize
			+ "-v" + minview + "_pruned.ply";

	cout << filename << endl;
	myfile.open(filename.c_str());

	/********** to be restored

	 vector<cv::Matx34d> cameras = getCameras();

	 **********/

	vector<cv::Point3d> cloud = getPointCloud();
	vector<cv::Vec3b> colors = getPointCloudRGB();

	CountPointViews();
	int goodPts = 0;

	assert(ptNViews.size() == cloud.size());
	if (ptNViews.size() != cloud.size())
		cout << "unequal size" << endl;

	for (unsigned int i = 0; i < ptNViews.size(); i++) {
		if (ptNViews[i] >= MIN_VIEWS_FOR_PRUNING) {
			goodPts++;
		}
	}

	cout << "draw goodPts = " << goodPts << " out of " << cloud.size() << endl;

	//write point cloud to .ply file
	myfile << "ply" << endl;
	myfile << "format ascii 1.0" << endl;

	myfile << "element vertex " << goodPts << endl;    // to be deleted

	myfile << "property float x" << endl;
	myfile << "property float y" << endl;
	myfile << "property float z" << endl;
	myfile << "property uchar red" << endl;
	myfile << "property uchar green" << endl;
	myfile << "property uchar blue" << endl;

	myfile << "end_header" << endl;

	for (unsigned int n = 0; n < cloud.size(); n++) {
		if (ptNViews[n] >= MIN_VIEWS_FOR_PRUNING) {
			float x = cloud.at(n).x;
			float y = cloud.at(n).y;
			float z = cloud.at(n).z;
			int r = colors.at(n)[0];
			int g = colors.at(n)[1];
			int b = colors.at(n)[2];
			myfile << x << " " << y << " " << z << " " << b << " " << g << " "
					<< r << endl;
		}
	}
	myfile.close();
}

int MultiViewPnP::FindHomographyInliers2Views(int vi, int vj) {
	vector<cv::KeyPoint> ikpts, jkpts;
	vector<cv::Point2f> ipts, jpts;
	GetAlignedPointsFromMatch(imgpts[vi], imgpts[vj],
			matches_matrix[make_pair(vi, vj)], ikpts, jkpts);
	KeyPointsToPoints(ikpts, ipts);
	KeyPointsToPoints(jkpts, jpts);

	double minVal, maxVal;
	cv::minMaxIdx(ipts, &minVal, &maxVal); //TODO flatten point2d?? or it takes max of width and height

	vector<uchar> status;
	cv::Mat H = cv::findHomography(ipts, jpts, status, CV_RANSAC,
			0.004 * maxVal); //threshold from Snavely07
	return cv::countNonZero(status); //number of inliers
}

// ntk
bool MultiViewPnP::IsGoodView(int view) {
	for (set<int>::iterator done_view = good_views.begin();
			done_view != good_views.end(); ++done_view) {
		if (view == *done_view) {
			return true;
		}
	}
	return false;
}

// ntk
/**
 * test whether 2 views are good for triangulation
 */
bool MultiViewPnP::IsGoodPair(int v1, int v2, bool forTriangulation) {
	std::cout << "================== IsGoodPair ==========" << "v1 = " << v1
			<< "   v2 = " << v2 << "\n";

	list<pair<int, pair<int, int> > > matches_sizes;

	int Hinliers;
	int percent = 200;

	for (std::map<std::pair<int, int>, std::vector<cv::DMatch> >::iterator i =
			matches_matrix.begin(); i != matches_matrix.end(); ++i) {

		if (((*i).first.first == v1) && ((*i).first.second == v2)) {

			if ((*i).second.size() < 100)
				matches_sizes.push_back(make_pair(100, (*i).first));
			else {
				Hinliers = FindHomographyInliers2Views((*i).first.first,
						(*i).first.second);
				percent = (int) (((double) Hinliers)
						/ ((double) (*i).second.size()) * 100.0);
				cout << "[" << (*i).first.first << "," << (*i).first.second
						<< " = " << percent << "] ";
				cout << endl;
			}
			break;
		}
	}

	if (forTriangulation) {
		if (percent <= THRESH_GOOD_VIEW_TRI) {
			return true;
		} else {
			return false;
		}
	} else {
		if (percent <= THRESH_GOOD_VIEW) {
			return true;
		} else {
			return false;
		}

	}

}

/**
 * Get an initial 3D point cloud from 2 views only
 */
bool MultiViewPnP::GetBaseLineTriangulation_vid(int v1, int v2) {
	std::cout
			<< "================== Baseline triangulation for Video input ===================\n";

	cv::Matx34d P(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0), P1(1, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 1, 0);

	std::vector<CloudPoint> tmp_pcloud;

	//Reconstruct from two views
	bool goodF = false;
	m_first_view = v1;
	m_second_view = v2;

	/**
	 //reverse iterate by number of matches
	 for(list<pair<int,pair<int,int> > >::iterator highest_pair = matches_sizes.begin();
	 highest_pair != matches_sizes.end() && !goodF;
	 ++highest_pair)
	 {
	 m_second_view = (*highest_pair).second.second;
	 m_first_view  = (*highest_pair).second.first;
	 **/

	std::cout << " -------- " << imgs_names[m_first_view] << " and "
			<< imgs_names[m_second_view] << " -------- " << std::endl;

	/***
	 std::cout << "(*highest_pair).first         = " << (*highest_pair).first << std::endl;
	 std::cout << "(*highest_pair).second.first  = " << (*highest_pair).second.first << std::endl;
	 std::cout << "(*highest_pair).second.second = " << (*highest_pair).second.second << std::endl;
	 ****/

	// See if the Fundamental Matrix between these two views is good
	goodF = FindCameraMatrices(K, Kinv, distortion_coeff, imgpts[m_first_view],
			imgpts[m_second_view], imgpts_good[m_first_view],
			imgpts_good[m_second_view], P, P1,
			matches_matrix[std::make_pair(m_first_view, m_second_view)],
			tmp_pcloud);

	std::cout << "goodF = " << goodF << std::endl;

	if (goodF) {
		vector<CloudPoint> new_triangulated;
		vector<int> add_to_cloud;

		// ntk
		/**
		 Pmats[m_first_view] = P;
		 Pmats[m_second_view] = P1;
		 **/

		//bool good_triangulation = TriangulatePointsBetweenViews(m_second_view,m_first_view,new_triangulated,  add_to_cloud);
		bool good_triangulation = TriangulatePointsBetweenViews_vid(P1, P,
				m_second_view, m_first_view, new_triangulated, add_to_cloud);

		if (!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
			std::cout << "triangulation failed" << std::endl;
			goodF = false;
			// ntk
			/***
			 Pmats[m_first_view] = 0;
			 Pmats[m_second_view] = 0;
			 ***/
			////// m_second_view++;
		} else {

			// ntk

			Pmats[m_first_view] = P;
			Pmats[m_second_view] = P1;

			std::cout << "before triangulation: " << pcloud.size();
			for (unsigned int j = 0; j < add_to_cloud.size(); j++) {
				if (add_to_cloud[j] == 1)
					pcloud.push_back(new_triangulated[j]);
			}
			std::cout << " after " << pcloud.size() << std::endl;
		}

	} else if (!goodF) {
		cerr
				<< "...........Cannot find a good pair of images to obtain a baseline triangulation"
				<< endl;
		//exit(0);
	}

	cout << ">>>>>>>> In GetBaseLineTriangulation, Pmats.size() = "
			<< Pmats.size() << endl;

	//cout << "Taking baseline from " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << endl;

	if (goodF) {
		return true;
	} else {
		return false;
	}
}

//ntk
void MultiViewPnP::Find2D3DCorrespondences_vid(int working_view,
		int anchor_view, std::vector<cv::Point3f>& ppcloud,
		std::vector<cv::Point2f>& imgPoints) {
	ppcloud.clear();
	imgPoints.clear();
	vector<int> pcloud_status(pcloud.size(), 0);

//  ntk
//	for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view)
//	{

	///////int old_view = *done_view;  // ntk
	int old_view = anchor_view;

	//check for matches_from_old_to_working between i'th frame and <old_view>'th frame (and thus the current cloud)
	std::vector<cv::DMatch> matches_from_old_to_working =
			matches_matrix[std::make_pair(old_view, working_view)];

	for (unsigned int match_from_old_view = 0;
			match_from_old_view < matches_from_old_to_working.size();
			match_from_old_view++) {
		// the index of the matching point in <old_view>
		int idx_in_old_view =
				matches_from_old_to_working[match_from_old_view].queryIdx;

		//scan the existing cloud (pcloud) to see if this point from <old_view> exists
		for (unsigned int pcldp = 0; pcldp < pcloud.size(); pcldp++) {
			// see if corresponding point was found in this point
			if (idx_in_old_view == pcloud[pcldp].imgpt_for_img[old_view]
					&& pcloud_status[pcldp] == 0) //prevent duplicates
							{
				//3d point in cloud
				ppcloud.push_back(pcloud[pcldp].pt);
				//2d point in image i
				imgPoints.push_back(
						imgpts[working_view][matches_from_old_to_working[match_from_old_view].trainIdx].pt);

				pcloud_status[pcldp] = 1;
				break;
			}
		}
	}
	cout << "found " << ppcloud.size() << " point correspondences between view "
			<< old_view << " and view " << working_view << endl;
	//}
}

bool MultiViewPnP::FindPoseEstimation(int working_view, cv::Mat_<double>& rvec,
		cv::Mat_<double>& t, cv::Mat_<double>& R,
		std::vector<cv::Point3f> ppcloud, std::vector<cv::Point2f> imgPoints) {
	if (ppcloud.size() <= 7 || imgPoints.size() <= 7
			|| ppcloud.size() != imgPoints.size()) {
		//something went wrong aligning 3D to 2D points..
		cerr << "couldn't find [enough] corresponding cloud points... (only "
				<< ppcloud.size() << ")" << endl;
		return false;
	}

	vector<int> inliers;
	double minVal, maxVal;
	cv::minMaxIdx(imgPoints, &minVal, &maxVal);
	CV_PROFILE("solvePnPRansac",
			cv::solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t,
					true, 1000, 0.006 * maxVal,
					0.25 * (double )(imgPoints.size()), inliers, CV_EPNP)
			;
	)

	vector<cv::Point2f> projected3D;
	cv::projectPoints(ppcloud, rvec, t, K, distortion_coeff, projected3D);

	if (inliers.size() == 0) { //get inliers
		for (int i = 0; i < projected3D.size(); i++) {
			if (norm(projected3D[i] - imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}
	}

	if (inliers.size() < (double) (imgPoints.size()) / 5.0) {
		cerr << "not enough inliers to consider a good pose (" << inliers.size()
				<< "/" << imgPoints.size() << ")" << endl;
		return false;
	}

	if (cv::norm(t) > 200.0) {
		cerr << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	cv::Rodrigues(rvec, R);
	if (!CheckCoherentRotation(R)) {
		cerr << "rotation is incoherent. we should try a different base view..."
				<< endl;
		return false;
	}

	std::cout << "found t = " << t << "\nR = \n" << R << std::endl;
	return true;
}

//ntk
bool MultiViewPnP::TriangulatePointsBetweenViews_vid(cv::Matx34d& P1,
		cv::Matx34d& P, int working_view, int older_view,
		vector<struct CloudPoint>& new_triangulated,
		vector<int>& add_to_cloud) {
	cout << " Triangulate " << imgs_names[working_view] << " and "
			<< imgs_names[older_view] << endl;
	//get the left camera matrix
	//TODO: potential bug - the P mat for <view> may not exist? or does it...

	// ntk
	/////cv::Matx34d P = Pmats[older_view];
	/////cv::Matx34d P1 = Pmats[working_view];

	std::vector<cv::KeyPoint> pt_set1, pt_set2;
	std::vector<cv::DMatch> matches = matches_matrix[std::make_pair(older_view,
			working_view)];
	GetAlignedPointsFromMatch(imgpts[older_view], imgpts[working_view], matches,
			pt_set1, pt_set2);

	//adding more triangulated points to general cloud
	double reproj_error = TriangulatePoints(pt_set1, pt_set2, K, Kinv,
			distortion_coeff, P, P1, new_triangulated, correspImg1Pt);
	std::cout << "triangulation reproj error " << reproj_error << std::endl;

	vector<uchar> trig_status;
	if (!TestTriangulation(new_triangulated, P, trig_status)
			|| !TestTriangulation(new_triangulated, P1, trig_status)) {
		cerr << "Triangulation did not succeed" << endl;
		return false;
	}

	//filter out outlier points with high reprojection
	vector<double> reprj_errors;
	for (int i = 0; i < new_triangulated.size(); i++) {
		reprj_errors.push_back(new_triangulated[i].reprojection_error);
	}
	std::sort(reprj_errors.begin(), reprj_errors.end());
	//get the 80% percentile
	double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2

	// ntk
	std::cout << "@@@@@@@@@@@@@@@@@@@@" << reprj_err_cutoff << std::endl;

	vector<CloudPoint> new_triangulated_filtered;
	std::vector<cv::DMatch> new_matches;
	for (int i = 0; i < new_triangulated.size(); i++) {
		if (trig_status[i] == 0)
			continue; //point was not in front of camera
		if (new_triangulated[i].reprojection_error > 16.0) {
			continue; //reject point
		}

		if (new_triangulated[i].reprojection_error < 4.0
				|| new_triangulated[i].reprojection_error < reprj_err_cutoff) {
			new_triangulated_filtered.push_back(new_triangulated[i]);
			new_matches.push_back(matches[i]);
		} else {
			continue;
		}
	}

	cout << "filtered out "
			<< (new_triangulated.size() - new_triangulated_filtered.size())
			<< " high-error points" << endl;

	//all points filtered?
	if (new_triangulated_filtered.size() <= 0)
		return false;

	new_triangulated = new_triangulated_filtered;

	matches = new_matches;
	matches_matrix[std::make_pair(older_view, working_view)] = new_matches; //just to make sure, remove if unneccesary
	matches_matrix[std::make_pair(working_view, older_view)] = FlipMatches(
			new_matches);
	add_to_cloud.clear();
	add_to_cloud.resize(new_triangulated.size(), 1);
	int found_other_views_count = 0;
	int num_views = imgs.size();

	//scan new triangulated points, if they were already triangulated before - strengthen cloud
	for (int j = 0; j < new_triangulated.size(); j++) {
		new_triangulated[j].imgpt_for_img = std::vector<int>(imgs.size(), -1);
		new_triangulated[j].imgpt_for_img[older_view] = matches[j].queryIdx; //2D reference to <older_view>
		new_triangulated[j].imgpt_for_img[working_view] = matches[j].trainIdx; //2D reference to <working_view>
		bool found_in_other_view = false;
		for (unsigned int view_ = 0; view_ < num_views; view_++) {
			if (view_ != older_view) {
				//Look for points in <view_> that match to points in <working_view>
				std::vector<cv::DMatch> submatches =
						matches_matrix[std::make_pair(view_, working_view)];
				for (unsigned int ii = 0; ii < submatches.size(); ii++) {
					if (submatches[ii].trainIdx == matches[j].trainIdx
							&& !found_in_other_view) {
						for (unsigned int pt3d = 0; pt3d < pcloud.size();
								pt3d++) {
							if (pcloud[pt3d].imgpt_for_img[view_]
									== submatches[ii].queryIdx) {
#pragma omp critical
								{
									pcloud[pt3d].imgpt_for_img[working_view] =
											matches[j].trainIdx;
									pcloud[pt3d].imgpt_for_img[older_view] =
											matches[j].queryIdx;
									found_in_other_view = true;
									add_to_cloud[j] = 0;
								}
							}
						}
					}
				}
			}
		}
#pragma omp critical
		{
			if (found_in_other_view) {
				found_other_views_count++;
			} else {
				add_to_cloud[j] = 1;
			}
		}
	}
	std::cout << found_other_views_count << "/" << new_triangulated.size()
			<< " points were found in other views, adding "
			<< cv::countNonZero(add_to_cloud) << " new\n";
	return true;
}

void MultiViewPnP::AdjustCurrentBundle_vid() {
	cout
			<< "======================== Bundle Adjustment (vid) ==========================\n";

	pointcloud_beforeBA = pcloud;
	GetRGBForPointCloud(pointcloud_beforeBA, pointCloudRGB_beforeBA);

	// ntk
	cv::Mat _cam_matrix = K;
	/**** for IR
	 float foc = 570.0;
	 float ou  = 285.0;
	 float ov  = 238.0;
	 ****/
	/**** for RGB (DSO B2) **
	 float foc = 720.0;
	 float ou  = 360.0;
	 float ov  = 288.0;
	 ****/
	/**** for RGB (punggol_input) **
	 float foc = 1056.0*1.0;
	 float ou  = 528.0;
	 float ov  = 288.0;
	 ****/
	/**** for RGB (HDB models, iPhone5 **/
	float foc = 1920.0;
	float ou = 960.0;
	float ov = 540.0;
	/****/
	/*** students of cs4243 using genetic algo gives **
	 float foc = 1226/2;
	 float ou  = 792.0;
	 float ov  = 363.3;
	 ****/

	_cam_matrix.at<double>(0, 0) = foc;
	_cam_matrix.at<double>(0, 1) = 0.0;
	_cam_matrix.at<double>(0, 2) = ou;
	_cam_matrix.at<double>(1, 0) = 0.0;
	_cam_matrix.at<double>(1, 1) = foc;
	_cam_matrix.at<double>(1, 2) = ov;
	_cam_matrix.at<double>(2, 0) = 0.0;
	_cam_matrix.at<double>(2, 1) = 0.0;
	_cam_matrix.at<double>(2, 2) = 1.0;
	cout << "++++++++ K = " << K.at<double>(0, 0) << ", " << K.at<double>(0, 1)
			<< ", " << K.at<double>(0, 2) << endl;
	cout << "++++++++ K = " << K.at<double>(1, 0) << ", " << K.at<double>(1, 1)
			<< ", " << K.at<double>(1, 2) << endl;
	cout << "++++++++ K = " << K.at<double>(2, 0) << ", " << K.at<double>(2, 1)
			<< ", " << K.at<double>(2, 2) << endl;
	/***/

	BAHandler BA;
	//BA.adjustBundle(pcloud,_cam_matrix,imgpts,Pmats);
	int nCams = good_views.size();
	BA.adjustBundle_vid(pcloud, _cam_matrix, imgpts, Pmats, mapViews, nCams);

	K = cam_matrix;
	Kinv = K.inv();
	cout << "use new K " << endl << K << endl;

	GetRGBForPointCloud(pcloud, pointCloudRGB);

	/******* to be restored

	 writePLY();

	 ********/

	writePLYPruned();
}

void MultiViewPnP::PruneMatchesBasedOnF() {
	for (unsigned int _i = 0; _i < imgs.size() - 1; _i++) {
		for (unsigned int _j = _i + 1; _j < imgs.size(); _j++) {
			int older_view = _i, working_view = _j;
			GetFundamentalMat(imgpts[older_view], imgpts[working_view],
					imgpts_good[older_view], imgpts_good[working_view],
					matches_matrix[std::make_pair(older_view, working_view)]);
			//update flip matches as well
			matches_matrix[std::make_pair(working_view, older_view)] =
					FlipMatches(
							matches_matrix[std::make_pair(older_view,
									working_view)]);
		}
	}
}

void MultiViewPnP::RecoverDepthFromImagesVid() {
	std::cout
			<< "======================================================================\n";
	std::cout
			<< "======================== Video Depth Recovery Start ========================\n";
	std::cout
			<< "======================================================================\n";

	feature_matcher = new FeatureMatcher(imgs, imgpts);
	for (unsigned int i = 0; i < imgs.size(); i++) {
		std::vector<int> row;
		mapViews.push_back(row);
	}

	// seek first pair to do triangulation
	int anchorView = 0; //current search view
	int previous_anchorView = 0; //previous search view

	int baselineTriangulationDone = 0; //no initial traingulation
	bool stop_search = false;
#ifdef _ZL_DEBUG
	ofstream fp;
	fp.open("log.txt",ofstream::out);
#endif
	while (!stop_search) {
		bool done_k = false;
		int end_search_for_k = anchorView + LOOK_AHEAD_NVIEWS; //search space
		if (end_search_for_k > (int) imgs.size()) {
			end_search_for_k = (int) imgs.size();
		}
		for (int k = anchorView + 1; k < end_search_for_k && (!done_k); k++) {
#ifdef _ZL_DEBUG
			fp<<"1111::k value: "<<k<<" anchorView: "<<anchorView<<" done_k:"<<done_k<<" img size:"<<imgs.size()<<" stop_search: "<<stop_search<<
			" good_views size:"<< good_views.size()<<endl;
#endif
			if (baselineTriangulationDone == 0) {
#ifdef _ZL_DEBUG
				fp<<"2222::k value: "<<k<<" anchorView: "<<anchorView<<"  done_k:"<<done_k<<"   img size:"<<imgs.size()<<" stop_search: "<<stop_search<<
				" good_views size:"<< good_views.size()<<endl;
#endif
				std::cout << "=====" << anchorView << "," << k << "====="
						<< std::endl;
				std::cout << "------------ Match " << imgs_names[anchorView]
						<< "," << imgs_names[k] << " ------------\n";
				std::vector<cv::DMatch> matches_tmp;
				feature_matcher->MatchFeatures(anchorView, k, &matches_tmp);
				matches_matrix[std::make_pair(anchorView, k)] = matches_tmp;
				std::vector<cv::DMatch> matches_tmp_flip = FlipMatches(
						matches_tmp);
				matches_matrix[std::make_pair(k, anchorView)] =
						matches_tmp_flip;
				cout << "@@@@@ pruning between pair (" << anchorView << "," << k
						<< ")" << endl;
				//Fundamental matrix itself is not used. Just to prune the matched matrix
				GetFundamentalMat(imgpts[anchorView], imgpts[k],
						imgpts_good[anchorView], imgpts_good[k],
						matches_matrix[std::make_pair(anchorView, k)]);
				//update flip matches as well
				matches_matrix[std::make_pair(k, anchorView)] = FlipMatches(
						matches_matrix[std::make_pair(anchorView, k)]);
			}

			// match k with most recent good views
			int startIndex = good_views.size() - WINDOW_SIZE;
			int count = 0;
			for (set<int>::iterator done_view = good_views.begin();done_view != good_views.end(); ++done_view) {
				cout << "*done_view = " << *done_view<< "......................." << endl;
				int view = *done_view;
				cout << "good_views.size() = " << good_views.size()	<< ",  current good_views  = " << view << endl;
				if (count < startIndex) {
					cout << "good_views skipped = " << view << endl;
					count++;
					continue;
				}
#ifdef _ZL_DEBUG
					fp<<"3333::k value: "<<k<<" anchorView: "<<anchorView<<"  done_view:"<<*done_view<<" img size:"<<imgs.size()<<" stop_search: "
					<<stop_search<<" good_views size:"<< good_views.size() << endl;
#endif
					std::vector<cv::DMatch> matches_tmp;
					feature_matcher->MatchFeatures(k, view, &matches_tmp);
					matches_matrix[std::make_pair(k, view)] = matches_tmp;
					std::vector<cv::DMatch> matches_tmp_flip = FlipMatches(
							matches_tmp);
					matches_matrix[std::make_pair(view, k)] = matches_tmp_flip;

					cout << "@@@@@ pruning between pair (" << k << "," << view
							<< ")" << endl;
					//Fundamental matrix itself is not used. Just to prune the matched matrix
					GetFundamentalMat(imgpts[k], imgpts[view], imgpts_good[k],
							imgpts_good[view],
							matches_matrix[std::make_pair(k, view)]);
					matches_matrix[std::make_pair(view, k)] = FlipMatches(
							matches_matrix[std::make_pair(k, view)]);
				}
			if (baselineTriangulationDone == 0) { //call once only for first good pairs. After this, we get initial point cloud
				if (IsGoodPair(anchorView, k, true)) {
					if (GetBaseLineTriangulation_vid(anchorView, k)) {
						mapViews[0].push_back(anchorView);
						mapViews[1].push_back(k);
						baselineTriangulationDone = 1;
						good_views.insert(anchorView);
						good_views.insert(k);
						anchorView = k;
						done_k = true;
						AdjustCurrentBundle_vid();
					}
				}
			} else {
				if (IsGoodPair(anchorView, k, false)) {
					vector<cv::Point3f> tmp3d;
					vector<cv::Point2f> tmp2d;
					Find2D3DCorrespondences_vid(k, anchorView, tmp3d, tmp2d);
					cv::Mat_<double> rvec(1, 3), t, R;
					bool pose_estimated = FindPoseEstimation(k, rvec, t, R,
							tmp3d, tmp2d);
					if (pose_estimated) { // pose estimated
						Pmats[k] = cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
								R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0),
								R(2, 1), R(2, 2), t(2));
						mapViews[good_views.size()].push_back(k);
						cout << "mapViews[good_views.size()] = "
								<< mapViews[good_views.size()][0];

						good_views.insert(k);
						anchorView = k;
						done_k = true;
						cout << "************ good_views.size() = "
								<< good_views.size() << endl;
						int startIndex = good_views.size() - WINDOW_SIZE - 1;

						int count = 0;
						for (set<int>::iterator done_view = good_views.begin();
								done_view != good_views.end(); ++done_view) {

							cout << "*done_view = " << *done_view
									<< "......................." << endl;
							int view = *done_view;
							cout << "good_views.size() = " << good_views.size()
									<< ",  current good_views  = " << view
									<< endl;
							if (count < startIndex) {
								cout << "good_views skipped = " << view << endl;
								count++;
								continue;
							}
#ifdef _ZL_DEBUG
							fp<<"4444::k value: "<<k<<" anchorView: "<<anchorView<<" done_view:"<<*done_view<<" img size:"<<imgs.size()<<" stop_search: "<<stop_search
							<<" good_views size:"<<good_views.size()<<endl;
#endif
							if (k != view) {
								vector<CloudPoint> new_triangulated;
								vector<int> add_to_cloud;
								bool good_triangulation =
										TriangulatePointsBetweenViews_vid(
												Pmats[view], Pmats[k], view, k,
												new_triangulated, add_to_cloud);
								if (!good_triangulation)
									continue;
								std::cout << "before triangulation: "
										<< pcloud.size();
								for (unsigned int j = 0;
										j < add_to_cloud.size(); j++) {
									if (add_to_cloud[j] == 1)
										pcloud.push_back(new_triangulated[j]);
								}
								std::cout << " after " << pcloud.size()
										<< std::endl;
								AdjustCurrentBundle_vid();
							}
						}
					}
				} else { // pose not estimated
					cout << "Not a good pair! " << endl;
					cout << " k = " << k << "   anchorView = " << anchorView
							<< endl;
				}
				if ((unsigned int) k >= imgs.size() - 1) {
					done_k = true;
					stop_search = true;
				}
			} //end of non-first traigulation
		} //end of current anchor view search

		cout << "baseLineTriangulationDone = " << baselineTriangulationDone	<< endl;
		cout << "done_k = " << done_k << endl;

		if (baselineTriangulationDone == 0) {
			anchorView++;
		}
		if ((unsigned int) anchorView == imgs.size() - 1) {
			stop_search = true;
		}

		if (previous_anchorView == anchorView) {
			stop_search = true;
		} else {
			previous_anchorView = anchorView;
		}

		cout << "good_views.size() = " << good_views.size() << endl;
		cout << "new anchorView = " << anchorView << endl;
		for (unsigned int v = 0; v < good_views.size(); v++) {
			cout << "mapViews = " << v << ", " << mapViews[v][0] << endl;
		}
	}
#ifdef _ZL_DEBUG
	fp.close();
#endif
	cout
			<< "======================================================================"
			<< endl;
	cout
			<< "========================= Depth Recovery DONE ========================"
			<< endl;
	cout
			<< "======================================================================"
			<< endl;
}

