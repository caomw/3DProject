

#include "essentialMatrixCalculation.h"

// Select the Best pair of essential matrix and corresponding projection matrix
cv::Mat bestMatrixCalculation(cv::vector<cv::Mat> E, cv::vector<cv::Mat> P, cv::vector<int> inliers){
    int largestInlier = 0;
    cv::Mat bestE;
    cv::Mat bestP;
    cv::vector<cv::Mat> result;
    for(size_t i=0; i < E.size(); i++) {
        if(inliers[i] > largestInlier){
            bestE = E[i];
            bestP = P[i];
            largestInlier = inliers[i];
        }
    }
    return bestE;
}

// Self Customised RANSAC mechanism
int ransac(cv::Mat E, double pts1[],double pts2[], int size){
    int numberOfInliners = 0;
    int threshold = 0.001;
    for(int i = 0; i < size; i++){
        float x1 = pts1[i*2];
		float y1 = pts1[i*2+1];
		float x2 = pts2[i*2];
		float y2 = pts2[i*2+1];
        float Q[9] = {x1*x2,x2*y1,x2,x1*y2,y1*y2,y2,x1,y1,1};
        if(Q[0]*E.at<double>(0,0)+Q[1]*E.at<double>(0,1)+Q[2]*E.at<double>(0,2)+Q[3]*E.at<double>(1,0)+Q[4]*E.at<double>(1,1)+Q[5]*E.at<double>(1,2)+Q[6]*E.at<double>(2,0)+Q[7]*E.at<double>(2,1)+Q[8]*E.at<double>(2,2) < threshold){
            numberOfInliners ++;
        }
    }
    return numberOfInliners;
}

cv::Mat essentialMatrixCalculation(double *pts1, double *pts2, int num_pts){
    
    cv::vector <cv::Mat> E; // essential matrix
    cv::vector <cv::Mat> P; // 3x4 projection matrix
    cv::vector <int> inliers;
    bool ret;
    
    ret = Solve5PointEssential(pts1, pts2, num_pts, E, P, inliers);
    
    double end = cv::getTickCount();
    
    if(ret) {
        std::cout << "Success! Found " <<  E.size() << " possible solutions" <<"\n";
        std::cout << "The best one has the highest inliers. An inlier is a point that is in front of both cameras." << "\n";
        std::cout << "\n";
        
        for(size_t i=0; i < E.size(); i++) {
            std::cout << "Solution " << (i+1) << "/" << E.size() << "\n";
            std::cout << "\n";
            std::cout << "E = " << E[i] << "\n";
            std::cout << "\n";
            
            if(cv::determinant(P[i](cv::Range(0,3), cv::Range(0,3))) < 0) {
                std::cout << "Detected a reflection in P. Multiplying all values by -1." << "\n";
                std::cout << "P = " << (P[i] * -1) << "\n";
            }
            else {
                std::cout << "P = " << P[i] << "\n";
            }
            
            std::cout << "\n";
            std::cout << "inliers = " << inliers[i] << "/" << num_pts << "\n";
            std::cout << "=========================================" << end<<"\n";
            std::cout << end<<"\n";
           
        }
    }
    else {
        std::cout << "Could not find a valid essential matrix" << "\n";
    }
    return bestMatrixCalculation(E, P, inliers);
}