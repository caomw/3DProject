#include "ValidateOpenCV.h"

int validateOpenCVInstall(string imgname) {
	Mat img = imread(imgname.c_str(), CV_LOAD_IMAGE_COLOR);
	int w = img.cols;
	int h = img.rows;
	Size s = Size(w, h);
	Mat grayImg = Mat(s, CV_8UC1);
	cvtColor(img, grayImg, COLOR_RGB2GRAY);
	namedWindow("Image", CV_WINDOW_AUTOSIZE);
	imshow("Image", img);
	waitKey(0);
	return 0;
}
