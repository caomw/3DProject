#include "SFMReconstruct.h"

int extractInt(string s) {
	int num = 0;
	for (unsigned int i = 0; i < s.length(); i++) {
		if (s.at(i) >= '0' && s.at(i) <= '9') {
			num = num * 10 + (s.at(i) - '0');
		}
	}
	return num;
}

void readImgInSequence(string picFolder, vector<cv::Mat> *imgs,vector<std::string> *img_names) {
	DIR *dir;
	struct dirent *ent;
	cv::Mat img;
	if ((dir = opendir(picFolder.c_str())) != NULL) {
		//create vectors for sorting
		vector<string> sortedFileNames;
		vector<string> fileNames;
		while ((ent = readdir(dir)) != NULL) {
			if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
				continue;
			fileNames.push_back(string(ent->d_name));
		}
		while (!fileNames.empty()) {
			vector<string>::iterator minFileName = fileNames.begin();
			for (vector<string>::iterator i = fileNames.begin();
					i != fileNames.end(); i++) {
				if (extractInt(*i) < extractInt(*minFileName))
					minFileName = i;
			}
			sortedFileNames.push_back(*minFileName);
			fileNames.erase(minFileName);
		}
		closedir(dir);
		string fileName;
		for (unsigned int i = 0; i < sortedFileNames.size(); i++) {
			fileName = sortedFileNames.at(i);
			img = cv::imread(picFolder + fileName, CV_LOAD_IMAGE_COLOR);
			if (!img.data) {
				cout << "could not open or find the image" << endl;
			}
			imgs->push_back(img);
			img_names->push_back(fileName);
		}
	} else {
		/* could not open directory */
		perror("");
	}
}
void writePLY(cv::Ptr<MultiViewPnP> distance,string outfile) {
	ofstream myfile;
	myfile.open(outfile.c_str());
	vector<cv::Matx34d> cameras = distance->getCameras();
	vector<cv::Point3d> cloud = distance->getPointCloud();
	vector<cv::Vec3b> colors = distance->getPointCloudRGB();
	myfile << "ply" << endl;
	myfile << "format ascii 1.0" << endl;
	myfile << "element vertex " << cloud.size() + cameras.size() * 4 << endl;
	myfile << "property float x" << endl;
	myfile << "property float y" << endl;
	myfile << "property float z" << endl;
	myfile << "property uchar red" << endl;
	myfile << "property uchar green" << endl;
	myfile << "property uchar blue" << endl;
	myfile << "element face 0" << endl;
	myfile << "property list uchar int vertex_indices" << endl;

	myfile << "element edge " << cameras.size() * 3 << endl;           // 3 axis
	myfile << "property int vertex1" << endl;
	myfile << "property int vertex2" << endl;
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

	for (unsigned int n = 0; n < cameras.size(); n++) {

		float Tx = cameras[n](0, 3);
		float Ty = cameras[n](1, 3);
		float Tz = cameras[n](2, 3);

		float Ix = Tx + cameras[n](0, 0);
		float Iy = Ty + cameras[n](0, 1);
		float Iz = Tz + cameras[n](0, 2);

		float Jx = Tx + cameras[n](1, 0);
		float Jy = Ty + cameras[n](1, 1);
		float Jz = Tz + cameras[n](1, 2);

		float Kx = Tx + cameras[n](2, 0);
		float Ky = Ty + cameras[n](2, 1);
		float Kz = Tz + cameras[n](2, 2);

		myfile << Tx << " " << Ty << " " << Tz << " " << 255 << " " << 255
				<< " " << 255 << endl;
		myfile << Ix << " " << Iy << " " << Iz << " " << 255 << " " << 0 << " "
				<< 0 << endl;
		myfile << Jx << " " << Jy << " " << Jz << " " << 0 << " " << 255 << " "
				<< 0 << endl;
		myfile << Kx << " " << Ky << " " << Kz << " " << 0 << " " << 0 << " "
				<< 255 << endl;

	}

	// draw the axis
	int offset = (int) cloud.size();
	for (unsigned int n = 0; n < cameras.size(); n++) {
		myfile << n * 4 + offset << " " << n * 4 + 1 + offset << " " << 255
				<< " " << 0 << " " << 0 << endl;
		myfile << n * 4 + offset << " " << n * 4 + 2 + offset << " " << 0 << " "
				<< 255 << " " << 0 << endl;
		myfile << n * 4 + offset << " " << n * 4 + 3 + offset << " " << 0 << " "
				<< 0 << " " << 255 << endl;

	}
}

int SFMReconstruct(string picFolder,string outfile) {
	vector<cv::Mat> imgs;
	std::vector<std::string> img_names;
	readImgInSequence(picFolder.c_str(), &imgs, &img_names);
	cv::Ptr<MultiViewPnP> distance = new MultiViewPnP(imgs, img_names);
	distance->use_rich_features = true;
	distance->RecoverDepthFromImagesVid();
	writePLY(distance,outfile);
	return 0;
}
