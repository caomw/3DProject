//Headers for registration
#include "FICPRegister.h"
#include "SFMReconstruct.h"
#include "ValidateOpenCV.h"

using namespace std;
using namespace cv;

int main() {
	int runType = 3;
	switch (runType) {
	case 0:
		validateOpenCVInstall("./DATA/05180.png");
		break;
	case 1:
		SFMReconstruct("/home/happierboy/Desktop/RA/3DProject/DATA/test/","./target.ply");
		break;
	case 2:
		FICPRegister("./target.ply", "./target.ply", "./");
		break;
	case 3:
	{
		string location = "Tuas_C";
		string dpath = "/home/happierboy/Desktop/RA/3DProject/TestSequence";
		printf("start IR reconstruction...\n");
		SFMReconstruct(dpath+"/"+location+"/IR/",location+"_IR.ply");
		printf("start RGB reconstruction...\n");
		SFMReconstruct(dpath+"/"+location+"/RGB/",location+"_RGB.ply");
		FICPRegister(location+"_IR.ply",location+"_RGB.ply", "./");
	}
	break;
	default:
		break;
	}
	return 0;
}

