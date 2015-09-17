#ifndef RECTIFY_STEREO_H
#define RECTIFY_STEREO_H
#include "..\KITTI_Data_Reader\KITTI_Data_Reader.h"
#include <opencv2\opencv.hpp>
using namespace cv;

class RectifyStereo{

public:
	RectifyStereo(){ isLoadCameraParam = false; }

	RectifyStereo(const char* fileName, int rectified = 0);//load parameter file and compute remap matrix.
	RectifyStereo(string fileName, int rectified = 0);//load parameter file and compute remap matrix.
	void loadParam(const char* fileName);//load parameter file and compute remap matrix.
	void loadParam(string fileName);//load parameter file and compute remap matrix.

	void rectifyImages(const Mat &L, const Mat &R, Mat &rL, Mat &rR);
	void getROI(const Mat &rL, const Mat &rR, Mat &roiL, Mat & roiR);
	Calib_Data_Type calibData;
	bool isLoadCameraParam;

private:
	void calRemapMatrix();
	
	bool rectified;
	Mat remapM[2][2];
	Rect roi[2];
};

#endif