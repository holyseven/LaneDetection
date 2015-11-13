#define _CRT_SECURE_NO_WARNINGS

#include "KITTI_Data_Reader\KITTI_Data_Reader.h"
#include "RectifyImages\RectifyStereo.h"
#include "ELAS_VisualOdometry\InterfaceProcessVISO.h"
#include "ELAS_VisualOdometry\image.h"
#include "ELAS_VisualOdometry\ELAS_Disparity_Interface.h"
#include "IPMImage\IPMImage.h"
#include "LaneDetector\LaneDetectionV2.h"

#include <iostream>
using namespace std;

void parse(string &DataSetFolderName, string &calibFileName, int &rectified,
	int &frameInterval, float &h, int &methodeDisparity, int &noDisparity,
	int &elasSetting, int &showTimeConsuming, float &pitch, string &formatImage)
{
	ifstream in("config.txt");
	if (!in.is_open())
	{
		cout << "error to open file : config.txt. Use defalut values." << endl;
		return;
	}
	in >> DataSetFolderName;
	in >> calibFileName;
	in >> rectified;
	in >> frameInterval;
	in >> h;
	in >> methodeDisparity;//0:elas ; 1:sgbm
	in >> noDisparity;//0:computes disparity map every frame
	in >> elasSetting;
	in >> showTimeConsuming;
	in >> pitch;
	in >> formatImage;
	in.close();
}

//with stereo
int main2(){
	//default values
	string DataSetFolderName;
	DataSetFolderName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\2011_09_26_drive_0029_sync";

	string calibFileName;
	calibFileName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\calib_cam_to_cam.txt";

	string formatImage;
	formatImage = ".png";

	//default settings:
	int rectified = 1; // 1 -- have already rectified; 0 -- need rectify
	int frameInterval = 1;
	float h = 1.65f;//m
	int methodeDisparity = 0;
	int noDisparity = 1;
	int elasSetting = Elas::ROBOTICS;
	int showTimeConsuming = 0;
	float pitch = 0;

	//read config.txt to settings:
	parse(DataSetFolderName, calibFileName, rectified, frameInterval, h,
		methodeDisparity, noDisparity, elasSetting, showTimeConsuming, pitch, formatImage);
	cout << "reading config.txt" << endl;


	KITTI_Data_Reader reader(DataSetFolderName, formatImage);
	cout << "Image number in this dir : " << reader.getMaxIndex() << endl;

	RectifyStereo rectifyStereo(calibFileName, rectified);
	if (!rectified && !rectifyStereo.isLoadCameraParam)
	{
		cout << "Camera Param is not loaded! Promgram exits..." << endl;
		return 1;
	}

	Calib_Data_Type calibData = rectifyStereo.calibData;

	CC::CC_SimpleIPM ipm;
	ipm.createModel(calibData.P_rect_00[0], calibData.P_rect_00[5], calibData.P_rect_00[2], calibData.P_rect_00[6],
		pitch, h);

	Elas::setting enumSetting = Elas::ROBOTICS;
	if (elasSetting == 1) // default 0
		enumSetting = Elas::MIDDLEBURY;

	Elas::parameters param(enumSetting);
	InterfaceProcessELAS procELAS(param);//instance for computing disparity map

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 256, 11);//sgbm for computing dm
	Ptr<StereoBM> bm = StereoBM::create(0, 15);//sgbm for computing dm


	Mat ipmImage;
	int64 t0, t1;

	reader.jumpToIndex(0);

	Mat _ = imread(reader.curImageFileName[0]);
	LaneDetection *lsd_ = new LaneDetection(_);
	lsd_->init(0, &ipm);

	//CC::CC_SimpleIPM ipm_r;
	//ipm_r.createModel(calibData.P_rect_01[0], calibData.P_rect_01[5], calibData.P_rect_01[2], calibData.P_rect_01[6],
	//	pitch, h);
	//LaneDetection *lsd_r = new LaneDetection(_);
	//lsd_r->init(0, &ipm_r);

	while (reader.generateNextDataFileName())
	{
		if (showTimeConsuming)
			t0 = getTickCount();
		string left_img_file_name = reader.curImageFileName[0];
		string right_img_file_name = reader.curImageFileName[1];

		Mat L = imread(left_img_file_name, -1);
		Mat R = imread(right_img_file_name, -1);

		Mat rL, rR;//rectified images : rL, rR
		if (rectified == 1)
		{
			L.copyTo(rL);
			R.copyTo(rR);
		}
		else
			rectifyStereo.rectifyImages(L, R, rL, rR);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Reading images and rectifying images : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}
		Mat disp;
		if (methodeDisparity == 0)
			procELAS.computeDisparity(rL, rR, disp);
		else if (methodeDisparity == 1)
		{
			sgbm->compute(rL, rR, disp);
			disp.convertTo(disp, CV_8U, 1.0 / 8);
		}
			
		//lsd_->method3(rL);
		lsd_->method4(rL, disp);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "LaneDetection : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		imshow("disparity map", disp);
		//if (waitKey(10) > 0)
			waitKey();
	}
	cout << "-------------------end------------------ " << endl;

	return 1;
}


//mono camera
int main() {
	//default values
	string DataSetFolderName;
	DataSetFolderName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\2011_09_26_drive_0029_sync";

	string calibFileName;
	calibFileName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\calib_cam_to_cam.txt";

	//default settings:
	int rectified = 1; // 1 -- have already rectified; 0 -- need rectify
	int frameInterval = 1;
	float h = 1.65f;//m
	int methodeDisparity = 0;
	int noDisparity = 1;
	int elasSetting = Elas::ROBOTICS;
	int showTimeConsuming = 0;
	float pitch = 0;
	string formatImage = ".png";

	//read config.txt to settings:
	parse(DataSetFolderName, calibFileName, rectified, frameInterval, h,
		methodeDisparity, noDisparity, elasSetting, showTimeConsuming, pitch, formatImage);
	cout << "reading config.txt" << endl;


	KITTI_Data_Reader reader(DataSetFolderName, formatImage);
	cout << "Image number in this dir : " << reader.getMaxIndex() << endl;

	RectifyStereo rectifyStereo(calibFileName, rectified);
	if (!rectified && !rectifyStereo.isLoadCameraParam)
	{
		cout << "Camera Param is not loaded! Promgram exits..." << endl;
		return 1;
	}

	Calib_Data_Type calibData = rectifyStereo.calibData;

	CC::CC_SimpleIPM ipm;
	ipm.createModel(calibData.P_rect_00[0], calibData.P_rect_00[5], calibData.P_rect_00[2], calibData.P_rect_00[6], 
		pitch, h);

	int frameNum = 0;
	Mat ipmImage;
	int64 t0, t1;

	reader.jumpToIndex(0);

	Mat _ = imread(reader.curImageFileName[0]);
	LaneDetection *lsd_ = new LaneDetection(_);
	lsd_->init(0, &ipm);
	
	//CC::CC_SimpleIPM ipm_r;
	//ipm_r.createModel(calibData.P_rect_01[0], calibData.P_rect_01[5], calibData.P_rect_01[2], calibData.P_rect_01[6],
	//	pitch, h);
	//LaneDetection *lsd_r = new LaneDetection(_);
	//lsd_r->init(0, &ipm_r);

	int frame_i = 0;
	while (reader.generateNextDataFileName())
	{
		cout << "frame----------------------" << frame_i++ << endl;
		if (showTimeConsuming)
			t0 = getTickCount();
		string left_img_file_name = reader.curImageFileName[0];
		string right_img_file_name = reader.curImageFileName[1];

		Mat L = imread(left_img_file_name, -1);
		Mat R = imread(right_img_file_name, -1);

		Mat rL, rR;//rectified images : rL, rR
		if (rectified == 1)
		{
			L.copyTo(rL);
			R.copyTo(rR);
		}
		else
			rectifyStereo.rectifyImages(L, R, rL, rR);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Reading images and rectifying images : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		//Mat maskRoad;
		//easyInterface(rL, maskRoad);
		//cvtColor(rL, rL, CV_BGR2GRAY);
		//adaptiveThreshold(rL, rL, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, 5);
		lsd_->method3(rL);
		//lsd_r->method3(rR, 1);



		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "LaneDetection : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		//imshow("maskRoad", maskRoad);
		//if (waitKey(10) > 0)
			waitKey();
	}
	cout << "-------------------end------------------ " << endl;
}