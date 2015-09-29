#define _CRT_SECURE_NO_WARNINGS

#include "KITTI_Data_Reader\KITTI_Data_Reader.h"
#include "RectifyImages\RectifyStereo.h"
#include "ELAS_VisualOdometry\InterfaceProcessVISO.h"
#include "ELAS_VisualOdometry\image.h"
#include "ELAS_VisualOdometry\ELAS_Disparity_Interface.h"
#include "IPMImage\IPMImage.h"
#include "LaneDetector\LaneDetection.h"

#include <iostream>
using namespace std;

void parse(string &DataSetFolderName, string &calibFileName, int &rectified,
	int &frameInterval, float &h, int &methodeDisparity, int &noDisparity,
	int &elasSetting, int &showTimeConsuming)
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
	in.close();
}

//with stereo
int main2(){
	//default values
	string DataSetFolderName; 
	DataSetFolderName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\2011_09_26_drive_0029_sync";

	string calibFileName; 
	calibFileName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\calib_cam_to_cam.txt";

	//default settings:
	int frameInterval = 7;
	float h = 1.2f;//m
	int rectified = 1; // 1 -- have already rectified; 0 -- need rectify
	int methodeDisparity = 0;
	int noDisparity = 0;
	int elasSetting = Elas::ROBOTICS;
	int showTimeConsuming = 0;

	//read config.txt to settings:
	parse(DataSetFolderName, calibFileName, rectified, frameInterval, h, 
		methodeDisparity, noDisparity, elasSetting, showTimeConsuming);
	cout << "reading config.txt" << endl;

	KITTI_Data_Reader reader(DataSetFolderName);
	cout << "Image number in this dir : " << reader.getMaxIndex() << endl;

	RectifyStereo rectifyStereo(calibFileName, rectified);
	if (!rectified && !rectifyStereo.isLoadCameraParam)
	{
		cout << "Camera Param is not loaded! Promgram exits..." << endl;
		return 1;
	}

	Calib_Data_Type calibData = rectifyStereo.calibData;

	//instance for computing Visual Odometry (camera pose change)
	InterfaceProcessVISO *procVISO = new InterfaceProcessVISO(calibData);
	

	Elas::setting enumSetting = Elas::ROBOTICS;
	if (elasSetting == 1) // default 0
		enumSetting = Elas::MIDDLEBURY;

	Elas::parameters param(enumSetting);
	InterfaceProcessELAS procELAS(param);//instance for computing disparity map

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 256, 11);//sgbm for computing dm
	Ptr<StereoBM> bm = StereoBM::create(0, 15);//sgbm for computing dm
	
	reader.generateNextDataFileName(0);
	string left_img_file_name = reader.curImageFileName[0];
	string right_img_file_name = reader.curImageFileName[1];

	Mat L = imread(left_img_file_name, 0);
	Mat R = imread(right_img_file_name, 0);
	Mat rL, rR;

	if (rectified == 1)
	{
		L.copyTo(rL);
		R.copyTo(rR);
	}
	else
		rectifyStereo.rectifyImages(L, R, rL, rR);

	Mat disp;
	procELAS.computeDisparity(rL, rR, disp);

	InterfaceProcessIPMImage *interface_ipm = new InterfaceProcessIPMImage(calibData, h, disp);//initialize pitch angle with disparity map
	interface_ipm->showVehiclePosition(false, true);

	int frameNum = 0;
	Mat ipmImage;
	int64 t0, t1;
	while (reader.generateNextDataFileName())
	{
		if (showTimeConsuming)
			t0 = getTickCount();
		string left_img_file_name = reader.curImageFileName[0];
		string right_img_file_name = reader.curImageFileName[1];

		Mat L = imread(left_img_file_name, -1);
		Mat R = imread(right_img_file_name, -1);
		Mat rL, rR;//rectified images : rL, rR

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Reading images : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		
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
			cout << "Rectifying images : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		if (frameNum == frameInterval)
		{
			procELAS.computeDisparity(rL, rR, disp);
			procVISO->~InterfaceProcessVISO();
			procVISO = new InterfaceProcessVISO(calibData);
			interface_ipm->~InterfaceProcessIPMImage();
			interface_ipm = new InterfaceProcessIPMImage(calibData, h, disp);
			interface_ipm->showVehiclePosition(false, true);
			frameNum = 0;

			if (showTimeConsuming)
			{
				t1 = getTickCount();
				cout << "Reinitialization : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
			}
		}
		
		//procVISO->processVISO(rL, rR);
		//cout << "[";
		//cout << procVISO->pose << "]" << endl;
		//cout << endl;
		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Computing pose (visual odometry) : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}
		

		///----------------------Disparity Map---------------------------
		Mat LforDisp, RforDisp;//Images for computing disparity map : LforDisp, RforDisp

		rectifyStereo.getROI(rL, rR, LforDisp, RforDisp);


		Mat disp;
		/*
		if (!noDisparity)
		{
			if (methodeDisparity == 0)
				procELAS.computeDisparity(LforDisp, RforDisp, disp);
			else if (methodeDisparity == 1)
			{
				sgbm->compute(LforDisp, RforDisp, disp);
				disp.convertTo(disp, CV_8U, 1.0 / 16);
			}
			else if (methodeDisparity == 2)
			{
				bm->compute(LforDisp, RforDisp, disp);
				disp.convertTo(disp, CV_8U, 1.0 / 16);
			}
		}
		*/
		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Computing disparity : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}
			

		///---------------------------------------------------------------

		
		string oxts_file_name = reader.curOxtsFileName;
		ifstream oxts_file(oxts_file_name);
		Oxts_Data_Type *oxtsData;//gps data
		if (oxts_file.is_open())
		{
			oxtsData = new Oxts_Data_Type;
			oxts_file >> *oxtsData;
		}
		else
		{
			oxtsData = NULL;
		}

		Mat ipmMask;
		interface_ipm->processIPM(rL, procVISO->pose, ipmImage, ipmMask, oxtsData);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "ipm : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}
		
		//detection of lines or lanes in ipmImage
		//LaneDetection lsd(ipmImage, ipmMask);
		//lsd.run(3, 1);

		LaneDetection lsd_(rL, calibData.K_00[2], calibData.K_00[5]);
		lsd_.run(3, 2);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "Lane detection : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		if (rL.data && rR.data)
		{
			imshow("rL", rL);
			imwrite("rl.png", rL);
			imshow("rR", rR);
		}
		if (ipmImage.data)
			imshow("ipm", ipmImage);
		if (disp.data)
		{
			imshow("disp", disp);
			imwrite("disp.png", disp);
		}

		//if (waitKey(10) > 0)
			waitKey();

		frameNum++;
		cout << "----------------frame end----------------------" << endl;
	}
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

	//read config.txt to settings:
	parse(DataSetFolderName, calibFileName, rectified, frameInterval, h,
		methodeDisparity, noDisparity, elasSetting, showTimeConsuming);
	cout << "reading config.txt" << endl;


	KITTI_Data_Reader reader(DataSetFolderName);
	cout << "Image number in this dir : " << reader.getMaxIndex() << endl;

	RectifyStereo rectifyStereo(calibFileName, rectified);
	if (!rectified && !rectifyStereo.isLoadCameraParam)
	{
		cout << "Camera Param is not loaded! Promgram exits..." << endl;
		return 1;
	}

	Calib_Data_Type calibData = rectifyStereo.calibData;

	string left_img_file_name = reader.curImageFileName[0];
	string right_img_file_name = reader.curImageFileName[1];

	InterfaceProcessIPMImage *interface_ipm = new InterfaceProcessIPMImage(calibData, h, 0);//initialize pitch angle with disparity map
	interface_ipm->showVehiclePosition(false, true);
	interface_ipm->ipm->updateFrameUsingStandardAssumption(Mat::eye(4, 4, CV_64FC1));

	int frameNum = 0;
	Mat ipmImage;
	int64 t0, t1;

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

		LaneDetection lsd_(rL, calibData.K_00[2], calibData.K_00[5]);
		lsd_.run(4, 3, interface_ipm->ipm);

		if (showTimeConsuming)
		{
			t1 = getTickCount();
			cout << "LaneDetection : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;
		}

		imshow("rL",rL);
		waitKey();
	}

}