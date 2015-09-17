#include "RectifyStereo.h"


RectifyStereo::RectifyStereo(const char* fileName, int _r)
{
	rectified = _r;
	loadParam(fileName);
}

RectifyStereo::RectifyStereo(string fileName, int _r){
	rectified = _r;
	loadParam(fileName.c_str());
}
void RectifyStereo::loadParam(string fileName){
	loadParam(fileName.c_str());
}

void RectifyStereo::loadParam(const char* fileName){
	ifstream if_in(fileName);
	if (!if_in.is_open())
	{
		isLoadCameraParam = false;
		cout << "load parameters error!" << endl;
		return;
	}

	if_in >> calibData;
	if_in.close();
	calRemapMatrix();
	isLoadCameraParam = true;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
		{
			if (remapM[i][j].empty())
				isLoadCameraParam = false;
		}
}

void RectifyStereo::calRemapMatrix(){
	Mat K_00, D_00, R_rect_00, P_rect_00;
	Mat K_01, D_01, R_rect_01, P_rect_01;
	Size S_00;
	Mat R_01, T_01;

	// Matlab computes these
	K_00 = Mat(SQRT_SIZE_K, SQRT_SIZE_K, CV_64FC1, calibData.K_00);
	D_00 = Mat(1, SIZE_D, CV_64FC1, calibData.D_00);
	K_01 = Mat(SQRT_SIZE_K, SQRT_SIZE_K, CV_64FC1, calibData.K_01);
	D_01 = Mat(1, SIZE_D, CV_64FC1, calibData.D_01);
	S_00 = Size(calibData.S_00[0], calibData.S_00[1]);
	R_01 = Mat(3, 3, CV_64FC1, calibData.R_01);
	T_01 = Mat(3, 1, CV_64FC1, calibData.T_01);

	// Check if R_rect_0X, P_rect_0X are valide.
	bool checkRP = true;
	for (int i = 0; i < SIZE_R && checkRP; i++)
	{
		if (abs(calibData.R_rect_00[i]) > 5000 || abs(calibData.R_rect_01[i]) > 5000)
			checkRP = false;
	}
	for (int i = 0; i < SIZE_P_ROWS * SIZE_P_COLS && checkRP; i++)
	{
		if (abs(calibData.P_rect_00[i]) > 5000 || abs(calibData.P_rect_01[i]) > 5000)
			checkRP = false;
	}

	Size S_rect_00;
	// If valide, read them directly
	if (checkRP == true)
	{
		R_rect_00 = Mat(SQRT_SIZE_R, SQRT_SIZE_R, CV_64FC1, calibData.R_rect_00);
		P_rect_00 = Mat(SIZE_P_ROWS, SIZE_P_COLS, CV_64FC1, calibData.P_rect_00);
		R_rect_01 = Mat(SQRT_SIZE_R, SQRT_SIZE_R, CV_64FC1, calibData.R_rect_01);
		P_rect_01 = Mat(SIZE_P_ROWS, SIZE_P_COLS, CV_64FC1, calibData.P_rect_01);
		S_rect_00 = Size(calibData.S_rect_00[0], calibData.S_rect_00[1]);

		//int _y = calibData.ROI_00[1];
		//if (_y < calibData.ROI_01[1])
		//	_y = calibData.ROI_01[1];
		//int _y2 = calibData.ROI_00[1] + calibData.ROI_00[3];
		//if (_y2 > calibData.ROI_01[1] + calibData.ROI_01[3])
		//	_y2 = calibData.ROI_01[1] + calibData.ROI_01[3];
		//int _width = calibData.ROI_00[2];
		//if (_width > calibData.ROI_01[2])
		//	_width = calibData.ROI_01[2];

		//roi[0] = Rect(calibData.ROI_00[0], _y, _width, _y2 - _y);
		//roi[1] = Rect(calibData.ROI_01[0], _y, _width, _y2 - _y);


	}
	// If not, compute them
	else
	{
		S_rect_00 = S_00;
		Mat Q;
		// Rectify and compute the rest camera parameters
		stereoRectify(K_00, D_00, K_01, D_01, S_00, R_01, T_01, //input
			R_rect_00, R_rect_01, P_rect_00, P_rect_01, // output
			Q, CV_CALIB_ZERO_DISPARITY, 0, S_rect_00, &roi[0], &roi[1]); //output
		
		memcpy(calibData.R_rect_00, (double*)R_rect_00.data, sizeof(calibData.R_rect_00));
		memcpy(calibData.P_rect_00, (double*)P_rect_00.data, sizeof(calibData.P_rect_00));

		memcpy(calibData.R_rect_01, (double*)R_rect_01.data, sizeof(calibData.R_rect_01));
		memcpy(calibData.P_rect_01, (double*)P_rect_01.data, sizeof(calibData.P_rect_01));

		calibData.ROI_00[0] = roi[0].x, calibData.ROI_00[1] = roi[0].y;
		calibData.ROI_00[2] = roi[0].width, calibData.ROI_00[3] = roi[0].height;

		calibData.ROI_01[0] = roi[1].x, calibData.ROI_01[1] = roi[1].y;
		calibData.ROI_01[2] = roi[1].width, calibData.ROI_01[3] = roi[1].height;

		calibData.S_rect_00[0] = S_rect_00.width, calibData.S_rect_00[1] = S_rect_00.height;
		calibData.S_rect_01[0] = S_rect_00.width, calibData.S_rect_01[1] = S_rect_00.height;
	}

	if (!rectified)
	{
		//compute remapM[2][2]
		initUndistortRectifyMap(
			K_00, D_00, R_rect_00, P_rect_00, S_rect_00, CV_32FC1,
			remapM[0][0], remapM[0][1]);

		initUndistortRectifyMap(
			K_01, D_01, R_rect_01, P_rect_01, S_rect_00, CV_32FC1,
			remapM[1][0], remapM[1][1]);
	}
}

void RectifyStereo::rectifyImages(const Mat &L, const Mat &R, Mat &rL, Mat &rR){
	if (!isLoadCameraParam)
	{
		cerr << "ERROR! Camera Parameters are not availabe!" << endl;
		return;
	}

	remap(L, rL, remapM[0][0], remapM[0][1], cv::INTER_LINEAR);
	remap(R, rR, remapM[1][0], remapM[1][1], cv::INTER_LINEAR);

}

void RectifyStereo::getROI(const Mat &rL, const Mat &rR, Mat &roiL, Mat & roiR){
	if (roi[0].area() != 0 && roi[1].area() != 0)
	{
		rL(roi[0]).copyTo(roiL);
		rR(roi[1]).copyTo(roiR);
	}
	else
	{
		rL.copyTo(roiL);
		rR.copyTo(roiR);
	}

	//resize(rR, rR, Size(roi[0].size()), 0, 0, INTER_AREA);

}

