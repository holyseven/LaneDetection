#include "InversePerspectiveMapping.h"


#include <iostream>
using namespace std;

CameraParameters::CameraParameters(){
	intrMat = Mat::eye(Size(4,4), CV_64FC1);
	rtMat = Mat::eye(Size(4, 4), CV_64FC1);
}

void CameraParameters::setIntrMat(const Mat &_intrMat){
	_intrMat.copyTo(intrMat);
}

void CameraParameters::setIntrMat(double fx, double fy, double cu, double cv){
	double _data[16] = { 
		fx, 0, cu, 0,
		0, fy, cv, 0,
		0, 0, 1, 0,
		0, 0, 0, 1 };

	setMat(_data, intrMat);
}

void CameraParameters::setMat(double *_data, Mat &_mat){
	if (_data == NULL)
	{
		cout << "data error!" << endl;
		return;
	}
	double* data_intrMat = _mat.ptr<double>(0);
	for (int i = 0, k = 0; i < intrMat.rows; i++)
		for (int j = 0; j < intrMat.cols; j++, k++)
			data_intrMat[k] = _data[k];
}



void CameraParameters::setRTMat(double rx /*pitch*/, double ry /*yaw*/, double rz /*roll*/, 
	double tx, double ty, double tz){

	double sx = sin(rx);
	double cx = cos(rx);
	double sy = sin(ry);
	double cy = cos(ry);
	double sz = sin(rz);
	double cz = cos(rz);

	double _data[16];

	//Rx * Ry * Rz
	//_data[0] = +cy*cz;				_data[1] = -cy*sz;				_data[2] = +sy;		_data[3] = tx;
	//_data[4] = +sx*sy*cz + cx*sz;	_data[5] = -sx*sy*sz + cx*cz;	_data[6] = -sx*cy;	_data[7] = ty;
	//_data[8] = -cx*sy*cz + sx*sz;	_data[9] = +cx*sy*sz + sx*cz;	_data[10] = +cx*cy; _data[11] = tz;
	//_data[12] = 0;					_data[13] = 0;					_data[14] = 0;		_data[15] = 1;

	//Rz * Rx * Ry
	_data[0] = +cy*cz - sx*sy*sz;	_data[1] = -cx*sz;			_data[2] = +cz*sy + cy*sx*sz;	_data[3] = tx;
	_data[4] = cy*sz + cz*sx*sy;	_data[5] = cx*cz;			_data[6] = sy*sz - cy*cz*sx;	_data[7] = ty;
	_data[8] = -cx*sy;			    _data[9] = sx;				_data[10] = +cx*cy;				_data[11] = tz;
	_data[12] = 0;					_data[13] = 0;              _data[14] = 0;					_data[15] = 1;

	setMat(_data, rtMat);


	//Rz * Rx * Ry
	//Tr.val[0][0] = +cy*cz - sx*sy*sz;	Tr.val[0][1] = -cx*sz;			Tr.val[0][2] = +cz*sy + cy*sx*sz;	Tr.val[0][3] = tx;
	//Tr.val[1][0] = cy*sz + cz*sx*sy;	Tr.val[1][1] = cx*cz;			Tr.val[1][2] = sy*sz - cy*cz*sx;	Tr.val[1][3] = ty;
	//Tr.val[2][0] = -cx*sy;			Tr.val[2][1] = sx;				Tr.val[2][2] = +cx*cy;				Tr.val[2][3] = tz;
	//Tr.val[3][0] = 0;					Tr.val[3][1] = 0;               Tr.val[3][2] = 0;					Tr.val[3][3] = 1;

	//Rx * Ry * Rz
	//rtMat.at<double>(0, 0) = +cy*cz;			rtMat.at<double>(0, 1) = -cy*sz;			 rtMat.at<double>(0, 2) = +sy;	  rtMat.at<double>(0, 3) = tx;
	//rtMat.at<double>(1, 0) = +sx*sy*cz + cx*sz; rtMat.at<double>(1, 1) = -sx*sy*sz + cx*cz;  rtMat.at<double>(1, 2) = -sx*cy; rtMat.at<double>(1, 3) = ty;
	//rtMat.at<double>(2, 0) = -cx*sy*cz + sx*sz; rtMat.at<double>(2, 1) = +cx*sy*sz + sx*cz;  rtMat.at<double>(2, 2) = +cx*cy; rtMat.at<double>(2, 3) = tz;
}

void CameraParameters::setRTMat(const Mat &_rtMat){
	_rtMat.copyTo(rtMat);
}




InversePerspectiveMapping::InversePerspectiveMapping(){
	rCW2UV = Mat::zeros(4, 4, CV_64FC1);
	rEqua = Mat::zeros(3, 3, CV_64FC1);
}

InversePerspectiveMapping::InversePerspectiveMapping(int r, int c){
	image_rows = r, image_cols = c;
	rCW2UV = Mat::zeros(4, 4, CV_64FC1);
	rEqua = Mat::zeros(3, 3, CV_64FC1);
	remapX = new double[r*c];
	memset(remapX, 0, r*c);
	remapZ = new double[r*c];
	memset(remapZ, 0, r*c);
}

void InversePerspectiveMapping::createModelForStandardAssumption(double fx, double fy, 
	double cu, double cv, 
	double h, double rx)
{
	firstFrameParam.setIntrMat(fx, fy, cu, cv);

	//InversePerspectiveMapping::StandardAssumption
	firstFrameParam.setRTMat(rx, 0, 0, 0, h * cos(rx), h * sin(rx));
}

void InversePerspectiveMapping::createModelForStandardAssumption(double fx, double fy, double cu, double cv, double h,
	const Mat &disp, bool estimate/**/)
{
	if (!estimate) return;
	double f = (fx + fy) / 2;
	double rx = estimateRx(cv, fx, disp);
	cout << rx << endl;
	createModelForStandardAssumption(fx, fy, cu, cv, h, rx);
}

InversePerspectiveMapping::~InversePerspectiveMapping(){
	delete remapX;
	delete remapZ;
}

void InversePerspectiveMapping::updateFrameUsingStandardAssumption(const Mat &pose){
	rCW2CI = pose.inv() * firstFrameParam.rtMat;
	//cout << " pose " << pose << endl;
	rCW2UV = firstFrameParam.intrMat * rCW2CI;
	//cout << " rCW2UV " << rCW2UV << endl;
	calcRemapMat();
}

void InversePerspectiveMapping::calcRemapMat(){
	double *ptr_rCW2UV = rCW2UV.ptr<double>(0);
	for (int v = image_rows / 2; v < image_rows; v++)
	{
		double c2 = ptr_rCW2UV[4] - ptr_rCW2UV[8] * v;
		double c3 = (ptr_rCW2UV[11] * v - ptr_rCW2UV[7]);
		double c4 = (ptr_rCW2UV[6] - ptr_rCW2UV[10] * v);

		for (int u = 0; u < image_cols; u++)
		{
			double c1 = ptr_rCW2UV[0] - ptr_rCW2UV[8] * u;
			double c5 = ptr_rCW2UV[2] - ptr_rCW2UV[10] * u;
			double c6 = ptr_rCW2UV[11] * u - ptr_rCW2UV[3];
			c5 = c5 / c1;
			c6 = c6 / c1;
			
			double z = (c3 - c6*c2) / (c4 - c5*c2);
			remapZ[v*image_cols + u] = z;
			remapX[v*image_cols + u] = (c6 - c5*z);

			double camera_z = rCW2CI.at<double>(2, 0)*(c6 - c5*z) + rCW2CI.at<double>(2, 2)*z
				+ rCW2CI.at<double>(2, 3);

			if (camera_z > 30)
			{
				remapZ[v*image_cols + u] = 0;
				remapX[v*image_cols + u] = 0;
			}
		}
	}
}

//void InversePerspectiveMapping::calcRemap(int u, int v, double &X, double &Z){
//	rEqua.at<double>(0, 0) = rCW2UV.at<double>(0, 0); rEqua.at<double>(0, 1) = rCW2UV.at<double>(0, 2); rEqua.at<double>(0, 2) = -u;
//	rEqua.at<double>(1, 0) = rCW2UV.at<double>(1, 0); rEqua.at<double>(1, 1) = rCW2UV.at<double>(1, 2); rEqua.at<double>(1, 2) = -v;
//	rEqua.at<double>(2, 0) = rCW2UV.at<double>(2, 0); rEqua.at<double>(2, 1) = rCW2UV.at<double>(2, 2); rEqua.at<double>(2, 2) = -1;
//
//	Mat rightTerm = Mat(3, 1, CV_64FC1);
//	rightTerm.at<double>(0) = -rCW2UV.at<double>(0, 3);
//	rightTerm.at<double>(1) = -rCW2UV.at<double>(1, 3);
//	rightTerm.at<double>(2) = -rCW2UV.at<double>(2, 3);
//
//	//cout << rEqua << endl;
//
//	Mat resolution = rEqua.inv() * rightTerm;
//	X = resolution.at<double>(0);
//	Z = resolution.at<double>(1);
//}


double InversePerspectiveMapping::estimateRx(double cv, double f, const Mat &disp){
	//v-disp
	Mat vdisp = Mat::zeros(disp.rows, 256, CV_8UC1);
	for (int _r = disp.rows/2; _r < disp.rows; _r++)
	{
		const uchar* ptr_row_disp = disp.ptr<uchar>(_r);
		uchar* ptr_row_vdisp = vdisp.ptr<uchar>(_r);

		for (int _c = 0; _c < disp.cols; _c++)
		{
			if (ptr_row_disp[_c] < 5) continue;
			if (ptr_row_vdisp[ptr_row_disp[_c]] < 255)
				ptr_row_vdisp[ptr_row_disp[_c]]++;
		}
	}
	//imshow("vdisp_raw", vdisp);
	//imwrite("1.png", vdisp);

	threshold(vdisp, vdisp, 0, 255, THRESH_OTSU);
	vector<Vec2f> lines;
	int vote = 200;
	do{
		HoughLines(vdisp, lines, 1, 0.01 * CV_PI / 180, vote, 0, 0, 100 * CV_PI / 180, 170 * CV_PI / 180);
		vote -= 20;
	} while (lines.size() == 0);
	
	cvtColor(vdisp, vdisp, CV_GRAY2BGR);
	double vanishing_point_y = 0;
	double mean_rho = 0, mean_theta = 0;
	for (int i = 0; i < lines.size(); i++)
	{
		mean_rho += lines[i][0], mean_theta += lines[i][1];
		//vanishing_point_y += rho / sin(theta);

		line(vdisp, Point(vdisp.cols, lines[i][0] / sin(lines[i][1]) - (vdisp.cols - 1)*cos(lines[i][1]) / sin(lines[i][1])),
			Point(0, lines[i][0] / sin(lines[i][1])), Scalar(255, 128, 0));
		//cout << vdisp.rows / 2 - rho / sin(theta) << endl;
	}
	mean_rho /= lines.size();
	mean_theta /= lines.size();
	vanishing_point_y = cv - mean_rho / sin(mean_theta);

	//cout << vanishing_point_y << endl;
	//imshow("v-disp", vdisp);
	//imwrite("2.png", vdisp);
	//waitKey();

	return atan2(vanishing_point_y, f);
}
