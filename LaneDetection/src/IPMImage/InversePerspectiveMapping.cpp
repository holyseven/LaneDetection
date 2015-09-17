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
	cout << "estimate pitch angle from disparity map : " << rx << endl;
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

void drawRoad(const Mat &disp, double rho, double theta){
	Mat drawDisp;
	cvtColor(disp, drawDisp, CV_GRAY2BGR);

	double a = -1 / tan(theta), b = rho / sin(theta);
	for (int r = b + 0.5; r < disp.rows; r++)
	{
		Vec3b* ptr_row_drawDisp = drawDisp.ptr<Vec3b>(r);
		const uchar* ptr_row_disp = disp.ptr<uchar>(r);


		double x = (r - b) / a;
		int x0 = x - 2.5, x1 = x + 2.5;
		for (int c = 0; c < disp.cols; c++)
		{
			int val = ptr_row_disp[c];
			if (val != 0 && val <= x1 && val >= x0)
			{
				ptr_row_drawDisp[c] = Vec3b(255, 0, 0);
			}
		}
	}

	imshow("road", drawDisp);
}


const int intervalMax = 100, intervalMin = 10, step_ = 16;
inline int betterVote(int nup, int u, int ndown, int d)
{
	return (int)(max(2.0, (double)(u - d)*(intervalMax - ndown) / (nup - ndown)) + d);
}
int g_better_vote = -1;
// very time consuming
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
	Mat raw_vdisp;
	vdisp.copyTo(raw_vdisp);

	threshold(vdisp, vdisp, 0, 255, THRESH_OTSU);
	vector<Vec2f> lines_0, lines_1;


	if (g_better_vote < 0)
	{
		for (int i = 150; i > 15; i-=10)
		{
			lines_0 = lines_1;
			HoughLines(vdisp, lines_1, 1, 0.01 * CV_PI / 180, i, 0, 0, 120 * CV_PI / 180, 150 * CV_PI / 180);
			int numOfLines = lines_1.size();
			if (numOfLines > intervalMax)
			{
				g_better_vote = i + 10;
				if (lines_0.size() == 0)
				{
					g_better_vote = i;
				}
				break;
			}
		}
	}

	int vote = g_better_vote;
	int bi_down = max(0, g_better_vote - step_), bi_up = g_better_vote + step_;
	int numOfLines_down = -1, numOfLines_up = -1;

	bool DownUpKnown = false;
	for (int times_i = 0; times_i < 5; times_i++)
	{

		HoughLines(vdisp, lines_1, 1, 0.01 * CV_PI / 180, vote, 0, 0, 120 * CV_PI / 180, 150 * CV_PI / 180);

		//cout << "-----------------------in loop------------------------" << endl;
		//cout << DownUpKnown << endl;
		//cout << "numOfLines_down: " << numOfLines_down << "," << "numOfLines_up : " << numOfLines_up << endl;
		//cout << "down : " << bi_down << " : up " << bi_up << endl << " vote:" << vote << endl;
		//cout << "lines_1.size() : " << lines_1.size() << endl;
		//cout << "-----------------------end in loop--------------------" << endl;

		int numOfLines = lines_1.size();
		if (numOfLines <= intervalMax && numOfLines >= intervalMin)//perfect case
		{
			break;
		}
		else
		{
			if (DownUpKnown)
			{
				if (numOfLines < intervalMin)
				{
					bi_up = vote;
					numOfLines_up = numOfLines;
					vote = betterVote(numOfLines_up, bi_up, numOfLines_down, bi_down);
				}
				else
				{
					bi_down = vote;
					numOfLines_down = numOfLines;
					vote = betterVote(numOfLines_up, bi_up, numOfLines_down, bi_down);
				}
			}
			else
			{
				if (numOfLines < intervalMin)
				{
					int n = numOfLines;
					while (n < intervalMin)
					{
						vote = bi_down;
						if (vote == 0)
							break;
						bi_down = max(0, vote - step_);
						numOfLines_up = n;
						HoughLines(vdisp, lines_1, 1, 0.01 * CV_PI / 180, vote, 0, 0, 120 * CV_PI / 180, 150 * CV_PI / 180);
						n = lines_1.size();
					}
					if (n <= intervalMax)
						break;

					bi_down = vote;
					numOfLines_down = n;
					bi_up = vote + step_;
					vote = betterVote(numOfLines_up, bi_up, numOfLines_down, bi_down);
				}
				else
				{
					int n = numOfLines;
					while (n > intervalMax)
					{
						vote = bi_up;
						bi_up = vote + step_;
						numOfLines_down = n;
						HoughLines(vdisp, lines_1, 1, 0.01 * CV_PI / 180, vote, 0, 0, 120 * CV_PI / 180, 150 * CV_PI / 180);
						n = lines_1.size();
					}
					if (n >= intervalMin)
						break;

					bi_up = vote;
					numOfLines_up = n;
					bi_down = max(0, vote - step_);
					vote = betterVote(numOfLines_up, bi_up, numOfLines_down, bi_down);
				}
				DownUpKnown = true;
			}
		}
	}

	vector<Vec2f> lines = lines_1;
	g_better_vote = vote;
	if (lines.size() == 0)
	{
		cout << "--------------------------" << endl;
		cout << "error! no lines detected! " << endl;
		cout << "--------------------------" << endl;
	}
	
	cvtColor(vdisp, vdisp, CV_GRAY2BGR);
	double vanishing_point_y = 0;
	double mean_rho = 0, mean_theta = 0;// y = -(cos(theta)/sin(theta))x + rho / sin(theta).
	for (int i = 0; i < lines.size(); i++)
	{
		mean_rho += lines[i][0], mean_theta += lines[i][1];
		
		line(vdisp, Point(vdisp.cols, lines[i][0] / sin(lines[i][1]) - (vdisp.cols - 1)*cos(lines[i][1]) / sin(lines[i][1])),
			Point(0, lines[i][0] / sin(lines[i][1])), Scalar(255, 128, 0));
	}
	mean_rho /= lines.size();
	mean_theta /= lines.size();
	vanishing_point_y = cv - mean_rho / sin(mean_theta);

	double theta = atan2(vanishing_point_y, f);

	imshow("v-disp", vdisp); 
	//imwrite("2.png", vdisp);
	//waitKey();

	//drawRoad(disp, mean_rho, mean_theta);
	return theta;
}
