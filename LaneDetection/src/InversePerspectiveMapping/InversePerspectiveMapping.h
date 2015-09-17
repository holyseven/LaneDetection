#ifndef Inverse_Perspective_Mapping_H
#define Inverse_Perspective_Mapping_H

#include <opencv2\opencv.hpp>
#include <string>
using namespace cv;
using namespace std;

class CameraParameters{
public:
	CameraParameters();
	void setIntrMat(double fx, double fy, double cu, double dv);
	void setIntrMat(const Mat &_intrMat);
	Mat intrMat;//4*4,double


	void setRTMat(double rx /*pitch*/, double ry /*yaw*/, double rz /*roll*/,
		double tx = 0, double ty = 0, double tz = 0);
	void setRTMat(const Mat &_rtMat);
	Mat rtMat;//4*4,double
	
private:
	void setMat(double *_data, Mat &_mat);
};



class InversePerspectiveMapping
{
public:
	enum {

		// ! The simplest model: suppose that the road is plat and is in surface Y = 0
		StandardAssumption



	};


	InversePerspectiveMapping();
	InversePerspectiveMapping(int image_rows, int image_cols);
	~InversePerspectiveMapping();
	
	void createModelForStandardAssumption(double fx, double fy, double cu, double cv, double h, 
		double rx);

	//estimate the pitch angle 'rx' using vanishing point with v-disparity
	void createModelForStandardAssumption(double fx, double fy, double cu, double cv, double h,
		const Mat &disp, bool estimateRx = true);

	//using v-disparity, return radian
	double estimateRx(double cv, double f, const Mat &disp);

	void updateFrameUsingStandardAssumption(const Mat &pose);
	//void calcRemap(int u, int v, double &X, double &Z);
	
	static inline void rotation2D(const Mat& rMat, Mat &p) { p = rMat * p; };

	double *remapX, *remapZ;
	int image_rows, image_cols;

private:
	Mat rCW2UV;//wolrd coordinates to (u,v)
	Mat rEqua;
	Mat rCW2CI;//
	CameraParameters firstFrameParam;// ! for model StandardAssumption
	void calcRemapMat();
};



#endif