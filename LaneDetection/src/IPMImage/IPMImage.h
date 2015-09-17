#ifndef IPMIMAGE_H
#define IPMIMAGE_H
#include <opencv2\opencv.hpp>
#include "InversePerspectiveMapping.h"
#include "..\KITTI_Data_Reader\KITTI_Data_Reader.h"
#include "..\ELAS_VisualOdometry\matrix.h"
using namespace cv;

#define IPM_Z_MIN 5
#define IPM_Z_MAX 25
#define IPM_X_MIN -10
#define IPM_X_MAX 10
#define IPM_IMAGE_SIZE_SCALE 20

class InterfaceProcessIPMImage{
public:
	InterfaceProcessIPMImage();
	InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, double rx);
	InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, const Mat &disp);
	~InterfaceProcessIPMImage(){ delete ipm; }
	




	void showVehiclePosition(bool infoVISO, bool infoGPS);

	void processIPM(const Mat &image, const Mat &pose, Mat &IPMImage, Mat &IPMImageMask = Mat(), Oxts_Data_Type *gpsData = NULL);
	void processIPM(const Mat &image, const Matrix &pose, Mat &IPMImage, Mat &IPMImageMask = Mat(), Oxts_Data_Type *gpsData = NULL);

	InversePerspectiveMapping *ipm;
	int height, width;
	Mat ipmImage;

private:
	bool showInfoVISO, showInfoGPS;
	Mat rotation_gps;
	double gpsData0[6];//x0,y0,h0,roll0,pitch0,yaw0
	void initRotationGPS(Oxts_Data_Type *gpsData0);
	double s, r; // for RADIANEARTH

	void drawVehiclePosition();
	void drawVehiclePosition(Oxts_Data_Type *gpsData);//gps position

	void resampleIPMImage(Mat &IPMImage, Mat &mask);

	// transformation of coordinates
	void XZtoIPMImage(double X, double Z, double &r, double &c);
	void IPMImageUVtoXZ(int r, int c, double &X, double &Z);
};



#endif