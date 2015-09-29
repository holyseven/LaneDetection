#include "IPMImage.h"

#define PI_180 CV_PI/180
#define RADIANEARTH 6378137

void InterfaceProcessIPMImage::initRotationGPS(Oxts_Data_Type *oxtsData)
{
	r = RADIANEARTH;
	s = cos(oxtsData->lat*PI_180);
	r = s*r;
	gpsData0[0] = r * oxtsData->lon * PI_180;
	gpsData0[1] = r * log(tan(PI_180 / 2 * (90 + oxtsData->lat)));
	gpsData0[2] = oxtsData->alt;
	gpsData0[3] = oxtsData->roll;
	gpsData0[4] = oxtsData->pitch;
	gpsData0[5] = oxtsData->yaw;

	double rotation_coor = atan2(oxtsData->ve, oxtsData->vn);
	rotation_gps = Mat::zeros(2, 2, CV_64FC1);
	double cr = cos(rotation_coor), sr = sin(rotation_coor);
	rotation_gps.at<double>(0, 0) = cr;
	rotation_gps.at<double>(0, 1) = -sr;
	rotation_gps.at<double>(1, 0) = sr;
	rotation_gps.at<double>(1, 1) = cr;
}


void InterfaceProcessIPMImage::drawVehiclePosition(Oxts_Data_Type *oxtsData)
{
	if (rotation_gps.empty())
	{
		initRotationGPS(oxtsData);
	}

	double gps_x = r * oxtsData->lon * PI_180 - gpsData0[0];
	double gps_y = r * log(tan(PI_180 / 2 * (90 + oxtsData->lat))) - gpsData0[1];
	Mat gps_p = Mat::zeros(2, 1, CV_64FC1);
	gps_p.at<double>(0) = gps_x;
	gps_p.at<double>(1) = gps_y;
	InversePerspectiveMapping::rotation2D(rotation_gps, gps_p);

	circle(ipmImage, Point((gps_p.at<double>(0) - IPM_X_MIN)*ipmImage.cols / (IPM_X_MAX - IPM_X_MIN),
		ipmImage.rows - (gps_p.at<double>(1) - IPM_Z_MIN)*ipmImage.rows / (IPM_Z_MAX - IPM_Z_MIN)),
		oxtsData->pos_accuracy *ipmImage.cols / (IPM_X_MAX - IPM_X_MIN),
		Scalar(255));
}