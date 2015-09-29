#ifndef INTERFACEPROCESSVISO_H
#define INTERFACEPROCESSVISO_H

#define _CRT_SECURE_NO_WARNINGS
#include "png++/png.hpp"
#include "viso_stereo.h"
#include "../KITTI_Data_Reader/KITTI_Data_Reader.h"
#include <opencv2\opencv.hpp>
using namespace cv;

//InterfaceProcessVISO computes camera pose.
//VisualOdometryStereo
class InterfaceProcessVISO{

public:
	InterfaceProcessVISO();
	InterfaceProcessVISO(VisualOdometryStereo::parameters _param,
		int width, int height);
	InterfaceProcessVISO(Calib_Data_Type calibData);
	~InterfaceProcessVISO(){ delete viso; };


	VisualOdometryStereo::parameters param;
	VisualOdometryStereo *viso;
	int width, height;
	Matrix pose;

	void processVISO(png::image< png::gray_pixel > left_img,
		png::image< png::gray_pixel > right_img);
	void processVISO(const Mat &left_img,
		const Mat &right_img);

private :
	void init(VisualOdometryStereo::parameters _param,
		int width, int height);
	uint8_t* left_img_data;
	uint8_t* right_img_data;
};

#endif
