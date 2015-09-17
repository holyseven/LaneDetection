#ifndef ELAS_DISPARITY_INTERFACE_H
#define ELAS_DISPARITY_INTERFACE_H
#include "elas.h"
#include "image.h"
#include <opencv2\opencv.hpp>
using namespace cv;

class InterfaceProcessELAS{

public:
	InterfaceProcessELAS();
	InterfaceProcessELAS(Elas::parameters param);
	~InterfaceProcessELAS(){ delete elas; };


	Elas::parameters param;
	Elas *elas;

	void computeDisparity(const Mat &left_img,
		const Mat &right_img, Mat &disp);
	
};

#endif