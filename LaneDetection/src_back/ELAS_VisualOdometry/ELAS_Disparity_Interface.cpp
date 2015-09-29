#include "ELAS_Disparity_Interface.h"

InterfaceProcessELAS::InterfaceProcessELAS()
{
	if (!param.postprocess_only_left)
	{
		param.postprocess_only_left = true;
	}
	elas = new Elas(param);
}

InterfaceProcessELAS::InterfaceProcessELAS(Elas::parameters _param)
{
	param = _param;
	if (!param.postprocess_only_left)
	{
		param.postprocess_only_left = true;
	}
	elas = new Elas(param);
}
void InterfaceProcessELAS::computeDisparity(const Mat &left_img,
	const Mat &right_img, Mat &disp)
{
	Mat L, R;
	if (left_img.channels() > 1)
		cvtColor(left_img, L, CV_BGR2GRAY);
	else
		left_img.copyTo(L);
	if (right_img.channels() > 1)
		cvtColor(right_img, R, CV_BGR2GRAY);
	else
		right_img.copyTo(R);

	
	image<uchar> *I1, *I2;
	I1 = loadFromCVMatGray(L);
	I2 = loadFromCVMatGray(R);

	int32_t width = I1->width();
	int32_t height = I1->height();

	const int32_t dims[3] = { width, height, width }; // bytes per line = width
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));

	elas->process(I1->data, I2->data, D1_data, D2_data, dims);




	// find maximum disparity for scaling output disparity images to [0..255]
	float disp_max = 0;
	for (int32_t i = 0; i<width*height; i++) {
		if (D1_data[i]>disp_max) disp_max = D1_data[i];
		if (D2_data[i]>disp_max) disp_max = D2_data[i];
	}

	// copy float to uchar
	image<uchar> *D1 = new image<uchar>(width, height);
	for (int32_t i = 0; i<width*height; i++) {
		D1->data[i] = (uint8_t)max(255.0*D1_data[i] / disp_max, 0.0);
	}

	// save images using cv::Mat so we can save any format' images. 
	//saveCVMat(D1, "disp.png");


	toCVMat(D1, disp);

	delete D1;
	delete D1_data;
	delete D2_data;
	delete I1;
	delete I2;
}