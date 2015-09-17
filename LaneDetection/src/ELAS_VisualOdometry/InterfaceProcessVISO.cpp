#include "InterfaceProcessVISO.h"

InterfaceProcessVISO::InterfaceProcessVISO(){
	viso = new VisualOdometryStereo(VisualOdometryStereo::parameters());
}
InterfaceProcessVISO::InterfaceProcessVISO(VisualOdometryStereo::parameters _param, int _width, int _height)
{
	param = _param;
	width = _width;
	height = _height;
	pose = Matrix::eye(4);
	viso = new VisualOdometryStereo(param);
}

InterfaceProcessVISO::InterfaceProcessVISO(Calib_Data_Type calibData)
{
	param.calib.f = calibData.P_rect_00[0]; // focal length in pixels
	param.calib.cu = calibData.P_rect_00[2]; // principal point (u-coordinate) in pixels
	param.calib.cv = calibData.P_rect_00[6]; // principal point (v-coordinate) in pixels
	param.base = sqrt(calibData.T_01[0] * calibData.T_01[0]
		+ calibData.T_01[1] * calibData.T_01[1]); // baseline in meters

	width = calibData.S_rect_00[0];
	height = calibData.S_rect_00[1];
	pose = Matrix::eye(4);
	viso = new VisualOdometryStereo(param);
}

void InterfaceProcessVISO::init(VisualOdometryStereo::parameters _param,
	int _width, int _height)
{
	param = _param;
	viso = new VisualOdometryStereo(param);
	width = _width;
	height = _height;
	pose = Matrix::eye(4);
}

void InterfaceProcessVISO::processVISO(const Mat &left_img,
	const Mat &right_img)
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


	left_img_data = (uint8_t*)L.data;
	right_img_data = (uint8_t*)R.data;

	int32_t dims[] = { width, height, width };
	if (viso->process(left_img_data, right_img_data, dims)) {
		// on success, update current pose
		pose = pose * Matrix::inv(viso->getMotion());
	}

}
