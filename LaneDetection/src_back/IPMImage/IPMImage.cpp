#include "IPMImage.h"


InterfaceProcessIPMImage::InterfaceProcessIPMImage(){
	ipm = new InversePerspectiveMapping();
	showInfoVISO = false;
	showInfoGPS = false;
}


InterfaceProcessIPMImage::InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, double rx){
	ipm = new InversePerspectiveMapping(calibData.S_rect_00[1], calibData.S_rect_00[0]);
	ipm->createModelForStandardAssumption(calibData.P_rect_00[0], calibData.P_rect_00[0],
		calibData.P_rect_00[2], calibData.P_rect_00[6], h, rx);
	width = calibData.S_rect_00[0];
	height = calibData.S_rect_00[1];
	ipmImage = Mat::zeros((IPM_Z_MAX - IPM_Z_MIN) * IPM_IMAGE_SIZE_SCALE,
		(IPM_X_MAX - IPM_X_MIN) * IPM_IMAGE_SIZE_SCALE, CV_8UC1);
	showInfoVISO = false;
	showInfoGPS = false;
}

InterfaceProcessIPMImage::InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, const Mat &disp){
	ipm = new InversePerspectiveMapping(calibData.S_rect_00[1], calibData.S_rect_00[0]);
	ipm->createModelForStandardAssumption(calibData.P_rect_00[0], calibData.P_rect_00[0],
		calibData.P_rect_00[2], calibData.P_rect_00[6], h, disp);
	width = calibData.S_rect_00[0];
	height = calibData.S_rect_00[1];
	ipmImage = Mat::zeros((IPM_Z_MAX - IPM_Z_MIN) * IPM_IMAGE_SIZE_SCALE,
		(IPM_X_MAX - IPM_X_MIN) * IPM_IMAGE_SIZE_SCALE, CV_8UC1);
	showInfoVISO = false;
	showInfoGPS = false;
}

void InterfaceProcessIPMImage::showVehiclePosition(bool infoVISO, bool infoGPS)
{
	showInfoVISO = infoVISO;
	showInfoGPS = infoGPS;
}

void InterfaceProcessIPMImage::processIPM(const Mat &image, const Mat &pose, Mat &outIPMImage, Mat &IPMImageMask,
	Oxts_Data_Type *gpsData)
{
	ipm->updateFrameUsingStandardAssumption(pose);

	Mat grayImage;
	if (image.channels() > 1)
		cvtColor(image, grayImage, CV_BGR2GRAY);
	else
		image.copyTo(grayImage);

	Mat mask = Mat::zeros(ipmImage.size(), CV_8UC1);
	for (int i = 0; i < height; i++)
	{
		if (i < height / 2) continue;
		uchar* ptr_row_grayImage = grayImage.ptr<uchar>(i);
		for (int j = 0; j < width; j++) //(i,j) gray image coordicates : row, col
		{
			double X, Z;
			X = ipm->remapX[i*width + j];
			Z = ipm->remapZ[i*width + j];
			
			if (Z < IPM_Z_MIN || Z >= IPM_Z_MAX) continue;
			if (X < IPM_X_MIN || X >= IPM_X_MAX) continue;

			double scale_reso = IPM_Z_MAX / 0.2;
			double resolution = Z / scale_reso;
			double r0, r1, c0, c1;
			XZtoIPMImage(X - resolution, Z - resolution, r0, c0);
			XZtoIPMImage(X + resolution, Z + resolution, r1, c1);
			if (r0 > r1)
			{
				double t = r1;
				r1 = r0;
				r0 = t;
			}
			if (c0 > c1)
			{
				double t = c1;
				c1 = c0;
				c0 = t;
			}
			int int_r0 = r0 + 0.5;
			int int_r1 = r1 + 0.5;
			int int_c0 = c0 + 0.5;
			int int_c1 = c1 + 0.5;

			for (int r = int_r0; r <= int_r1 && r < ipmImage.rows; r++)
			{
				if (r < 0) continue;
				uchar *ptr_row_ipm_image = ipmImage.ptr<uchar>(r);
				uchar *ptr_row_mask = mask.ptr<uchar>(r);

				for (int c = int_c0; c <= int_c1 && c < ipmImage.cols; c++)
				{
					if (c < 0) continue;
					ptr_row_mask[c] = 255;
					if (ptr_row_ipm_image[c] == 0)
						ptr_row_ipm_image[c] = ptr_row_grayImage[j];
					else
						ptr_row_ipm_image[c] = 0.5 * ptr_row_ipm_image[c] + 0.5 * ptr_row_grayImage[j];
				}
					
			}
		}
	}

	//imwrite("ipm_before_resample.png", ipmImage);
	resampleIPMImage(ipmImage, mask);
	//imwrite("ipm_after_resample.png", ipmImage);

	if (showInfoVISO)
		;
	if (showInfoGPS && (gpsData != NULL))
		drawVehiclePosition(gpsData);

	ipmImage.copyTo(outIPMImage);
	mask.copyTo(IPMImageMask);
}

void InterfaceProcessIPMImage::XZtoIPMImage(double X, double Z, double &r, double &c){
	r = (Z - IPM_Z_MIN) * ((float)ipmImage.rows) / (IPM_Z_MAX - IPM_Z_MIN);
	r = ipmImage.rows - r - 1;
	c = (X - IPM_X_MIN) * ((float)ipmImage.cols) / (IPM_X_MAX - IPM_X_MIN);
}

void InterfaceProcessIPMImage::IPMImageUVtoXZ(int r, int c, double &X, double &Z){

}

void InterfaceProcessIPMImage::processIPM(const Mat &image, const Matrix &pose, Mat &IPMImage, Mat &IPMImageMask,
	Oxts_Data_Type *gpsData)
{
	Mat mat_pose = Mat::eye(pose.m, pose.n, CV_64FC1);
	double* ptr_mat_pose = mat_pose.ptr<double>(0);
	for (int i = 0; i < mat_pose.rows; i++)
		for (int j = 0; j < mat_pose.cols; j++)
			ptr_mat_pose[i*mat_pose.cols + j] = pose.val[i][j];
	
	processIPM(image, mat_pose, IPMImage, IPMImageMask, gpsData);
}

void InterfaceProcessIPMImage::drawVehiclePosition()
{
}

void InterfaceProcessIPMImage::resampleIPMImage(Mat &IPMImage, Mat &mask)
{
	//method 1:
	//interpolation between row - 1 and row + 1
	//interpolation only for one black row
	//row[2]
	//row[1]
	//row[0]

	int rows = IPMImage.rows, cols = IPMImage.cols;
	uchar *ptr_row_data[3];
	bool black[3] = { true, true, true };

	for (int _r = rows - 2; _r > 0; _r--)
	{
		for (int index_3 = 0; index_3 < 2; index_3++)
		{
			ptr_row_data[index_3] = ptr_row_data[index_3+1];
			black[index_3] = black[index_3+1];
		}

		black[2] = true;
		ptr_row_data[2] = IPMImage.ptr<uchar>(_r - 1);
		for (int _c = 0; _c < cols; _c++)
		{
			if (ptr_row_data[2][_c] != 0)
			{
				black[2] = false;
				break;
			}
		}

		if (!black[0] && black[1] && !black[2])
		{
			uchar *ptr_row_mask = mask.ptr<uchar>(_r);
			for (int _c = 0; _c < cols; _c++)
			{
				ptr_row_mask[_c] = 255;
				ptr_row_data[1][_c] = 0.5 * ptr_row_data[0][_c] + 0.5 * ptr_row_data[2][_c];
			}
		}
	}
	// end of method 1



}