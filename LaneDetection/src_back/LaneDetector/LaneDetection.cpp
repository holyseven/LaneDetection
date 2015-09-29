#include "LaneDetection.h"

#include <iostream>

using namespace std;
LaneDetection::LaneDetection(){
}

LaneDetection::LaneDetection(const Mat &_ipmImage, const int x, const int y){
	_ipmImage.copyTo(ipmImage);
	_ipmImage.copyTo(processImage);
	center_x = x;
	center_y = y;
}

LaneDetection::LaneDetection(const Mat &_ipmImage, const Mat &_ipmMask, const int x, const int y){
	_ipmImage.copyTo(ipmImage);
	_ipmImage.copyTo(processImage);
	_ipmMask.copyTo(ipmMask);
	center_x = x;
	center_y = y;
}

void LaneDetection::detectionLineLSD(Mat &processImage) {
	unsigned int X = processImage.cols;  /* x image size */
	unsigned int Y = processImage.rows;  /* y image size */
	unsigned int XY = X * Y; /* numer of pixels */

	/* create a simple image: left half black, right half gray */
	//Vec3b intensity;
	//image = new_image_double(X,Y);
	//for(x=0;x<X;x++)
	//	for(y=0;y<Y;y++){
	//		//image->data[ x + y * image->xsize ] = x<X/2 ? 0.0 : 64.0; /* image(x,y) */
	//		intensity = inputImage.at<Vec3b>(y,x);
	//		image->data[ x + y * image->xsize ] = (double) (intensity.val[2]+intensity.val[1]+intensity.val[0])/3;
	//	}

	if (processImage.channels() == 1)
		processImage.copyTo(processGrayImage);
	else if (processImage.channels() == 3)
		cvtColor(processImage, processGrayImage, CV_BGR2GRAY);
	

	/*blur the image*/
	//blur(grayImage, grayImage, Size(5, 5));


	image_double image = new_image_double(X, Y);
	/*copy the data*/
	uchar* ptr_gray_image = processGrayImage.ptr<uchar>(0);
	for (int _pixel = 0; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}

	/* call LSD */
	ntuple_list out = lsd(image);

	/* print output */
	//LOG_INFO("line segments found: " << out->size);

	Mat colorImage;
	if (processImage.channels() == 1)
		cvtColor(processImage, processImage, CV_GRAY2BGR);

	/*draw the lines*/
 	for (int i = 0; i < out->size; i++)
	{
		int b = (unsigned)theRNG() & 255;
		int g = i & 255;
		int r = i / 255;
		int thickness = (int)out->values[i * out->dim + 4] / 2;
		int lineType = 8;
		Point start = cv::Point(out->values[i * out->dim + 0] + 0.5, out->values[i * out->dim + 1] + 0.5),
			end = cv::Point(out->values[i * out->dim + 2] + 0.5, out->values[i * out->dim + 3] + 0.5);

		line(processImage, start, end, Scalar(b, g, r), 1, lineType);
	}

	/* free memory */
	free_image_double(image);
	free_ntuple_list(out);


}

void LaneDetection::blobProcess(Mat &processImage)
{
	ConnectedComponentsTypes;
	Mat labelImage, stats, centroids;
	int num = connectedComponentsWithStats(processImage, labelImage, stats, centroids);

	//cout << "stats: " << stats << endl;
	//cout << "centroids: " << centroids << endl;

	int *ptr_labelImage = labelImage.ptr<int>(0);
	int rows = labelImage.rows, cols = labelImage.cols;
	int pixels = rows *cols;
	//Mat colorImage;
	//cvtColor(processImage, colorImage, CV_GRAY2BGR);

	for (int i = 0; i < num; i++)
	{
		int *ptr_row_stats = stats.ptr<int>(i);
		bool possible_noise = ptr_row_stats[CC_STAT_HEIGHT] < 10 || ptr_row_stats[CC_STAT_AREA] < 5;
		if (!possible_noise) continue;

		for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
		{
			int *ptr_row_label = labelImage.ptr<int>(y);
			uchar *ptr_row_processImage = processImage.ptr<uchar>(y);
			for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
			{
				if (ptr_row_label[x] == i)
				{
					ptr_row_processImage[x] = 0;
				}
			}
		}
		
	}

	
}

void LaneDetection::detectionLineLSDInPerspImage(Mat &processImage){
	unsigned int X = processImage.cols;  /* x image size */
	unsigned int Y = processImage.rows;  /* y image size */
	unsigned int XY = X * Y; /* numer of pixels */

	if (processImage.channels() == 1)
		processImage.copyTo(processGrayImage);
	else if (processImage.channels() == 3)
		cvtColor(processImage, processGrayImage, CV_BGR2GRAY);


	image_double image = new_image_double(X, Y);
	/*copy the data*/
	uchar* ptr_gray_image = processGrayImage.ptr<uchar>(0);
	for (int _pixel = XY * 0.4; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}

	/* call LSD */
	//ntuple_list line_out = lsd(image);
	ntuple_list line_out = LineSegmentDetection(image, 0.8, 0.6, 2.0, 22.5, 0.0, 0.7, 1024, 255.0, 0);

	if (processImage.channels() == 1)
		cvtColor(processImage, processImage, CV_GRAY2BGR);

	AlgoFilterLanes(line_out);

	/* free memory */
	free_image_double(image);
	free_ntuple_list(line_out);

}

void LaneDetection::blobProcess2(Mat &processImage, const Mat &ipmImage)
{
	Mat labelImage, stats, centroids;
	int num = connectedComponentsWithStats(processImage, labelImage, stats, centroids);

	int *ptr_labelImage = labelImage.ptr<int>(0);
	int rows = labelImage.rows, cols = labelImage.cols;
	int pixels = rows *cols;

	double *colorSum = new double[num];
	int *vote = new int[num];

	double meanArea = 0;
	for (int i = 0; i < num; i++)
	{
		int *ptr_row_stats = stats.ptr<int>(i);
		if (ptr_row_stats[CC_STAT_WIDTH] * ptr_row_stats[CC_STAT_HEIGHT] > 0.5 * pixels)
		{
			continue;
		}
		meanArea += ptr_row_stats[CC_STAT_AREA];
	}
	meanArea = meanArea / num;
	cvtColor(processImage, processImage, CV_GRAY2BGR);
	for (int i = 0; i < num; i++)
	{
		colorSum[i] = 0;
		vote[i] = 0;
		int *ptr_row_stats = stats.ptr<int>(i);
		if (ptr_row_stats[CC_STAT_WIDTH] * ptr_row_stats[CC_STAT_HEIGHT] > 0.5 * pixels)
		{
			continue;
		}

		for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
		{
			int *ptr_row_label = labelImage.ptr<int>(y);
			const uchar *ptr_row_ipmImage = ipmImage.ptr<uchar>(y);
			for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
			{
				if (ptr_row_label[x] == i)
				{
					colorSum[i] += ptr_row_ipmImage[x];
				}
			}
		}
		colorSum[i] = colorSum[i] / ptr_row_stats[CC_STAT_AREA];

		int v = (colorSum[i] - 128) / 32 + 0.5;
		
		cout << "[" << ptr_row_stats[CC_STAT_LEFT] << "," << ptr_row_stats[CC_STAT_TOP] << ";"
			<< ptr_row_stats[CC_STAT_WIDTH] << "," << ptr_row_stats[CC_STAT_HEIGHT] << "]" << " : ";
		cout << v << ",";
		vote[i] += v;

		double val = ((double)ptr_row_stats[CC_STAT_HEIGHT] / ptr_row_stats[CC_STAT_WIDTH] - 4);
		if (val < 0)
			v = val + 0.5;
		else if (val > 10)
			v = 4;
		else if (val > 7)
			v = 3;
		else if (val > 4)
			v = 2;
		else if (val >= 0)
			v = 1;
		cout << v << ",";
		vote[i] += v;

		val = (ptr_row_stats[CC_STAT_AREA] - meanArea / 2);
		if (val < 0)
			v = val / meanArea * 10 + 0.5;
		else if (val > 4 * meanArea)
			v = 4;
		else if (val > 2 * meanArea)
			v = 3;
		else if (val > meanArea)
			v = 2;
		else if (val >= 0)
			v = 1;
		cout << v << "," << endl;;
		vote[i] += v;

		//if (vote[i] > 0) continue;

		
		for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
		{
			int *ptr_row_label = labelImage.ptr<int>(y);
			Vec3b *ptr_row_processImage = processImage.ptr<Vec3b>(y);
			for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
			{
				if (ptr_row_label[x] == i)
				{
					if (vote[i] > 5)
						ptr_row_processImage[x] = Vec3b(255, 0, 0);
					else if (vote[i] > 0)
						ptr_row_processImage[x] = Vec3b(0, 255, 0);
					else
						ptr_row_processImage[x] = Vec3b(0, 0, 255);
				}
			}
		}
	}
	
	delete colorSum;
	delete vote;

	//int indexMaxArea = -1;
	//int maxArea = -1;

	//int indexMaxHeight = -1;
	//int maxHeight = -1;
	//for (int i = 0; i < num; i++)
	//{
	//	int *ptr_row_stats = stats.ptr<int>(i);
	//	if (ptr_row_stats[CC_STAT_WIDTH] * ptr_row_stats[CC_STAT_HEIGHT] > 0.5 * pixels)
	//	{
	//		continue;
	//	}
	//	if (maxArea < ptr_row_stats[CC_STAT_AREA])
	//	{
	//		maxArea = ptr_row_stats[CC_STAT_AREA];
	//		indexMaxArea = i;
	//	}
	//	if (maxHeight < ptr_row_stats[CC_STAT_HEIGHT])
	//	{
	//		maxHeight = ptr_row_stats[CC_STAT_HEIGHT];
	//		indexMaxHeight = i;
	//	}
	//}



	//int *ptr_row_stats = stats.ptr<int>(indexMaxArea);

	//for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
	//{
	//	int *ptr_row_label = labelImage.ptr<int>(y);
	//	uchar *ptr_row_processImage = processImage.ptr<uchar>(y);
	//	for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
	//	{
	//		if (ptr_row_label[x] == indexMaxArea)
	//		{
	//			ptr_row_processImage[x] = 180;
	//		}
	//	}
	//}

	//ptr_row_stats = stats.ptr<int>(indexMaxHeight);

	//for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
	//{
	//	int *ptr_row_label = labelImage.ptr<int>(y);
	//	uchar *ptr_row_processImage = processImage.ptr<uchar>(y);
	//	for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
	//	{
	//		if (ptr_row_label[x] == indexMaxHeight)
	//		{
	//			ptr_row_processImage[x] = 100;
	//		}
	//	}
	//}

}


void LaneDetection::method3(int nameIndex){
	if (!processImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	Mat temp_image;
	processImage.copyTo(temp_image);
	detectionLineLSD(temp_image);
	char windowName[64];
	sprintf(windowName, "%d", nameIndex);

	imshow(string(windowName) + "step0", temp_image);
	imwrite(string(windowName) + "step0.png", temp_image);

	detectionLineLSDInPerspImage(processImage);
	imshow(string(windowName) + "step1", processImage);
	imwrite(string(windowName) + "step1.png", processImage);

}


void LaneDetection::method2(){
	if (!processImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	//detectionLineLSD(processImage);
	//imwrite("processImage.png", processImage);

	Mat step1 = Mat::zeros(processImage.size(), CV_8UC1);
	
	const int m = 3, T = 50, T_s = 0;
	for (int _r = 0; _r < step1.rows; _r++)
	{
		uchar *ptr_row_processImage = processImage.ptr<uchar>(_r);
		uchar *ptr_row_ipmMask = ipmMask.ptr<uchar>(_r);
		for (int _c = m; _c < step1.cols - m; _c++)
		{
			if (ptr_row_ipmMask[_c] == 0 || ptr_row_ipmMask[_c - m] == 0 || ptr_row_ipmMask[_c + m] == 0)
				continue;
			int bm1 = ptr_row_processImage[_c] - ptr_row_processImage[_c - m];
			if (bm1 < T_s) continue;
			int bm2 = ptr_row_processImage[_c] - ptr_row_processImage[_c + m];
			if (bm2 < T_s) continue;
			if (bm1 + bm2 < T)
				continue;
			step1.at<uchar>(_r, _c) = 255;
		}
	}

	imshow("step0", step1);

	blobProcess(step1);
	imshow("step1", step1);
	blobProcess2(step1, processImage);
	imshow("step2", step1);
	detectionLineLSD(step1);

	imshow("step3", step1);
	imwrite("step1.png", step1);
	imwrite("processImage.png", processImage);
}


void LaneDetection::method1(){
	if (!processImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	//imshow("LaneDetection input image", processImage);
	//imwrite("ipm.png", processImage);

	//Mat image;
	//processImage.copyTo(image);
	//detectionLineLSD(image);
	//imshow("image", image);
	//imwrite("ipm_lines.png", image);

	Sobel(processImage, processImage, CV_8U, 1, 0, 1, 2);
	Sobel(processImage, processImage, CV_8U, 1, 0, 1, 2);
	imshow("sobel-2", processImage);

	imshow("sobel-2_", processImage);
	imwrite("sobel-2.png", processImage);
	threshold(processImage, processImage, 0, 255, THRESH_OTSU);
	

	blobProcess(processImage);

	//imshow("blobProcess", processImage);
	imwrite("lsd1.png", processImage);

	detectionLineLSD(processImage);
	imshow("lsd", processImage);
	imwrite("lsd2.png", processImage);

}