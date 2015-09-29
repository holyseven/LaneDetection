#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;
#include <vector>
#include "../LSD1.5/lsd.h"
#include "../IPMImage/InversePerspectiveMapping.h"

class Segment2d
{
public:
	Point2d p1;
	Point2d p2;
	int indexZone[4];//0 : y of small; 1 : y of big; 3 : x of small; 4 : x of big (of p1 and p2)
	Segment2d(){};
	Segment2d(Point2d _p1, Point2d _p2);

	double getSlope(){
		if (!_ini_slope) computeSlope();
		return slope;
	}
	double getLength(){
		if (!_ini_length) computeLength();
		return length;
	}

	bool isNeighbor(Segment2d s);

	std::vector<Segment2d> getValidRect(Segment2d s, int rows);

private:
	bool _ini_slope;
	bool _ini_length;
	double slope;
	double length;
	void computeSlope();
	void computeLength();
};


class Pair2d{
public:
	Segment2d s1;
	Segment2d s2;
	Point2d validRect[4];//0,1 is on s1; 2,3 is on s2.
	Point2d _p1, _p2;//_p1 = (validRect[0] + validRect[2]) / 2; _p2 = (validRect[1] + validRect[3]) / 2;
	Segment2d _p12; // _p12 = Segment2d(_p1, _p2);

	Pair2d(Segment2d _s1, Segment2d _s2) { 
		s1 = _s1, s2 = _s2; 
		hasModeled = false; horizontal = false;
		mean_color = -1;
		mean_width = -1;
	}
	Pair2d(Segment2d _s1, Segment2d _s2, Point2d p0, Point2d p1, Point2d p2, Point2d p3)
	{ 
		s1 = _s1, s2 = _s2;
		validRect[0] = p0;
		validRect[1] = p1;
		validRect[2] = p2;
		validRect[3] = p3;
		hasModeled = false; horizontal = false;
		mean_color = -1;
		mean_width = -1;
	}

	void computeLineModel();
	Point2d intersec(Pair2d *p2);//intersection of two lines.
	bool parallele(Pair2d *p2, double prec = 22.5);//prec : precision in unit of degree.
	bool aligned(Pair2d* p2, double prec = 5);//prec: precision in unit of pixel.
	double weight(Pair2d* p2);
	void setMeanColor(double _color){ mean_color = _color; };
	void setMeanWidth(double _width){ mean_width = _width; };
	double mean_width;
	double mean_color;

private:
	double a, b, c;//line model : x = ay + b, or y = c;
	bool hasModeled;
	bool horizontal;

	
};

class LaneDetection
{
public:
	LaneDetection();
	LaneDetection(const Mat &ipmImage, const int center_x = 690, const int center_y = 233);
	LaneDetection(const Mat &ipmImage, const Mat &ipmMask, const int center_x = 690, const int center_y = 233);

	void run(int method = 1, int nameIndex_showImage = 0, InversePerspectiveMapping* ipm = NULL) {
		if (method == 1)
			method1();
		if (method == 2)
			method2();
		if (method == 3)
			method3(nameIndex_showImage);
		if (method == 4)
			method4(nameIndex_showImage, ipm);
	}


	void detectionLineLSD(Mat &processImage);
	void detectionLineLSDInPerspImage(Mat &processImage);
	void blobProcess(Mat &processImage);
	void blobProcess2(Mat &processImage, const Mat &ipmImage);

	void method1();
	void method2();
	void method3(int);
	void method4(int, InversePerspectiveMapping* ipm);

	Mat ipmImage;
	Mat processImage;
	Mat processGrayImage;
	Mat ipmMask;
	int center_x;
	int center_y;

	void AlgoFilterLanes(ntuple_list lines_out);
	void AlgoFilterLanes_back(ntuple_list lines_out);

	int nameIndex;
};






#endif // LANEDETECTION_H