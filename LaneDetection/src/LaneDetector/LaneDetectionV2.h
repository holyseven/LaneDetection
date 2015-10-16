#pragma once

#include <opencv2\opencv.hpp>
using namespace cv;

#include "../LSD1.5/lsd.h"
#include "../ConverterCoordinates/CC.h"
#include "EKF.h"


#define half_pi CV_PI / 2

class Segment2d
{
public:
	Point2d p1;
	Point2d p2;
	int indexZone[4];//0 : y of small; 1 : y of big; 2 : x of small; 3 : x of big (of p1 and p2)
	Segment2d() {};
	Segment2d(Point2d _p1, Point2d _p2, int scale = 3);

	double getSlope() {
		if (!_ini_slope) computeSlope();
		return slope;
	}
	double getLength() {
		if (!_ini_length) computeLength();
		return length;
	}

	double computeDistance(Point2d p);

	bool isNeighbor(Segment2d s);
	bool isNeighbor(Point2d s, double distMax = 20);

	//s = Point2d(0, 0). seg is this.
	bool isInView(double x_min = -20, double x_max = 15, double z_min = -2, double z_max = 80);

	bool isClose(Point2d s);
	Point2d intersec(Segment2d *s);

	bool maybePair(Segment2d *s);
	std::vector<Segment2d> getValidRect(Segment2d s);

private:
	bool _ini_slope;
	bool _ini_length;
	double slope;
	double length;
	void computeSlope();
	void computeLength();
	int _scale;
};

struct MyMarking{
	Segment2d seg;
	double width;
	int type;
	int index_in_lanes;
	int index_in_a_lane;
};

class Pair2d {
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
	void setMeanColor(double _color) { mean_color = _color; };
	void setMeanWidth(double _width) { mean_width = _width; };
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
	struct Param{
		double slope_dif;//
		double rect_dist_max, rect_dist_min;
		double color_dif;
	} ;

	//LaneDetection();
	LaneDetection(const Mat &img = Mat());

	void init(int nameIndex_showImage = 0, CC::CC_SimpleIPM* _ipm = NULL) {
		nameIndex = nameIndex_showImage;
		ipm = _ipm;
	}

	//step1 : LSD detection of lines
	//work on rawGrayImage
	ntuple_list resultLSD();

	//step2 : update ipm (estimate rx)
	//work on ipm and kf
	void updateIPM(std::vector<Pair2d> pairs, std::vector<Pair2d> pairs_in_image);
	void updateIPM2(std::vector<Pair2d> pairs_in_image);

	void findPairs(std::vector<Pair2d> &pairs, std::vector<Pair2d> &pairs_in_image, const Mat &maskRoad = Mat());

	void detectionLineLSD(Mat &processImage);

	void method1();
	void method2(const Mat &img);
	std::vector<Pair2d> method3(const Mat &img, int winFlag = 0, const Mat &maskRoad = Mat());
	std::vector<Pair2d> method4(const Mat &rL, const Mat &disp, int winFlag = 0);//stereo

	void segmentationRoad(Mat &maskRoad);

	//double estimateRx(std::vector<Segment2d> *segments_in_image);

	Mat rawImage;
	Mat rawGrayImage;
	Mat rawColorImage;

	int nameIndex;
	CC::CC_SimpleIPM* ipm;
	KalmanFilter kf;
	EKalmanFilter ekf;
	Point2d vp;//vanishing point
	double vpy;

	const int x_min = -10, x_max = 15;//x_min = -20, x_max = 15
	const int z_min = 6, z_max = 50;//z_min = 6, z_max = 80

private:
	ntuple_list lsd_result;
	Vec3b roadColor;
};