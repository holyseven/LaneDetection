#include "LaneDetectionV2.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
using namespace std;
#define EPS_HERE 1.0e-2
#define INFINI_HERE 1.0e6

//#define DEBUG_FOUT
#define DEBUG_drawImage

#ifdef DEBUG_FOUT
ofstream fout_2;
#endif

inline double kernel(Point2d x) {
	return exp2(-x.ddot(x)/100);
}
inline double kernel(double x){
	return exp2(-x*x / 10);
}

//(p1-p2)*(p1-p2)
inline double pow_dist_p2p(Point2d p1, Point2d p2)
{
	return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

//distance between two points
inline double dist_p2p(Point2d p1, Point2d p2)
{
	return sqrt(pow_dist_p2p(p1, p2));
}

inline double slope_seg(Point2d p1, Point2d p2)
{
	if (abs(p2.x - p1.x) < EPS_HERE) return INFINI_HERE;
	return (p2.y - p1.y) / (p2.x - p1.x);
}

double dist_p2line(Point2d p, Segment2d seg)
{
	if (abs(seg.p1.x - seg.p2.x) < EPS_HERE)
	{
		return abs(p.x - seg.p1.x);
	}
	if (seg.p2.y - seg.p1.y < EPS_HERE)
	{
		return abs(p.y - seg.p1.y);
	}

	double k = seg.getSlope();
	double foot_x = (k * k * seg.p1.x + k * (p.y - seg.p1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg.p1.x) + seg.p1.y;

	return dist_p2p(p, Point2d(foot_x, foot_y));
}

Rect2d getBoundingBox_d(Point2d p0, Point2d p1, Point2d p2, Point2d p3)
{
	double tf_x = p0.x, tf_y = p0.y;
	double rb_x = p0.x, rb_y = p0.y;
	if (p1.x < tf_x) tf_x = p1.x;
	if (p1.x > rb_x) rb_x = p1.x;
	if (p1.y < tf_y) tf_y = p1.y;
	if (p1.y > rb_y) rb_y = p1.y;

	if (p2.x < tf_x) tf_x = p2.x;
	if (p2.x > rb_x) rb_x = p2.x;
	if (p2.y < tf_y) tf_y = p2.y;
	if (p2.y > rb_y) rb_y = p2.y;

	if (p3.x < tf_x) tf_x = p3.x;
	if (p3.x > rb_x) rb_x = p3.x;
	if (p3.y < tf_y) tf_y = p3.y;
	if (p3.y > rb_y) rb_y = p3.y;

	return Rect2d(tf_x, tf_y, rb_x - tf_x, rb_y - tf_y);
}


Point2d foot_p2line(Point2d p, Segment2d seg) {
	if (abs(seg.p1.x - seg.p2.x) < EPS_HERE)
	{
		return Point2d(seg.p1.x, p.y);
	}
	if (seg.p2.y - seg.p1.y < EPS_HERE)
	{
		return Point2d(p.x, seg.p1.y);
	}

	double k = seg.getSlope();
	double foot_x = (k * k * seg.p1.x + k * (p.y - seg.p1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg.p1.x) + seg.p1.y;
	return Point2d(foot_x, foot_y);
}

Segment2d::Segment2d(Point2d _p1, Point2d _p2, int scale)
{
	_scale = scale;
	if (_p1.y < _p2.y)
	{
		p1 = _p1;
		p2 = _p2;
	}
	else
	{
		p2 = _p1;
		p1 = _p2;
	}

	indexZone[0] = (int)p1.y / scale;
	indexZone[1] = (int)p2.y / scale;
	if (p1.x <= p2.x)
	{
		indexZone[2] = (int)p1.x / scale;
		indexZone[3] = (int)p2.x / scale;
	}
	else
	{
		indexZone[2] = (int)p2.x / scale;
		indexZone[3] = (int)p1.x / scale;
	}
	_ini_slope = false;
	_ini_length = false;
}

void Segment2d::computeSlope() {
	slope = slope_seg(p1, p2);
	_ini_slope = true;
}

void Segment2d::computeLength() {
	length = dist_p2p(p1, p2);
	_ini_length = true;
}

double Segment2d::computeDistance(Point2d p)
{
	return dist_p2line(p, *this);
}

bool Segment2d::isNeighbor(Segment2d s) {
	bool b2 = s.indexZone[2] > indexZone[2] - 2 && s.indexZone[2] < indexZone[2] + 2;
	if (b2)
		return true;

	b2 = s.indexZone[3] > indexZone[3] - 2 && s.indexZone[3] < indexZone[3] + 2;
	if (b2)
		return true;

	return false;
}

bool Segment2d::isClose(Point2d s) {
	bool b1 = s.y / _scale > indexZone[0] - 5 && s.y / _scale < indexZone[1] + 5;
	bool b2 = s.x / _scale > indexZone[2] - 5 && s.x / _scale < indexZone[3] + 5;
	if (b1 && b2)
		return true;

	return false;
}

bool Segment2d::isNeighbor(Point2d s, double distMax) {
	if (dist_p2line(s, *this) < distMax)
		return true;

	return false;
}

bool Segment2d::isInView(double x_min, double x_max, double z_min, double z_max)
{
	if (this->p1.x > x_min && this->p1.x < x_max && this->p1.y > z_min && this->p1.y < z_max)
		return true;
	if (this->p2.x > x_min && this->p2.x < x_max && this->p2.y > z_min && this->p2.y < z_max)
		return true;
	return false;
}

bool Segment2d::maybePair(Segment2d *seg)
{
	if (seg->p1.y < 0)
		return false;
	if (seg->getLength() > this->getLength())
	{
		return false;
	}
	//simple check, no great computation
	if (!this->isNeighbor(*seg))
	{
		return false;
	}

	//slope difference
	double dif_slope = atan(this->getSlope());
	dif_slope -= atan(seg->getSlope());
	if (dif_slope > half_pi)
		dif_slope -= CV_PI;
	if (dif_slope < -half_pi)
		dif_slope += CV_PI;

	if (abs(dif_slope) > 5 * CV_PI / 180)
	{
#ifdef DEBUG_FOUT
		fout_2 << "abs(dif_slope) > 5 * CV_PI / 180 " << endl;
		fout_2 << dif_slope << endl;
#endif
		return false;
	}

	//pass all conditions above
	return true;
}

vector<Segment2d> Segment2d::getValidRect(Segment2d s)
{
	vector<Segment2d> vec;

	Point2d foot1 = foot_p2line(p1, s);
	Point2d foot2 = foot_p2line(p2, s);
	//distance compare
	//if (min(dist_p2p(foot1, p1), dist_p2p(foot2, p2)) > thresh)
	//{
	//	return vec;
	//}
	Segment2d foot_12(foot1, foot2);

	//possible to improve
	if (s.p1.y > foot_12.p2.y || s.p2.y < foot_12.p1.y)
	{

		return vec;
	}


	//possible to improve
	Point2d arry[4] = { s.p1, foot1, foot2, s.p2 };
	for (int i = 1, j = 0; i < 4; i++)
	{
		Point2d temp = arry[i];
		for (j = i - 1; j >= 0 && temp.y <= arry[j].y; j--)
		{
			arry[j + 1] = arry[j];
		}
		arry[j + 1] = temp;
	}

	if (arry[3].y - arry[0].y < 0.01)
	{
		for (int i = 1, j = 0; i < 4; i++)
		{
			Point2d temp = arry[i];
			for (j = i - 1; j >= 0 && temp.x <= arry[j].x; j--)
			{
				arry[j + 1] = arry[j];
			}
			arry[j + 1] = temp;
		}
	}

	//length compare
	Segment2d valid_foot_12(arry[1], arry[2]);
	if (valid_foot_12.getLength() < s.getLength() * 0.1)
	{
		return vec;//empty
	}


	Point2d new_p1, new_p2;

	new_p1 = foot_p2line(valid_foot_12.p1, *this);
	new_p2 = foot_p2line(valid_foot_12.p2, *this);

	//distance compare
	double thresh = 0.7;
	if (dist_p2p(new_p1, valid_foot_12.p1) > thresh && dist_p2p(new_p2, valid_foot_12.p2) > thresh)
	{
		return vec;
	}
	thresh = 0.1;
	if (dist_p2p(new_p1, valid_foot_12.p1) < thresh || dist_p2p(new_p2, valid_foot_12.p2) < thresh)
	{
		return vec;
	}



	Segment2d new_p12(new_p1, new_p2);

	Point2d t1 = valid_foot_12.p2 - valid_foot_12.p1;
	Point2d t2 = new_p12.p1 - valid_foot_12.p1;
	Point2d t3 = new_p12.p2 - valid_foot_12.p1;
	if (t1.ddot(t2) > t1.ddot(t3))
	{
		Point2d t = new_p12.p1;
		new_p12.p1 = new_p12.p2;
		new_p12.p2 = t;
	}

	if ((new_p1.x - valid_foot_12.p1.x) * (new_p2.x - valid_foot_12.p2.x) < 0)
	{
#ifdef DEBUG_FOUT
		fout_2 << "(new_p1.x - this->p1.x) * (new_p2.x - this->p2.x) " << endl;
		fout_2 << new_p1 << ", " << new_p2 << "; " << this->p1 << ", " << this->p2 << endl;
#endif
		return vec;
	}

	vec.push_back(new_p12);
	vec.push_back(valid_foot_12);

	return vec;
}

Point2d Segment2d::intersec(Segment2d *s)
{
	double slope_dif = s->getSlope() - this->getSlope();
	if (abs(slope_dif) < EPS_HERE)
		return Point2d(-1, -1);

	double x = this->p1.y - s->p1.y + s->getSlope() * s->p1.x - this->getSlope() * this->p1.x;
	x = x / slope_dif;
	double y = this->p1.y + this->getSlope() * (x - this->p1.x);

	return Point2d(x, y);
	
}

void Pair2d::computeLineModel()
{
	double x1, y1, x2, y2;
	_p1 = (validRect[0] + validRect[2]) / 2;
	x1 = _p1.x, y1 = _p1.y;
	_p2 = (validRect[1] + validRect[3]) / 2;
	x2 = _p2.x, y2 = _p2.y;
	_p12 = Segment2d(_p1, _p2);

	if (abs(y1 - y2) < EPS_HERE)
	{
		horizontal = true;
		c = (y1 + y2) / 2;
		a = -1;
		b = -1;
	}
	else
	{
		horizontal = false;
		a = (x2 - x1) / (y2 - y1);
		b = x1 - a * y1;
		c = -1;
	}

	hasModeled = true;
}

double Pair2d::weight(Pair2d* p2)
{
	if (!this->hasModeled) computeLineModel();
	if (!p2->hasModeled) p2->computeLineModel();

	//const double ANG = 0.2618; // 15 degree
	//const double ANG = 0.174533; // 10 degree
	//const double ANG = 0.0174533; // 1 degree
	//const double SCALE_ANG = 0.174533 * 0.174533;
	//const double DIFDIRDIST = 10; // pixels.
	//const double SCALE_DIRDIST = DIFDIRDIST * DIFDIRDIST * 100;
	//const double DIFCOLOR = 10;
	//const double SCALE_DIFCOLOR = 2500;
	const double DIFWIDTH = 0.1;
	const double SCALE_DIFWIDTH = 0.5;
	const double DIFALIDIST = 0.1;
	const double SCALE_DIFALIDIST = 0.5;

	//double dif_angle = 0;
	//if (this->horizontal && p2->horizontal)
	//{
	//}
	//else
	//{
	//	dif_angle = abs(atan(this->a) - atan(p2->a));//radian.
	//}
	//double part_ang = dif_angle - ANG;
	//if (part_ang < 0)
	//	part_ang = 1;
	//else
	//	part_ang = exp(-part_ang * part_ang / SCALE_ANG);

	//double part_color = abs(this->mean_color - p2->mean_color) - DIFCOLOR;
	//if (part_color < 0)
	//	part_color = 1;
	//else
	//	part_color = exp(-part_color * part_color / SCALE_DIFCOLOR);

	double part_width = abs(this->mean_width - p2->mean_width) - DIFWIDTH;
	if (part_width < 0)
		part_width = 1;
	else
		part_width = exp(-part_width * part_width / SCALE_DIFWIDTH);


	//double dif_align_dist = 0.5 * (dist_p2segment(p2->_p1, this->_p12.p1, this->_p12.p2) 
	//	+ dist_p2segment(p2->_p2, this->_p12.p1, this->_p12.p2));
	double dif_align_dist = max(dist_p2line(0.5 * (p2->_p1 + p2->_p2), this->_p12)
		, dist_p2line(0.5 * (this->_p1 + this->_p2), p2->_p12));
	double part_align_dist = dif_align_dist - DIFALIDIST;
	if (part_align_dist < 0)
		part_align_dist = 1;
	else
		part_align_dist = exp(-part_align_dist * part_align_dist / SCALE_DIFALIDIST);


	return part_align_dist * part_width;
}

LaneDetection::LaneDetection(const Mat &img) {
	kf.init(2, 2, 0);
	kf.transitionMatrix = (Mat_<float>(2, 2) << 1, 0, 0, 1);

	//(constant)
	setIdentity(kf.measurementMatrix);

	//has added to the process by opencv. (constant)
	setIdentity(kf.processNoiseCov, Scalar::all(10));
	setIdentity(kf.measurementNoiseCov, Scalar::all(100));

	//(variant)
	setIdentity(kf.errorCovPost, Scalar::all(100));


	if (img.data)
	{
		vp = Point2d(img.cols / 2, img.rows / 2);
		vpy = img.rows / 2;
	}
	else
	{
		vp = Point2d(700, 150);
		vpy = 150;
	}
		
	kf.statePost = (Mat_<float>(2, 1) << vp.x, vp.y);

	ekf.init(2, 1, 0);
	ekf.transitionMatrix = (Mat_<float>(2, 2) << 1, 0, 0, 1);
	setIdentity(ekf.processNoiseCov, Scalar::all(10));
	setIdentity(ekf.errorCovPost, Scalar::all(100));
	ekf.statePost = (Mat_<float>(2, 1) << vp.x, vp.y);
}

ntuple_list LaneDetection::resultLSD() {
	unsigned int X = rawGrayImage.cols;
	unsigned int Y = rawGrayImage.rows;
	unsigned int XY = X * Y;

	image_double image = new_image_double(X, Y);
	
	const uchar* ptr_gray_image = rawGrayImage.ptr<uchar>(0);
	for (int _pixel = 0; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}
	if (lsd_result == NULL || lsd_result->values == NULL)
		;
	else
		free_ntuple_list(lsd_result);

	lsd_result = lsd(image);

#ifdef DEBUG_FOUT
	Mat colorImage;
	rawColorImage.copyTo(colorImage);
	/*draw the lines*/
	for (int i = 0; i < lsd_result->size; i++)
	{
		int b = (unsigned)theRNG() & 255;
		int g = i & 255;
		int r = i / 255;
		int thickness = (int)lsd_result->values[i * lsd_result->dim + 4] / 2;
		int lineType = 8;
		Point start = cv::Point(lsd_result->values[i * lsd_result->dim + 0] + 0.5, lsd_result->values[i * lsd_result->dim + 1] + 0.5),
			end = cv::Point(lsd_result->values[i * lsd_result->dim + 2] + 0.5, lsd_result->values[i * lsd_result->dim + 3] + 0.5);

		line(colorImage, start, end, Scalar(b, g, r), 1, lineType);
	}
	imwrite("lsd.png", colorImage);
#endif
	/* free memory */
	free_image_double(image);

	return lsd_result;
}

void LaneDetection::updateIPM(vector<Pair2d> pairs, vector<Pair2d> pairs_in_image)
{
	vector<Segment2d> segments_in_image;
	vector<Point2d> intersecs;
	int n_pairs = pairs.size();
	Mat updateIPM_img;
	rawColorImage.copyTo(updateIPM_img);
	for (int i = 0; i < n_pairs; i++)
	{
		//if (abs(atan(pairs[i]._p12.getSlope())) < CV_PI / 4)
		//	continue;

		line(updateIPM_img, pairs_in_image[i].s1.p1, pairs_in_image[i].s1.p2, Scalar(255, 0, 0));
		line(updateIPM_img, pairs_in_image[i].s2.p1, pairs_in_image[i].s2.p2, Scalar(255, 0, 0));
		segments_in_image.push_back(pairs_in_image[i].s1);
		segments_in_image.push_back(pairs_in_image[i].s2);
	}
	
	int n_size = segments_in_image.size();

	if (n_size == 0)
		return;


	int nb = 0;

	for (int i = 0; i < n_size; i++)
	{
		Segment2d *s = &segments_in_image[i];

		for (int j = i + 1; j < n_size; j++)
		{
			Point2d p = s->intersec(&segments_in_image[j]);
			if (p.x < 0 || p.y < 0)
				continue;
			if (p.x > rawImage.cols || p.y > rawImage.rows)
				continue;
			nb++;
			//vp += p;
			//circle(step2, p, 1, Scalar(0, 0, 255));
			intersecs.push_back(p);
		}
	}
	//cout << "nb " << nb << endl;
	//mean shift
	Point2d _vp = vp;
	int time = 0;
	int max_iter = 20;
	while (time < max_iter)
	{
		
		time++;
		Point2d v(0, 0);
		double b = 0;
		for (int k = 0; k < nb; k++)
		{
			Point2d x = _vp - intersecs[k];
			//cout << x << ", ";
			double ker = kernel(x);
			v += ker * intersecs[k];
			b += ker;
		}
		//cout << v << ", " << b << endl;
		v = v / b;
		if ((v - _vp).ddot(v - _vp) < EPS_HERE)
		{
			_vp = v;
			break;
		}
		_vp = v;
		
	}

	if (time == max_iter)
	{
		cout << time << endl;
		return;
	}
		
	vp = _vp;
	if (vp.y > rawImage.rows * 0.6)
	{
		vp.y = rawImage.rows * 0.6;
	}
		
	if (vp.y < rawImage.rows * 0.35)
	{
		vp.y = rawImage.rows * 0.35;
	}

	line(updateIPM_img, Point(0, vp.y), Point(updateIPM_img.cols - 1, vp.y), Scalar(0, 0, 255));
	line(updateIPM_img, vp, Point(updateIPM_img.cols * 0.5f, updateIPM_img.rows -1), Scalar(0, 0, 255));
	circle(updateIPM_img, vp, 5, Scalar(0, 255, 255));


	//kf
	kf.predict();
	Mat measure = Mat::zeros(2, 1, CV_32F);
	//randn(measure, Scalar::all(0), Scalar::all(kf.measurementNoiseCov.at<float>(0)));
	measure += (Mat_<float>(2, 1) << vp.x, vp.y);
	kf.correct(measure);
	float *ptr_measure = kf.statePost.ptr<float>(0);
	vp = Point2d(ptr_measure[0], ptr_measure[1]);


	line(updateIPM_img, Point(0, vp.y), Point(updateIPM_img.cols - 1, vp.y), Scalar(255, 0, 0));
	line(updateIPM_img, vp, Point(updateIPM_img.cols * 0.5f, updateIPM_img.rows - 1), Scalar(0, 0, 255));
	circle(updateIPM_img, vp, 5, Scalar(255, 255, 0));
	imshow("updateIPM_img", updateIPM_img);

	//update ipm
	double fx, fy, cu, cv;
	ipm->getCameraParam(fx, fy, cu, cv);
	double rx = atan2(cv - vp.y, (fx + fy) / 2);
	double _, h;
	ipm->getRxAndH(_, h);
	ipm->createModel(fx, fy, cu, cv, rx, h);

	cout << "rx :    -------------  " << rx << endl;
}


float hx(Point2d vp, Point2d m)
{
	return atan((vp.y - m.y) / (vp.x - m.x));
}

void LaneDetection::updateIPM2(vector<Pair2d> pairs_in_image)
{
	vector<Segment2d> segments_in_image;
	int n_pairs = pairs_in_image.size();


	for (int i = 0; i < n_pairs; i++)
	{
		//if (abs(atan(pairs[i]._p12.getSlope())) < CV_PI / 4)
		//	continue;

		//line(updateIPM_img, pairs_in_image[i].s1.p1, pairs_in_image[i].s1.p2, Scalar(255, 0, 0));
		//line(updateIPM_img, pairs_in_image[i].s2.p1, pairs_in_image[i].s2.p2, Scalar(255, 0, 0));
		segments_in_image.push_back(pairs_in_image[i].s1);
		segments_in_image.push_back(pairs_in_image[i].s2);
	}

	int n_size = segments_in_image.size();

	if (n_size == 0)
		return;


	Mat prex = ekf.predict();
	Point2d prevp(prex.at<float>(0), prex.at<float>(1));
	double leng_max = 700;
	for (int n_iter = 0; n_iter < 2; n_iter++)
	{
		for (int i = 0; i < n_size; i++)
		{
			prevp = Point2d(ekf.statePre.at<float>(0), ekf.statePre.at<float>(1));
			Segment2d *s = &segments_in_image[i];
			Point2d mi = 0.5*(s->p1 + s->p2);
			//cout << atan(s->getSlope()) << ";;;;;" << hx(prevp, mi) << endl;
			float yi = ekf.residual(atan(s->getSlope()), hx(prevp, mi));
			if (abs(yi) > CV_PI / 9)
				continue;

			float d2 = pow(s->getLength(), 2);
			if (leng_max < s->getLength())
			{
				leng_max = s->getLength();
			}
			ekf.measurementMatrix = (Mat_<float>(1, 2) << (mi.y - prevp.y) / d2, (prevp.x - mi.x) / d2);
			ekf.measurementNoiseCov = (20 - s->getLength() * 19.0 / leng_max) * CV_PI / 180;
			//cout << s->getLength() << "  " << ekf.measurementNoiseCov << endl;
			ekf.correct().copyTo(ekf.statePre);
		}
	}
	float *ptr_vp = ekf.statePost.ptr<float>(0);
	//cout << "nb " << nb << endl;
	if (ptr_vp[1] > rawImage.rows * 0.7)
	{
		ptr_vp[1] = rawImage.rows * 0.7;
	}

	if (ptr_vp[1] < rawImage.rows * 0.3)
	{
		ptr_vp[1] = rawImage.rows * 0.3;
	}
	vp = Point2d(ptr_vp[0], ptr_vp[1]);

	//cout << vp << endl;


	//update ipm
	double fx, fy, cu, cv;
	ipm->getCameraParam(fx, fy, cu, cv);
	double rx = atan2(cv - vp.y, (fx + fy) / 2);
	double _, h;
	ipm->getRxAndH(_, h);
	ipm->createModel(fx, fy, cu, cv, rx, h);

	//cout << "rx :    -------------  " << rx << endl;
}


void LaneDetection::detectionLineLSD(Mat &resultImage) {
	unsigned int X = rawGrayImage.cols;  /* x image size */
	unsigned int Y = rawGrayImage.rows;  /* y image size */
	unsigned int XY = X * Y; /* numer of pixels */

	image_double image = new_image_double(X, Y);
	/*copy the data*/
	const uchar* ptr_gray_image = rawGrayImage.ptr<uchar>(0);
	for (int _pixel = 0; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}

	/* call LSD */
	ntuple_list out = lsd(image);

	/* print output */
	//LOG_INFO("line segments found: " << out->size);

	Mat colorImage;
	rawColorImage.copyTo(colorImage);

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

		line(colorImage, start, end, Scalar(b, g, r), 1, lineType);
	}

	/* free memory */
	free_image_double(image);
	free_ntuple_list(out);

	colorImage.copyTo(resultImage);
}

void LaneDetection::method1()
{

}
void LaneDetection::method2(const Mat &img)
{
	if (ipm == NULL)
	{
		cout << "ipm is null .. " << endl;
		return;
	}

	img.copyTo(rawImage);
	if (rawImage.channels() == 3)
	{
		cvtColor(rawImage, rawGrayImage, CV_BGR2GRAY);
		rawColorImage = rawImage;
	}
	else if (rawImage.channels() == 1) 
	{
		rawImage.copyTo(rawGrayImage);
		cvtColor(rawGrayImage, rawColorImage, CV_GRAY2BGR);
	}

	if (!rawGrayImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}

	int64 t0, t1;
	t0 = getTickCount();

	//step1 : call LSD 
	resultLSD();
	t1 = getTickCount();
	cout << "step1 lsd : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	//step2: choose segments in this zone
	int x_min = -30, x_max = 30;
	int z_min = 0, z_max = 80;

	vector<Segment2d> segments;
	vector<Segment2d> segments_in_image;
	int dim = lsd_result->dim;

	for (int i = 0; i < lsd_result->size; i++)
	{
		double x, z;
		double u = lsd_result->values[i * lsd_result->dim + 0];
		double v = lsd_result->values[i * lsd_result->dim + 1];
		Point2d p1_in_image(u, v);
		ipm->convert(u, v, x, z);
		if (x < x_min || x > x_max)
			continue;
		if (z < z_min || z > z_max)
			continue;

		Point2d p1(x, z);

		u = lsd_result->values[i * lsd_result->dim + 2];
		v = lsd_result->values[i * lsd_result->dim + 3];
		Point2d p2_in_image(u, v);
		ipm->convert(u, v, x, z);
		if (x < x_min || x > x_max)
			continue;
		if (z < z_min || z > z_max)
			continue;
		Point2d p2(x, z);

		segments.push_back(Segment2d(p1, p2));
		segments_in_image.push_back(Segment2d(p1_in_image, p2_in_image));
	}
	
	t1 = getTickCount();
	cout << "step2 : choose segments in this zone " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	//step 3 : find pairs
	vector<Pair2d> pairs;
	vector<Pair2d> pairs_in_image;
	int n_segments = segments.size();
	for (int i = 0; i < n_segments; i++)
	{
		Segment2d *p12 = &segments[i];//p1.y < p2.y
		if (p12->p1.y < 0)
			continue;

		Segment2d *p12_img = &segments_in_image[i];
		for (int j = 0; j < n_segments; j++)
		{
			if (j == i) continue;
			Segment2d *seg = &segments[j];

			//several conditions
			if (!p12->maybePair(seg))
				continue;

			Segment2d *seg_img = &segments_in_image[j];

			vector<Segment2d> entire_v = p12->getValidRect(*seg);
			//valid?
			if (entire_v.empty())
			{
				//fout_2 << i << ", " << j << "continue in v.empty()" << endl;
				continue;
			}
			double d1 = dist_p2p(entire_v[0].p1, entire_v[1].p1);
			if (d1 < 0.1)
			{
				continue;
			}

			Pair2d pair2d(*p12, *seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			double mean_width = d1;
			pair2d.setMeanWidth(mean_width);

			pairs.push_back(pair2d);
			pairs_in_image.push_back(Pair2d(*p12_img, *seg_img));
		}
	}

	t1 = getTickCount();
	cout << "step 3 : find pairs " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	int n_pairs = pairs.size();
	for (int i = 0; i < n_pairs; i++)
	{
		Pair2d *pair_i = &pairs[i];
		pair_i->computeLineModel();
	}

	//step4 : updateIPM (kf, meanshift, ipm)
	updateIPM(pairs, pairs_in_image);
	t1 = getTickCount();
	cout << "step4 updateIPM : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	//step 5 : compute weights between each two and regroupe
	vector< vector <int> > vec_ints;
	vec_ints.resize(n_pairs);
	Mat weights = Mat::zeros(n_pairs, n_pairs, CV_64FC1);
	for (int i = 0; i < n_pairs; i++)
	{
		vec_ints[i].push_back(i);
		Pair2d *pair_i = &pairs[i];
		double *ptr_row_weights = weights.ptr<double>(i);
		for (int j = i + 1; j < n_pairs; j++)
		{
			double weight = pair_i->weight(&pairs[j]);
			ptr_row_weights[j] = weight;
			//weights.at<double>(j, i) = weight;
			if (weight > 0.9)
			{
				vec_ints[i].push_back(j);
				vec_ints[j].push_back(i);
			}
		}
	}
	bool *drawn = new bool[n_pairs];
	memset(drawn, false, n_pairs);


	int scale = 7;

	int image_cols = (x_max - x_min) * scale;
	int image_rows = (z_max - z_min) * scale;
	Mat cameraScene = Mat::zeros(image_rows, image_cols, CV_8UC3);
	Point2d round(0.5, 0.5);

	Mat ipm_image;
	rawColorImage.copyTo(ipm_image);
	for (int i = 0; i < n_pairs; i++)
	{
		double length = 0;
		vector<int> vec_i;
		vector<int> stk;
		stk.push_back(i);

		while (!stk.empty())
		{
			int p = stk[stk.size() - 1];
			stk.pop_back();
			if (!drawn[p])
			{
				vec_i.push_back(p);
				drawn[p] = true;
				length += dist_p2p(pairs[p]._p1, pairs[p]._p2);
			}

			for (int j = 0; j < vec_ints[p].size(); j++)
			{
				if (!drawn[vec_ints[p][j]])
					stk.push_back(vec_ints[p][j]);
			}
		}

		if (length < 2)
			continue;

		//just draw pictures
		for (int k = 0; k < vec_i.size(); k++)
		{
			Pair2d *pair_k = &pairs[vec_i[k]];

			Point2d p1(pair_k->s1.p1.x - x_min, z_max - pair_k->s1.p1.y);
			Point2d p2(pair_k->s1.p2.x - x_min, z_max - pair_k->s1.p2.y);
			line(cameraScene, scale * p1 + round, scale * p2 + round, Scalar(255, 255, i));

			p1 = Point2d(pair_k->s2.p1.x - x_min, z_max - pair_k->s2.p1.y);
			p2 = Point2d(pair_k->s2.p2.x - x_min, z_max - pair_k->s2.p2.y);
			line(cameraScene, scale * p1 + round, scale * p2 + round, Scalar(255, 0, i));

		}
		for (int k = 0; k < vec_i.size(); k++)
		{

			Pair2d *pair_k = &pairs_in_image[vec_i[k]];
			line(ipm_image, pair_k->s1.p1, pair_k->s1.p2, Scalar(255, 0, i));
			line(ipm_image, pair_k->s2.p1, pair_k->s2.p2, Scalar(255, 255, i));
		}


		//Mat step;
		//rawColorImage.copyTo(step);
		//int b = (unsigned)theRNG() & 255;
		//int g = (unsigned)theRNG() & 255;
		//int r = i & 255;
		//for (int k = 0; k < vec_i.size(); k++)
		//{
		//	Pair2d *pair_k = &pairs_in_image[vec_i[k]];
		//	line(step, pair_k->s1.p1, pair_k->s1.p2, Scalar(255, 0, r));
		//	line(step, pair_k->s2.p1, pair_k->s2.p2, Scalar(255, 255, r));
		//	//line(step, pair_k->_p12.p1, pair_k->_p12.p2, Scalar(255, 0, r));

		//	//cout << vec_i[k] << "," << pair_k->s1.p1 << " " << pair_k->s1.p2 << "," << pair_k->s2.p1 << " " << pair_k->s2.p2 << "; ";
		//	//cout << pair_k->_p12.p1 <<  pair_k->_p12.p2 << endl;

		//}
		////cout << endl;
		//if (vec_i.size() != 0)
		//{
		//	char fileName[20];
		//	sprintf_s(fileName, "step%d.png", i);

		//	imwrite(fileName, step);
		//}

	}
	imshow("ipm_image", cameraScene);
	imshow("cameraScene", ipm_image );
	//imwrite("cameraScene.png", cameraScene);

	t1 = getTickCount();
	cout << "final : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	free_ntuple_list(lsd_result);
	delete drawn;
}


void LaneDetection::findPairs(vector<Pair2d> &pairs, vector<Pair2d> &pairs_in_image, const Mat &maskRoad)
{
	//select segments in this zone
	//int x_min = -20, x_max = 15;
	//int z_min = 6, z_max = 80;

	vector<Segment2d> segments_in_image;
	vector<Segment2d> segments;

	int num_sample = 10;
	for (int i = 0; i < lsd_result->size; i++)
	{
		double x, z;
		double u = lsd_result->values[i * lsd_result->dim + 0];
		double v = lsd_result->values[i * lsd_result->dim + 1];
		Point2d p1_in_image(u, v);
		ipm->convert(u, v, x, z);
		bool b1 = x > x_min && x < x_max && z > z_min && z < z_max;
		Point2d p1(x, z);

		double u2 = lsd_result->values[i * lsd_result->dim + 2];
		double v2 = lsd_result->values[i * lsd_result->dim + 3];
		Point2d p2_in_image(u2, v2);
		ipm->convert(u2, v2, x, z);
		bool b2 = x > x_min && x < x_max && z > z_min && z < z_max;
		Point2d p2(x, z);

		bool b_mask = true;


		if ( (b1 || b2) && b_mask)
		{
			segments.push_back(Segment2d(p1, p2));
			segments_in_image.push_back(Segment2d(p1_in_image, p2_in_image));
		}

	}

#ifdef DEBUG_FOUT
	Mat colorImage;
	rawColorImage.copyTo(colorImage);
	/*draw the lines*/
	for (int i = 0; i < segments_in_image.size(); i++)
	{
		Segment2d *p12 = &segments_in_image[i];//p1.y < p2.y
		int g = i & 255;
		int r = i / 255;

		line(colorImage, p12->p1, p12->p2, Scalar(255, g, r), 1);
	}
	imwrite("inViewSegments.png", colorImage);
#endif

	vector<double> slopes;

	//step 3 : find pairs
	pairs.clear();
	pairs_in_image.clear();
	int n_segments = segments.size();
	int width = 1;
	for (int i = 0; i < n_segments; i++)
	{
		Segment2d *p12 = &segments[i];//p1.y < p2.y
		if (p12->p1.y < 0)
			continue;

		Segment2d *p12_img = &segments_in_image[i];
		for (int j = 0; j < n_segments; j++)
		{
			if (j == i) continue;
			Segment2d *seg = &segments[j];
			Segment2d *seg_img = &segments_in_image[j];

			//several conditions
			if (!p12->maybePair(seg))
			{
#ifdef DEBUG_FOUT
				fout_2 << "!p12->maybePair(seg)  " << endl;
				fout_2 << i << "," << j << "[" << p12_img->p1 << "," << p12_img->p2 << "] and [" << seg_img->p1 << "," << seg_img->p2 << endl;
#endif
				continue;
			}
				

			

			vector<Segment2d> entire_v = p12->getValidRect(*seg);
			//valid?
			if (entire_v.empty())
			{
				//fout_2 << i << ", " << j << "continue in v.empty()" << endl;
				continue;
			}
			double d1 = dist_p2p(entire_v[0].p1, entire_v[1].p1);
			if (d1 < 0.1)
			{
#ifdef DEBUG_FOUT
				fout_2 << "d1 < 0.1  " << endl;
				fout_2 << i << "," << j << "[" << p12_img->p1 << "," << p12_img->p2 << "] and [" << seg_img->p1 << "," << seg_img->p2 << endl;
#endif
				continue;
			}

			//color compare
			Point2d translation[3];
			translation[0] = Point2d(0, 0);
			translation[1] = entire_v[0].p1 - entire_v[1].p1;
			translation[2] = -translation[1];
			if (translation[1].x > 0)
			{
				translation[2] = entire_v[0].p1 - entire_v[1].p1;
				translation[1] = -translation[2];
			}

			//translation[1] -= Point2d(0.1, 0);
			//translation[2] += Point2d(0.1, 0);

			vector<Vec3b> vec_color[3];
			Vec3d mean_color[3];


			Point2d src_p[4];
			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 2; j++)
				{
					j == 0 ? src_p[i * 2 + j] = entire_v[i].p1 : src_p[i * 2 + j] = entire_v[i].p2;
				}
			}


			for (int _trans = 0; _trans < 3; _trans++)
			{
				Point2d trans = translation[_trans];
				Point2d dst_p[4];
				for (int n = 0; n < 4; n++)
				{
					ipm->convert_inv(src_p[n].x + trans.x, src_p[n].y + trans.y, dst_p[n].x, dst_p[n].y);
				}

				Point2d center = (src_p[0] + src_p[1] + src_p[2] + src_p[3]) / 4;
				Point2d dst_center;
				ipm->convert_inv(center.x + trans.x, center.y + trans.y, dst_center.x, dst_center.y);

				int cen_x = dst_center.x, cen_y = dst_center.y;
				for (int cen_x_i = -1; cen_x_i < 2; cen_x_i++)
				{
					for (int cen_y_i = -1; cen_y_i < 2; cen_y_i++)
					{
						if (cen_y + cen_y_i < 0 || cen_y + cen_y_i > rawColorImage.rows - 1)
							continue;
						if (cen_x + cen_x_i < 0 || cen_x + cen_x_i > rawColorImage.cols - 1)
							continue;
						vec_color[_trans].push_back(rawColorImage.at<Vec3b>(cen_y + cen_y_i, cen_x + cen_x_i));
					}
				}
				

				//Rect box = getBoundingBox_d(dst_p[0], dst_p[1],
				//	dst_p[2], dst_p[3]);

				//for (int _y = box.y; _y < box.y + box.height; _y++)
				//{
				//	if (_y >= rawGrayImage.rows || _y < 0) continue;
				//	const Vec3b* ptr_row_processImage = rawColorImage.ptr<Vec3b>(_y);
				//	for (int _x = box.x; _x < box.x + box.width; _x++)
				//	{
				//		if (_x >= rawGrayImage.cols || _x < 0) continue;
				//		Point2d p(_x, _y);

				//		int direc = (p - dst_p[0]).cross(p - dst_p[1]) > -0.0 ? 1 : -1;

				//		if (direc != ((p - dst_p[1]).cross(p - dst_p[3]) > -0.0 ? 1 : -1))
				//			continue;
				//		if (direc != ((p - dst_p[3]).cross(p - dst_p[2]) > -0.0 ? 1 : -1))
				//			continue;
				//		if (direc != ((p - dst_p[2]).cross(p - dst_p[0]) > -0.0 ? 1 : -1))
				//			continue;

				//		vec_color[_trans].push_back(ptr_row_processImage[_x]);
				//	}
				//}


				if (vec_color[_trans].empty())
				{
					//cout << "empty" << ", ";
				}
				else
				{
					Mat mean, stdDev;
					meanStdDev(vec_color[_trans], mean, stdDev);
					mean_color[_trans] = Vec3d(mean.at<double>(0), mean.at<double>(1), mean.at<double>(2));

					//line(pairImage, dst_p[0], dst_p[1], Scalar(255, 0, 0));
					//line(pairImage, dst_p[1], dst_p[3], Scalar(255, 255, 0));
					//line(pairImage, dst_p[2], dst_p[3], Scalar(255, 0, 0));
					//line(pairImage, dst_p[2], dst_p[0], Scalar(255, 255, 0));

					//cout << mean_color[_trans] << ",";
				}
				
			}
			//cout << endl;
			double g[3];
			for (int g_n = 0; g_n < 3; g_n++)
			{
				g[g_n] = 0.114*mean_color[g_n][0] + 0.587*mean_color[g_n][1] + 0.299*mean_color[g_n][2];
			}
			bool color_matched = true;
			for (int g_n = 0; g_n < 3; g_n++)
			{
				color_matched = color_matched &&
					mean_color[0][g_n] > mean_color[1][g_n] + 10
					&& mean_color[0][g_n] > mean_color[2][g_n] + 10;
			}

			color_matched = color_matched && abs(mean_color[0][0] - mean_color[0][1]) < 50
				&& abs(mean_color[0][0] - mean_color[0][2]) < 50;

			//bool color_matched = (g[0] > g[1] + 10) && (g[0] > g[2] + 10);
			if (!color_matched)
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << mean_color[0] << "," << mean_color[1] << "," << mean_color[2];
				//fout_2 << "," << stdDev_color[0] << endl;
				//fout_2 << "continue in color_matched" << endl;
#ifdef DEBUG_FOUT
				fout_2 << "!color_matched  " << endl;
				fout_2 << i << "," << j << "[" << p12_img->p1 << "," << p12_img->p2 << "] and [" << seg_img->p1 << "," << seg_img->p2 << endl;
#endif
				continue;
			}

			//some lanes are like (255, 255, 200)
			//double dif_bg = mean_color[0][0] - mean_color[0][1];
			//double dif_rg = mean_color[0][2] - mean_color[0][1];
			//color_matched = (dif_bg * dif_bg + dif_rg * dif_rg < 400);
			//if (!color_matched)
			//{
			//	continue;
			//}

			Pair2d pair2d(*p12, *seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			double mean_width = d1;
			pair2d.setMeanWidth(mean_width);


			
			pair2d.computeLineModel();


			p12_img;
			seg_img;
			bool b_mask = false;
			if (maskRoad.data)
			{
				imshow("mask", maskRoad);
				double u = p12_img->p1.x;
				double v = p12_img->p1.y;
				double u2 = p12_img->p2.x;
				double v2 = p12_img->p2.y;
				for (int i_sample = 0; i_sample < num_sample; i_sample++)
				{
					int u_s = i_sample * (u - u2) / num_sample + u2;
					int v_s = i_sample * (v - v2) / num_sample + v2;
					if (u_s <= 0) u_s = 1;
					if (u_s >= rawColorImage.cols - 1) u_s = rawColorImage.cols - 2;
					if (v_s <= 0) v_s = 1;
					if (v_s >= rawColorImage.rows - 1) v_s = rawColorImage.rows - 2;

					const uchar* ptr_maskRoad = maskRoad.ptr<uchar>(v_s - 1);
					const uchar* ptr_maskRoad_2 = maskRoad.ptr<uchar>(v_s);
					const uchar* ptr_maskRoad_3 = maskRoad.ptr<uchar>(v_s + 1);

					for (int c_sample = -1; c_sample < 2; c_sample++)
					{
						if (ptr_maskRoad[c_sample + u_s] > 10
							|| ptr_maskRoad_2[c_sample + u_s] > 10
							|| ptr_maskRoad_3[c_sample + u_s] > 10)
						{
							b_mask = true;
							break;
						}
					}

					if (b_mask)
						break;
				}

				u = seg_img->p1.x;
				v = seg_img->p1.y;
				u2 = seg_img->p2.x;
				v2 = seg_img->p2.y;
				for (int i_sample = 0; i_sample < num_sample; i_sample++)
				{
					int u_s = i_sample * (u - u2) / num_sample + u2;
					int v_s = i_sample * (v - v2) / num_sample + v2;
					if (u_s <= 0) u_s = 1;
					if (u_s >= rawColorImage.cols - 1) u_s = rawColorImage.cols - 2;
					if (v_s <= 0) v_s = 1;
					if (v_s >= rawColorImage.rows - 1) v_s = rawColorImage.rows - 2;

					const uchar* ptr_maskRoad = maskRoad.ptr<uchar>(v_s - 1);
					const uchar* ptr_maskRoad_2 = maskRoad.ptr<uchar>(v_s);
					const uchar* ptr_maskRoad_3 = maskRoad.ptr<uchar>(v_s + 1);

					for (int c_sample = -1; c_sample < 2; c_sample++)
					{
						if (ptr_maskRoad[c_sample + u_s] > 10
							|| ptr_maskRoad_2[c_sample + u_s] > 10
							|| ptr_maskRoad_3[c_sample + u_s] > 10)
						{
							b_mask = true;
							break;
						}
					}

					if (b_mask)
						break;
				}

			}
			else
				b_mask = true;

			if (!b_mask)
			{
				continue;
			}

			//double angle = atan(pair2d._p12.getSlope());
			//if (angle < 0) angle += CV_PI;
			//slopes.push_back(angle);

			pairs.push_back(pair2d);
			pairs_in_image.push_back(Pair2d(*p12_img, *seg_img));
			
		}
	}


	////mean shift to find slope center
	//vector<Pair2d> _pairs_copy = pairs;
	//vector<Pair2d> _pairs_in_image_copy = pairs_in_image;
	//pairs.clear();
	//pairs_in_image.clear();
	//double _center = CV_PI / 2;
	//int time = 0;
	//int max_iter = 20;
	//while (time < max_iter)
	//{
	//	time++;
	//	double v = 0;
	//	double b = 0;
	//	for (int k = 0; k < slopes.size(); k++)
	//	{
	//		double x = _center - slopes[k];
	//		//cout << x << ", ";
	//		double ker = kernel(x);
	//		v += ker * slopes[k];
	//		b += ker;
	//	}
	//	//cout << v << ", " << b << endl;
	//	v = v / b;
	//	_center = v;
	//	if ((v - _center)*(v - _center) < EPS_HERE)
	//	{
	//		break;
	//	}
	//}

	//for (int i = 0; i < _pairs_copy.size(); i++)
	//{
	//	if (abs(slopes[i] - _center) > 0.2)
	//		continue;

	//	pairs.push_back(_pairs_copy[i]);
	//	pairs_in_image.push_back(_pairs_in_image_copy[i]);
	//}

#ifdef DEBUG_FOUT
	imwrite("pairImage.png", pairImage);
#endif
}


vector<Pair2d> LaneDetection::method3(const Mat &img, int winFlag, const Mat &maskRoad)
{
#ifdef DEBUG_FOUT
	fout_2.open("debug_fout.txt");
#endif
	vector<Pair2d> pairs;
	vector<Pair2d> pairs_in_image;
	if (ipm == NULL)
	{
		cout << "ipm is null .. " << endl;
		return pairs;
	}
	
	img.copyTo(rawImage);
	if (rawImage.channels() == 3)
	{
		cvtColor(rawImage, rawGrayImage, CV_BGR2GRAY);
		rawColorImage = rawImage;
	}
	else if (rawImage.channels() == 1)
	{
		rawImage.copyTo(rawGrayImage);
		cvtColor(rawGrayImage, rawColorImage, CV_GRAY2BGR);
	}

	if (!rawGrayImage.data)
	{
		cout << "processImage is empty. " << endl;
		return pairs;
	}

	resultLSD();

	//Mat maskRoad;
	//segmentationRoad(maskRoad);

	//imshow("maskRoad", maskRoad);


	findPairs(pairs, pairs_in_image, maskRoad);
	//double rx, h;
	//ipm->getRxAndH(rx, h);
	//cout << "1 "<< rx << "," << h << endl;
	updateIPM2(pairs_in_image);
	//ipm->getRxAndH(rx, h);
	//cout << "2 "<<  rx << "," << h << endl;
	findPairs(pairs, pairs_in_image, maskRoad);

#ifdef DEBUG_drawImage
	Mat updateIPM_img;
	rawColorImage.copyTo(updateIPM_img);
	line(updateIPM_img, Point(0, vp.y), Point(updateIPM_img.cols - 1, vp.y), Scalar(255, 0, 0));
	line(updateIPM_img, vp, Point(updateIPM_img.cols * 0.5f, updateIPM_img.rows - 1), Scalar(0, 0, 255));
	circle(updateIPM_img, vp, 5, Scalar(255, 255, 0));
	char winName[64];
	sprintf_s(winName, "updateIPM_img %d", winFlag);
	imshow(winName, updateIPM_img);

	Mat pairImage;
	rawColorImage.copyTo(pairImage);
	for (int i = 0; i < pairs_in_image.size(); i++)
	{
		line(pairImage, pairs_in_image[i].s1.p1, pairs_in_image[i].s1.p2, Scalar(0, 255, 0));
		line(pairImage, pairs_in_image[i].s2.p1, pairs_in_image[i].s2.p2, Scalar(0, 255, 0));
	}
	sprintf_s(winName, "pairImage %d", winFlag);
	imshow(winName, pairImage);

	int scale = 10;
	Mat ipm_image = Mat::zeros(scale*(z_max-z_min), scale*(x_max - x_min), CV_8UC3);

	for (int i = 0; i < pairs.size(); i++)
	{
		for (int s = 0; s < 2; s++)
		{
			const Segment2d *seg_img = (s == 0 ? &pairs[i].s1 : &pairs[i].s2);
			double x = seg_img->p1.x, z = seg_img->p1.y;
			Point2d p1(x - x_min, z_max - z);
			x = seg_img->p2.x, z = seg_img->p2.y;
			Point2d p2(x - x_min, z_max - z);
			line(ipm_image, scale * p1, scale * p2, Scalar(255, 0, 0));
		}

		Segment2d *seg_img = &pairs[i]._p12;
		double x = seg_img->p1.x, z = seg_img->p1.y;
		Point2d p1(x - x_min, z_max - z);
		x = seg_img->p2.x, z = seg_img->p2.y;
		Point2d p2(x - x_min, z_max - z);
		line(ipm_image, scale * p1, scale * p2, Scalar(255, 255, 0));
	}


	sprintf_s(winName, "ipm_image %d", winFlag);
	imshow(winName, ipm_image);
#endif


#ifdef DEBUG_FOUT
	fout_2.close();
#endif
	return pairs;
}


vector<Pair2d> LaneDetection::method4(const Mat &rL, const Mat &disp, int winFlag)
{
#ifdef DEBUG_FOUT
	fout_2.open("debug_fout.txt");
#endif
	vector<Pair2d> pairs;
	vector<Pair2d> pairs_in_image;
	if (ipm == NULL)
	{
		cout << "ipm is null .. " << endl;
		return pairs;
	}

	rL.copyTo(rawImage);
	if (rawImage.channels() == 3)
	{
		cvtColor(rawImage, rawGrayImage, CV_BGR2GRAY);
		rawColorImage = rawImage;
	}
	else if (rawImage.channels() == 1)
	{
		rawImage.copyTo(rawGrayImage);
		cvtColor(rawGrayImage, rawColorImage, CV_GRAY2BGR);
	}

	if (!rawGrayImage.data)
	{
		cout << "processImage is empty. " << endl;
		return pairs;
	}

	resultLSD();

	//Mat maskRoad;
	//segmentationRoad(maskRoad);

	//imshow("maskRoad", maskRoad);


	findPairs(pairs, pairs_in_image);
	//double rx, h;
	//ipm->getRxAndH(rx, h);
	//cout << "1 "<< rx << "," << h << endl;
	updateIPM2(pairs_in_image);
	//ipm->getRxAndH(rx, h);
	//cout << "2 "<<  rx << "," << h << endl;
	findPairs(pairs, pairs_in_image);

#ifdef DEBUG_drawImage
	Mat updateIPM_img;
	rawColorImage.copyTo(updateIPM_img);
	line(updateIPM_img, Point(0, vp.y), Point(updateIPM_img.cols - 1, vp.y), Scalar(255, 0, 0));
	line(updateIPM_img, vp, Point(updateIPM_img.cols * 0.5f, updateIPM_img.rows - 1), Scalar(0, 0, 255));
	circle(updateIPM_img, vp, 5, Scalar(255, 255, 0));
	char winName[64];
	sprintf_s(winName, "updateIPM_img %d", winFlag);
	imshow(winName, updateIPM_img);

	Mat pairImage;
	rawColorImage.copyTo(pairImage);
	for (int i = 0; i < pairs_in_image.size(); i++)
	{
		line(pairImage, pairs_in_image[i].s1.p1, pairs_in_image[i].s1.p2, Scalar(0, 255, 0));
		line(pairImage, pairs_in_image[i].s2.p1, pairs_in_image[i].s2.p2, Scalar(0, 255, 0));
	}
	sprintf_s(winName, "pairImage %d", winFlag);
	imshow(winName, pairImage);

	int scale = 10;
	Mat ipm_image = Mat::zeros(scale*(z_max - z_min), scale*(x_max - x_min), CV_8UC3);

	for (int i = 0; i < pairs.size(); i++)
	{
		for (int s = 0; s < 2; s++)
		{
			const Segment2d *seg_img = (s == 0 ? &pairs[i].s1 : &pairs[i].s2);
			double x = seg_img->p1.x, z = seg_img->p1.y;
			Point2d p1(x - x_min, z_max - z);
			x = seg_img->p2.x, z = seg_img->p2.y;
			Point2d p2(x - x_min, z_max - z);
			line(ipm_image, scale * p1, scale * p2, Scalar(255, 0, 0));
		}

		Segment2d *seg_img = &pairs[i]._p12;
		double x = seg_img->p1.x, z = seg_img->p1.y;
		Point2d p1(x - x_min, z_max - z);
		x = seg_img->p2.x, z = seg_img->p2.y;
		Point2d p2(x - x_min, z_max - z);
		line(ipm_image, scale * p1, scale * p2, Scalar(255, 255, 0));
	}


	sprintf_s(winName, "ipm_image %d", winFlag);
	imshow(winName, ipm_image);
#endif


#ifdef DEBUG_FOUT
	fout_2.close();
#endif
	return pairs;
}

void LaneDetection::segmentationRoad(Mat &maskRoad)
{
	rawColorImage.copyTo(maskRoad);

	double u, v;
	ipm->convert_inv(x_min, z_min, u, v);
	Point2d p1(u, v);
	ipm->convert_inv(x_max, z_min, u, v);
	Point2d p2(u, v);
	ipm->convert_inv(x_min, z_max, u, v);
	Point2d p3(u, v);
	ipm->convert_inv(x_max, z_max, u, v);
	Point2d p4(u, v);

	//line(maskRoad, p1, p2, Scalar(255, 0, 0), 2);
	line(maskRoad, p1, p3, Scalar(255, 0, 0), 2);
	line(maskRoad, p2, p4, Scalar(255, 0, 0), 2);
	line(maskRoad, p3, p4, Scalar(255, 0, 0), 2);

	Vec3b *ptr_maskRoad = maskRoad.ptr<Vec3b>(0);
	for (int p = 0; p < maskRoad.rows * maskRoad.cols; p++)
	{
		int b = ptr_maskRoad[p][0];
		int g = ptr_maskRoad[p][1];
		int r = ptr_maskRoad[p][2];

		ptr_maskRoad[p] = Vec3b(abs(b - g), abs(g - r), 0);
	}

	floodFill(maskRoad, Point(rawColorImage.cols*0.49, rawColorImage.rows * 0.9),
		Scalar(0, 0, 255), 0, Scalar(1, 1, 1), Scalar(1, 1, 1), 4);
	floodFill(maskRoad, Point(rawColorImage.cols*0.5, rawColorImage.rows * 0.9),
		Scalar(0, 0, 255), 0, Scalar(1, 1, 1), Scalar(1, 1, 1), 4);
	floodFill(maskRoad, Point(rawColorImage.cols*0.51, rawColorImage.rows * 0.9),
		Scalar(0, 0, 255), 0, Scalar(1, 1, 1), Scalar(1, 1, 1), 4);

	vector<Vec3b> seedsRoad;
	seedsRoad.push_back(rawColorImage.at<Vec3b>(rawColorImage.rows * 0.9, rawColorImage.cols*0.5));
	seedsRoad.push_back(rawColorImage.at<Vec3b>(rawColorImage.rows * 0.9, rawColorImage.cols*0.51));
	seedsRoad.push_back(rawColorImage.at<Vec3b>(rawColorImage.rows * 0.9, rawColorImage.cols*0.52));
	seedsRoad.push_back(rawColorImage.at<Vec3b>(rawColorImage.rows * 0.9, rawColorImage.cols*0.49));
	seedsRoad.push_back(rawColorImage.at<Vec3b>(rawColorImage.rows * 0.9, rawColorImage.cols*0.48));

	roadColor = 1.0 / seedsRoad.size() * seedsRoad[0];
	for (int seeds_i = 1; seeds_i < seedsRoad.size(); seeds_i++)
	{
		roadColor += 1.0 / seedsRoad.size() * seedsRoad[seeds_i];
	}
	
}
