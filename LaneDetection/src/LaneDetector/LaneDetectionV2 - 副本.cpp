#include "LaneDetectionV2.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
using namespace std;
#define EPS_HERE 1.0e-2
#define INFINI_HERE 1.0e6
ofstream fout_2;

inline double kernel(Point2d x) {
	return exp2(-x.ddot(x) / 8100);
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

bool Segment2d::isNeighbor(Segment2d s) {
	bool b2 = s.indexZone[2] > indexZone[2] - 2 && s.indexZone[2] < indexZone[2] + 2;
	if (b2)
		return true;

	b2 = s.indexZone[3] > indexZone[3] - 2 && s.indexZone[3] < indexZone[3] + 2;
	if (b2)
		return true;

	return false;
}

bool Segment2d::isNeighbor(Point2d s) {
	bool b1 = s.y / _scale > indexZone[0] - 5 && s.y / _scale < indexZone[1] + 5;
	bool b2 = s.x / _scale > indexZone[2] - 5 && s.x / _scale < indexZone[3] + 5;
	if (b1 && b2)
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
	double thresh = 1.0;
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
	thresh = 1.0;
	if (dist_p2p(new_p1, valid_foot_12.p1) > thresh && dist_p2p(new_p2, valid_foot_12.p2) > thresh)
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

	setIdentity(kf.measurementMatrix);
	setIdentity(kf.processNoiseCov, Scalar::all(10));
	setIdentity(kf.measurementNoiseCov, Scalar::all(20));
	setIdentity(kf.errorCovPost, Scalar::all(10));
	
	if (img.data)
	{
		vp = Point2d(img.cols / 2, img.rows / 2);
	}
	else
	{
		vp = Point2d(700, 150);
	}
		
	kf.statePost = (Mat_<float>(2, 1) << vp.x, vp.y);
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

	lsd_result = lsd(image);

	/* free memory */
	free_image_double(image);

	return lsd_result;
}

void LaneDetection::updateIPM(vector<Pair2d> pairs, vector<Pair2d> pairs_in_image)
{
	vector<Segment2d> segments_in_image;

	int x_min = -20, x_max = 20;
	int z_min = 0, z_max = 30;
	double length_min = 9;

	int n_pairs = pairs.size();
	Mat updateIPM_img;
	rawColorImage.copyTo(updateIPM_img);
	for (int i = 0; i < n_pairs; i++)
	{
		double x, z;
		double u = pairs_in_image[i].s1;
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
		p2 -= p1;

		//if (p2.ddot(p2) < length_min)
		//	continue;
		//cout << atan(p2.y / p2.x) << endl;
		if (abs(p2.x) > EPS_HERE)
		{
			if (abs(atan(p2.y / p2.x)) < CV_PI / 4)
				continue;
		}

		segments_in_image.push_back(Segment2d(p1_in_image, p2_in_image));
		line(updateIPM_img, p1_in_image, p2_in_image, Scalar(0, 255, 0));
	}
	//imshow("updateIPM_img", updateIPM_img);


	int n_size = segments_in_image.size();
	int nb = 0;
	vector<Point2d> intersecs;
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

	//mean shift
	Point2d _vp = vp;
	int time = 0;
	while (time < 10)
	{
		time++;
		Point2d v(0, 0);
		double b = 0;
		for (int k = 0; k < nb; k++)
		{
			Point2d x = _vp - intersecs[k];
			double ker = kernel(x);
			v += ker * intersecs[k];
			b += ker;
		}
		v = v / b;
		if ((v - _vp).ddot(v - _vp) < EPS_HERE)
		{
			_vp = v;
			break;
		}
		_vp = v;
	}
	cout << time << endl;
	//if (time == 10)
	//	return;
	vp = _vp;


	line(updateIPM_img, Point(0, vp.y), Point(updateIPM_img.cols - 1, vp.y), Scalar(0, 0, 255));
	line(updateIPM_img, vp, Point(updateIPM_img.cols * 0.5f, updateIPM_img.rows -1), Scalar(0, 0, 255));
	circle(updateIPM_img, vp, 5, Scalar(0, 255, 255));

	//kf
	kf.predict();
	Mat measure = (Mat_<float>(2, 1) << vp.x, vp.y);
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
	//ipm->createModel(fx, fy, cu, cv, rx, h);

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

void LaneDetection::method1() {
	if (ipm == NULL)
	{
		cout << "ipm is null .. " << endl;
		return;
	}
	if (!rawGrayImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	
	//for lsd test.
	//Mat temp_image;
	//detectionLineLSD(temp_image);
	//char windowName[64];
	//sprintf(windowName, "%d", nameIndex);
	//imwrite(string(windowName) + "step0.png", temp_image);
	//end for lsd test


	int64 t0, t1;
	t0 = getTickCount();
	//fout_2.open("pairs.txt");
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
	ntuple_list lsd_result = lsd(image);

	free_image_double(image);

	t1 = getTickCount();
	cout << "-----------in lsd : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	int dim = lsd_result->dim;
	int n_size = lsd_result->size;

	vector<Segment2d> segments;
	segments.resize(n_size);

	vector<Segment2d> segments_in_image;
	segments_in_image.resize(n_size);

	for (int i = 0; i < n_size; i++)
	{
		double x, z;
		double u = lsd_result->values[i * lsd_result->dim + 0];
		double v = lsd_result->values[i * lsd_result->dim + 1];
		Point2d p1_in_image(u, v);
		ipm->convert(u, v, x, z);
		Point2d p1(x, z);

		u = lsd_result->values[i * lsd_result->dim + 2];
		v = lsd_result->values[i * lsd_result->dim + 3];
		Point2d p2_in_image(u, v);
		ipm->convert(u, v, x, z);
		Point2d p2(x, z);

		segments[i] = Segment2d(p1, p2);
		segments_in_image[i] = Segment2d(p1_in_image, p2_in_image);
	}

	free_ntuple_list(lsd_result);

	vector<Pair2d> pairs;
	vector<Pair2d> pairs_in_image;

	//Mat step1;
	//rawColorImage.copyTo(step1);
	for (int i = 0; i < n_size; i++)
	{
		Segment2d *p12 = &segments[i];//p1.y < p2.y
		if (p12->p1.y < 0)
			continue;

		Segment2d *p12_img = &segments_in_image[i];//p1.y < p2.y
		for (int j = 0; j < n_size; j++)
		{
			if (j == i) continue;
			Segment2d *seg = &segments[j];
			if (seg->p1.y < 0)
				continue;
			Segment2d *seg_img = &segments_in_image[j];

			if (seg->getLength() > p12->getLength())
			{
				continue;
			}

			//simple check
			if (!p12->isNeighbor(*seg))
			{
				//fout_2 << i << ", " << j << "continue in isNeighbor" << endl;
				continue;
			}

			//slope difference?
			double dif_slope = atan(p12->getSlope());
			dif_slope -= atan(seg->getSlope());
			if (dif_slope > half_pi)
				dif_slope -= CV_PI;
			if (dif_slope < -half_pi)
				dif_slope += CV_PI;

			if (abs(dif_slope) > 5 * CV_PI / 180)
			{
				//fout_2 << i << ", " << j << "continue in abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 10 * CV_PI / 180" << endl;
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

			Pair2d pair2d(*p12, *seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			double mean_width = d1;
			pair2d.setMeanWidth(mean_width);

			//line(step1, p12_img->p1, p12_img->p2, Scalar(255, pairs.size(), i));
			//line(step1, seg_img->p1, seg_img->p2, Scalar(128, pairs.size(), j));


			pairs.push_back(pair2d);
			pairs_in_image.push_back(Pair2d(*p12_img, *seg_img));
		}
	}

	//imwrite("step_1.png", step1);
	t1 = getTickCount();
	cout << "-----------step_1 : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	int n_pairs = pairs.size();

	for (int i = 0; i < pairs.size(); i++)
	{
		Pair2d *pair_i = &pairs[i];
		pair_i->computeLineModel();
	}
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
			if (i == 50 && j == 51)
				int c = 0;
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

	t1 = getTickCount();
	cout << "-----------step_2 : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	int x_min = -30, x_max = 30, z_min = 0, z_max = 80;
	int scale = 10;

	int image_cols = (x_max - x_min) * scale;
	int image_rows = (z_max - z_min) * scale;
	Mat cameraScene = Mat::zeros(image_rows, image_cols, CV_8UC3);
	Point2d round(0.5, 0.5);

	Mat step1;
	rawColorImage.copyTo(step1);
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


		for (int k = 0; k < vec_i.size(); k++)
		{
			Pair2d *pair_k = &pairs[vec_i[k]];

			Point2d p1(pair_k->s1.p1.x - x_min, z_max - pair_k->s1.p1.y);
			Point2d p2(pair_k->s1.p2.x - x_min, z_max - pair_k->s1.p2.y);
			line(cameraScene, scale * p1 + round, scale * p2 + round, Scalar(255, 255, i));

			p1 = Point2d(pair_k->s2.p1.x - x_min, z_max - pair_k->s2.p1.y);
			p2 = Point2d(pair_k->s2.p2.x - x_min, z_max - pair_k->s2.p2.y);
			line(cameraScene, scale * p1 + round, scale * p2 + round, Scalar(255, 0, i));

			//line(step, pair_k->_p12.p1, pair_k->_p12.p2, Scalar(255, 0, r));

			//cout << vec_i[k] << "," << pair_k->s1.p1 << " " << pair_k->s1.p2 << "," << pair_k->s2.p1 << " " << pair_k->s2.p2 << "; ";
			//cout << pair_k->_p12.p1 <<  pair_k->_p12.p2 << endl;

		}

		for (int k = 0; k < vec_i.size(); k++)
		{

			Pair2d *pair_k = &pairs_in_image[vec_i[k]];
			line(step1, pair_k->s1.p1, pair_k->s1.p2, Scalar(255, 0, i));
			line(step1, pair_k->s2.p1, pair_k->s2.p2, Scalar(255, 255, i));

			//line(step, pair_k->_p12.p1, pair_k->_p12.p2, Scalar(255, 0, r));

			//cout << vec_i[k] << "," << pair_k->s1.p1 << " " << pair_k->s1.p2 << "," << pair_k->s2.p1 << " " << pair_k->s2.p2 << "; ";
			//cout << pair_k->_p12.p1 <<  pair_k->_p12.p2 << endl;

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
	imshow("cameraScene.png", step1);
	imshow("cameraScene", cameraScene);
	//imwrite("cameraScene.png", cameraScene);
	//waitKey(10);

	t1 = getTickCount();
	cout << "-----------in remap image 2 world : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	fout_2 << "-------------------" << endl;
	fout_2.close();
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

	//step3: choose segments in this zone
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
	cout << "step3 : choose segments in this zone " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	//step 4 : find pairs
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

			Pair2d pair2d(*p12, *seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			double mean_width = d1;
			pair2d.setMeanWidth(mean_width);

			pairs.push_back(pair2d);
			pairs_in_image.push_back(Pair2d(*p12_img, *seg_img));
		}
	}

	t1 = getTickCount();
	cout << "step 4 : find pairs " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;


	//step2 : updateIPM (kf, meanshift, ipm)
	updateIPM(pairs, pairs_in_image);
	t1 = getTickCount();
	cout << "step2 updateIPM : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;


	//step 5 : compute liens between each two and regroupe
	int n_pairs = pairs.size();
	for (int i = 0; i < n_pairs; i++)
	{
		Pair2d *pair_i = &pairs[i];
		pair_i->computeLineModel();
	}
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
			line(ipm_image, scale * p1 + round, scale * p2 + round, Scalar(255, 255, i));

			p1 = Point2d(pair_k->s2.p1.x - x_min, z_max - pair_k->s2.p1.y);
			p2 = Point2d(pair_k->s2.p2.x - x_min, z_max - pair_k->s2.p2.y);
			line(ipm_image, scale * p1 + round, scale * p2 + round, Scalar(255, 0, i));

		}
		for (int k = 0; k < vec_i.size(); k++)
		{

			Pair2d *pair_k = &pairs_in_image[vec_i[k]];
			line(cameraScene, pair_k->s1.p1, pair_k->s1.p2, Scalar(255, 0, i));
			line(cameraScene, pair_k->s2.p1, pair_k->s2.p2, Scalar(255, 255, i));
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
	imshow("ipm_image", ipm_image);
	imshow("cameraScene", cameraScene);
	//imwrite("cameraScene.png", cameraScene);

	t1 = getTickCount();
	cout << "final : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

}


double LaneDetection::estimateRx(vector<Segment2d> *segments_in_image)
{
	Mat step2;
	rawColorImage.copyTo(step2);
	int n_size = segments_in_image->size();
	Point2d vp(step2.cols / 2, step2.rows / 2);
	int nb = 0;

	vector<Point2d> intersecs;
	for (int i = 0; i < n_size; i++)
	{
		Segment2d *s = &segments_in_image->operator[](i);

		for (int j = i + 1; j < n_size; j++)
		{
			Point2d p = s->intersec(&segments_in_image->operator[](j));
			if (p.x < 0 || p.y < 0)
				continue;
			if (p.x > step2.cols || p.y > step2.rows)
				continue;
			nb++;
			//vp += p;
			//circle(step2, p, 1, Scalar(0, 0, 255));
			intersecs.push_back(p);
		}
	}

	//mean shift
	int time = 0;
	while (time < 10)
	{
		time++;
		Point2d v(0, 0);
		double b = 0;
		for (int k = 0; k < nb; k++)
		{
			Point2d x = vp - intersecs[k];
			double ker = kernel(x);
			v += ker * intersecs[k];
			b += ker;
		}
		v = v / b;
		if ((v - vp).ddot(v - vp) < EPS_HERE)
		{
			vp = v;
			break;
		}
		vp = v;
	}

	circle(step2, vp, 5, Scalar(255, 255, 0));

	//kf
	kf.predict();
	Mat measure = (Mat_<float>(2, 1) << vp.x, vp.y);
	kf.correct(measure);
	
	float *ptr_measure = kf.statePost.ptr<float>(0);
	vp = Point2d(ptr_measure[0], ptr_measure[1]);


	double fx, fy, cu, cv;
	ipm->getCameraParam(fx, fy, cu, cv);
	double rx = atan2(cv - vp.y, (fx + fy) / 2);


	double _, h;
	ipm->getRxAndH(_, h);
	ipm->createModel(fx, fy, cu, cv, rx, h);

	circle(step2, vp, 5, Scalar(255, 0, 0));
	imshow("step2", step2);
	return rx;
}