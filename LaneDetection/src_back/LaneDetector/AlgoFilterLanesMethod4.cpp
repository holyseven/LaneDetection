#include "LaneDetection.h"
#include <fstream>
#include <stdlib.h>

#define EPS_HERE 1.0e-2
#define INFINI_HERE 1.0e6
ofstream fout_2;


//(p1-p2)*(p1-p2)
inline double pow_dist_p2p(Point2d p1, Point2d p2)
{
	return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

//(p1-p0)x(p2-p0)
inline double cross(Point2d p0, Point2d p1, Point2d p2)
{
	return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
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

Rect getBoundingBox(Point2d p0, Point2d p1, Point2d p2, Point2d p3)
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

	return Rect(tf_x, tf_y, rb_x - tf_x + 0.5, rb_y - tf_y + 0.5);
}

//distance between two points
inline double dist_p2p(Point2d p1, Point2d p2)
{
	return sqrt(pow_dist_p2p(p1, p2));
}

inline bool adjacent_p2p(Point2d p1, Point2d p2) {
	return ((abs(p1.x - p2.x) < 2) && (abs(p1.y - p2.y) < 2));
}

inline double slope_seg(Point2d p1, Point2d p2)
{
	if (abs(p2.x - p1.x) < EPS_HERE) return INFINI_HERE;
	return (p2.y - p1.y) / (p2.x - p1.x);
}

//compute the distance between a point p and a segment (seg1, seg2).
//If the foot is in this segment, all will be ok.
//If not, this fonction will return the smaller one between p, seg1 and p, seg2.
double dist_p2segment(Point2d p, Point2d seg1, Point2d seg2) {
	if (abs(seg1.x - seg2.x) < EPS_HERE)
	{
		int val = ((p.y - seg1.y) > 0 ? 1 : -1) * ((p.y - seg2.y) > 0 ? 1 : -1);
		if (val < 0)
			return abs(p.x - seg1.x);
		else
			return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}
	if (abs(seg1.y - seg2.y) < EPS_HERE)
	{
		int val = ((p.x - seg1.x) > 0 ? 1 : -1) * ((p.x - seg2.x) > 0 ? 1 : -1);
		if (val < 0)
			return abs(p.y - seg1.y);
		else
			return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}

	double k = (seg2.y - seg1.y) / (seg2.x - seg1.x);
	double foot_x = (k * k * seg1.x + k * (p.y - seg1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg1.x) + seg1.y;

	int val = (foot_x - seg1.x > 0 ? 1 : -1) * (foot_x - seg2.x > 0 ? 1 : -1);
	if (val > 0)
	{
		return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}
	else
	{
		return dist_p2p(p, Point2d(foot_x, foot_y));
	}
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

bool foot_p2segment(Point2d p, Point2d seg1, Point2d seg2, Point2d &foot)
{
	if (abs(seg1.x - seg2.x) < EPS_HERE)
	{
		int val = ((p.y - seg1.y) > 0 ? 1 : -1) * ((p.y - seg2.y) > 0 ? 1 : -1);
		if (val < 0)
		{
			foot = Point2d(seg1.x, p.y);
			return true;
		}
		else
		{
			return false;
		}

	}
	if (abs(seg1.y - seg2.y) < EPS_HERE)
	{
		int val = ((p.x - seg1.x) > 0 ? 1 : -1) * ((p.x - seg2.x) > 0 ? 1 : -1);
		if (val < 0)
		{
			foot = Point2d(p.x, seg1.y);
			return true;
		}
		else
			return false;
	}

	double k = (seg2.y - seg1.y) / (seg2.x - seg1.x);
	double foot_x = (k * k * seg1.x + k * (p.y - seg1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg1.x) + seg1.y;

	int val = (foot_x - seg1.x > 0 ? 1 : -1) * (foot_x - seg2.x > 0 ? 1 : -1);
	if (val < 0)
	{
		foot = Point2d(foot_x, foot_y);
		return true;
	}
	else
	{
		return false;
	}
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


Segment2d::Segment2d(Point2d _p1, Point2d _p2)
{
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

	indexZone[0] = (int)p1.y / 5;
	indexZone[1] = (int)p2.y / 5;
	if (p1.x <= p2.x)
	{
		indexZone[2] = (int)p1.x / 5;
		indexZone[3] = (int)p2.x / 5;
	}
	else
	{
		indexZone[2] = (int)p2.x / 5;
		indexZone[3] = (int)p1.x / 5;
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

vector<Segment2d> Segment2d::getValidRect(Segment2d s, int rows)
{
	vector<Segment2d> vec;

	Point2d foot1 = foot_p2line(p1, s);

	//distance compare
	double thresh = 1.0;
	if (dist_p2p(foot1, p1) > thresh)
	{
		//fout_2 << foot1 << p1 << thresh << endl;
		//fout_2 << "continue in dist_p2p(foot2, p2) > thresh" << endl;
		return vec;
	}
	Point2d foot2 = foot_p2line(p2, s);
	if (dist_p2p(foot2, p2) > thresh)
	{
		//fout_2 << foot2 << p2 << thresh << endl;
		//fout_2 << "continue in dist_p2p(foot2, p2) > thresh" << endl;
		return vec;
	}
	Segment2d foot_12(foot1, foot2);

	//possible to improve
	if (s.p1.y > foot_12.p2.y || s.p2.y < foot_12.p1.y)
	{
		//fout_2 << p1 << p2 << s.p1 << s.p2 << endl;
		//fout_2 << "continue in s.p1.y > foot_12.p2.y || s.p2.y < foot_12.p1.y" << endl;
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
		//fout_2 << p1 << p2 << s.p1 << s.p2 << endl;
		//fout_2 << "continue in valid_foot_12.getLength() < s.getLength() * 0.2" << endl;
		return vec;//empty
	}


	Point2d new_p1, new_p2;

	new_p1 = foot_p2line(valid_foot_12.p1, *this);
	new_p2 = foot_p2line(valid_foot_12.p2, *this);

	//distance compare
	thresh = 0.5;
	if (dist_p2p(new_p1, valid_foot_12.p1) > thresh)
	{
		//fout_2 << new_p1 << valid_foot_12.p1 << thresh << endl;
		//fout_2 << "dist_p2p(new_p1, valid_foot_12.p1) > thresh" << endl;
		return vec;
	}

	if (dist_p2p(new_p2, valid_foot_12.p2) > thresh)
	{
		//fout_2 << new_p2 << valid_foot_12.p2 << thresh << endl;
		//fout_2 << "dist_p2p(new_p2, valid_foot_12.p2) > thresh" << endl;
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

	double dif_angle = 0;
	if (this->horizontal && p2->horizontal)
	{
	}
	else
	{
		dif_angle = abs(atan(this->a) - atan(p2->a));//radian.
	}

	//const double ANG = 0.2618; // 15 degree
	//const double ANG = 0.174533; // 10 degree
	const double ANG = 0.0174533; // 1 degree
	const double SCALE_ANG = 0.08726646 * 0.08726646;
	//const double DIFDIRDIST = 10; // pixels.
	//const double SCALE_DIRDIST = DIFDIRDIST * DIFDIRDIST * 100;
	const double DIFCOLOR = 10;
	const double SCALE_DIFCOLOR = 2500;
	const double DIFWIDTH = 0.1;
	const double SCALE_DIFWIDTH = 0.25;
	const double DIFALIDIST = 0.1;
	const double SCALE_DIFALIDIST = 0.25;

	double part_ang = dif_angle - ANG;
	if (part_ang < 0)
		part_ang = 1;
	else
		part_ang = exp(-part_ang * part_ang / SCALE_ANG);

	double part_color = abs(this->mean_color - p2->mean_color) - DIFCOLOR;
	if (part_color < 0)
		part_color = 1;
	else
		part_color = exp(-part_color * part_color / SCALE_DIFCOLOR);

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

	return part_ang * part_align_dist * part_width;
}




void LaneDetection::AlgoFilterLanes(ntuple_list lines_out){}

void LaneDetection::method4(int index_windows_name, InversePerspectiveMapping* ipm) {
	if (ipm == NULL)
	{
		cout << "ipm is null .. " << endl;
		return;
	}
	if (!processImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	fout_2.open("pairs.txt");
	Mat temp_image;
	processImage.copyTo(temp_image);
	detectionLineLSD(temp_image);
	char windowName[64];
	sprintf(windowName, "%d", nameIndex);
	imwrite(string(windowName) + "step0.png", temp_image);


	processImage.copyTo(temp_image);

	unsigned int X = temp_image.cols;  /* x image size */
	unsigned int Y = temp_image.rows;  /* y image size */
	unsigned int XY = X * Y; /* numer of pixels */

	if (temp_image.channels() == 1)
		temp_image.copyTo(processGrayImage);
	else if (temp_image.channels() == 3)
		cvtColor(temp_image, processGrayImage, CV_BGR2GRAY);

	image_double image = new_image_double(X, Y);
	/*copy the data*/
	uchar* ptr_gray_image = processGrayImage.ptr<uchar>(0);
	for (int _pixel = 0; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}

	int64 t0, t1;
	t0 = getTickCount();

	/* call LSD */
	ntuple_list lsd_result = lsd(image);
	t1 = getTickCount();
	cout << "-----------in lsd : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	int dim = lsd_result->dim;
	int n_size = lsd_result->size;


	vector<Pair2d> pairs;
	vector<Pair2d> pairs_in_image;

	Mat step1;
	cvtColor(processGrayImage, step1, CV_GRAY2BGR);
	double half_pi = 90 * CV_PI / 180;
	for (int i = 0; i < lsd_result->size; i++)
	{
		Point2d p1 = ipm->remapImg2World(lsd_result->values[i * lsd_result->dim + 0], lsd_result->values[i * lsd_result->dim + 1]);
		Point2d p2 = ipm->remapImg2World(lsd_result->values[i * lsd_result->dim + 2], lsd_result->values[i * lsd_result->dim + 3]);
		Segment2d p12 = Segment2d(p1, p2);//p1.y < p2.y
		//cout << i << ": " << p1 << ", " << p2 << endl;
		for (int j = 0; j < n_size; j++)
		{
			if (j == i) continue;
			Point2d seg1 = ipm->remapImg2World(lsd_result->values[j * lsd_result->dim + 0], lsd_result->values[j * lsd_result->dim + 1]);
			Point2d seg2 = ipm->remapImg2World(lsd_result->values[j * lsd_result->dim + 2], lsd_result->values[j * lsd_result->dim + 3]);
			Segment2d seg(seg1, seg2);

			Point2d p1_img = Point2d(lsd_result->values[i * lsd_result->dim + 0], lsd_result->values[i * lsd_result->dim + 1]);
			Point2d p2_img = Point2d(lsd_result->values[i * lsd_result->dim + 2], lsd_result->values[i * lsd_result->dim + 3]);
			Segment2d p12_img = Segment2d(p1_img, p2_img);//p1.y < p2.y

			Point2d seg1_img = Point2d(lsd_result->values[j * lsd_result->dim + 0], lsd_result->values[j * lsd_result->dim + 1]);
			Point2d seg2_img = Point2d(lsd_result->values[j * lsd_result->dim + 2], lsd_result->values[j * lsd_result->dim + 3]);
			Segment2d seg_img(seg1_img, seg2_img);

			if (seg.getLength() > p12.getLength())
			{
				continue;
			}

			//simple check
			if (!p12.isNeighbor(seg))
			{
				continue;
			}

			//slope difference?
			double dif_slope = atan(p12.getSlope());
			dif_slope -= atan(seg.getSlope());
			if (dif_slope > half_pi)
				dif_slope -= CV_PI;
			if (dif_slope < -half_pi)
				dif_slope += CV_PI;

			if (abs(dif_slope) > 15 * CV_PI / 180)
			{
				fout_2 << i << "," <<j  << ": " << p1_img << p2_img << seg1_img << seg2_img 
					<< p1 << p2 << seg1 << seg2  
					<< p12.getSlope() << "," << seg.getSlope() << endl;
				fout_2 << "continue in abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 10 * CV_PI / 180" << endl;
				continue;
			}

			vector<Segment2d> entire_v = p12.getValidRect(seg, processGrayImage.rows);
			//valid?
			if (entire_v.empty())
			{
				fout_2 << i << "," << j  << ": " << p1_img << p2_img << seg1_img << seg2_img
					<< endl;
				fout_2 << "continue in v.empty()" << endl;
				continue;
			}
			double d1 = dist_p2p(entire_v[0].p1, entire_v[1].p1);
			if (d1 > 1 || d1 < EPS_HERE)
			{
				fout_2 << p1 << p2 << seg1 << seg2 << endl;
				fout_2 << "continue in d1 > 1 || d1 < EPS_HERE" << endl;
				continue;
			}
			Pair2d pair2d(p12, seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			double mean_width = d1;
			pair2d.setMeanWidth(mean_width);



			pairs.push_back(pair2d);

			line(step1, p1_img, p2_img, Scalar(255, 0, i));
			line(step1, seg1_img, seg2_img, Scalar(255, 255, j));

			pairs_in_image.push_back(Pair2d(p12_img, seg_img));
		}
	}

	imwrite("step_1.png", step1);
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
			double weight = pair_i->weight(&pairs[j]);
			ptr_row_weights[j] = weight;
			weights.at<double>(j, i) = weight;
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

	for (int i = 0; i < n_pairs; i++)
	{
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
			}

			for (int j = 0; j < vec_ints[p].size(); j++)
			{
				if (!drawn[vec_ints[p][j]])
					stk.push_back(vec_ints[p][j]);
			}
		}


		Mat step;
		cvtColor(processGrayImage, step, CV_GRAY2BGR);
		int b = (unsigned)theRNG() & 255;
		int g = (unsigned)theRNG() & 255;
		int r = i & 255;
		for (int k = 0; k < vec_i.size(); k++)
		{
			Pair2d *pair_k = &pairs_in_image[vec_i[k]];
			line(step, pair_k->s1.p1, pair_k->s1.p2, Scalar(255, 0, r));
			line(step, pair_k->s2.p1, pair_k->s2.p2, Scalar(255, 255, r));
			//line(step, pair_k->_p12.p1, pair_k->_p12.p2, Scalar(255, 0, r));

			//cout << vec_i[k] << "," << pair_k->s1.p1 << " " << pair_k->s1.p2 << "," << pair_k->s2.p1 << " " << pair_k->s2.p2 << "; ";
			//cout << pair_k->_p12.p1 <<  pair_k->_p12.p2 << endl;

		}
		//cout << endl;
		if (vec_i.size() != 0)
		{
			char fileName[20];
			sprintf_s(fileName, "step%d.png", i);

			imwrite(fileName, step);
		}

	}


	t1 = getTickCount();
	cout << "-----------in remap image 2 world : " << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;

	fout_2 << "-------------------" << endl;
	fout_2.close();
}

