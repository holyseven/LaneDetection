#include "LaneDetection.h"
#include <fstream>
#include <stdlib.h>

#define EPS_HERE 1.0e-1
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

inline bool adjacent_p2p(Point2d p1, Point2d p2){
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
double dist_p2segment(Point2d p, Point2d seg1, Point2d seg2){
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

Point2d foot_p2line(Point2d p, Segment2d seg){
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

	//if (p2.y - p1.y < 1.0)
	//{
	//	Point2d t;
	//	if (p1.x > p2.x)
	//	{
	//		t = p1;
	//		p1 = p2;
	//		p2 = t;
	//	}
	//}
	
	indexZone[0] = (int)p1.y / 50 ;
	indexZone[1] = (int)p2.y / 50 ;
	if (p1.x <= p2.x)
	{
		indexZone[2] = (int)p1.x / 50;
		indexZone[3] = (int)p2.x / 50;
	}
	else
	{
		indexZone[2] = (int)p2.x / 50;
		indexZone[3] = (int)p1.x / 50;
	}
	_ini_slope = false;
	_ini_length = false;
}

void Segment2d::computeSlope(){
	slope = slope_seg(p1, p2);
	_ini_slope = true;
}

void Segment2d::computeLength(){
	length = dist_p2p(p1, p2);
	_ini_length = true;
}

bool Segment2d::isNeighbor(Segment2d s){
	bool b1 = s.indexZone[0] > indexZone[0] - 2 && s.indexZone[0] < indexZone[0] + 2;
	bool b2 = s.indexZone[2] > indexZone[2] - 2 && s.indexZone[2] < indexZone[2] + 2;
	if (b1 && b2)
		return true;

	b1 = s.indexZone[1] > indexZone[1] - 2 && s.indexZone[1] < indexZone[1] + 2;
	b2= s.indexZone[3] > indexZone[3] - 2 && s.indexZone[3] < indexZone[3] + 2;
	if (b1 && b2)
		return true;

	bool b = s.indexZone[0] > indexZone[0] - 1 && s.indexZone[1] < indexZone[1] + 1
		&& s.indexZone[2] > indexZone[2] - 1 && s.indexZone[3] < indexZone[3] + 1;
	if (b)
		return true;

	b = min( (s.indexZone[1] - indexZone[0] + 1) * (s.indexZone[3] - indexZone[2] + 1)
		, (indexZone[1] - s.indexZone[0] + 1) * (indexZone[3] - s.indexZone[2] + 1) ) > 7;

	if (b)
		return true;

	return false;
}

vector<Segment2d> Segment2d::getValidRect(Segment2d s, int rows)
{
	vector<Segment2d> vec;

	Point2d foot1 = foot_p2line(p1, s);

	//distance compare
	double thresh = 50;
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

	if (arry[3].y - arry[0].y < 1.0)
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
	thresh = 40;
	if (new_p1.y + valid_foot_12.p1.y < 8 * rows / 5)
		thresh = 30;
	if (new_p1.y + valid_foot_12.p1.y < 6 * rows / 5)
		thresh = 15;
	if (dist_p2p(new_p1, valid_foot_12.p1) > thresh)
	{
		//fout_2 << new_p1 << valid_foot_12.p1 << thresh << endl;
		//fout_2 << "dist_p2p(new_p1, valid_foot_12.p1) > thresh" << endl;
		return vec;
	}

	thresh = 40;
	if (new_p2.y + valid_foot_12.p2.y < 8 * rows / 5)
		thresh = 30;
	if (new_p2.y + valid_foot_12.p2.y < 6 * rows / 5)
		thresh = 15;
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

	if (abs(y1 - y2) < 1.0)
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

	fout_2 << a << "," << b << "," << c << endl;

	hasModeled = true;
}

Point2d Pair2d::intersec(Pair2d *p2)
{
	if (!this->hasModeled) computeLineModel();
	if (!p2->hasModeled) p2->computeLineModel();

	if (!this->horizontal && !p2->horizontal)
	{
		if (abs(this->a - p2->a) < EPS_HERE)
		{
			return Point2d(-1, -1);
		}
		else
		{
			double y = (this->b - p2->b) / (p2->a - this->a);
			double x = this->a * y + this->b;
			return Point2d(x, y);
		}
	}
	else if (this->horizontal && !p2->horizontal)
	{
		return Point2d(p2->a * this->c + p2->b, this->c);
	}
	else if (!this->horizontal && p2->horizontal)
	{
		return Point2d(this->a * p2->c + this->b, p2->c);
	}
	else
	{
		return Point2d(-1, -1);
	}
}

bool Pair2d::parallele(Pair2d *p2, double prec_d){
	if (!this->hasModeled) computeLineModel();
	if (!p2->hasModeled) p2->computeLineModel();

	if (this->horizontal && p2->horizontal)
	{
		return true;
	}
	else
	{
		if (abs(atan(this->a) - atan(p2->a)) < prec_d * CV_PI / 180)
			return true;
		else
			return false;
	}
}

bool Pair2d::aligned(Pair2d *p2, double prec_p)
{
	if (!this->hasModeled) computeLineModel();
	if (!p2->hasModeled) p2->computeLineModel();

	if (dist_p2line(p2->_p1, this->_p12) > prec_p)
	{
		return false;
	}

	if (dist_p2line(p2->_p2, this->_p12) > prec_p)
	{
		return false;
	}

	return true;
}

double Pair2d::weight(Pair2d* p2)
{
	if (!this->hasModeled) computeLineModel();
	if (!p2->hasModeled) p2->computeLineModel();


	double dif_angle = 0;
	if (this->horizontal && p2->horizontal)
	{
		;
	}
	else
	{
		dif_angle = abs(atan(this->a) - atan(p2->a));//radian.
	}

	double dif_direc_dist = 0;
	dif_direc_dist = min(dist_p2p(this->_p1, p2->_p2), dist_p2p(this->_p2, p2->_p1));

	//const double ANG = 0.2618; // 15 degree
	//const double ANG = 0.174533; // 10 degree
	const double ANG = 0.08726646; // 5 degree
	const double SCALE_ANG = 0.174533 * 0.174533;
	//const double DIFDIRDIST = 10; // pixels.
	//const double SCALE_DIRDIST = DIFDIRDIST * DIFDIRDIST * 100;
	const double DIFCOLOR = 10;
	const double SCALE_DIFCOLOR = 2500;
	const double DIFWIDTH = 1;
	const double SCALE_DIFWIDTH = 100;
	const double DIFALIDIST = 1;
	const double SCALE_DIFALIDIST = 100;
	
	double part_ang = dif_angle - ANG;
	if (part_ang < 0)
		part_ang = 1;
	else
		part_ang = exp(-part_ang * part_ang / SCALE_ANG);

	//double  part_direc_dist = dif_direc_dist - DIFDIRDIST;
	//if (part_direc_dist < 0)
	//	part_direc_dist = 1;
	//else
	//	part_direc_dist = exp(-part_direc_dist * part_direc_dist / SCALE_DIRDIST);
	
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
		,dist_p2line(0.5 * (this->_p1 + this->_p2), p2->_p12));
	double part_align_dist = dif_align_dist - DIFALIDIST;
	if (part_align_dist < 0)
		part_align_dist = 1;
	else
		part_align_dist = exp(-part_align_dist * part_align_dist / SCALE_DIFALIDIST);

	return part_ang * part_align_dist * part_width;
}

/*
VARIATIONS:
	-- S: Ensemble de segments obtenus par LSD.

Pour chaque segment 'Si' dans S
{
	Pour tous les autres segments 'Sj' dans S
	{
		si la longueur de Sj est plus grande que celle de Si
		ou Sj est très loin de Si
­­­­		ou la différence de la pente de Sj et de Si est hors la tolérance
			continue;

		//sous quelques conditions
		//retourner vide s'il ne satisfait pas ces conditions.
		v = Si.getValidRect(Sj);

		Si v est vide
			continue;
		
		v est compos?de deux bords, et on fait deux fois la translation pour obtenir les voisins de v.
		Comparer les couleurs de v et de ses voisins.
		Si le couleur de v n'est pas plus clair que celui de ses voisins
			continue;

		Si et Sj est considér?comme un couple (Pair2d).
	}
}

On utilise un segment '_p12' pour représenter un couple.
On choit les deux points de ce segment dont chaque est le milieu de deux extrémités de v.

VARIATIONS:
	-- pairs: tous les couples obtenus par l'étape dernière.

Pour chaque couple 'pair_i' dans pairs
{
	Pour tous les autres couples 'pair_j' dans pairs
	{
		si pair_i et pair_j ne sont pas alignes
			continue;

		pair_j sera connect?par pair_i.
	}

	Le nombre de branches de pair_i n'est pas fixée.
}

Regrouper tous les noeuds qui sont connectés.


*/

void LaneDetection::AlgoFilterLanes(ntuple_list line_out)
{
	fout_2.open("pairs.txt");
	unsigned int dim = line_out->dim;
	int n_size = line_out->size;
	int cols = processGrayImage.cols;
	int rows = processGrayImage.rows;
	
	Mat lines_candidats = Mat::zeros(processGrayImage.size(), CV_32SC1);
	vector<Pair2d> pairs;

	Mat step1;
	processImage.copyTo(step1);

	for (int i = 0; i < n_size; i++)
	{
		if (line_out->values[i * dim + 1] < 0.6 * rows &&
			line_out->values[i * dim + 3] < 0.6 * rows)
			continue;

		const Point2d p1(line_out->values[i * dim + 0], line_out->values[i * dim + 1]);
		const Point2d p2(line_out->values[i * dim + 2], line_out->values[i * dim + 3]);
		Segment2d p12 = Segment2d(p1, p2);//p1.y < p2.y

		for (int j = 0; j < n_size; j++)
		{
			if (j == i) continue;
			Point2d seg1(line_out->values[j * dim + 0], line_out->values[j * dim + 1]);
			Point2d seg2(line_out->values[j * dim + 2], line_out->values[j * dim + 3]);
			Segment2d seg(seg1, seg2);
			
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
			if (abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 15 * CV_PI / 180)
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << p12.getSlope() << "," << seg.getSlope() << endl;
				//fout_2 << "continue in abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 10 * CV_PI / 180" << endl;
				continue;
			}

			vector<Segment2d> entire_v = p12.getValidRect(seg, processGrayImage.rows);
			//valid?
			if (entire_v.empty())
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << endl;
				//fout_2 << "continue in v.empty()" << endl;
				continue;
			}

			//color compare
			Point2d translation[3];
			translation[0] = Point2d(0, 0);
			translation[1] = entire_v[0].p1 - entire_v[1].p1;
			translation[2] = -translation[1];

			vector<uchar> vec_color[3];
			double mean_color[3];
			double stdDev_color[3];
			
			
			vector<Segment2d> v;
			v.push_back(Segment2d(entire_v[0].p1 - translation[1] / 4, entire_v[0].p2 - translation[1] / 4));
			v.push_back(Segment2d(entire_v[1].p1 + translation[1] / 4, entire_v[1].p2 + translation[1] / 4));
				
			for (int _trans = 0; _trans < 3; _trans++)
			{
				Point2d trans = translation[_trans];
				Rect box = getBoundingBox(v[0].p1 + trans, v[0].p2 + trans,
					v[1].p1 + trans, v[1].p2 + trans);
				for (int _y = box.y; _y < box.y + box.height; _y++)
				{
					if (_y >= step1.rows || _y < 0) continue;
					uchar* ptr_row_processImage = processGrayImage.ptr<uchar>(_y);
					for (int _x = box.x; _x < box.x + box.width; _x++)
					{
						if (_x >= step1.cols || _x < 0) continue;
						Point2d p(_x, _y);
						int direc = cross(p, v[0].p1 + trans, v[0].p2 + trans) > -0.0 ? 1 : -1;

						if (direc != (cross(p, v[0].p2 + trans, v[1].p2 + trans) > -0.0 ? 1 : -1))
							continue;
						if (direc != (cross(p, v[1].p2 + trans, v[1].p1 + trans) > -0.0 ? 1 : -1))
							continue;
						if (direc != (cross(p, v[1].p1 + trans, v[0].p1 + trans) > -0.0 ? 1 : -1))
							continue;

						vec_color[_trans].push_back(ptr_row_processImage[_x]);
					}
				}
				if (vec_color[_trans].empty())
					;
				else
				{
					Mat mean, stdDev;
					meanStdDev(vec_color[_trans], mean, stdDev);
					mean_color[_trans] = mean.at<double>(0);
					stdDev_color[_trans] = stdDev.at<double>(0);
				}
			}

			//double min_value = -1;
			//minMaxIdx(vec_color[0], &min_value);

			bool color_matched = (mean_color[0] > mean_color[1] + 10) && (mean_color[0] > mean_color[2] + 10)
				&& stdDev_color[0] < 50;

			if (!color_matched)
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << mean_color[0] << "," << mean_color[1] << "," << mean_color[2];
				//fout_2 << "," << stdDev_color[0] << endl;
				//fout_2 << "continue in color_matched" << endl;

				continue;
			}

			//fout_2 << p1 << p2 << seg1 << seg2 << endl;
			//fout_2 << "Paired!!!!!!" << endl;

			//cout << v[0].p1 << v[0].p2 << v[1].p1 << v[1].p2 << endl;

			//line(processImage, v[0].p1, v[0].p2, Scalar(255, 0, 0));
			//line(processImage, v[1].p1, v[1].p2, Scalar(0, 255, 0));
			//line(processImage, v[0].p1, v[1].p1, Scalar(0, 0, 255));
			//line(processImage, v[0].p2, v[1].p2, Scalar(0, 255, 255));

			line(step1, p1, p2, Scalar(255, 0, pairs.size()));
			line(step1, seg1, seg2, Scalar(0, 255, pairs.size()));
			//cout << v[0].p1 << "," << v[0].p2 << "£¬ " << v[1].p1 << v[1].p2 << "," << endl;
			Pair2d pair2d(p12, seg, entire_v[0].p1, entire_v[0].p2, entire_v[1].p1, entire_v[1].p2);
			pair2d.setMeanColor(mean_color[0]- 0.5 * mean_color[1] - 0.5 * mean_color[2]);

			double mean_width = 0;

			//double scale_cxy = dist_p2p(Point2d(center_x, center_y),
			//	0.5 * (entire_v[0].p1 + entire_v[1].p1));
			//if (scale_cxy < 50) scale_cxy = 50;
			//mean_width += dist_p2p(entire_v[0].p1, entire_v[1].p1) / scale_cxy;

			//scale_cxy = dist_p2p(Point2d(center_x, center_y),
			//	0.5 * (entire_v[0].p2 + entire_v[1].p2));
			//if (scale_cxy < 50) scale_cxy = 50;
			//mean_width += dist_p2p(entire_v[0].p2, entire_v[1].p2) / scale_cxy;

			double scale_row = 0.25 * (entire_v[0].p1.y + entire_v[1].p1.y + entire_v[0].p2.y + entire_v[1].p2.y) - center_y;
			if (scale_row < 20) scale_row = 20;
			double d1 = dist_p2p(entire_v[0].p1, entire_v[1].p1) / scale_row ;

			mean_width = 10 * d1;
			//cout << pairs.size() << ": " << mean_width << endl;

			pair2d.setMeanWidth(mean_width);
			pairs.push_back(pair2d);
			//imshow("step1", processImage);
			//waitKey();
		}//end for j
	}

	imwrite("step_1.png", step1);

	int n_pairs = pairs.size();

	vector<Point2d> validIntersec;
	for (int i = 0; i < pairs.size(); i++)
	{
		Pair2d *pair_i = &pairs[i];
		pair_i->computeLineModel();
	}

	fout_2 << n_pairs << endl;

	vector< vector <int> > vec_ints;
	vec_ints.resize(n_pairs);
	Mat weights = Mat::zeros(n_pairs, n_pairs, CV_64FC1);
	for (int i = 0; i < n_pairs; i++)
	{
		if (i == 40)
			int c = 9;
		//vector <int> vec_int;
		vec_ints[i].push_back(i);
		Pair2d *pair_i = &pairs[i];
		double *ptr_row_weights = weights.ptr<double>(i);
		for (int j = i + 1; j < n_pairs; j++)
		{
			if (j == 156)
				int c = 0;
			double weight = pair_i->weight(&pairs[j]);
			ptr_row_weights[j] = weight;
			weights.at<double>(j, i) = weight;
			if (weight > 0.9)
			{
				vec_ints[i].push_back(j);
				vec_ints[j].push_back(i);
			}
		}
		//vec_ints.push_back(vec_int);
	}

	//for (int i = 0; i < n_pairs; i++)
	//{
	//	if (vec_ints[i].size() > 2)
	//		continue;

	//	//vector <int> vec_int;
	//	Pair2d *pair_i = &pairs[i];

	//	double *ptr_row_weights = weights.ptr<double>(i);
	//	int maxInd = -1;
	//	double maxVal = -1;
	//	for (int j = 0; j < n_pairs; j++)
	//	{
	//		if (vec_ints[i][0] == j || i == j)
	//			continue;
	//		if (vec_ints[i].size() == 2)
	//		{
	//			if (vec_ints[i][1] == j)
	//				continue;
	//		}

	//		if (maxVal < ptr_row_weights[j])
	//		{
	//			maxVal = ptr_row_weights[j];
	//			maxInd = j;
	//		}
	//	}
	//	if (maxVal > 0.2)
	//	{
	//		vec_ints[i].push_back(maxInd);
	//		vec_ints[maxInd].push_back(i);
	//	}

	//}


	fout_2 << "weights: " << weights << endl;
	
	bool *drawn = new bool[n_pairs];
	memset(drawn, false, n_pairs);
	
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

		double mean_color = 0;
		double mean_width = 0;
		double mean_long = 0;
		for (int k = 0; k < vec_i.size(); k++)
		{
			Pair2d *pair_k = &pairs[vec_i[k]];
			mean_color += pair_k->mean_color;
			mean_width += pair_k->mean_width;
			mean_long += dist_p2p(pair_k->_p1, pair_k->_p2);
		}
		mean_color /= vec_i.size();
		mean_width /= vec_i.size();
		//mean_long /= vec_i.size();

		//if (mean_long / mean_width < 5)
		//	continue;

		Mat step;
		processImage.copyTo(step);
		int b = (unsigned)theRNG() & 255;
		int g = (unsigned)theRNG() & 255;
		int r = i;
		for (int k = 0; k < vec_i.size(); k++)
		{
			Pair2d *pair_k = &pairs[vec_i[k]];
			line(step, pair_k->s1.p1, pair_k->s1.p2, Scalar(b, g, r));
			line(step, pair_k->s2.p1, pair_k->s2.p2, Scalar(b, g, r));
			line(step, pair_k->_p12.p1, pair_k->_p12.p2, Scalar(255, 0, r));
			
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


		fout_2 << endl;
	}

	fout_2 << "-------------------" << endl;
	fout_2.close();
}




