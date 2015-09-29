
#ifndef KITTI_DATA_READER_H
#define KITTI_DATA_READER_H

#include <string>
#include <fstream>
using namespace std;

class Oxts_Data_Type{
public:
	Oxts_Data_Type(){};

	double lat;// latitude of the oxts - unit(deg)
	double lon;// longitude of the oxts - unit(deg)
	double alt;// altitude of the oxts - unit(m)
	double roll;// roll angle(rad), 0 = level, positive = left side up, range : -pi   .. + pi
	double pitch;// pitch angle(rad), 0 = level, positive = front down, range : -pi / 2 .. + pi / 2
	double yaw;// heading(rad), 0 = east, positive = counter clockwise, range : -pi   .. + pi
	double vn;// velocity towards north(m / s)
	double ve;// velocity towards east(m / s)

	double vf;// : forward velocity, i.e.parallel to earth - surface(m / s)
	double vl;// : leftward velocity, i.e.parallel to earth - surface(m / s)
	double vu;// : upward velocity, i.e.perpendicular to earth - surface(m / s)
	double ax;// : acceleration in x, i.e.in direction of vehicle front(m / s ^ 2)
	double ay;// : acceleration in y, i.e.in direction of vehicle left(m / s ^ 2)
	double az;// : acceleration in z, i.e.in direction of vehicle top(m / s ^ 2)
	double af;// : forward acceleration(m / s ^ 2)
	double al;// : leftward acceleration(m / s ^ 2)
	double au;// : upward acceleration(m / s ^ 2)
	double wx;// : angular rate around x(rad / s)
	double wy;// : angular rate around y(rad / s)
	double wz;// : angular rate around z(rad / s)
	double wf;// : angular rate around forward axis(rad / s)
	double wl;// : angular rate around leftward axis(rad / s)
	double wu;// : angular rate around upward axis(rad / s)

	double pos_accuracy;// velocity accuracy(north / east in m)

				//vel_accuracy : velocity accuracy(north / east in m / s)
				//navstat : navigation status(see navstat_to_string)
				//numsats : number of satellites tracked by primary GPS receiver
				//posmode : position mode of primary GPS receiver(see gps_mode_to_string)
				//velmode : velocity mode of primary GPS receiver(see gps_mode_to_string)
				//orimode : orientation mode of primary GPS receiver(see gps_mode_to_string)

	friend ifstream& operator >> (ifstream& in, Oxts_Data_Type& M);
};

#define SIZE_S 2
#define SQRT_SIZE_K 3
#define SIZE_K SQRT_SIZE_K * SQRT_SIZE_K
#define SIZE_D 5
#define SQRT_SIZE_R 3
#define SIZE_R SQRT_SIZE_R * SQRT_SIZE_R
#define SIZE_T 3
#define SIZE_P_ROWS 3
#define SIZE_P_COLS 4
#define SIZE_P SIZE_P_ROWS * SIZE_P_COLS
#define SIZE_TIME 64
#define SIZE_CORNER_DIST 32
#define SIZE_ROI 4

class Calib_Data_Type{
public:
	Calib_Data_Type(){
		for (int i = 0; i < SIZE_S; i++)
		{
			S_rect_00[i] = -10001;
		}
		for (int i = 0; i < SIZE_R; i++)
		{
			R_rect_00[i] = -10001;
			R_rect_01[i] = -10001;
		}
		for (int i = 0; i < SIZE_P; i++)
		{
			P_rect_00[i] = -10001;
			P_rect_01[i] = -10001;
		}
	};

	char calib_time[SIZE_TIME];
	char corner_dist[SIZE_CORNER_DIST];

	double S_00[SIZE_S];//raw image size; S_00: 1.392000e+03 5.120000e+02
	double K_00[SIZE_K];//raw camera intrinsic parameters : 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
	double D_00[SIZE_D];//distortion coefficients : -3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 - 7.233722e-02
	double R_00[SIZE_R];// : 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
	double T_00[SIZE_T];//translation with respect to the first camera : 2.573699e-16 - 1.059758e-16 1.614870e-16
	int ROI_00[SIZE_ROI];//x,y,width,height
	double S_rect_00[SIZE_S];// : 1.242000e+03 3.750000e+02
	double R_rect_00[SIZE_R];// : 9.999239e-01 9.837760e-03 - 7.445048e-03 - 9.869795e-03 9.999421e-01 - 4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01
	double P_rect_00[SIZE_P];//useful : 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00

	double S_01[SIZE_S];//raw image size; S_00: 1.392000e+03 5.120000e+02
	double K_01[SIZE_K];//raw camera intrinsic parameters : 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
	double D_01[SIZE_D];//distortion coefficients : -3.728755e-01 2.037299e-01 2.219027e-03 1.383707e-03 - 7.233722e-02
	double R_01[SIZE_R];// : 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
	double T_01[SIZE_T];// : -5.370000e-01 4.822061e-03 - 1.252488e-02
	int ROI_01[SIZE_ROI];//x,y,width,height
	double S_rect_01[SIZE_S];// : 1.242000e+03 3.750000e+02
	double R_rect_01[SIZE_R];// : 9.996878e-01 - 8.976826e-03 2.331651e-02 8.876121e-03 9.999508e-01 4.418952e-03 - 2.335503e-02 - 4.210612e-03 9.997184e-01
	double P_rect_01[SIZE_P];// : 7.215377e+02 0.000000e+00 6.095593e+02 - 3.875744e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00

	friend ifstream& operator >> (ifstream &in, Calib_Data_Type &calib_data);
	friend ofstream& operator << (ofstream &of, const Calib_Data_Type &calib_data);
};

//With respecting the format of KITTI data set, forder address as input baseDir, 
//KITTI_Data_Reader will count the number of frames and generate file names of images, 
//oxts files and velodyne files. And it will update automaticly for next frame after 
//reading this frame.
class KITTI_Data_Reader
{
public:
	string baseDir;

	KITTI_Data_Reader();
	KITTI_Data_Reader(string _baseDir);

	bool generateNextDataFileName(int _dataIndex = -1);

	string curImageFileName[4];//left gray, right gray, left color, right color
	string curOxtsFileName;
	string curVelodyneFileName;
	void setMaxIndex(int n) { maxIndex = n; }
	int getMaxIndex() { return maxIndex; }
private:
	void generateDataDirs();
	int getMaxIndexFromTimeStamps();
	string imageDirs[4];//left gray, right gray, left color, right color
	string oxtsDir;//gps data file dir
	string velodyneDir;//laser data dir
	int dataIndex;
	int maxIndex;
};



#endif