#include "KITTI_Data_Reader.h"
#include <fstream>

/*
int GetAllFiles(string path)
{
	long hFile = 0;
	struct _finddata_t fileinfo;
	const char* p = path.append("\\*.png").c_str();
	int numFiles = 0;
	if ((hFile = _findfirst(p, &fileinfo)) != -1)
	{
	
		do
		{
			//if ((fileinfo.attrib &  _A_SUBDIR))
			//{
			//	if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
			//	{
			//		files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			//		GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			//	}
			//}
			//else
			//{
			//	files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			//}
			numFiles++;
		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}
	return numFiles - 1;
}
*/

string generateFileNameFormat(int _index)
{
	if (_index < 0) _index = 0;

	char s[11];
	sprintf_s(s, "%010d", _index);
	return s;
}

int getImagesNum(string path)
{
	const int MAX_NUM = 10000;
	int num = 0;
	for (int i = 0; i < MAX_NUM; i++)
	{
		string s = generateFileNameFormat(i);
		s = path + "\\" + s + ".png";
		ifstream fileStream(s);
		if (!fileStream.is_open())
		{
			num = i;
			break;
		}
	}
	return num - 1;
}



KITTI_Data_Reader::KITTI_Data_Reader(){
	baseDir = "";
}

KITTI_Data_Reader::KITTI_Data_Reader(string _baseDir){
	baseDir = _baseDir;
	dataIndex = 0;
	generateDataDirs();
	maxIndex = getMaxIndexFromTimeStamps();
	if (maxIndex < 0)
		maxIndex = getImagesNum(imageDirs[0] + "data");
}

int KITTI_Data_Reader::getMaxIndexFromTimeStamps(){
	ifstream in(imageDirs[0] + "timestamps.txt");
	if (!in.is_open()) return -1;

	int numLines = 0;
	string lineNoUse;
	while (getline(in, lineNoUse))
		numLines++;

	return numLines - 1;
}

void KITTI_Data_Reader::generateDataDirs(){
	imageDirs[0] = baseDir + "\\image_00\\";
	imageDirs[1] = baseDir + "\\image_01\\";
	imageDirs[2] = baseDir + "\\image_02\\";
	imageDirs[3] = baseDir + "\\image_03\\";

	oxtsDir = baseDir + "\\oxts\\";
	velodyneDir = baseDir + "\\velodyneDir\\";
}

bool KITTI_Data_Reader::generateNextDataFileName(int _index){
	if (_index < 0)
	{
		_index = dataIndex;
		dataIndex++;
	}

	if (_index > maxIndex) return false;

	string fileName = generateFileNameFormat(_index);
	for (int i = 0; i < 4; i++)
		curImageFileName[i] = imageDirs[i] + "data\\" + fileName + ".png";

	curOxtsFileName = oxtsDir + "data\\" + fileName + ".txt";
	curVelodyneFileName = velodyneDir + "data\\" + fileName + ".bin";

	
	return true;
}

ifstream& operator >> (ifstream &in, Oxts_Data_Type &oxts_data)
{
	in >> oxts_data.lat >> oxts_data.lon >> oxts_data.alt;
	in >> oxts_data.roll >> oxts_data.pitch >> oxts_data.yaw;
	in >> oxts_data.vn >> oxts_data.ve;

	in >> oxts_data.vf >> oxts_data.vl >> oxts_data.vu;
	in >> oxts_data.ax >> oxts_data.ay >> oxts_data.az;
	in >> oxts_data.af >> oxts_data.al >> oxts_data.au;
	in >> oxts_data.wx >> oxts_data.wy >> oxts_data.wz;
	in >> oxts_data.wf >> oxts_data.wl >> oxts_data.wu;

	in >> oxts_data.pos_accuracy;

	//vel_accuracy : velocity accuracy(north / east in m / s)
	//navstat : navigation status(see navstat_to_string)
	//numsats : number of satellites tracked by primary GPS receiver
	//posmode : position mode of primary GPS receiver(see gps_mode_to_string)
	//velmode : velocity mode of primary GPS receiver(see gps_mode_to_string)
	//orimode : orientation mode of primary GPS receiver(see gps_mode_to_string)

	return in;
}

ifstream& operator >> (ifstream &in, Calib_Data_Type &calib_data)
{
	in.getline(calib_data.calib_time, SIZE_TIME);
	in.getline(calib_data.corner_dist, SIZE_CORNER_DIST);

	while (!in.eof())
	{
		string prefix;
		in >> prefix;

		if (prefix == "S_00:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_00[i];
		}
		if (prefix == "K_00:")
		{
			for (int i = 0; i < SIZE_K; i++)
				in >> calib_data.K_00[i];
		}
		if (prefix == "D_00:")
		{
			for (int i = 0; i < SIZE_D; i++)
				in >> calib_data.D_00[i];
		}
		if (prefix == "S_01:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_01[i];
		}
		if (prefix == "K_01:")
		{
			for (int i = 0; i < SIZE_K; i++)
				in >> calib_data.K_01[i];
		}
		if (prefix == "D_01:")
		{
			for (int i = 0; i < SIZE_D; i++)
				in >> calib_data.D_01[i];
		}
		if (prefix == "R_01:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_01[i];
		}
		if (prefix == "T_01:")
		{
			for (int i = 0; i < SIZE_T; i++)
				in >> calib_data.T_01[i];
		}

		if (prefix == "ROI_00:")
		{
			for (int i = 0; i < SIZE_ROI; i++)
				in >> calib_data.ROI_00[i];
		}
		if (prefix == "S_rect_00:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_rect_00[i];
		}

		if (prefix == "R_rect_00:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_rect_00[i];
		}

		if (prefix == "P_rect_00:")
		{
			for (int i = 0; i < SIZE_P; i++)
				in >> calib_data.P_rect_00[i];
		}

		if (prefix == "ROI_01:")
		{
			for (int i = 0; i < SIZE_ROI; i++)
				in >> calib_data.ROI_01[i];
		}

		if (prefix == "S_rect_01:")
		{
			for (int i = 0; i < SIZE_S; i++)
				in >> calib_data.S_rect_01[i];
		}

		if (prefix == "R_rect_01:")
		{
			for (int i = 0; i < SIZE_R; i++)
				in >> calib_data.R_rect_01[i];
		}

		if (prefix == "P_rect_01:")
		{
			for (int i = 0; i < SIZE_P; i++)
				in >> calib_data.P_rect_01[i];
		}
	}

	return in;
}

ofstream& operator << (ofstream &of, const Calib_Data_Type &calib_data)
{
	of << calib_data.calib_time << "\n";
	of << calib_data.corner_dist << "\n";

	of << "S_00: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_00[i] << " ";
	of << "\n";

	of << "K_00: ";
	for (int i = 0; i < SIZE_K; i++)
		of << calib_data.K_00[i] << " ";
	of << "\n";

	of << "D_00: ";
	for (int i = 0; i < SIZE_D; i++)
		of << calib_data.D_00[i] << " ";
	of << "\n";

	of << "ROI_00: ";
	for (int i = 0; i < SIZE_ROI; i++)
		of << calib_data.ROI_00[i] << " ";
	of << "\n";

	of << "S_rect_00: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_rect_00[i] << " ";
	of << "\n";

	of << "R_rect_00: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_rect_00[i] << " ";
	of << "\n";

	of << "P_rect_00: ";
	for (int i = 0, n = 0; i < SIZE_P; i++)
	{
		of << calib_data.P_rect_00[i] << " ";
	}
	of << "\n";

	of << "S_01: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_01[i] << " ";
	of << "\n";

	of << "K_01: ";
	for (int i = 0; i < SIZE_K; i++)
		of << calib_data.K_01[i] << " ";
	of << "\n";

	of << "D_01: ";
	for (int i = 0; i < SIZE_D; i++)
		of << calib_data.D_01[i] << " ";
	of << "\n";

	of << "K_01: ";
	for (int i = 0; i < SIZE_K; i++)
		of << calib_data.K_01[i] << " ";
	of << "\n";

	of << "R_01: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_01[i] << " ";
	of << "\n";

	of << "T_01: ";
	for (int i = 0; i < SIZE_T; i++)
		of << calib_data.T_01[i] << " ";
	of << "\n";

	of << "ROI_01: ";
	for (int i = 0; i < SIZE_ROI; i++)
		of << calib_data.ROI_01[i] << " ";
	of << "\n";

	of << "S_rect_01: ";
	for (int i = 0; i < SIZE_S; i++)
		of << calib_data.S_rect_01[i] << " ";
	of << "\n";

	of << "R_rect_01: ";
	for (int i = 0; i < SIZE_R; i++)
		of << calib_data.R_rect_01[i] << " ";
	of << "\n";

	of << "P_rect_01: ";
	for (int i = 0, n = 0; i < SIZE_P; i++)
	{
		of << calib_data.P_rect_01[i] << " ";
	}
	of << "\n";

	return of;
}

