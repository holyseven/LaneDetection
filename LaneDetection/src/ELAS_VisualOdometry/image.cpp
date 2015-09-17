#include "image.h"

void pnm_read(std::ifstream &file, char *buf) {
	char doc[BUF_SIZE];
	char c;

	file >> c;
	while (c == '#') {
		file.getline(doc, BUF_SIZE);
		file >> c;
	}
	file.putback(c);

	file.width(BUF_SIZE);
	file >> buf;
	file.ignore();
}

image<uchar> *loadPGM(const char *name) {
	char buf[BUF_SIZE];

	// read header
	std::ifstream file(name, std::ios::in | std::ios::binary);
	pnm_read(file, buf);
	if (strncmp(buf, "P5", 2)) {
		std::cout << "ERROR: Could not read file " << name << std::endl;
		throw pnm_error();
	}

	pnm_read(file, buf);
	int width = atoi(buf);
	pnm_read(file, buf);
	int height = atoi(buf);

	pnm_read(file, buf);
	if (atoi(buf) > UCHAR_MAX) {
		std::cout << "ERROR: Could not read file " << name << std::endl;
		throw pnm_error();
	}

	// read data
	image<uchar> *im = new image<uchar>(width, height);
	file.read((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));

	return im;
}

void savePGM(image<uchar> *im, const char *name) {
	int width = im->width();
	int height = im->height();
	std::ofstream file(name, std::ios::out | std::ios::binary);

	file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
	file.write((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));
}

image<uchar> *loadFromCVMatGray(const cv::Mat &cvImage){
	int width = cvImage.cols;
	int height = cvImage.rows;
	image<uchar> *im = new image<uchar>(width, height);
	for (int i = 0; i < width * height; i++)
		im->data[i] = cvImage.data[i];
	return im;
}

void saveCVMat(image<uchar> *im, const char* name){
	int width = im->width();
	int height = im->height();

	cv::Mat cvImage(height, width, CV_8UC1);

	for (int i = 0; i < height * width; i++)
		cvImage.data[i] = im->data[i];

	cv::imwrite(name, cvImage);
}

void toCVMat(image<uchar> *im, cv::Mat &cvImage){
	int width = im->width();
	int height = im->height();

	cvImage = cv::Mat(height, width, CV_8UC1);

	for (int i = 0; i < height * width; i++)
		cvImage.data[i] = im->data[i];
}