/*
 * DisplayImage.cpp
 *
 *  Created on: 22.07.2012
 *      Author: valentin
 */

#include <cv.h>
#include <highgui.h>
//using namespace cv;
//using namespace std;

//#define PI 3.1415926535898

//class B{    void MyPrint () {     }};
//class B1 extends B{
//    void MyPrint () {
//        //Переопределение метода
//                }
//};
//class B2 extends B{
//    void MyPrint () {
//        //Переопределение метода
//    }
//};

class ObjectFinder {
private:
	// original image
	cv::Mat img;
public:
	ObjectFinder();
};

class MarkerFinder /*extends ObjectFinder */{
private:
	// original image
	cv::Mat img;
	// vector containing the end points
	// of the detected lines
	std::vector<cv::Vec2i> dots;
public:
	//MarkerFinder() {
	//}	;
	std::vector<cv::Vec2i> find(cv::Mat& image, int radius = 1000) {
		dots.clear();
		int minX = 0, minY = 0;
		do {
			brightFinder(image, &minX, &minY);
			if (minX != 0 && minY != 0) {
				// Поищем нет ли рядом точеньки...
				std::vector<cv::Vec2i>::const_iterator it2 = dots.begin();
				bool found = false;
				while (it2 != dots.end() && !found) {
					if (((((*it2)[0] - minX) * ((*it2)[0] - minX))
							+ ((minY - (*it2)[1]) * (minY - (*it2)[1])))
							< radius)
						found = true;
					++it2;

				}
				if (!found) {
					cv::Vec2i np;	//= new cv::Vec2i();
					np[0] = minX;
					np[1] = minY;
					dots.push_back(np);
				}
				minX++;
				minY++;
			}

		} while (minX != 0 && minY != 0);

		return dots;
	}
	// Draw the detected lines on an image
	void draw(/*cv::Mat &image, */cv::Mat &target, cv::Scalar color =
			cv::Scalar(128, 128, 255),int thickness=5, int size = 20) {
		// Draw the lines
		std::vector<cv::Vec2i>::const_iterator it2 = dots.begin();
		int minX, minY;
		while (it2 != dots.end()) {
			minX = (*it2)[0];
			minY = (*it2)[1];
			//brightFinder(image, &minX, &minY);
			if ((minX == 0) && (minY == 0)) {
				return;
			}
			//cout << "found";
			cv::Point ptc(minX, minY);
			cv::circle(target, ptc, size, color, thickness, 8);
			cv::Point pt1(minX - size, minY);
			cv::Point pt2(minX + size, minY);
			cv::line(target, pt1, pt2, color);
			cv::Point pt3(minX, minY - size);
			cv::Point pt4(minX, minY + size);
			cv::line(target, pt3, pt4, color);
			++it2;
		}
	}
	//находим белое пятно
	void brightFinder(cv::Mat &image, int* x, int* y) const {
		//Ищем самое яркое пятно
		int min = 128;
		int minX = 0;
		int minY = 0;

		int nl = image.rows;	// number of lines
		// total number of elements per line
		int nc = image.cols * image.channels();
		//cout<<image.channels();
		if (*y == 0)
			*y = nl / 4;
		nl -= nl / 4;
		if (*x == 0)
			*x = nc / 4;
		nc -= nc / 4;
		for (int j = *y; j < nl; j++) {
			// get the address of row j
			uchar* data = image.ptr<uchar>(j);
			for (int i = *x; i < nc; i++) {
				// process each pixel ---------------------
				if (data[i] <= min) {
					min = data[i];
					minX = i;
					minY = j;
					j = nl;
					i = nc;
				}
			}
		}

		if (nl <= minY || nc <= minX) {
			minX = 0;
			minY = 0;
		}
		*x = minX;
		*y = minY;
	}
};

class LineFinder {
private:
// original image
	cv::Mat img;
// vector containing the end points
// of the detected lines
	std::vector<cv::Vec4i> lines;
// accumulator resolution parameters
	double deltaRho;
	double deltaTheta;
// minimum number of votes that a line
// must receive before being considered
	int minVote;
// min length for a line
	double minLength;
// max allowed gap along the line
	double maxGap;
public:
// Default accumulator resolution is 1 pixel by 1 degree
// no gap, no mimimum length
	LineFinder() :
			deltaRho(1), deltaTheta(CV_PI / 180), minVote(10), minLength(0.), maxGap(
					0.) {
	}

// Set the resolution of the accumulator
	void setAccResolution(double dRho, double dTheta) {
		deltaRho = dRho;
		deltaTheta = dTheta;
	}
// Set the minimum number of votes
	void setMinVote(int minv) {
		minVote = minv;
	}
// Set line length and gap
	void setLineLengthAndGap(double length, double gap) {
		minLength = length;
		maxGap = gap;
	}
// Apply probabilistic Hough Transform
	std::vector<cv::Vec4i> find(cv::Mat& image) {
		lines.clear();
		cv::HoughLinesP(image, lines, deltaRho, deltaTheta, minVote, minLength,
				maxGap);
		return lines;
	}
	// Draw the detected lines on an image
	void draw(cv::Mat &image,
			cv::Scalar color = cv::Scalar(32, 255, 32),int thickness=5) {
		// Draw the lines
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end()) {
			cv::Point pt1((*it2)[0], (*it2)[1]);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			cv::line(image, pt1, pt2, color,thickness);
			++it2;
		}
	}
};

class CircleFinder {
private:
	// original image
	cv::Mat img;
	// vector containing the end points
	// of the detected lines
	std::vector<cv::Vec3f> circles;
public:
	CircleFinder() {
	}
	;
	std::vector<cv::Vec3f> find(cv::Mat& image) {
		//cv::GaussianBlur(image, image, cv::Size(5, 5), 1.5);
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(image, circles, CV_HOUGH_GRADIENT,
				10,	// accumulator resolution (size of the image / 2)
				image.cols/5// minimum distance between two circles
				//200, // Canny high threshold
				//100, // minimum number of votes
				//5, 1000
				); // min and max radius
		return circles;
	}
	// Draw the detected lines on an image
	void draw(/*cv::Mat &image, */cv::Mat &target,
			cv::Scalar color = cv::Scalar(128, 128, 255), int size = 20) {
		//this->find(image);
		// Draw the circles
		std::vector<cv::Vec3f>::const_iterator itc = circles.begin();
		while (itc != circles.end()) {
			cv::circle(target, cv::Point((*itc)[0], (*itc)[1]), // circle centre
			(*itc)[2],// circle radius
			color,//cv::Scalar(255), // color
					8);	// thickness
			++itc;
		}
	}

};

int main(int argc, char** argv) {
	cv::Mat image;
	cv::Mat contours;
	if (argc == 2) {
		image = cv::imread(argv[1], 1);
		if (!image.data) {
			printf("not found ");
			//printf(argv[1]);
			return (-1);
		}
		cv::imshow("input file", image);
	}
	//cv::Mat imageROI= image(cv::Rect(110,260,35,40));
	// Get the Hue histogram
	//int minSat=65;
	//ColorHistogram hc;
	//cv::MatND colorhist=
	//hc.getHueHistogram(imageROI,minSat);//
	// Open the default camera
	cv::VideoCapture capture(0);
	// Check if we succeeded
	if (!capture.isOpened()) {
		std::cout << "Video capture failed, please check the camera."
				<< std::endl;
		return -1;
	} else {
		std::cout << "Video camera capture successful!" << std::endl;
	}

	int Rmin = 5, Rmax = 255;
	for (;;) {
		cv::Mat frame, hsv, frameBitmap;
		cv::Mat grayFrame,ResultFrame;
		cv::Mat gaussGrayFrame;
		cv::Mat edges;
		// create vector of 3 images
		std::vector<cv::Mat> planes;

		LineFinder lFinder;
		MarkerFinder mFinder;
		CircleFinder cFinder;

		capture >> frame; // get a new frame from camera
		// split 1 3-channel image into 3 1-channel images
		cv::split(frame, planes);
		//planes[2]^=planes[2];
		cv::cvtColor(frame, hsv, CV_BGR2HSV);
		cv::inRange(planes[2], cv::Scalar(0, 0, Rmin),
				cv::Scalar(250, 250, Rmax), frameBitmap);
		//frameBitmap.inv();
		//frameBitmap = planes[2];                                                        // Переводим в bitmap
//	        medianBlur(frameBitmap,frameBitmap,5); // фильтруем шумы
		//cv::split(hsv,planes);
//	        cv::inRange(planes[2],cvScalar(Rmin),cvScalar(Rmax),planes[2]);
		//Convert the frame into a gray Frame
		cv::cvtColor(frame, grayFrame, CV_BGR2GRAY);

		//Apply a Gaussian Blur on the gray-level Frame
		cv::GaussianBlur(grayFrame, gaussGrayFrame, cv::Size(17, 17), 1.5, 1.5);

		//Apply Canny Algorithm
		cv::Canny(gaussGrayFrame, // gray-level source image
				edges,          // output contours
				50,              // low threshold
				150,             // high threshold// чем больше тем меньше помех. маркер на 250 -супер. Но остальное  -нужно контрастность/свет
				3,false);             // aperture size
		//End Canny Algorithm

		lFinder.setLineLengthAndGap(30, 10);
		lFinder.setMinVote(5);

		//Detect lines
		std::vector<cv::Vec2i> markers =mFinder.find(frameBitmap);

		std::vector<cv::Vec4i> lines = lFinder.find(edges);
		std::vector<cv::Vec3f> circles =cFinder.find(edges);

		cv::cvtColor(grayFrame,ResultFrame,CV_GRAY2BGR);
		mFinder.draw(ResultFrame);
		lFinder.draw(ResultFrame);
        cFinder.draw(ResultFrame);
		//int threshold=250;
		//cv::threshold(frameBitmap, frameBitmap,threshold, 255, cv::THRESH_BINARY);

		cv::imshow("Red Dots", frameBitmap);


		cv::imshow("Camera Preview", frame);
		cv::imshow("ResultFrame", ResultFrame);
		cv::imshow("edges", edges);

//		cv::Mat corners, dilated_corners;
//		cv::preCornerDetect(image, corners, 3);
//		// dilation with 3x3 rectangular structuring element
//		cv::dilate(corners, dilated_corners, cv::Mat(), 1);
//		cv::Mat corner_mask = corners == dilated_corners;

		if (cv::waitKey(30) >= 0)
			break;
	}

	return 0;
}
