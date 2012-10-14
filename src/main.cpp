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
	//ObjectFinder();
};

class MarkerFinder: public ObjectFinder {
private:
	// original image
	//cv::Mat img;
	// vector containing the end points
	// of the detected lines
	std::vector<cv::Vec2i> dots;
public:
	//MarkerFinder() {
	//}	;
	std::vector<cv::Vec2i> find(cv::Mat& image, int radius = 1000) {
		dots.clear();
		int minX = 0, minY = 0;
		for (int q = 0; q < 4; q++) {
			minX = 0, minY = 0;
			brightFinder(image, &minX, &minY, q);
			//if (minX != 0 && minY != 0)
			{
//				// Поищем нет ли рядом точеньки...
//				std::vector<cv::Vec2i>::iterator it2 = dots.begin();
//				bool found = false;
//				while (it2 != dots.end() && !found) {
//					if (((((*it2)[0] - minX) * ((*it2)[0] - minX))
//							+ ((minY - (*it2)[1]) * (minY - (*it2)[1])))
//							< radius) {
//						found = true;
//						break;
//					}
//					++it2;
//
//				}
//				if (!found) {
				cv::Vec2i np;	//= new cv::Vec2i();
				np[0] = minX;
				np[1] = minY;
				dots.push_back(np);
//				}
//				else
//				{
//					(*it2)[0] = ((*it2)[0]+minX)/2;
//					(*it2)[1] = ((*it2)[1]+minY)/2;
//				}
//				minX++;
//				minY++;
			}

		} //while (minX != 0 && minY != 0);

		return dots;
	}
	// Draw the detected lines on an image
	void draw(/*cv::Mat &image, */cv::Mat &target, cv::Scalar color =
			cv::Scalar(128, 128, 255), int thickness = 1, int size = 20) {
		// Draw the lines
		std::vector<cv::Vec2i>::const_iterator it2 = dots.begin();
		int minX, minY;
		int q = 0;
		cv::Vec2i p1, p2, p3, p4;
		while (it2 != dots.end()) {
			minX = (*it2)[0];
			minY = (*it2)[1];
			switch (q) {
			case 0:
				p1[0] = minX;
				p1[1] = minY;
				break;
			case 1:
				p2[0] = minX;
				p2[1] = minY;
				break;
			case 2:
				p3 = (*it2);
				break;
			case 3:
				p4 = (*it2);
				break;
			}
			//brightFinder(image, &minX, &minY);
			if ((minX == 0) && (minY == 0)) {
				++it2;
				q++;
				continue;
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
			++q;

		}
		// рисуем 4 расстояния между точками
		if (p1[0] != 0 && p1[1] != 0 && p2[0] != 0 && p2[1] != 0) {
			cv::Point pts(p1[0], p1[1]), pte(p2[0], p2[1]);
			cv::line(target, pts, pte, color);
			int d = abs((int) (p1[0] - p2[0])) + abs((int) (p1[1] - p2[1]));
			char str[11];
			sprintf(str, " <- %d -> ", d);
			cv::putText(target, str, p1, cv::FONT_HERSHEY_SIMPLEX, 1, color);
			//std::cout << d<< " ";
		}
		if (p3[0] != 0 && p3[1] != 0 && p1[0] != 0 && p1[1] != 0) {
			cv::Point pts(p3[0], p3[1]), pte(p1[0], p1[1]), ptm(
					abs((int) (p1[0] + p3[0])) / 2,
					abs((int) (p1[1] + p3[1])) / 2);
			cv::line(target, pts, pte, color);
			int d = abs((int) (p3[0] - p1[0])) + abs((int) (p3[1] - p1[1]));
			char str[11];
			sprintf(str, " <- %d  ", d);
			cv::putText(target, str, ptm, cv::FONT_HERSHEY_SIMPLEX, 1, color);
			//std::cout << d<< " ";
		}
		if (p3[0] != 0 && p3[1] != 0 && p4[0] != 0 && p4[1] != 0) {
			cv::Point pts(p3[0], p3[1]), pte(p4[0], p4[1]);
			cv::line(target, pts, pte, color);
			int d = abs((int) (p3[0] - p4[0])) + abs((int) (p3[1] - p4[1]));
			char str[11];
			sprintf(str, " <- %d -> ", d);
			cv::putText(target, str, p3, cv::FONT_HERSHEY_SIMPLEX, 1, color);
		}
		if (p2[0] != 0 && p2[1] != 0 && p4[0] != 0 && p4[1] != 0) {
			cv::Point pts(p2[0], p2[1]), pte(p4[0], p4[1]), ptm(
					abs((int) (p4[0] + p2[0])) / 2,
					abs((int) (p4[1] + p2[1])) / 2);
			cv::line(target, pts, pte, color);
			int d = abs((int) (p2[0] - p4[0])) + abs((int) (p2[1] - p4[1]));
			char str[11];
			sprintf(str, " <- %d  ", d);
			cv::putText(target, str, ptm, cv::FONT_HERSHEY_SIMPLEX, 1, color);
			//std::cout << d<< " ";
		}
	}
	//находим самое яркое пятно
	void brightFinder(cv::Mat &image, int* x, int* y, int qadrant) const {
		//Ищем самое яркое пятно
		int min = 175;
		int minX = 0;
		int minY = 0;

		int nl = image.rows;	// number of lines
		// total number of elements per line
		int nc = image.cols * image.channels();
		//cout<<image.channels();
		switch (qadrant) {
		case 0:
			*y = nl / 4;
			nl /= 2;
			*x = nc / 4;
			nc /= 2;
			break;
		case 1:
			*y = nl / 4;
			nl /= 2;
			*x = nc / 2;
			nc -= nc / 4;
			break;
		case 2:
			*y = nl / 2;
			nl -= nl / 4;
			*x = nc / 4;
			nc /= 2;
			break;
		case 3:
			*y = nl / 2;
			nl -= nl / 4;
			*x = nc / 2;
			nc -= nc / 4;
			break;
		}

		for (int j = *y; j < nl; j++) {
			// get the address of row j
			uchar* data = image.ptr<uchar>(j);
			for (int i = *x; i < nc; i++) {
				// process each pixel ---------------------
				if (data[i] >= min) {
					min = data[i];
					minX = i;
					minY = j;
					//j = nl;
					//i = nc;
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

class LineFinder: public ObjectFinder {
private:
// original image
//	cv::Mat img;
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
	void draw(cv::Mat &image, cv::Scalar color = cv::Scalar(32, 255, 32),
			int thickness = 2) {
		// Draw the lines
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end()) {
			cv::Point pt1((*it2)[0], (*it2)[1]);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			cv::line(image, pt1, pt2, color, thickness);
			++it2;
		}
	}
};

class FaceFinder: public ObjectFinder {
private:
//	char* cascadeName =
//			"../../data/haarcascades/haarcascade_frontalface_alt.xml";
//	char* nestedCascadeName =
//			"../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
	//static cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
	std::vector<cv::Rect> faces;
	cv::CascadeClassifier face_cascade;
	bool loaded;
public:
	FaceFinder() {
		loaded = face_cascade.load("haarcascade_frontalface_alt.xml");
		if (!loaded) {
			printf("--(!)Error loading haarcascade_frontalface_alt.xml\n");
		};
	}
	;
	std::vector<cv::Rect> find( cv::Mat image) {
		if ( loaded) face_cascade.detectMultiScale( image, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
		return faces;
	}
	void draw(/*cv::Mat &image, */cv::Mat &target, cv::Scalar color =
			cv::Scalar(128, 128, 255), int size = 20) {
		std::vector<cv::Rect>::const_iterator itc = faces.begin();
		while (itc != faces.end()) {
			cv::Point center( (*itc).x + (*itc).width*0.5, (*itc).y + (*itc).height*0.5 );
		      ellipse( target, center, cv::Size( (*itc).width*0.5, (*itc).height*0.5), 0, 0, 360, color, 2, 8, 0 );
			++itc;
		}
	}
};

class CircleFinder: public ObjectFinder {
private:
	// original image
//	cv::Mat img;
	// vector containing the end points
	// of the detected lines
	std::vector<cv::Vec3f> circles;
public:
	CircleFinder() {
	}
	;
	std::vector<cv::Vec3f> find(cv::Mat image) {
		//cv::GaussianBlur(image, image, cv::Size(5, 5),1.5,1.5);
		cv::medianBlur(image, image, 5);
		cv::HoughCircles(image, circles, CV_HOUGH_GRADIENT, 2// accumulator resolution (size of the image / 2)
				, image.rows / 15	// minimum distance between two circles
				, 200 // Canny high threshold
				, 150 // minimum number of votes
				, 20, 300 // min and max radius
				);
		return circles;
	}
	// Draw the detected lines on an image
	void draw(/*cv::Mat &image, */cv::Mat &target, cv::Scalar color =
			cv::Scalar(128, 128, 255), int size = 20) {
		//this->find(image);
		// Draw the circles
		std::vector<cv::Vec3f>::const_iterator itc = circles.begin();
		while (itc != circles.end()) {
			cv::circle(target, cv::Point((*itc)[0], (*itc)[1]), // circle centre
			(*itc)[2], // circle radius
					color, //cv::Scalar(255), // color
					2);	// thickness
			++itc;
		}
//	    for( size_t i = 0; i < circles.size(); i++ )
//	    {
//	    	cv::Vec3i c = circles[i];
//	    	cv::circle( target, cv::Point(c[0], c[1]), c[2], cv::Scalar(0,0,255), 2, CV_AA);
//	    	cv::circle( target, cv::Point(c[0], c[1]), 2, cv::Scalar(0,255,0), 2, CV_AA);
//	    }
	}
};

int main(int argc, char** argv) {
	cv::Mat image;
	cv::Mat contours;
	if (argc == 2) {
		image = cv::imread(argv[1], 1);
		if (!image.data) {
			printf("not found ");
			return (-1);
		}
		cv::cvtColor(image, image, CV_BGR2GRAY);
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
	int Rmin = 100, Rmax = 255;
	int Gmin = 50, Gmax = 255;
	int Bmin = 50, Bmax = 255;
	for (;;) {
		cv::Mat frame, hsv, frameBitmap;
		cv::Mat grayFrame, ResultFrame;
		cv::Mat gaussGrayFrame;
		cv::Mat edges;
		// create vector of 3 images
		std::vector<cv::Mat> planes;

		LineFinder lFinder;
		MarkerFinder mFinder;
		CircleFinder cFinder;
		FaceFinder fFinder;

		capture >> frame; // get a new frame from camera
		// split 1 3-channel image into 3 1-channel images
		cv::split(frame, planes);
		//planes[2]^=planes[2];
		cv::cvtColor(frame, hsv, CV_BGR2HSV);
		cv::inRange(
				//planes[2]
				hsv, cv::Scalar(Bmin, Gmin, Rmin), cv::Scalar(Bmax, Gmax, Rmax),
				frameBitmap);

		//cv::medianBlur(frameBitmap, frameBitmap, 5); // фильтруем шумы
		cv::cvtColor(frame, grayFrame, CV_BGR2GRAY);
		//cv::equalizeHist(grayFrame, grayFrame);
		frameBitmap = grayFrame;
		//Apply a Gaussian Blur on the gray-level Frame
		//cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 1.5, 1.5);

		//Apply Canny Algorithm
		cv::Canny(grayFrame, // gray-level source image
				edges,          // output contours
				100,              // low threshold
				200, // high threshold// чем больше тем меньше помех. маркер на 250 -супер. Но остальное  -нужно контрастность/свет
				3, false);             // aperture size
		//End Canny Algorithm

		lFinder.setLineLengthAndGap(100, 5);
		lFinder.setMinVote(150);

		//Detect lines
		//std::vector<cv::Vec2i> markers = mFinder.find(frameBitmap);
		cv::GaussianBlur(edges, edges, cv::Size(5, 5), 1.5, 1.5);
		/////////cv::medianBlur(edges, edges, 5); // фильтруем шумы
		std::vector<cv::Vec3f> circles = cFinder.find(grayFrame);
		std::vector<cv::Vec4i> lines   = lFinder.find(edges);
		//std::vector<cv::Rect>  faces   = fFinder.find(grayFrame);
		cv::cvtColor(grayFrame, ResultFrame, CV_GRAY2BGR);
		ResultFrame = frame;
		mFinder.draw(ResultFrame);
		lFinder.draw(ResultFrame);
		cFinder.draw(ResultFrame);
        fFinder.draw(ResultFrame);
//		cv::imshow("Red Dots", frameBitmap);
		cv::imshow("grayFrame", grayFrame);
//		cv::imshow("Camera Preview", frame);

		cv::imshow("edges", edges);
		cv::imshow("ResultFrame", ResultFrame);
//		cv::Mat corners, dilated_corners;
//		cv::preCornerDetect(image, corners, 3);
//		// dilation with 3x3 rectangular structuring element
//		cv::dilate(corners, dilated_corners, cv::Mat(), 1);
//		cv::Mat corner_mask = corners == dilated_corners;
//		cv::imshow("input file", image);
		if (cv::waitKey(30) >= 0)
			break;
	}

	return 0;
}
