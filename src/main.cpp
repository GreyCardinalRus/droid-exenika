/*
 * DisplayImage.cpp
 *
 *  Created on: 22.07.2012
 *      Author: valentin
 */

#include <cv.h>
#include <highgui.h>
using namespace cv;
using namespace std;

#define PI 3.1415926535898

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
			deltaRho(1), deltaTheta(PI / 180), minVote(10), minLength(0.), maxGap(
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
	std::vector<cv::Vec4i> findLines(cv::Mat& binary) {
		lines.clear();
		cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength,
				maxGap);
		return lines;
	}
	// Draw the detected lines on an image
	void drawDetectedLines(cv::Mat &image,
			cv::Scalar color = cv::Scalar(255, 255, 255)) {
		// Draw the lines
		std::vector<cv::Vec4i>::const_iterator it2 = lines.begin();
		while (it2 != lines.end()) {
			cv::Point pt1((*it2)[0], (*it2)[1]);
			cv::Point pt2((*it2)[2], (*it2)[3]);
			cv::line(image, pt1, pt2, color);
			++it2;
		}
	}
};
int main(int argc, char** argv) {
//	printf("started \n");
//	Mat image;
//	cv::Mat contours;
//	if (argc != 2) {
//		printf("No image data \n");
//		return -1;
//	}
//	image = imread(argv[1], 1);
//	if (!image.data) {
//		printf("not found ");
//		//printf(argv[1]);
//		return (-1);
//	}
//
//	// Create LineFinder instance
//	LineFinder finder;
//	// Set probabilistic Hough parameters
//	finder.setLineLengthAndGap(100, 20);
//	finder.setMinVote(80);
//	// Detect lines and draw them
//	std::vector<cv::Vec4i> lines = finder.findLines(contours);
////	finder.drawDetectedLines(image);
//	cv::namedWindow("Detected Lines with HoughP");
//	cv::imshow("Detected Lines with HoughP", image);
//	waitKey(0);
//
	   // Open the default camera
	    cv::VideoCapture capture(0);
	    // Check if we succeeded
	    if(!capture.isOpened())
	    {
	        std::cout<<"Video capture failed, please check the camera."<<std::endl;
	        return -1;
	    }else{
	        std::cout<<"Video camera capture successful!"<<std::endl;
	    }
//	    int Rmin = 0;
//	    int Rmax = 256;
//
//	    int Gmin = 0;
//	    int Gmax = 256;
//
//	    int Bmin = 0;
//	    int Bmax = 256;
//
//	    int RGBmax = 256;
int Rmin=5,Rmax=255;
	    for(;;) {
	        cv::Mat frame,hsv,frameBitmap;
	        cv::Mat grayFrame;
	        cv::Mat gaussGrayFrame;
	        cv::Mat edges;
	        // create vector of 3 images
	        std::vector<cv::Mat> planes;

	        LineFinder finder;

	        capture >> frame; // get a new frame from camera
	        // split 1 3-channel image into 3 1-channel images
	        cv::split(frame,planes);
	        //planes[2]^=planes[2];
	        cv::cvtColor(frame,hsv, CV_BGR2HSV );
	        inRange(planes[2],Scalar(0,0,Rmin),Scalar(250,250,Rmax),frameBitmap);
	        //frameBitmap = planes[2];                                                        // Переводим в bitmap
	      // medianBlur(frameBitmap,frameBitmap,5); // фильтруем шумы
	        //cv::split(hsv,planes);
//	        cv::inRange(planes[2],cvScalar(Rmin),cvScalar(Rmax),planes[2]);
	        //Convert the frame into a gray Frame
	        cv::cvtColor(frame, grayFrame, CV_BGR2GRAY);

	        //Apply a Gaussian Blur on the gray-level Frame
	        cv::GaussianBlur(grayFrame, gaussGrayFrame, cv::Size(7,7), 1.5, 1.5);

	        //Apply Canny Algorithm
	        cv::Canny(
	            gaussGrayFrame, // gray-level source image
	            edges,          // output contours
	            0,              // low threshold
	            30,             // high threshold
	            3);             // aperture size
	        //End Canny Algorithm

	        	finder.setLineLengthAndGap(100, 3);
	        	finder.setMinVote(80);
	        //Detect lines
	        std::vector<cv::Vec4i> lines = finder.findLines(edges);
	        //Draw the detected lines
	        finder.drawDetectedLines(frame);

	        //int threshold=250;
	        //cv::threshold(frameBitmap, frameBitmap,threshold, 255, cv::THRESH_BINARY);
	        cv::imshow("Camera Preview", frameBitmap);

	        if(cv::waitKey(30) >= 0) break;
	    }

	return 0;
}
