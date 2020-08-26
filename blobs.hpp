/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 *
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

 //class description
/**
 * \class BasicBlob
 * \brief Class to describe a basic blob and associated functions
 *
 * 
 */

#ifndef BLOBS_H_INCLUDE
#define BLOBS_H_INCLUDE

#include "opencv2/opencv.hpp"
using namespace cv; //avoid using 'cv' to declare OpenCV functions and variables (cv::Mat or Mat)


// Maximun number of char in the blob's format
const int MAX_FORMAT = 1024;

/// Type of labels for blobs
typedef enum {	
	UNKNOWN=0, 
	PERSON=1, 
	GROUP=2, 
	CAR=3, 
	OBJECT=4
} CLASS;

struct cvBlob {
	int     ID;  /* blob ID        */
	int   x, y;  /* blob position  */
	int   w, h;  /* blob sizes     */	
	//CLASS label; /* type of blob   */
	//char format[MAX_FORMAT];
	int area;
};

inline cvBlob initBlob(int id, int x, int y, int w, int h, int area)
{
	cvBlob B = { id,x,y,w,h, area};
	return B;
}

//Headers:

//blob drawing functions
Mat paintBlobImage(Mat frame, Point2f predicted_points, bool labelled, std::vector<cvBlob> bloblist);

void floodFill(int x, int y, Mat& aux, int& maxX, int& maxY, int& minX, int& minY, int connectivity, int &area);

//blob extraction functions
int extractBlobs(Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity);
Point2f removeSmallBlobs(std::vector<cvBlob> bloblist_in, std::vector<cvBlob> &bloblist_out, int min_width, int min_height);

void showFinalTrajector(cv::Mat _background, std::vector<Point2f> pointTrajectory, std::vector<Point2f> measuredPoints, std::string trajectoryOutputPath);

KalmanFilter createConstantVelocityKalmanFilter();
KalmanFilter createConstantAccelerationKalmanFilter();

void doTheKalmanFiltering(Point2f measurement_pair, KalmanFilter &KF, std::vector<Point2f> &measured_points, std::vector<Point2f> &predicted_points, bool &detection_started, bool &hasMeasurement, Point2f &currentPoint, int sizeOfState);

#endif

