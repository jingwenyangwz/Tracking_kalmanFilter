#include "blobs.hpp"
#include <cmath>
#include <algorithm>
#include <utility>
#include <math.h>


 Mat paintBlobImage(cv::Mat frame, Point2f predicted_point, bool labelled, std::vector<cvBlob> bloblist)
{
	cv::Mat localImage;

	frame.copyTo(localImage);
	int height = 50;
	int width = 50;
	if(bloblist.size() > 0){
		height = floor(bloblist[0].h/2);
		width = floor(bloblist[0].w/2);
	}

	if(predicted_point.x != -1){
		Scalar color(255, 255, 255);
		//circle(localImage, predicted_point, 25, Scalar( 0, 0, 255), 5, LINE_8);
		ellipse(localImage, predicted_point, Size(width, height), 0, 0, 360, Scalar( 0, 0, 255), 4, LINE_8);
		if (labelled){
			putText(localImage, "Corrected", predicted_point, FONT_HERSHEY_SIMPLEX, 3, color, 2.0);
		}
		else{
			putText(localImage, "Predicted", predicted_point, FONT_HERSHEY_SIMPLEX, 3, color, 2.0);
		}
	}


	return localImage;
}

void showFinalTrajector(cv::Mat _background, std::vector<Point2f> pointTrajectory, std::vector<Point2f> measuredPoints, std::string trajectoryOutputPath){
	//Point pt = pointTrajectory[0];
	if(measuredPoints.size() >1){
		cv::drawMarker(_background, measuredPoints[0], Scalar( 255, 0, 0),MARKER_CROSS , 6, 4, LINE_8);
		for(int i = 1; i < measuredPoints.size(); i++){
			cv::line(_background, measuredPoints[i-1], measuredPoints[i], Scalar( 255, 0, 0), 2, LINE_8);
			Point2f pt = measuredPoints[i];
			cv::drawMarker(_background, measuredPoints[i], Scalar( 255, 0, 0), MARKER_CROSS, 6, 4, LINE_8);
		}
	}

	if(pointTrajectory.size()>1){
		circle(_background, pointTrajectory[0], 4, Scalar( 0, 0, 255), 1, LINE_8);
		for(int i = 1; i < pointTrajectory.size(); i++){
			cv::line(_background, pointTrajectory[i-1], pointTrajectory[i], Scalar( 0, 0, 255), 2, LINE_8);
			Point2f pt = pointTrajectory[i];
			circle(_background, pt, 4, Scalar( 0, 0, 255), 2, LINE_8);
			//ellipse(_background, pt, Size(50,50), 0, 0, 360, Scalar( 0, 0, 255), 1, LINE_8);
		}
	}

	namedWindow("Tracking results", 1 );
	putText(_background, "Measured x_k", Point(5,25), FONT_HERSHEY_PLAIN, 2.0, Scalar( 255, 0, 0), 2.0);
	putText(_background, "Estimated x_k", Point(5,50), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255), 2.0);
	imshow("Tracking results", _background);

	//imwrite("/home/peter/eclipse-workspace/Ex1_LoadModifySave/trajectory.png", _background);
	imwrite(trajectoryOutputPath, _background);
}

//Our recursive implementation of the flood fill algorithm
//x,y: seed pixel (or current)
//aux: the image we want to iterate on
//maxX,Y minX,Y: stores the smallest rectangle contains our blob
//connectivity: what kind of neighbourhood to examine
//area: stores the amount of pixels, that our blob covers.
void floodFill(int x, int y, Mat& aux, int& maxX, int& maxY, int& minX, int& minY, int connectivity, int &area){
	 if(x >= 0 && y >= 0 && x < aux.rows && y < aux.cols && aux.at<int>(x,y) == 255){
		 if(x >= maxX){
			 maxX = x;
		 }
		 if(y >= maxY){
			 maxY = y;
		 }
		 if(x <= minX){
			 minX = x;
		 }
		 if(y <= minY){
			 minY = y;
		 }
		 area++;
		 aux.at<int>(x, y) = 0;

		 if(connectivity == 8) floodFill(x-1,y-1,aux, maxX, maxY, minX, minY, connectivity, area);

		 floodFill(x,y-1,aux, maxX, maxY, minX, minY, connectivity, area);

		 if(connectivity == 8) floodFill(x+1,y-1,aux, maxX, maxY, minX, minY, connectivity, area);

		 floodFill(x-1,y,aux, maxX, maxY, minX, minY, connectivity, area);
		 floodFill(x+1,y,aux, maxX, maxY, minX, minY, connectivity, area);

		 if(connectivity == 8) floodFill(x-1,y+1,aux, maxX, maxY, minX, minY, connectivity, area);
		 floodFill(x,y,aux, maxX, maxY, minX, minY, connectivity, area);
		 if(connectivity == 8) floodFill(x+1,y+1,aux, maxX, maxY, minX, minY, connectivity, area);
	 }
 }


// Extract the blobs, object candidates for tracking
int extractBlobs(cv::Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity)
{	
	Mat aux; // image to be updated each time a blob is detected (blob cleared)
	fgmask.convertTo(aux,CV_32SC1);
	
	cv::Mat img = cv::Mat::zeros(aux.cols,aux.rows,CV_8UC1);

	bloblist.clear();
			
	//Connected component analysis
	int counter = 0;
	for(int i = 0; i < fgmask.rows; i++){
		for(int j = 0; j < fgmask.cols; j++){
			if(aux.at<int>(i,j) == 255){
				cv::Point seed(j,i);
				cv::Rect rect;
				//cv::floodFill(aux, seed, 0, &rect, cv::Scalar(1), cv::Scalar(1), connectivity);
				//cvBlob blob=initBlob(counter, rect.x, rect.y, rect.width, rect.height);
				int maxX =i;
				int minX = i;
				int maxY = j;
				int minY = j;
				int area = 0;
				//our implementation of flood fill algorithm
				floodFill(i, j, aux, maxX, maxY, minX, minY, connectivity, area);
				counter++;
				cvBlob blob=initBlob(counter, minY, minX, maxY-minY, maxX-minX, area);
				//collect each detected blobs:
				bloblist.push_back(blob);
			}
		}
	}
	return 1;
}

//Selection among object candidates, finds the singel biggest one
Point2f removeSmallBlobs(std::vector<cvBlob> bloblist_in, std::vector<cvBlob> &bloblist_out, int min_width, int min_height)
{
	bloblist_out.clear();
	std::pair<int,int> center(NULL,NULL);
	Point2f pt(NULL, NULL);
	//If there are no detections at all, dont start working
	if (bloblist_in.size() == 0){

		return pt;
	}
	int indexWithBiggestArea = -1;
	int max_area = 0;
	//iterates on the blobs, and finds the biggest one, which is also bigger than the min_width and min_height
	for(int i = 0; i < bloblist_in.size(); i++)
	{
		//int current_area = bloblist_in[i].h * bloblist_in[i].w;
		int current_area = bloblist_in[i].area;
		if ( current_area >  max_area && bloblist_in[i].h > min_height && bloblist_in[i].w > min_width){
			indexWithBiggestArea = i;
			max_area = current_area;
		}

	}
	//if there are none above the threshold
	if(indexWithBiggestArea == -1){
		return pt;
	}
	cvBlob finalBlob = bloblist_in[indexWithBiggestArea];
	//center.first = floor(finalBlob.x+finalBlob.w/2);
	//center.second = floor(finalBlob.y+finalBlob.h/2);

	//put it in a comfortable format
	pt.x = finalBlob.x+finalBlob.w/2;
	pt.y = finalBlob.y+finalBlob.h/2;
	bloblist_out.push_back(finalBlob);
	return pt;
}

//Definition of Kalman filter
//in our
KalmanFilter createConstantVelocityKalmanFilter(){
	KalmanFilter KF(4, 2, 0);
	KF.transitionMatrix = (Mat_<float>(4,4) <<
			1, 0, 1, 0,
			0, 1, 0, 1,
			0, 0, 1, 0,
			0, 0, 0, 1);
	KF.measurementMatrix = (Mat_<float>(2,4) << 1, 0, 0, 0, 0, 1, 0, 0);


	KF.processNoiseCov = (Mat_<float>(4,4) <<
			25, 0, 0, 0,
			0, 25, 0, 0,
			0, 0, 10, 0,
			0, 0, 0, 10); // Q, 4x4


	setIdentity(KF.measurementNoiseCov, Scalar::all(0)); //R, 2x2


	setIdentity(KF.errorCovPost, Scalar::all(1));

	return KF;
}

KalmanFilter createConstantAccelerationKalmanFilter(){
	KalmanFilter KF(6, 2, 0);
	KF.transitionMatrix = (Mat_<float>(6,6) <<
			1, 0, 1, 0, 0.5, 0,
			0, 1, 0, 1, 0, 0.5,
			0, 0, 1, 0, 1, 0,
			0, 0, 0, 1, 0, 1,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1);
	KF.measurementMatrix = (Mat_<float>(2,6) <<
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0);


	KF.processNoiseCov = (Mat_<float>(6,6) <<
				25, 0, 0, 0, 0 , 0,
				0, 25, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0,
				0, 0, 0, 10, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1);

	setIdentity(KF.measurementNoiseCov, Scalar::all(25));
	setIdentity(KF.errorCovPost, Scalar::all(1));

	return KF;
}


void doTheKalmanFiltering(Point2f measurement_pair,
		KalmanFilter &KF,
		std::vector<Point2f> &measured_points,
		std::vector<Point2f> &predicted_points,
		bool &detection_started,
		bool &hasMeasurement,
		Point2f &currentPoint,
		int sizeOfState){
	Mat state(sizeOfState, 1, CV_32F);
	//initilaize the tracking if it hasnt been yet
	if(measurement_pair.x != NULL && !detection_started){
		//it is initialited with the current measurement as center, and random speed
		if(sizeOfState == 4){
			KF.statePost = (Mat_<float>(4,1) << measurement_pair.x, measurement_pair.y, rand() % 5 -2, rand() % 5 -2);
		}

		if(sizeOfState == 6){
			KF.statePost = (Mat_<float>(sizeOfState,1) << measurement_pair.x,
				measurement_pair.y,
				rand() % 5 -2,
				rand() % 5 -2,
				rand() % 3 -1,
				rand() % 3 -1);
		}


		detection_started = true;
	}else if(detection_started){
		// if the tracking is working we predict the next state based on the model
		Mat prediction = KF.predict();
		// depening on if we had measurement, we will corret
		if(measurement_pair.x != NULL){
			measured_points.push_back(measurement_pair);
			Mat measurement = Mat::zeros(2, 1, CV_32F);
			measurement.at<float>(0) = measurement_pair.x;
			measurement.at<float>(1) = measurement_pair.y;
			std::cout << "Measurement\n";
			std::cout << measurement_pair.x << " " << measurement_pair.y << std::endl;
			state = KF.correct(measurement);
			hasMeasurement = true;
		}else {
			// if there were no measurement, the state will be only the prediction
			state = prediction;
		}
		//state = KF.statePost;
		std::cout << "State:" << std::endl;
		for(int i = 0; i < 2; i++){
			std::cout << state.at<float>(i) << " ";
		}
		std::cout << "\n----------" << std::endl;
		currentPoint.x = state.at<float>(0);
		currentPoint.y = state.at<float>(1);
		// we save the predictions
		predicted_points.push_back(currentPoint);
	}
}





