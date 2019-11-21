#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

class cameraCalibration
{
public:
	cameraCalibration();
	~cameraCalibration();

	bool calibrate(string chessboardImagesPath, Size chessboardSize, float squareLenght, bool showResults);
	Mat undistort(Mat image, bool showResults);
	vector<Mat> get_rvecs();
	vector<Mat> get_tvecs();
	Mat get_cameraMatrix();
	Mat get_distortionCoefficients();

private:

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	Mat cameraMatrix;
	Mat distortionCoefficients;
};

cameraCalibration::cameraCalibration() {
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distortionCoefficients = Mat::zeros(8, 1, CV_64F);
}

cameraCalibration::~cameraCalibration() {}

bool cameraCalibration::calibrate(string chessboardImagesPath, Size chessboardSize, float squareLenght, bool showResults) {
	// Make a list of calibration images
	vector<String> fileNames;
	glob(chessboardImagesPath, fileNames);

	if (fileNames.empty())
		return false;

	// Arrays to store object points and image points from all the images
	vector<vector<Point2f>> imagePoints;	// 2D points in image plane
	vector<vector<Point3f>> objectPoints;	// 3D points in real world space

	// Step through the list and search for chessboard corners
	while (fileNames.size() > 0) {
		Mat image = imread(fileNames.back());
		// Find the chessboard corners
		vector<Point2f> corners;
		if (findChessboardCorners(image, chessboardSize, corners, CALIB_CB_ADAPTIVE_THRESH)) {
			imagePoints.push_back(corners);
			if (showResults) {
				// Draw and display the corners
				drawChessboardCorners(image, chessboardSize, corners, true);
				imshow("Chessboard corners", image);
				waitKey(50);
			}
		}

		fileNames.pop_back();
	}

	// Prepare object points, like (0,0,0), (1,0,0), (2,0,0)....,(8,5,0)
	vector<Point3f> objp;
	for (int i = 0; i < chessboardSize.height; i++) {
		for (int j = 0; j < chessboardSize.width; j++) {
			objp.push_back(Point3f(j * squareLenght, i * squareLenght, 0.0f));
		}
	}
	objectPoints.push_back(objp);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	return calibrateCamera(objectPoints, imagePoints, chessboardSize, cameraMatrix, distortionCoefficients, rvecs, tvecs);
}
	
Mat cameraCalibration::undistort(Mat image, bool showResults) {
	Mat image_undistorted;
	cv::undistort(image, image_undistorted, cameraMatrix, distortionCoefficients);

	if (showResults) {
		// Draw and display the corners
		imshow("Distorted image", image);
		imshow("Undistorted image", image_undistorted);
	}

	return image_undistorted;
}

vector<Mat> cameraCalibration::get_rvecs()
{
	return rvecs;
}

vector<Mat> cameraCalibration::get_tvecs()
{
	return tvecs;
}

Mat cameraCalibration::get_cameraMatrix()
{
	return cameraMatrix;
}

Mat cameraCalibration::get_distortionCoefficients()
{
	return distortionCoefficients;
}
