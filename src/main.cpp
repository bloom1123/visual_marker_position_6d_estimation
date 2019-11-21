#include "cameraCalibration.h"
#include <opencv2/aruco.hpp>

using namespace cv;

int main( int argc, const char** argv )
{
	//////////////////////////////
	//	Command line parser		//
	//////////////////////////////

	CommandLineParser parser(argc, argv,
	"{ help h usage ? 	|| print help message}"
	"{ @path cp 		|| camera calibration chessboard images}"
	"{ @path mp 		|| aruco marker images}");
	parser.about("This program demostrate the use of visual marker 6d position estimation.\n"
				 "It calibrates the camera, detect de visual markers and estimate its 6D position.\n\n"
				 "Call: ./visual_marker_6d_position_estimation -path calibration_images -path marker_images\n\n");

	if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

	string calibration_images  = parser.get<string>("cp");
	string marker_images = parser.get<string>("mp");

	if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
	
	///////////////////////////////////////////////
	//	Calibrate the camera to generate the	//
	//	cameraMatrix and distortionCoefficients	//
	//////////////////////////////////////////////

	cameraCalibration cameraCalibration;

	if (cameraCalibration.calibrate(calibration_images, Size(6, 9), 0.023625, true)) {

		//////////////////////////////
		//	Aruco marker detection	//
		//////////////////////////////

		Ptr<aruco::Dictionary> dictionary = getPredefinedDictionary(aruco::DICT_4X4_100);

		vector<String> fileNames;
		glob(marker_images, fileNames);

		if (fileNames.empty())
			return -1;

		// Arrays to store object points and image points from all the images
		vector<vector<Point2f>> imagePoints;	// 2D points in image plane
		vector<vector<Point3f>> objectPoints;	// 3D points in real world space

		// Step through the list and search for chessboard corners
		while (fileNames.size() > 0) {
			
			Mat image, imageCopy;
			image = imread(fileNames.back());

			image.copyTo(imageCopy);
			vector<int> ids;
			vector<vector<Point2f>> corners;
			detectMarkers(image, dictionary, corners, ids);

			// if at least one marker detected
			if (ids.size() > 0) {
				aruco::drawDetectedMarkers(imageCopy, corners, ids);
					
				std::vector<cv::Vec3d> rvecs, tvecs;
				aruco::estimatePoseSingleMarkers(corners, 0.12, cameraCalibration.get_cameraMatrix(), cameraCalibration.get_distortionCoefficients(), rvecs, tvecs);

				// draw axis for each marker
				for(int i=0; i<ids.size(); i++)
					aruco::drawAxis(imageCopy, cameraCalibration.get_cameraMatrix(), cameraCalibration.get_distortionCoefficients(), rvecs[i], tvecs[i], 0.1);
			}
				
			imshow("Aruco visual marker", imageCopy);

			waitKey(2000);

			fileNames.pop_back();
		}
	}

	return 0;
}
