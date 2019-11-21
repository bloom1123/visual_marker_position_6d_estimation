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
	"{ @path p 			|| camera calibration chessboard images}"
	"{ @image i 			|| aruco marker image file name}");
	parser.about("This program demostrate the use of visual marker 6d position estimation.\n"
				 "It calibrates the camera, detect de visual markers and estimate its 6D position.\n\n"
				 "Call: ./visual_marker_6d_position_estimation -path calibration_images -image marker_images/1.png\n\n");

	if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

	string calibration_images  = parser.get<string>("p");
	string marker_image = parser.get<string>("i");

	if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }

	cameraCalibration cameraCalibration;

	if (cameraCalibration.calibrate(calibration_images, Size(6, 9), 0.023625, true)) {

		//////////////////////////////
		//	Aruco marker detection	//
		//////////////////////////////

		VideoCapture inputVideo;
		inputVideo.open(0);
		Ptr<aruco::Dictionary> dictionary = getPredefinedDictionary(aruco::DICT_4X4_100);

		while (inputVideo.grab()) {
			Mat image, imageCopy;
			inputVideo.retrieve(image);
			image.copyTo(imageCopy);
			vector<int> ids;
			vector<vector<Point2f>> corners;
			detectMarkers(image, dictionary, corners, ids);

			// if at least one marker detected
			if (ids.size() > 0) 
			{
				aruco::drawDetectedMarkers(imageCopy, corners, ids);
				
				std::vector<cv::Vec3d> rvecs, tvecs;
				aruco::estimatePoseSingleMarkers(corners, 0.12, cameraCalibration.get_cameraMatrix(), cameraCalibration.get_distortionCoefficients(), rvecs, tvecs);

				// draw axis for each marker
				for(int i=0; i<ids.size(); i++)
					aruco::drawAxis(imageCopy, cameraCalibration.get_cameraMatrix(), cameraCalibration.get_distortionCoefficients(), rvecs[i], tvecs[i], 0.1);
			}
				
			imshow("Live", imageCopy);

			char key = (char) cv::waitKey(100);

			if (key == 27)
				break;
		}
	}

	return 0;
}
