#include "stdafx.h"

#include "Camera.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1980, 1080)); //1000, 600

	

	//load intrinsic and dist_coefficients from actual camera feed :: 1920x1080
	bool readOk = load_camparam("cam_param.xml", _cam_real_intrinsic, _cam_real_dist_coeff);
	if (!readOk) {
		cerr << "Cannot load cam_param file." << endl;
		return;
	}

	//initialize the aruco board parameteres for lab 4
	size_square = 36; // user specified, mm
	size_mark = 21; // user specified, mm
	board_size = Size(5, 7);
	dictionary_id = aruco::DICT_6X6_250;

	track_board = false;

}

CCamera::~CCamera()
{
}

void CCamera::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 0;

	_cam_setting_x = 52; // units in mm
	_cam_setting_y = 147; // units in mm
	//_cam_setting_x = 0; // units in mm
	//_cam_setting_y = 0; // units in mm
	_cam_setting_z = 500; // units in mm

	_cam_setting_roll = 0; // units in degrees
	_cam_setting_pitch = -180; // units in degrees
	_cam_setting_yaw = -180; // units in degrees

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 4;
	//_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);
	
	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();
}

/////////////////////////////////////////////////////////////////////////
// Lab 3
/////////////////////////////////////////////////////////////////////////

void CCamera::calculate_intrinsic()
{
	//internal properties of the camera; where is it located in space
	_cam_virtual_intrinsic = (Mat1f(3, 3) <<  (1/_pixel_size),							0,							_principal_point.x,
																	0,								(1/_pixel_size),				_principal_point.y,
																	0,										0,									1					 )
									*(Mat1f(3, 4) <<  float(_cam_setting_f)/1000,			0,									0,					0,
																	0,							float(_cam_setting_f)/1000,			0,					0,
																	0,										0,									1,					0);
	//cout << _cam_virtual_intrinsic << endl;
}

void CCamera::calculate_extrinsic()
{
	float _cam_setting_roll_rad = _cam_setting_roll / 57.2958;
	float _cam_setting_pitch_rad = _cam_setting_pitch / 57.2958;
	float _cam_setting_yaw_rad = _cam_setting_yaw / 57.2958;
	//external properties of the camera; describe the pose of the camera with respect to the world
	_cam_virtual_extrinsic = (Mat1f(4, 4) << (cos(_cam_setting_yaw_rad) * cos(_cam_setting_pitch_rad)), ((cos(_cam_setting_yaw_rad) * sin(_cam_setting_pitch_rad) * sin(_cam_setting_roll_rad)) - (sin(_cam_setting_yaw_rad) * cos(_cam_setting_roll_rad))), ((cos(_cam_setting_yaw_rad) * sin(_cam_setting_pitch_rad) * cos(_cam_setting_roll_rad)) + (sin(_cam_setting_yaw_rad) * sin(_cam_setting_roll_rad))), float(_cam_setting_x)/1000,
														  (sin(_cam_setting_yaw_rad) * cos(_cam_setting_pitch_rad)), ((sin(_cam_setting_yaw_rad) * sin(_cam_setting_pitch_rad) * sin(_cam_setting_roll_rad)) + (cos(_cam_setting_yaw_rad) * cos(_cam_setting_roll_rad))), ((sin(_cam_setting_yaw_rad) * sin(_cam_setting_pitch_rad) * cos(_cam_setting_roll_rad)) - (cos(_cam_setting_yaw_rad) * sin(_cam_setting_roll_rad))), float(_cam_setting_y)/1000,
														  (-sin(_cam_setting_pitch_rad))									  , (cos(_cam_setting_pitch_rad) * sin(_cam_setting_roll_rad))																														 , (cos(_cam_setting_pitch_rad) * cos(_cam_setting_roll_rad))																														, float(_cam_setting_z)/1000,
														  (0)																		  , (0)																																																 , (0)																																																, (1));
}

void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	// take in pt3d_mat and multiply it by the intrinsic and extrinsic -> returns 3x1 (u, v, w)
	Mat transform_uvw;
	transform_uvw = _cam_virtual_intrinsic * _cam_virtual_extrinsic.inv() * pt3d_mat;

	// set pt equal to x = u/w etc. pt.x to reference
	pt.x = (transform_uvw.at<float>(0, 0)) / (transform_uvw.at<float>(2, 0));
	pt.y = (transform_uvw.at<float>(1, 0)) / (transform_uvw.at<float>(2, 0));
	//cout << pt << endl;
}

void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	//create this
	vector<Mat> transform_uvw;
	Mat transform_uvw_single;
	cv::Point2f pt;

	for (int i = 0; i < pts3d_mat.size(); i++) {
		transform_uvw_single = _cam_virtual_intrinsic * _cam_virtual_extrinsic.inv() * pts3d_mat.at(i);
		pt.x = (transform_uvw_single.at<float>(0, 0)) / (transform_uvw_single.at<float>(2, 0));
		pt.y = (transform_uvw_single.at<float>(1, 0)) / (transform_uvw_single.at<float>(2, 0));
		pts2d.push_back(pt);
	}
}

void CCamera::update_settings(Mat& im)
{
	Point _camera_setting_window;
	if (track_board == true) {
		_cam_setting_x = tvec[0];
		_cam_setting_y = tvec[1];
		_cam_setting_z = tvec[2];
	}


	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	if (cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board))
	{
		track_board = true;
	}

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}


	// Use this line if only this settings window in use
	//cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}

/////////////////////////////////////////////////////////////////////////
// Lab 4
/////////////////////////////////////////////////////////////////////////

void CCamera::init_real_cam(int webcam)
{
	//start the camera and set the size
	inputVideo.open(webcam);

	//size
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
}

void CCamera::get_cam_img(Mat& frame)
{
	//check if the video is open
	if (inputVideo.isOpened() == true) 
	{
		inputVideo >> frame;
	}
}

bool CCamera::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

bool CCamera::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);
	
	if (!fs.isOpened())
	{
		return false;
	}
	
	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

void CCamera::createChArUcoBoard()
{
	Mat im;
	float size_square = 36; // user specified, mm
	float size_mark = 21; // user specified, mm
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_id);
	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
	
	board->draw(cv::Size(600, 500), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCamera::calibrate_board(int cam_id)
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;
  
	float size_aruco_square = (36 / 1000); // MEASURE THESE, mm
	float size_aruco_mark = (21 / 1000); // MEASURE THESE, mm

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	VideoCapture inputVideo;
	inputVideo.open(cam_id);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920); //1280
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); //720

	// Collect data from live video 
	while (inputVideo.grab()) 
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(draw_im);
		
		// First pass detect markers
		aruco::detectMarkers(im, dictionary, corners, corner_ids, detectorParams, rejected_corners);
		// Second pass detect markers
		aruco::refineDetectedMarkers(im, board, corners, corner_ids, rejected_corners);

		// Refine charuco corners
		if (corner_ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0)
		{
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0)
		{
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}
		
		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0) 
		{
			cout << "Frame captured" << endl;
			calib_corner.push_back(corners);
			calib_id.push_back(corner_ids);
			calib_im.push_back(im);
			calib_im_size = im.size();
		}
	}

	if (calib_id.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(calib_corner.size());
	for (unsigned int i = 0; i < calib_corner.size(); i++) {
		markerCounterPerFrame.push_back((int)calib_corner[i].size());
		for (unsigned int j = 0; j < calib_corner[i].size(); j++) {
			allCornersConcatenated.push_back(calib_corner[i][j]);
			allIdsConcatenated.push_back(calib_id[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, calib_im_size, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

	// prepare data for charuco calibration
	int nFrames = (int)calib_corner.size();
	vector< Mat > allCharucoCorners;
	vector< Mat > allCharucoIds;
	vector< Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for (int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		Mat currentCharucoCorners, currentCharucoIds;
		aruco::interpolateCornersCharuco(calib_corner[i], calib_id[i], calib_im[i], charucoboard,
			currentCharucoCorners, currentCharucoIds, cameraMatrix,
			distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(calib_im[i]);
	}

	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	// calibrate camera using charuco
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, calib_im_size, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = save_camparam("cam_param.xml", cameraMatrix, distCoeffs);
	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Rep Error Aruco: " << arucoRepErr << endl;
	cout << "Calibration saved to " << "cam_param.xml" << endl;

	// show interpolated charuco corners for debugging
		for (unsigned int frame = 0; frame < filteredImages.size(); frame++) 
		{
			Mat imageCopy = filteredImages[frame].clone();
			
			if (calib_id[frame].size() > 0) {

				if (allCharucoCorners[frame].total() > 0) 
				{
					aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],allCharucoIds[frame]);
				}
			}

			imshow("out", imageCopy);
			char key = (char)waitKey(0);
			if (key == 27) break;
	}
}

void CCamera::update_settings_aruco(Mat& im, cv::Vec3d tvec)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");

	//update the x, y, and z sliders here
	_cam_setting_x = tvec[0];
	_cam_setting_y = tvec[1];
	_cam_setting_z = tvec[2];

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board);

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}


	cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}

void CCamera::create_pose_aruco(Mat& image)
{
	///////////////////////////////////////
	//detect the board with CalibrationPose
	///////////////////////////////////////
	{
			Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_id);
			Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
			cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f> > markerCorners;

			//detect markers
			cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

			// if at least one marker detected
			if (markerIds.size() > 0) {
				cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
				std::vector<cv::Point2f> charucoCorners;
				std::vector<int> charucoIds;
				cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, _cam_real_intrinsic, _cam_real_dist_coeff);

				// if at least one charuco corner detected
				if (charucoIds.size() > 0) {
					cv::Scalar color = cv::Scalar(255, 0, 0);
					cv::aruco::drawDetectedCornersCharuco(image, charucoCorners, charucoIds, color);
					bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec);
				}
			}
	}
}

void CCamera::transform_to_image_aruco(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	//variable declarations
	std::vector<Point3f> point_pts;
	cv::Point3f point;

	for (int i = 0; i < pts3d_mat.size(); i++)
	{
		//x value
		point.x = pts3d_mat[i].at<float>(0, 0);
		//y value
		point.y = pts3d_mat[i].at<float>(1, 0);
		//z value
		point.z = pts3d_mat[i].at<float>(2, 0);
		//fill point_pts vector with points
		point_pts.push_back(point);
	}
	projectPoints(point_pts, rvec, tvec, _cam_real_intrinsic, _cam_real_dist_coeff, pts2d);
}

/////////////////////////////////////////////////////////////////////////
// Lab 5
/////////////////////////////////////////////////////////////////////////

/*
void CCamera::update_robot_settings(Mat& im)
{
	//these get initialized to x = 0 and y = 0;
	Point _camera_setting_window = 0;
	Point _robot_setting_window = 0;

	_robot_setting_window.x = 1720;
	_robot_setting_window.y = 0;

	if (track_board == true) {
		_cam_setting_x = tvec[0];
		_cam_setting_y = tvec[1];
		_cam_setting_z = tvec[2];
	}

	//building the two windows
	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");
	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;

	cvui::window(im, _robot_setting_window.x, _robot_setting_window.y, 200, 350, "Robot Settings");
	_robot_setting_window.x = 1725;
	_robot_setting_window.y = 25;


	//first setting
		//camera setting for focal length
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");
		//robot setting for joint0
	cvui::trackbar(im, _robot_setting_window.x, _robot_setting_window.y, 180, &_cam_setting_j0, -180, 180);
	cvui::text(im, _robot_setting_window.x + 180, _robot_setting_window.y + 20, "j0");

	//second setting
		//camera setting for X translation
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");
		//camera setting for joint1
	_robot_setting_window.y += 45;
	cvui::trackbar(im, _robot_setting_window.x, _robot_setting_window.y, 180, &_cam_setting_j1, -180, 180);
	cvui::text(im, _robot_setting_window.x + 180, _robot_setting_window.y + 20, "j1");

	//third setting
		//camera setting for Y translation
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");
		//camera setting for joint2
	_robot_setting_window.y += 45;
	cvui::trackbar(im, _robot_setting_window.x, _robot_setting_window.y, 180, &_cam_setting_j2, -180, 180);
	cvui::text(im, _robot_setting_window.x + 180, _robot_setting_window.y + 20, "j2");

	//fourth setting
		//camera setting for Z translation
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");
	//camera setting for joint2
	_robot_setting_window.y += 45;
	cvui::trackbar(im, _robot_setting_window.x, _robot_setting_window.y, 180, &_cam_setting_j3, 0, 150);
	cvui::text(im, _robot_setting_window.x + 180, _robot_setting_window.y + 20, "j3");

	//fifth setting
		//camera setting for roll
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");


		//reset button for robot settings
	_robot_setting_window.y += 45;
	if (cvui::button(im, _robot_setting_window.x, _robot_setting_window.y, 100, 30, "Reset"))
	{
		init_robot_params();
	}

		//animate button for robot settings

	//sixth setting
		//camera setting for pitch
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	//seventh setting
		//camera setting for yaw
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	//track board pushbutton
	_camera_setting_window.y += 45;
	if (cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board))
	{
		track_board = true;
	}

	//reset button for camera settings
	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}
*/

