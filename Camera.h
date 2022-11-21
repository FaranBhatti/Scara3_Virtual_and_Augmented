////////////////////////////////////////////////////////////////
// camera.h
// Created Sept 10th, 2022 by Faran Bhatti
// Last edited Nov 20th, 2022
////////////////////////////////////////////////////////////////

#pragma once

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace dnn;

class CCamera
{
public:
	CCamera();
	~CCamera();

private:
	/////////////////////////////
	// Lab 3

	// Virtual Camera parameters
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	// Calculating the camera intrinsic matrix
	void calculate_intrinsic();
	// Calculating the camera extrinsic matrix
	void calculate_extrinsic();

	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;

	/////////////////////////////
	// Lab 4
	
	// Real webcam
	float size_square; // user specified, mm
	float size_mark; // user specified, mm
	Size board_size;
	int dictionary_id;
	Size _image_size;
	VideoCapture inputVideo;
	Mat _canvas;
	int webcam = 0;

	// Real Camera parameters
	Mat _cam_real_intrinsic;
	Mat _cam_real_extrinsic;
	Mat _cam_real_dist_coeff;
	Mat _cam_real_rotation;
	cv::Vec3d rvec, tvec;

	// Checkbox for tracking the board
	bool track_board;

public:

	/////////////////////////////
	// Lab 3

	/**
	 * Initializations of camera values
	 *
	 * @param image_size, utilized to determine the principal point
	*/
	void init(Size image_size);
	/**
	 * Transforming the given points into image onto Virtual Camera Environment
	 *
	 * @param pt3d_mat, points to be multiplied by intrinsic and extrinsic camera matrices 
	 * @param pt, points to be stored for the image
	*/
	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	/**
	 * Transforming the given points into image onto Virtual Camera Environment
	 *
	 * @param pt3d_mat, points to be multiplied by intrinsic and extrinsic camera matrices
	 * @param pt, vector of points for the augmentation
	*/
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);
	/**
	 * Updating the parameters for the camera
	 *
	 * @param im, image canvas to update the settings on
	*/
	void update_settings(Mat& im);

	/////////////////////////////
	// Lab 4
	/**
	 * Initializing the webcam
	 *
	 * @param webcam, webcam connected to use (default : 0)
	*/
	void init_real_cam(int webcam);
	/**
	 * Grabbing a frame from the video feed
	 *
	 * @param frame, storing the frame from the video feed
	*/
	void get_cam_img(Mat& frame);
	/**
	 * Saving the camera calibration parameters
	 *
	 * @param filename, name of file
	 * @param cam, camera utilized to calibrate to
	 * @param dist, distortion coefficients
	*/
	bool save_camparam(string filename, Mat& cam, Mat& dist);
	/**
	 * Loading the camera calibration parameters
	 *
	 * @param filename, name of file
	 * @param cam, camera utilized to calibrate to
	 * @param dist, distortion coefficients
	*/
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	// Detecting ChArUcoBoard on the real camera environment
	void createChArUcoBoard();
	/**
	 * Calibrating the ChArUcoBoard to the given camera
	 *
	 * @param cam_id, webcam connected to use (default : 0)
	*/
	void calibrate_board(int cam_id);
	/**
	 * Updating the settings on real camera environment
	 *
	 * @param im, canvas to update settings on
	 * @param tvec, coordinates x, y, z controlled by settings
	*/
	void update_settings_aruco(Mat& im, cv::Vec3d tvec);

	/**
	 * Transforming the given points into image onto Real Camera Environment
	 *
	 * @param pt3d_mat, vector of points to be multiplied by intrinsic and extrinsic camera matrices
	 * @param pt, vector of points on the real camera environment
	*/
	void transform_to_image_aruco(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);
	/**
	 * Creating the pose on the ChArUcoBoard
	 *
	 * @param image, canvas to create the pose on
	*/
	void create_pose_aruco(Mat& image);
};

