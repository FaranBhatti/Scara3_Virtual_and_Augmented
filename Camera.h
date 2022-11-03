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

	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;

	Mat _cam_virtual_extrinsic;

	void calculate_intrinsic();
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
	Mat _cam_real_intrinsic;
	Mat _cam_real_extrinsic;
	Mat _cam_real_dist_coeff;
	Mat _cam_real_rotation;
	cv::Vec3d rvec, tvec;
	bool track_board;

	/////////////////////////////
	// Lab 5

public:

	/////////////////////////////
	// Lab 3

	void init(Size image_size);
	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);
	void update_settings(Mat& im);

	/////////////////////////////
	// Lab 4
	void init_real_cam(int webcam);
	void get_cam_img(Mat& frame);
	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	void createChArUcoBoard();
	void calibrate_board(int cam_id);
	void update_settings_aruco(Mat& im, cv::Vec3d tvec);

	void transform_to_image_aruco(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);
	void create_pose_aruco(Mat& image);
	//bool detect_board_pose();
	//bool detect_marker_pose();

	/////////////////////////////
	// Lab 5
};

