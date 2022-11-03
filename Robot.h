#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;



class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;
	float X, Y, Z;
	int webcam = 0;
	float _time, _time_old, _time_change, _angle_z;

	// robot setting variables
	int _cam_setting_j0;
	int _cam_setting_j1;
	int _cam_setting_j2;
	int _cam_setting_j3;
	int _do_animate;

	/////////////////////////////
	// Lab 3

	vector<vector<Mat>> _simple_robot;

	CCamera _virtualcam;

	//CuArm uarm;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	/////////////////////////////
	// Lab 4

	CCamera _realcam;

	void drawCoord_realcam(Mat& im, std::vector<Mat> coord3d);
	void drawBox_realcam(Mat& im, std::vector<Mat> box3d, Scalar colour);

	/////////////////////////////
	// Lab 5

public:
	Mat createHT(Vec3d t, Vec3d r);

	/////////////////////////////
	// Lab 3

	void create_simple_robot();
	void draw_simple_robot();

	/////////////////////////////
	// Lab 4

	void draw_augmented_robot();
	void create_augmented_robot();
	void update_angle();

	/////////////////////////////
   // Lab 5
	void update_var_inc(int& var, int& step_size);
	void update_var_dec(int& var, int& step_size);
	void init();
	void update_settings(Mat& im);
	Mat fkin(float q1_deg, float q2_deg, float q3_deg, float q4);
		//part a
	void create_scara_robot();
	void draw_scara_robot();
		//part b
	void create_scara_robot_augmented();
	void draw_scara_robot_augmented();
};

