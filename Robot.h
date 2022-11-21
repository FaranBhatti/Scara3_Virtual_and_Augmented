////////////////////////////////////////////////////////////////
// robot.h
// Created Sept 10th, 2022 by Faran Bhatti
// Last edited Nov 20th, 2022
////////////////////////////////////////////////////////////////

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
	int webcam = 0;
	float _time, _time_old, _time_change, _angle_z;

	// robot setting variables
	int _cam_setting_j0;
	int _cam_setting_j1;
	int _cam_setting_j2;
	int _cam_setting_j3;
	int _do_animate;

	// robot reverse kin variables
	int X, Y, Z, theta;
	bool joint_control;
	int _do_animate_joint;

	/////////////////////////////
	// Lab 3

	vector<vector<Mat>> _simple_robot;

	CCamera _virtualcam;

	//CuArm uarm;

	/**
	 * Creating a Box
	 *
	 * @param w, width of the box
	 * @param h, height of the box
	 * @param d, depth of the box
	 * @return vector <Mat> box, which is a box with the origin at the center of the box
	*/
	std::vector<Mat> createBox(float w, float h, float d);

	/**
	 * Creating coordinate
	 * @return vector <Mat>, coordinates at origin (transform by multiplying HT matrix)
	*/
	std::vector<Mat> createCoord();

	/**
	 * Transforming a given set of points with respect to a given matrix
	 *
	 * @param points, vector <Mat> of points being transformed
	 * @param T, matrix which is multiplying each respective point
	*/
	void transformPoints(std::vector<Mat>& points, Mat T);
	
	/**
	 * Drawing a Box in virtual camera environment
	 *
	 * @param im, image from to draw box on
	 * @param box3d, height of the box
	 * @param colour, colour of the box
	 */
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	/**
	 * Drawing a coordinate in virtual camera environment
	 *
	 * @param im, image from to draw box on
	 * @param coord3d, points of the coordinate in 3d
	*/
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	/////////////////////////////
	// Lab 4

	CCamera _realcam;

	/**
	 * Drawing a coordinate in real camera environment
	 *
	 * @param im, image from to draw box on
	 * @param coord3d, points of the coordinate in 3d
	*/
	void drawCoord_realcam(Mat& im, std::vector<Mat> coord3d);
	/**
	 * Drawing a Box in real camera environment
	 *
	 * @param im, image from to draw box on
	 * @param box3d, points of the coordinates in 3d
	 * @param colour, colour of the box
	 */
	void drawBox_realcam(Mat& im, std::vector<Mat> box3d, Scalar colour);

	/////////////////////////////
	// Lab 5

public:
	/**
	 * Creating Homogeneous Transformation Matrix
	 *
	 * @param t, 3d vector with coordinates for translational vector
	 * @param r, 3d vector with values for roll, pitch, and yaw
	 * @return 4x4 matrix containing homogeneous coordinates (r and t vec augmented)
	*/
	Mat createHT(Vec3d t, Vec3d r);

	/////////////////////////////
	// Lab 3

	// Creating the simple robot in virtual camera environment
	void create_simple_robot();
	// Drawing the simple robot in virtual camera environment
	void draw_simple_robot();

	/////////////////////////////
	// Lab 4

	// Updating the angle w/respect to time
	void update_angle();
	// Creating the augmented robot in virtual camera environment
	void create_augmented_robot();
	// Drawing the augmented robot in virtual camera environment
	void draw_augmented_robot();



	/////////////////////////////
   // Lab 5

	/**
	 * Increasing a given variable given a step size
	 *
	 * @param var, variable to update
	 * @param step_size, increment to update variable by
	*/
	void update_var_inc(int& var, int& step_size);
	/**
	 * Decreasing a given variable given a step size
	 *
	 * @param var, variable to update
	 * @param step_size, amount to decrement the variable
	*/
	void update_var_dec(int& var, int& step_size);
	// Initializations for robot
	void init();
	/**
	 * Updating the settings corresponding to the robot
	 *
	 * @param im, image frame to apply the update to
	*/
	void update_robot_settings(Mat& im);
		//part a
			// Creating the Scara Robot in Virtual Camera Environment
			void create_scara_robot();
			// Drawing the Scara Robot in Virtual Camera Environment
			void draw_scara_robot();
		//part b
			// Creating the Scara Robot in Real Camera Environment
			void create_scara_robot_augmented();
			// Drawing the Scara Robot in Real Camera Environment
			void draw_scara_robot_augmented();
			// Forward Kinematics matrix for troubleshooting purposes
			void fkin();

	/////////////////////////////
	// Lab 6

		//part a
			/**
			 * Calculating the position of end effector given a target end effector pose
			 *
			 * @param X, desired X coordinate of end effector
			 * @param Y, desired Y coordinate of end effector
			 * @param Z, desired Z coordinate of end effector
			 * @param theta, desired rotation of end effector
			*/
			void revkin(int& X, int& Y, int& Z, int& theta);
			// Creating the Scara Robot with Reverse Kinematics in Virtual Camera Environment
			void create_robot_revkin();
			// Drawing the Scara Robot with Reverse Kinematics in Virtual Camera Environment
			void draw_robot_revkin();
		//part b
			// Creating the Scara Robot with Reverse Kinematics in Real Camera Environment
			void create_scara_robot_revkin_augmented();
			// Drawing the Scara Robot with Reverse Kinematics in Real Camera Environment
			void draw_scara_robot_revkin_augmented();
};

