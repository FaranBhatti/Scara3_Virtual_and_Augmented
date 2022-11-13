#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"
#define M_PI           3.14159265358979323846  /* pi */

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing

	//For lab 3: 1000, 600
	//For lab 4: 1920, 1080
	_image_size = Size(1920, 1080);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	//startup the webcam
	_realcam.init_real_cam(0);

	//timing
	_time_old = getTickCount() / getTickFrequency();

	//initializations for robot parameters
	init();

  ///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	_angle_z = 0;

	_do_animate = 0;
	_cam_setting_j0 = 0;
	_cam_setting_j1 = 0;
	_cam_setting_j2 = 0;
	_cam_setting_j3 = 0;

	//joint parameters
	X = 0;
	Y = 0;
	Z = 0;
	theta = 0;
	joint_control = false;
	_do_animate_joint = 0;
}

/////////////////////////////
// Lab 3

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	float roll = r[0] / 57.2958;
	float pitch = r[1] / 57.2958;
	float yaw = r[2] / 57.2958;
	float tx = t[0];
	float ty = t[1];
	float tz = t[2];

	// r[0] = roll, r[1] = pitch, r[2] = yaw
	// t[0] = x-coord, t[1] = y-coord, t[2] = z-coord
	//same as extrinsic in camera but variables are different ie. roll pitch yaw are r vectors
	return (Mat1f(4, 4) << (cos(yaw) * cos(pitch))		, ((cos(yaw) * sin(pitch) * sin(roll)) - (sin(yaw) * cos(roll)))		, ((cos(yaw) * sin(pitch) * cos(roll)) + (sin(yaw) * sin(roll)))		, tx,
								  (sin(yaw) * cos(pitch))		, ((sin(yaw) * sin(pitch) * sin(roll)) + (cos(yaw) * cos(roll)))		, ((sin(yaw) * sin(pitch) * cos(roll)) - (cos(yaw) * sin(roll)))		, ty,
								  (-sin(pitch))					, (cos(pitch) * sin(roll))															, (cos(pitch) * cos(roll))															, tz,
								  (0)									, (0)																						, (0)																						, (1));
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;
	
	//virtual camera = 0.072, real camera = 72
	float axis_length = 0.072;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}

void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };
	 
	_virtualcam.transform_to_image(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1.5);
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 2); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 2); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 2); // Z=BLUE
}

void CRobot::create_simple_robot()
{
	//vectors of all the boxes
	//						           x,     y,   z
	cv::Vec3d base_trans = {	  0,	   0,	  0 };		//base : red
	cv::Vec3d body_trans = {	  0,	0.05,	  0 };		//body : red
	cv::Vec3d head_trans = {	  0,	0.15,   0 };		//head : red
	cv::Vec3d Larm_trans = { -0.05,	0.10,	  0 };		//left arm: blue
	cv::Vec3d Rarm_trans = {  0.05,  0.10,	  0 };		//right arm: green

	//					  roll, pitch, yaw
	cv::Vec3d rot = { 0,	    0,    0 };	//rotation vector

	//creating all the boxes
	std::vector<Mat> base = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> body = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> head = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> Larm = createBox(0.05, 0.05, 0.05);
	std::vector<Mat> Rarm = createBox(0.05, 0.05, 0.05);

	//translate and then rotate
	transformPoints(base, createHT(base_trans, rot));
	transformPoints(body, createHT(body_trans, rot));
	transformPoints(head, createHT(head_trans, rot));
	transformPoints(Larm, createHT(Larm_trans, rot));
	transformPoints(Rarm, createHT(Rarm_trans, rot));
	
	// drawing the boxes here
	drawBox(_canvas, base, CV_RGB(255, 0, 0));
	drawBox(_canvas, body, CV_RGB(255, 0, 0));
	drawBox(_canvas, head, CV_RGB(255, 0, 0));
	drawBox(_canvas, Larm, CV_RGB(0, 0, 255));
	drawBox(_canvas, Rarm, CV_RGB(0, 255, 0));

}

void CRobot::draw_simple_robot()
{
	//grab the image
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	
	//creating and drawing origin coordinates here
	std::vector<Mat> Origin = createCoord();
	drawCoord(_canvas, Origin);

	//update the settings
	_virtualcam.update_settings(_canvas);

	//create and draw the robot
	create_simple_robot();

	//draw the canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

/////////////////////////////
// Lab 4

void CRobot::drawCoord_realcam(Mat& im, std::vector<Mat> coord3d)
{
	vector<Point2f> point;

	_realcam.transform_to_image_aruco(coord3d, point);

	line(im, point.at(0), point.at(1), CV_RGB(255, 0, 0), 1); // X=RED
	line(im, point.at(0), point.at(2), CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, point.at(0), point.at(3), CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::drawBox_realcam(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_realcam.transform_to_image_aruco(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::update_angle() 
{
	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;
	_angle_z += 5;
}

void CRobot::create_augmented_robot()
{
	//vectors of all the boxes
	//						 x,   y,    z
	cv::Vec3d vec1 = { 0,	0,	  0 };		//base : red
	cv::Vec3d vec2 = { 0,	0,	  50 };		//body : red
	cv::Vec3d vec3 = { 0,	0,	  150 };		//head : red
	cv::Vec3d vec4 = { 50,	0,	  100 };		//left arm: blue
	cv::Vec3d vec5 = { -50,	0,	  100 };		//right arm: green

	//								roll, pitch, yaw
	cv::Vec3d rot = { 0,	  0,    _angle_z };	//rotation vector

	//creating all the boxes
	std::vector<Mat> box1 = createBox(50, 50, 50);
	std::vector<Mat> box2 = createBox(50, 50, 50);
	std::vector<Mat> box3 = createBox(50, 50, 50);
	std::vector<Mat> box4 = createBox(50, 50, 50);
	std::vector<Mat> box5 = createBox(50, 50, 50);

	//translate
	transformPoints(box1, createHT(vec1, 0));
	transformPoints(box2, createHT(vec2, 0));
	transformPoints(box3, createHT(vec3, 0));
	transformPoints(box4, createHT(vec4, 0));
	transformPoints(box5, createHT(vec5, 0));

	//then rotate
	transformPoints(box1, createHT(0, rot));
	transformPoints(box2, createHT(0, rot));
	transformPoints(box3, createHT(0, rot));
	transformPoints(box4, createHT(0, rot));
	transformPoints(box5, createHT(0, rot));


	//drawing the boxes
	drawBox_realcam(_canvas, box1, CV_RGB(255, 0, 0));
	drawBox_realcam(_canvas, box2, CV_RGB(255, 0, 0));
	drawBox_realcam(_canvas, box3, CV_RGB(255, 0, 0));
	drawBox_realcam(_canvas, box4, CV_RGB(0, 0, 255));
	drawBox_realcam(_canvas, box5, CV_RGB(0, 255, 0));
}

void CRobot::draw_augmented_robot()
{
	
	//update
	update_angle();

	//grab image
	_realcam.get_cam_img(_canvas);

	//get pose of board
	_realcam.create_pose_aruco(_canvas);

	//draw a coordinate
	std::vector<Mat> Origin = createCoord();
	drawCoord_realcam(_canvas, Origin);

	//draw robot
	create_augmented_robot();

	//show sliders
	_realcam.update_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
	
	//_realcam.calibrate_board(0);

}

/////////////////////////////
// Lab 5
void CRobot::update_var_inc(int& var, int& step_size)
{
	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;
	var += step_size;
}

void CRobot::update_var_dec(int& var, int& step_size)
{
	//update time
	_time = getTickCount() / getTickFrequency();
	_time_change = _time - _time_old;
	_time_old = _time;
	var -= step_size;
}

void CRobot::update_robot_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	//robot setting for joint0
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_cam_setting_j0, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j0");

	//camera setting for joint1
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_cam_setting_j1, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j1");

	//camera setting for joint2
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_cam_setting_j2, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j2");

	//camera setting for joint3
	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_cam_setting_j3, 0, 150);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j3");

	//reset button for robot settings
	_setting_window.y += 45;
	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Reset"))
	{
		init();
	}

	_setting_window.x += 100;
	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	_setting_window.x -= 100;
	_setting_window.y += 30;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &X, -300, 300);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "X");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &Y, -300, 300);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Y");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &Z, -300, 300);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Z");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &theta, -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "Th");

	_setting_window.y += 65;
	if (cvui::checkbox(im, _setting_window.x, _setting_window.y, "Joint Ctrl", &joint_control))
	{
		//toggle for determining pose(j0, j1, j2, j3) given pose(X, Y, Z, theta)
	}

	_setting_window.x += 90;
	_setting_window.y -= 15;
	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate_joint = 1;
	}
	
	if (_do_animate != 0)
	{
		int step_size = 5;
		if (_do_animate == 1)
		{
			// state 1
			if (1)	{
				//going fwd 180 deg for j0
				update_var_inc(_cam_setting_j0, step_size);

				if (_cam_setting_j0 == 180) { _do_animate = 2; }
			}
		}
		else if (_do_animate == 2) {
			// state 2
			if (1) 
			{ 
				//going back 0 deg for j0
				update_var_dec(_cam_setting_j0, step_size);
				if (_cam_setting_j0 == 0) { _do_animate = 3; }
			}
		}
		else if (_do_animate == 3) {
			// state 3
			if (1)
			{
				//going back -180 deg for j0
				update_var_dec(_cam_setting_j0, step_size);
				if (_cam_setting_j0 == -180) { _do_animate = 4; }
			}
		}
		else if (_do_animate == 4) {
			// state 4
			if (1)
			{
				//going back 0 deg for j0
				update_var_inc(_cam_setting_j0, step_size);
				if (_cam_setting_j0 == 0) { _do_animate = 5; }
			}
		}
		else if (_do_animate == 5) {
			// state 5
			if (1)
			{
				//going fwd 180 deg for j1
				update_var_inc(_cam_setting_j1, step_size);
				if (_cam_setting_j1 == 180) { _do_animate = 6; }
			}
		}
		else if (_do_animate == 6) {
			// state 6
			if (1)
			{
				//going back to 0 deg for j1
				update_var_dec(_cam_setting_j1, step_size);
				if (_cam_setting_j1 == 0) { _do_animate = 7; }
			}
		}
		else if (_do_animate == 7) {
			// state 7
			if (1)
			{
				//going back -180 deg for j1
				update_var_dec(_cam_setting_j1, step_size);
				if (_cam_setting_j1 == -180) { _do_animate = 8; }
			}
		}
		else if (_do_animate == 8) {
			// state 8
			if (1)
			{
				//going back to 0 deg for j1
				update_var_inc(_cam_setting_j1, step_size);
				if (_cam_setting_j1 == 0) { _do_animate = 9; }
			}
		}
		else if (_do_animate == 9) {
			// state 9
			if (1)
			{
				//going fwd 180 deg for j2
				update_var_inc(_cam_setting_j2, step_size);
				if (_cam_setting_j2 == 180) { _do_animate = 10; }
			}
		}
		else if (_do_animate == 10) {
			// state 10
			if (1)
			{
				//going back to 0 deg for j2
				update_var_dec(_cam_setting_j2, step_size);
				if (_cam_setting_j2 == 0) { _do_animate = 11; }
			}
		}
		else if (_do_animate == 11) {
			// state 11
			if (1)
			{
				//going back -180 deg for j2
				update_var_dec(_cam_setting_j2, step_size);
				if (_cam_setting_j2 == -180) { _do_animate = 12; }
			}
		}
		else if (_do_animate == 12) {
			// state 12
			if (1)
			{
				//going back 0 deg for j2
				update_var_inc(_cam_setting_j2, step_size);
				if (_cam_setting_j2 == 0) { _do_animate = 13; }
			}
		}
		else if (_do_animate == 13) {
		// state 13
		if (1)
		{
			//going down 150mm for j3
			update_var_inc(_cam_setting_j3, step_size);
			if (_cam_setting_j3 == 150) { _do_animate = 14; }
		}
		}
		else if (_do_animate == 14) {
			// state 14
			if (1) {
				update_var_dec(_cam_setting_j3, step_size);
				if (_cam_setting_j3 == 0) 
				{
					_do_animate = 0;
					init();
				}
			}
		}
	}

	cvui::update();
}

void CRobot::fkin()
{
	//variables
		//a variables distance along the x-axis
	float a = 0.15;
	float q1 = _cam_setting_j0 / 57.2958;
	float q2 = _cam_setting_j1 / 57.2958;
	float q3 = _cam_setting_j2 / 57.2958;
	float q4 = _cam_setting_j3;
	Mat fwdkin;
	//0.225 - 0.175 - 0.075 - j2_trans[0] 
	
	
	//Ry
	//encapsulation for homogeneous matrix
	float phi_3 = (cos(q1) * sin(q2)) + (cos(q2) * sin(q1));
	float phi_2 = (cos(q1) * cos(q2)) - (sin(q1) * sin(q2));
	float phi_1 = (cos(q3) * phi_2) - (sin(q3) * phi_3);

	fwdkin =	(Mat1f(4, 4) <<	(phi_1),												(0),			((cos(q3) * phi_3) + (sin(q3) * phi_2)),	((a * phi_2) + (a * cos(q1)) - 0.3),
										(0),													(1),			(0),													(-q4),
										((-cos(q3) * phi_3) - (sin(q3) * phi_2)), (0),			(phi_1),												(-(a * phi_3) - (a * sin(q1))),
										(0),													(0),			(0),													(1));
	cout << fwdkin << endl;
}

void CRobot::create_scara_robot()
{
	//when you draw on to your video scale everything by 1000

	//translation vectors of all the links
	//each link is 15cm in width, 5cm in height, 5cm in depth
	//										x,			y,			z
	cv::Vec3d plat_trans =		{	0,			0.075,	0 };
	cv::Vec3d j0_trans =			{ 0.075,		-0.05,	0 };
	cv::Vec3d j1_trans =			{  0,			0,			0 };
	cv::Vec3d j2_trans =			{ -0.15,		-0.025,	0 };
	//rotational vectors
	//							roll, pitch, yaw
	cv::Vec3d j0_rot = {	0,	    0,    0 };
	cv::Vec3d j1_rot = { 0,	    0,    0 };
	cv::Vec3d j2_rot = { 0,	    0,    0 };

	//drawing the rigid bodies and platform
	std::vector<Mat> platform = createBox(0.05, 0.15, 0.05);	//platform :: white
	std::vector<Mat> rb0 =		 createBox(0.15, 0.05, 0.05);	//j0::red
	std::vector<Mat> rb1 =		 createBox(0.15, 0.05, 0.05);	//j1::green
	std::vector<Mat> rb2 =		 createBox(0.15, 0.05, 0.05);	//j2::blue

	//transformation matrices for placing the boxes
	Mat plat = createHT(plat_trans, 0);
	Mat T_rb0 = plat * createHT(j0_trans, j0_rot);
	Mat T_rb1 = T_rb0 * createHT(j1_trans, j1_rot);
	Mat T_rb2_rot = T_rb1 * createHT(j2_trans, j2_rot);

	//platform
		//putting it into the frame
	transformPoints(platform, plat);
		//updating the box point
	Mat PLAT0 = createHT(Vec3d{ 0, 0, 0 }, Vec3d{ 0, 0, 0 });
		//creating the coordinate for platform
	std::vector<Mat> plat_coord = createCoord();
		//transforming the coordinates
	transformPoints(plat_coord, PLAT0);
		//drawing the coordinates 
	drawCoord(_canvas, plat_coord);
		//drawing the box
	transformPoints(platform, PLAT0);

	//1st rigid body
		//putting it into the frame
	transformPoints(rb0, T_rb0);
		//updating the box point
	Mat TRB0 = createHT(Vec3d{ 0, 0.15, 0 }, Vec3d{ 0, float(_cam_setting_j0), 0 });
		//creating the coordinate for rb0
	std::vector<Mat> rb0_coord = createCoord();
		//transforming the coordinates
	transformPoints(rb0_coord, TRB0);
		//drawing the coordinates
	drawCoord(_canvas, rb0_coord);
		//drawing the box
	transformPoints(rb0, TRB0);

	//2nd rigid body
		//putting it into the frame
	transformPoints(rb1, T_rb1);
		//updating the box point
	Mat TRB1 = createHT(Vec3d{ 0.15, 0, 0 }, Vec3d{ 0, float(_cam_setting_j1), 0 });
		//creating the coordinate for rb1
	std::vector<Mat> rb1_coord = createCoord();
		//transforming the coordinates
	transformPoints(rb1_coord, TRB0 * TRB1);
		//drawing the coordinates
	drawCoord(_canvas, rb1_coord);
		//drawing the box
	transformPoints(rb1, TRB0*TRB1);

	//3rd rigid body
		//putting it into the frame
	transformPoints(rb2, T_rb2_rot);
		//updating the box points
	Mat TRB2_rot_and_trans = createHT(Vec3d{ 0.175, -(float(_cam_setting_j3) / 1000), 0 }, Vec3d{ float(_cam_setting_j2), 0, 270 });
	Mat TRB2_trans = createHT(Vec3d{ 0.175, -(float(_cam_setting_j3) / 1000), 0 }, Vec3d{ 0, 0, 270 });
	Mat TRB2_rot = createHT(Vec3d{ 0.175, 0, 0 }, Vec3d{ float(_cam_setting_j2), 0, 270 });
		//creating the coordinates for rb2
	std::vector<Mat> rb2_trans_coord = createCoord();
	std::vector<Mat> rb2_rot_coord = createCoord();
		//transforming the coordinates
	transformPoints(rb2_trans_coord, TRB0 * TRB1 * TRB2_trans);
	transformPoints(rb2_rot_coord, TRB0 * TRB1 * TRB2_rot);
		//drawing the coordinates
	drawCoord(_canvas, rb2_trans_coord);
	drawCoord(_canvas, rb2_rot_coord);
		//drawing the box
	transformPoints(rb2, TRB0 * TRB1 * TRB2_rot_and_trans);

	//drawing the rigid bodies
	drawBox(_canvas, rb0, CV_RGB(255, 0, 0));
	drawBox(_canvas, rb1, CV_RGB(0, 255, 0));
	drawBox(_canvas, rb2, CV_RGB(0, 0, 255));
	drawBox(_canvas, platform, CV_RGB(255, 255, 255));
}

void CRobot::draw_scara_robot()
{
	//grab image (update when you do part 2 of lab on video)
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw robot
	create_scara_robot();

	//show sliders
	_virtualcam.update_settings(_canvas);
	update_robot_settings(_canvas);

	//print fkin results
	fkin();

	//show canvas (update when you do part 2 of lab on video)
	cv::imshow(CANVAS_NAME, _canvas);
}

void CRobot::create_scara_robot_augmented()
{
	//when you draw on to your video scale everything by 1000

	//translation vectors of all the links
	//each link is 15cm in width, 5cm in height, 5cm in depth
	//										x,			y,			z
	cv::Vec3d plat_trans =	{	0,			0,	75 };
	cv::Vec3d j0_trans =		{ 75,		0,	-50 };
	cv::Vec3d j1_trans =		{ 0,			0,			0 };
	cv::Vec3d j2_trans =		{ -150,		0,	-25 };
	//rotational vectors
	//							roll, pitch, yaw
	cv::Vec3d j0_rot = { 0,	    0,    0 };
	cv::Vec3d j1_rot = { 0,	    0,    0 };
	cv::Vec3d j2_rot = { 0,	    0,    0 };

	//drawing the rigid bodies and platform
	std::vector<Mat> platform = createBox(50, 50, 150);	//platform :: white
	std::vector<Mat> rb0 = createBox(150, 50, 50);	//j0::red
	std::vector<Mat> rb1 = createBox(150, 50, 50);	//j1::green
	std::vector<Mat> rb2 = createBox(150, 50, 50);	//j2::blue

	//transformation matrices for placing the boxes
	Mat plat = createHT(plat_trans, Vec3d{ 0, 0, 0 });
	Mat T_rb0 = plat * createHT(j0_trans, j0_rot);
	Mat T_rb1 = T_rb0 * createHT(j1_trans, j1_rot);
	Mat T_rb2_rot = T_rb1 * createHT(j2_trans, j2_rot);

	//platform
		//putting it into the frame
	transformPoints(platform, plat);
	//updating the box point
	Mat PLAT0 = createHT(Vec3d{ 0, 0, 0 }, Vec3d{ 0, 0, 0 });
	//creating the coordinate for platform
	std::vector<Mat> plat_coord = createCoord();
	//transforming the coordinates
	transformPoints(plat_coord, PLAT0);
	//drawing the coordinates 
	drawCoord_realcam(_canvas, plat_coord);
	//drawing the box
	transformPoints(platform, PLAT0);

	//1st rigid body
		//putting it into the frame
	transformPoints(rb0, T_rb0);
	//updating the box point
	Mat TRB0 = createHT(Vec3d{ 0, 0, 150 }, Vec3d{ 0, 0, float(_cam_setting_j0) });
	//creating the coordinate for rb0
	std::vector<Mat> rb0_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb0_coord, TRB0);
	//drawing the coordinates
	drawCoord_realcam(_canvas, rb0_coord);
	//drawing the box
	transformPoints(rb0, TRB0);

	//2nd rigid body
		//putting it into the frame
	transformPoints(rb1, T_rb1);
	//updating the box point
	Mat TRB1 = createHT(Vec3d{ 150, 0, 0 }, Vec3d{ 0, 0, float(_cam_setting_j1) });
	//creating the coordinate for rb1
	std::vector<Mat> rb1_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb1_coord, TRB0 * TRB1);
	//drawing the coordinates
	drawCoord_realcam(_canvas, rb1_coord);
	//drawing the box
	transformPoints(rb1, TRB0 * TRB1);

	//3rd rigid body
		//putting it into the frame
	transformPoints(rb2, T_rb2_rot);
	//updating the box points
	Mat TRB2_rot_and_trans = createHT(Vec3d{ 175, 0, -float(_cam_setting_j3) }, Vec3d{ float(_cam_setting_j2), 90, 0 });
	Mat TRB2_trans = createHT(Vec3d{ 175, 0, -float(_cam_setting_j3) }, Vec3d{ 0, 90, 0 });
	Mat TRB2_rot = createHT(Vec3d{ 175, 0, 0 }, Vec3d{ float(_cam_setting_j2), 90, 0 });
	//creating the coordinates for rb2 
	std::vector<Mat> rb2_trans_coord = createCoord();
	std::vector<Mat> rb2_rot_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb2_trans_coord, TRB0 * TRB1 * TRB2_trans);
	transformPoints(rb2_rot_coord, TRB0 * TRB1 * TRB2_rot);
	//drawing the coordinates
	drawCoord_realcam(_canvas, rb2_trans_coord);
	drawCoord_realcam(_canvas, rb2_rot_coord);
	//drawing the box
	transformPoints(rb2, TRB0 * TRB1 * TRB2_rot_and_trans);

	//drawing the rigid bodies
	drawBox_realcam(_canvas, rb0, CV_RGB(255, 0, 0));
	drawBox_realcam(_canvas, rb1, CV_RGB(0, 255, 0));
	drawBox_realcam(_canvas, rb2, CV_RGB(0, 0, 255));
	drawBox_realcam(_canvas, platform, CV_RGB(255, 255, 255));
}

void CRobot::draw_scara_robot_augmented()
{
	//grab image
	_realcam.get_cam_img(_canvas);

	//get pose of board
	_realcam.create_pose_aruco(_canvas);

	//draw robot
	create_scara_robot_augmented();

	//show sliders
	_realcam.update_settings(_canvas);
	update_robot_settings(_canvas);

	//show canvas
	cv::imshow(CANVAS_NAME, _canvas);
}

/////////////////////////////
// Lab 6

void CRobot::revkin(int& X, int& Y, int& Z, int& theta)
{
	float Xf = float(X) / 1000;
	float Yf = float(Y) / 1000;
	float Zf = float(Z) / 1000;
	float thetaf = theta;
	
	//if (X != 0 && X != 300 && X != -300 && Y != 0 && Y != 300 && Y != -300)
	{
		//solving for j0 joint
			//var for simplifying eqn for j0
			float negnumb = (-100 * pow(Xf, 4)) - (200 * pow(Xf, 2) * pow(Yf, 2)) + (9 * pow(Xf, 2)) - (100 * pow(Yf, 4)) + (9 * pow(Yf, 2));
			float var = return_real(negnumb);
			//float var = sqrt((-100 * pow(Xf, 4)) - (200 * pow(Xf, 2) * pow(Yf, 2)) + (9 * pow(Xf, 2)) - (100 * pow(Yf, 4)) + (9 * pow(Yf, 2)));
			//solving for _cam_setting_j0 given the target end effector pose
			_cam_setting_j0 = (2 * (atan((3 * Yf + var) / ((10 * pow(Xf, 2)) + (3 * Xf) + (10 * pow(Yf, 2))))) * 360 / M_PI);
			cout << "j0 is = " << _cam_setting_j0 << endl;
		//solving for j1 joint
			float negnumb1 = -100 * pow(Xf, 2) - 100 * pow(Yf, 2) + 9;
			float var1 = return_real(negnumb1);
			_cam_setting_j1 = (-2 * (atan((var1) / (10 * sqrt(pow(Xf, 2) + pow(Yf, 2))))) * 360 / M_PI);
			cout << "j1 is = " << _cam_setting_j1 << endl;
		//solving for j2 joint
	}
	
	
}

float CRobot::return_real(float& negnumber)
{
	float real;
	complex<float> number(negnumber, 0);
	complex<float> result = sqrt(number);
	real = result.real();
	return real;
}
float CRobot::return_imag(float& negnumber)
{
	float real;
	complex<float> number(negnumber, 0);
	complex<float> result = sqrt(number);
	real = result.imag();
	return real;
}

void CRobot::create_robot_revkin()
{
	//when you draw on to your video scale everything by 1000

	//translation vectors of all the links
	//each link is 15cm in width, 5cm in height, 5cm in depth
	//										x,			y,			z
	cv::Vec3d plat_trans = { 0,			0.075,	0 };
	cv::Vec3d j0_trans = { 0.075,		-0.05,	0 };
	cv::Vec3d j1_trans = { 0,			0,			0 };
	cv::Vec3d j2_trans = { -0.15,		-0.025,	0 };
	//rotational vectors
	//							roll, pitch, yaw
	cv::Vec3d j0_rot = { 0,	    0,    0 };
	cv::Vec3d j1_rot = { 0,	    0,    0 };
	cv::Vec3d j2_rot = { 0,	    0,    0 };

	//drawing the rigid bodies and platform
	std::vector<Mat> platform = createBox(0.05, 0.15, 0.05);	//platform :: white
	std::vector<Mat> rb0 = createBox(0.15, 0.05, 0.05);	//j0::red
	std::vector<Mat> rb1 = createBox(0.15, 0.05, 0.05);	//j1::green
	std::vector<Mat> rb2 = createBox(0.15, 0.05, 0.05);	//j2::blue

	//transformation matrices for placing the boxes
	Mat plat = createHT(plat_trans, 0);
	Mat T_rb0 = plat * createHT(j0_trans, j0_rot);
	Mat T_rb1 = T_rb0 * createHT(j1_trans, j1_rot);
	Mat T_rb2_rot = T_rb1 * createHT(j2_trans, j2_rot);

	//platform
		//putting it into the frame
	transformPoints(platform, plat);
	//updating the box point
	Mat PLAT0 = createHT(Vec3d{ 0, 0, 0 }, Vec3d{ 0, 0, 0 });
	//creating the coordinate for platform
	std::vector<Mat> plat_coord = createCoord();
	//transforming the coordinates
	transformPoints(plat_coord, PLAT0);
	//drawing the coordinates 
	drawCoord(_canvas, plat_coord);
	//drawing the box
	transformPoints(platform, PLAT0);

	//1st rigid body
		//putting it into the frame
	transformPoints(rb0, T_rb0);
	//updating the box point
	Mat TRB0 = createHT(Vec3d{ 0, 0.15, 0 }, Vec3d{ 0, float(_cam_setting_j0), 0 });
	//creating the coordinate for rb0
	std::vector<Mat> rb0_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb0_coord, TRB0);
	//drawing the coordinates
	drawCoord(_canvas, rb0_coord);
	//drawing the box
	transformPoints(rb0, TRB0);

	//2nd rigid body
		//putting it into the frame
	transformPoints(rb1, T_rb1);
	//updating the box point
	Mat TRB1 = createHT(Vec3d{ 0.15, 0, 0 }, Vec3d{ 0, float(_cam_setting_j1), 0 });
	//creating the coordinate for rb1
	std::vector<Mat> rb1_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb1_coord, TRB0 * TRB1);
	//drawing the coordinates
	drawCoord(_canvas, rb1_coord);
	//drawing the box
	transformPoints(rb1, TRB0 * TRB1);

	//3rd rigid body
		//putting it into the frame
	transformPoints(rb2, T_rb2_rot);
	//updating the box points
	Mat TRB2_rot_and_trans = createHT(Vec3d{ 0.175, -(float(_cam_setting_j3) / 1000), 0 }, Vec3d{ float(_cam_setting_j2), 0, 270 });
	Mat TRB2_trans = createHT(Vec3d{ 0.175, -(float(_cam_setting_j3) / 1000), 0 }, Vec3d{ 0, 0, 270 });
	Mat TRB2_rot = createHT(Vec3d{ 0.175, 0, 0 }, Vec3d{ float(_cam_setting_j2), 0, 270 });
	//creating the coordinates for rb2
	std::vector<Mat> rb2_trans_coord = createCoord();
	std::vector<Mat> rb2_rot_coord = createCoord();
	//transforming the coordinates
	transformPoints(rb2_trans_coord, TRB0 * TRB1 * TRB2_trans);
	transformPoints(rb2_rot_coord, TRB0 * TRB1 * TRB2_rot);
	//drawing the coordinates
	drawCoord(_canvas, rb2_trans_coord);
	drawCoord(_canvas, rb2_rot_coord);
	//drawing the box
	transformPoints(rb2, TRB0 * TRB1 * TRB2_rot_and_trans);

	//drawing the rigid bodies
	drawBox(_canvas, rb0, CV_RGB(255, 0, 0));
	drawBox(_canvas, rb1, CV_RGB(0, 255, 0));
	drawBox(_canvas, rb2, CV_RGB(0, 0, 255));
	drawBox(_canvas, platform, CV_RGB(255, 255, 255));
}

void CRobot::draw_robot_revkin() 
{
	//grab image (update when you do part 2 of lab on video)
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	//draw robot
	create_robot_revkin();

	//show sliders
	_virtualcam.update_settings(_canvas);
	update_robot_settings(_canvas);

	//print fkin results
	fkin();
	revkin(X, Y, Z, theta);

	//show canvas (update when you do part 2 of lab on video)
	cv::imshow(CANVAS_NAME, _canvas);
}