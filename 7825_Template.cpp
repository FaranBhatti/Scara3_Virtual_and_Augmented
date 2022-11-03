////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept. 9, 2020 by Craig Hennessey
// Edited Oct. 23rd, 2022 by Faran Bhatti
// Last updated October 23rd, 2022
////////////////////////////////////////////////////////////////
#include "stdafx.h"

using namespace std;
using namespace cv;

using namespace dnn;
using namespace aruco;




#include <opencv2/face/facemark.hpp>

// Add simple GUI elements
#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "Robot.h"

void lab1()
{
  // MATLAB
}

void lab2()
{
  // MATLAB
}

void lab3(int cam_id)
{
  char exit_key = -1;
  CRobot robot;
  

  //remember to update the axis size
  robot.create_simple_robot();

  while (exit_key != 'q')
  {
    robot.draw_simple_robot();
    exit_key = waitKey(10);
  }
}

void lab4(int cam_id)
{
   char exit_key = -1;
   CRobot robot;

   while (exit_key != 'q') 
   {
      robot.draw_augmented_robot();
      exit_key = waitKey(10);
   }
}

void lab5(int cam_id)
{
   int select = -1;
   while (select != 0)
   {
      cout << "\n(1) Lab 5a - Forward Kinematics Virtual";
      cout << "\n(2) Lab 5b - Forward Kinematics Real";
      cout << "\n(0) Exit\n";
      cin >> select;
      switch (select)
      {
      case 1:
      {
         //Scara robot on virtual camera
         char exit_key = -1;
         CRobot robot;

         while (exit_key != 'q')
         {
            robot.draw_scara_robot();
            exit_key = waitKey(10);
         }
      }
         break;
      case 2:
      {
         //Scara robot on real camera
         char exit_key = -1;
         CRobot robot;

         while (exit_key != 'q')
         {
            robot.draw_scara_robot_augmented();
            exit_key = waitKey(10);
         }
      }
         break;
      }
   }
   /* COMMENT BLOCK FOR RUNNING VIRTUAL CAMERA SCARA
   //update frame size for when displaying on image vs. display, and scaling for boxes
   char exit_key = -1;
   CRobot robot;

   while (exit_key != 'q')
   {
      robot.draw_scara_robot();
      exit_key = waitKey(10);
   }
   */
   
}

void lab6(int cam_id)
{
  
}

void lab7(int cam_id)
{
 
}

void lab8(int cam_id)
{

}

void lab9(int cam_id)
{

}

int main(int argc, char* argv[])
{
  int cam_id = 0;
  int sel = -1;

  while (sel != 0)
  {
    cout << "\n*****************************************************";
    cout << "\n(1) Lab 1 - Coordinate Transforms 2D";
    cout << "\n(2) Lab 2 - Coordinate Transforms 3D";
    cout << "\n(3) Lab 3 - Virtual Camera";
    cout << "\n(4) Lab 4 - Camera Calibration";
    cout << "\n(5) Lab 5 - Forward Kinematics";
    cout << "\n(6) Lab 6 - Inverse Kinematics";
    cout << "\n(7) Lab 7 - Trajectories";
    cout << "\n(8) Lab 8 - Object Tracking";
    cout << "\n(9) Lab 9 - DNN";
    cout << "\n(0) Exit";
    cout << "\n>> ";

    cin >> sel;
    switch (sel)
    {
    case 1: lab1(); break;
    case 2: lab2(); break;
    case 3: lab3(cam_id); break;
    case 4: lab4(cam_id); break;
    case 5: lab5(cam_id); break;
    case 6: lab6(cam_id); break;
    case 7: lab7(cam_id); break;
    case 8: lab8(cam_id); break;
    case 9: lab9(cam_id); break;
    }
  }

  return 1;
}