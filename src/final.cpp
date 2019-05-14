#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <actionlib/client/simple_action_client.h>
#include "frontier_exploration/ExploreTaskAction.h"
#include <vector>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <cmath>

// Initial Point.
float X = 0;
float Y = 0;

// Area Bounded by two points
float X1 = -9.0 + 1;
float Y1 = -9.0 + 1;
// Shifted by half a unit to allow robot to rotate
float X2 = 9 - 1;
float Y2 = 9 - 1;

// Amound of steps used to calculate step size.
float stepsX = 4;
float stepsY = 4;
float stepSizeX = fabs(X2-X1)/stepsX;
float stepSizeY = fabs(Y2-Y1)/stepsY;

// Boolean cases
bool incX = true;
bool completed = false;
bool sendNextStep = true;
bool cp = true;

int dir = 0;

void index2Coordinates(int i) {
  float XX = floor(i/800)*0.05 - 20;
  float YY = (i%800)*0.05 - 20;
  ROS_INFO_STREAM("(" << XX << ", " << YY << ")");
}

// Calculates next point in sweep expects robot to be at initial point
void calculateNextPoint() {

  // CASE: We are at destination
  if(X == X2 && Y == Y2) {
    completed = true;
    ROS_INFO("Point Last");

  // CASE: We walk towards Point two's bound
  } else if(incX && X < X2) {
    ROS_INFO("mr -- Down");
    X += stepSizeX;

  // CASE: We walk towards Point one's bound
  } else if((!incX) && X > X1) {
    ROS_INFO("mr -- Up");
    X -= stepSizeX;
  // CASE: We bump into Point one or two's bound
} else if(X <= X1 || X >= X2) {
    incX = !incX;
    Y += stepSizeY;
    ros::Duration(0.5).sleep();
    ROS_INFO("mr -- Right");
  } else {
    ROS_INFO("ERROR: NO CASE!!!");
    if(incX){
      ROS_INFO("incX");
    } else {
      ROS_INFO("descX");
    }
  }

  ROS_INFO_STREAM("(" << X << "," << Y << ")");
}

int main(int argc,char **argv) {

    ros::init(argc,argv,"final");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("Connected to action server");

    enum state {
      GO_TO_AREA,
      SWEEP,
      ARRIVED,
      SPIN,
      LOST
    };

    state botState = GO_TO_AREA;

    move_base_msgs::MoveBaseGoal goal;

    ros::Rate rate(1);

    while (ros::ok()) {
      ros::spinOnce();

      ROS_INFO("...");
      switch (botState) {

        case GO_TO_AREA: {

          // goal.target_pose.header.frame_id = "map";
          // // goal.target_pose.header.stamp = ros::Time::now();
          // //
          // // // goal.target_pose.pose.position.x = 0;
          // // // goal.target_pose.pose.position.y = 2.2;
          // // // goal.target_pose.pose.orientation.w = 1.0;
          // // //
          // // // ac.sendGoal(goal);
          // ac.waitForResult();


          ROS_INFO("Going to Area");
          X = X1;
          Y = Y1;
          ROS_INFO_STREAM("(" << X << "," << Y << ")");

          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();

          goal.target_pose.pose.position.x = X;
          goal.target_pose.pose.position.y = Y;

          goal.target_pose.pose.orientation.w = 1.0;

          ac.sendGoal(goal);
          ac.waitForResult();

          if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("Succ: Arrived to area");
            botState = SPIN;
          } else {
            ROS_INFO_STREAM("Err: Failed arrived to area");
            ros::Duration(3).sleep();
            botState = LOST;
          }
          break;
        }

        case SWEEP: {

          if (completed) {
            botState = ARRIVED;
          } else if (sendNextStep) {
            sendNextStep = false;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // Setting Point
            calculateNextPoint();
            goal.target_pose.pose.position.x = X;
            goal.target_pose.pose.position.y = Y;

            // Setting Direction
            if (incX) {
              // Facing down in Gazebo
              goal.target_pose.pose.orientation.x = 0;
              goal.target_pose.pose.orientation.y = 0;
              goal.target_pose.pose.orientation.z = 0;
              goal.target_pose.pose.orientation.w = 1.0;
            } else {
              // Facing up in Gazebo
              goal.target_pose.pose.orientation.x = 0;
              goal.target_pose.pose.orientation.y = 0;
              goal.target_pose.pose.orientation.z = 1.0;
              goal.target_pose.pose.orientation.w = 0.0;
            }

            ac.sendGoal(goal);
            ac.waitForResult(ros::Duration(45.0));

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
              ROS_INFO_STREAM("Succ");
              sendNextStep = true;
              botState = SPIN;
            } else {
              ROS_INFO_STREAM("Err: Timed out");
              ros::Duration(3).sleep();
              sendNextStep = false;
              botState = LOST;
            }
          }

          break;
        }

        case SPIN: {
          tf2::Quaternion quat_tf;
          geometry_msgs::Quaternion quat_msg;
          dir = 0;
          while (dir < 3) {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = X;
            goal.target_pose.pose.position.y = Y;

            if (dir == 0) {
              ROS_INFO_STREAM("Spin " << dir);
              quat_tf.setRPY(0,0,0 *(M_PI/180));
            } else if (dir == 1) {
              ROS_INFO_STREAM("Spin " << dir);
              quat_tf.setRPY(0,0,120 *(M_PI/180));
            } else if (dir == 2) {
              ROS_INFO_STREAM("Spin " << dir);
              quat_tf.setRPY(0,0,240 *(M_PI/180));
            }
            quat_msg = tf2::toMsg(quat_tf);
            goal.target_pose.pose.orientation = quat_msg;


            ac.sendGoal(goal);
            ac.waitForResult(ros::Duration(7.0));

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
              ROS_INFO_STREAM("Succ: Spin");
              if (dir < 3) {
                dir++;
              }
              if (dir == 3) {
                sendNextStep = true;
                botState = SWEEP;
              }
            } else {
              ROS_INFO_STREAM("Err: Spin timmed our or failed");
              dir++;
            }
          }
          if (dir >= 3) {
            sendNextStep = true;
            botState = SWEEP;
          }
          break;
        }

        case ARRIVED: {
          ROS_INFO("Succ: Scan of area");
          ros::shutdown();
          break;
        }

        case LOST: {
          ROS_INFO("Err: Going to next point");
          sendNextStep = true;
          botState = SWEEP;
          break;
        }
      }
    }

    return 0;
}
