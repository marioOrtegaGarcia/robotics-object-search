#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>

float strt_pt_x = 4;
float strt_pt_y = 0;
float end_pt_x = 9;
float end_pt_y = 6;
float arr_of_pts[18] = {-5,5, 0,5 , 5,5 , 5,0 , 0,0 , -5,0 , -5,-5 , 0,-5 , 5,-5 };



int main(int argc,char **argv) {

    ros::init(argc,argv,"final_node");
    ros::NodeHandle nh;

    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal;

    
   
   
    ros::Duration(3).sleep();
    for (int i = 0; i < 18; i += 2){
        ros::spinOnce();
        
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = arr_of_pts[i];
        goal.target_pose.pose.position.y = arr_of_pts[ i + 1];
        ROS_INFO_STREAM("x = " << goal.target_pose.pose.position.x);
        ROS_INFO_STREAM("y = " << goal.target_pose.pose.position.y);
        goal.target_pose.pose.orientation.w = 1.0;

        ac.sendGoal(goal);
        ac.waitForResult();
        ros::Duration(5).sleep();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO_STREAM("SUCCESS");
            

        }
        else {
            ROS_INFO_STREAM("FAILED TRY AGAIN");
            i = i - 2;
            goal.target_pose.pose.position.x = arr_of_pts[i];
            goal.target_pose.pose.position.y = arr_of_pts[ i + 1] -1;
            ros::Duration(3).sleep();
            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO_STREAM("SUCCESS");
                i = i+2;
                continue;
            }
        }


    
    } 
    nh.shutdown();
}
