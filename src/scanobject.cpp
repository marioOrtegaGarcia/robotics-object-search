#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

float X, Y;
float yaw;
float X_1, X_2, Y_1, Y_2;
bool initiated = false;
float laserDerrivative[720];
std::vector<int> p;
int peaks = 0;


void poseReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  X = msg.pose.pose.position.x;
  Y = msg.pose.pose.position.y;
  tf2::Quaternion q;
  tf2::convert(msg.pose.pose.orientation, q);
  yaw = tf2::getYaw(q);
  initiated = true;
  // ROS_INFO_STREAM("Loc(" << X << ", " << Y);
}

void scanMessageReceived(const sensor_msgs::LaserScan&msg) {
  // ROS_INFO_STREAM("Received scan. Finding closest obstacle");
  // float closest = msg.ranges[0];
  float highestPeak;
  laserDerrivative[0] = msg.ranges[0] - msg.ranges[1];
  highestPeak = laserDerrivative[0];
  p.clear();
  // Finding closest index and also computing the derrivative array and finding the highestPeak
  for (int i = 0 ; i < msg.ranges.size()-1 ; i++ ) {
    if(i > 180 && i < 540) laserDerrivative[i] = fabs(msg.ranges[i+1] - msg.ranges[i]);
    else laserDerrivative[i] = 0;
  }
  // ROS_INFO("Peaks");
  peaks = 0;
  for (size_t j = 0; j < 720; j++) {
    if (laserDerrivative[j] > 0.8 ) {
      peaks++;
      p.push_back(j);
      // ROS_INFO_STREAM("Peak @ index " << j << " Distance " << msg.ranges[j] << " " << msg.ranges[j+1] << " Del " << laserDerrivative[j]);
      //<< " Location ("<<  X+laserDerrivative[j]*cos(j*msg.angle_increment) <<", " << Y+laserDerrivative[j]*sin(j*msg.angle_increment) << ")" << " del " << laserDerrivative[j] << ", ");
    }
  }
  // ROS_INFO_STREAM("Peaks found: " << peaks);
  if (peaks == 8) {
    float avg = 0;
    float totalAVG = 0;
    float sum = 0;
    float total = 0;
    for (size_t i = 0; i < p.size()-1; i+=2) {
      for (size_t j = p[i]; j < p[i+1]; j++) {
          sum += msg.ranges[j];
          total++;
      }
      avg = sum / total;
      totalAVG += avg;
      // if (avg < 5) ROS_INFO_STREAM("Calculated Avg: " << avg);hypotf(msg.ranges[p[0]], msg.ranges[p[1]]);
    // ROS_INFO_STREAM("Hypot: " << hy);

    }
    if (avg < 5) ROS_INFO_STREAM("FND TABLE @PNT(" << X+avg*cos(yaw) << ", "<< Y+avg*sin(yaw) << ") avg: " << totalAVG/total << " hypot 0,1: " << hypotf(msg.ranges[p[0]], msg.ranges[p[1]]) << " hypot 1,4: " << hypotf(msg.ranges[p[1]], msg.ranges[p[4]]));

  }

  if (peaks == 2) {
    float avg = 0;
    float totalAVG = 0;
    float sum = 0;
    float total = 0;
    float hy = hypotf(msg.ranges[p[0]], msg.ranges[p[1]]);
    // ROS_INFO_STREAM("Hypot: " << hy);
    for (size_t i = 0; i < p.size()-1; i+=2) {
      for (size_t j = p[i]; j < p[i+1]; j++) {
          sum += msg.ranges[j];
          total++;
      }
      avg = sum / total;
      totalAVG += avg;
      // if (avg < 3) ROS_INFO_STREAM("Calculated Avg: " << avg);
    }
     if (avg < 3 && ((hy < 5.6 && hy > 4) || (hy > 2 && hy < 3)) ) ROS_INFO_STREAM("FND MAILBOX @PNT(" << X+avg*cos(yaw) << ", "<< Y+avg*sin(yaw) << ") avg rng: " << totalAVG/total
    << " Real Hypot: " << hypotf(msg.ranges[p[0]]*cos((p[1]-p[0])*msg.angle_increment) , msg.ranges[p[1]]*sin((p[1]-p[0])*msg.angle_increment))
     << " Hypot: " << hy);
  }
}

int main(int argc,char ** argv) {
  ros::init(argc,argv,"scanobject");
  ros::NodeHandle nh;

  ros::Subscriber subScan = nh.subscribe("/scan", 1000, &scanMessageReceived);
  ros::Subscriber subPose = nh.subscribe("/amcl_pose", 1000, &poseReceived);





  ros::spin();
}
