#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "follow_person/PersonFollowedData.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include <bica/Component.h>
#include <cmath>
#include <string>

const float Pi = 3.1416;

class pdAlgorithm: public bica::Component
{

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_, info_sub_;
  ros::Publisher speed_pub_, talk_publisher_;
  std_msgs::String data_;

  int width;
  float minDist, maxDist, Kp, Kd, prevErrorLinear, prevErrorAngular, maxV, maxW, prevV;

public:
  pdAlgorithm()
  {
    sub_ = n_.subscribe("/person_followed_data", 1, &pdAlgorithm::cb, this);
    info_sub_ = n_.subscribe("/camera/rgb/camera_info", 1, &pdAlgorithm::cb_info, this);
    speed_pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    talk_publisher_ = n_.advertise<std_msgs::String>("/talk", 1);

    width = 0;
    minDist = 900.0;
    maxDist = 1200.0;
    maxV = 0.5;
    Kp = 0.3;
    Kd = 0.7;
    prevV = 0;
    maxW = Pi / 2;
    prevErrorLinear = 0;
    prevErrorAngular = 0;
  }

  float linear(float d)
  {
    //Calculo la salida del control proporcional
    if(d > maxDist){
      std_msgs::String s;
      //s.data = "Come closer, please";
      //talk_publisher_.publish(s);
      d = maxDist;
    }
    float currentErrorLinear = (d - minDist) / (maxDist - minDist); //error entre 0 y 1
    float output = currentErrorLinear * Kp + (currentErrorLinear - prevErrorLinear) * Kd;


    prevErrorLinear = currentErrorLinear;
    float v = output * maxV;
    if(abs(v - prevV) >= maxV / 8.0){
      v = v / 8.0;
    }
    if(v < 0.0){
      v = prevV;
    }
    prevV = v;
    return v;
  }

  float angular(int p)
  {
    //calculo el error entre 0 y 1
    float currentErrorAngular = ((float)(width / 2) - (float)p) / (float)(width / 2);
    float output = currentErrorAngular * Kp + (currentErrorAngular - prevErrorAngular) * Kd;

    return output * maxW;

    prevErrorAngular = currentErrorAngular;
  }

  void cb(const follow_person::PersonFollowedData::ConstPtr& msg)
  {
    if(isActive()){
      if(width != 0){
        float dist = msg->dist;
        int centralPixel = msg->centralPixel;
        float v = pdAlgorithm::linear(dist);
        float w = pdAlgorithm::angular(centralPixel);
        geometry_msgs::Twist vel;
        vel.linear.x = v;
        vel.angular.z = w;

        speed_pub_.publish(vel);
      }
    }
  }
  void cb_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    width = msg->width;
    info_sub_.shutdown();
  }

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PD_Algorithm");
  pdAlgorithm pd;
  ros::spin();

}
