//Publicates the number of persons
#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "visual_perception_msgs/BoundingBoxPersonArray.h"
#include "visual_perception_msgs/BoundingBoxPerson.h"
#include <vector>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

const float MinPersonProb = 0.6;

class Stimator
{
private:
  ros::NodeHandle n;
  ros::Publisher person_stimation_publisher, prueba_pub;
  ros::Subscriber sub_node, object_subscriber;
  std::string obj;
public:
  Stimator()
  {
    obj = "person";
    person_stimation_publisher = n.advertise<visual_perception_msgs::BoundingBoxPersonArray>("/person_stimate", 1);
    sub_node = n.subscribe("/darknet_ros/bounding_boxes", 1, &Stimator::cb, this);
    object_subscriber = n.subscribe("/orders", 1, &Stimator::objectCallback, this);
    prueba_pub = n.advertise<std_msgs::String>("/prueba", 1);

  }


  void cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    int boxessize = msg->bounding_boxes.size();
    std::vector<visual_perception_msgs::BoundingBoxPerson> v;
    visual_perception_msgs::BoundingBoxPerson element;
    for (int i = 0; i < boxessize; i++)
    {
      //ROS_ERROR("object to detect: %s", obj.c_str());
      if(msg->bounding_boxes[i].Class == obj){
        if(msg->bounding_boxes[i].probability > MinPersonProb){

          element.xmin = msg->bounding_boxes[i].xmin;
          element.xmax = msg->bounding_boxes[i].xmax;
          element.ymin = msg->bounding_boxes[i].ymin;
          element.ymax = msg->bounding_boxes[i].ymax;
          v.push_back(element);
        }
      }
    }
    visual_perception_msgs::BoundingBoxPersonArray a;
    a.persons_array = v;
    person_stimation_publisher.publish(a);
  }

  void objectCallback(const std_msgs::String::ConstPtr& msg)
  {
    obj = msg->data;
    std_msgs::String m;
    m.data = obj;
    prueba_pub.publish(m);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_ros_subscriber");
  Stimator stimator;
  ros::spin();
  /*
  while(ros::ok()){
    step();
    ros::spinOnce();
    rate.sleep();
  }
  */
}
