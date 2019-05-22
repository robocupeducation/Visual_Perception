#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "std_msgs/String.h"
#include <vector>

//const float MinProb = 0.4;

class Detector
{
private:
  float MinProb;

  ros::NodeHandle n;
  std::vector<std::string> objects_collection;
  ros::Publisher object_publisher;
  ros::Subscriber sub_node;
  std::string object;
public:
  Detector()
  {
    MinProb = 0.4;
    object_publisher = n.advertise<std_msgs::String>("/object_detected", 1);
    Detector::getParams(n);
    sub_node = n.subscribe("/darknet_ros/bounding_boxes", 1, &Detector::callback, this);
  }
  void getParams(ros::NodeHandle n)
  {
    std::vector<std::string> v;
    std::string s;
    n.getParam("obj1", s);
    objects_collection.push_back(s);
    n.getParam("obj2", s);
    objects_collection.push_back(s);
  }

  bool objectFound(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    int size = msg->bounding_boxes.size();
    int count = 0;
    bool finish = 0;
    while((!finish) && (count < size))
    {
      if(msg->bounding_boxes[count].probability > MinProb){
        darknet_ros_msgs::BoundingBox obj = msg->bounding_boxes[count];
        int i = 0;
        int size_objects = objects_collection.size();
        while((!finish) && (i < size_objects))
        {
          if(obj.Class == objects_collection[i]){
            finish = 1;
            object = obj.Class;
          }else{
            i++;
          }
        }

      }
      count++;
    }
    return finish;
  }

  void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    if(Detector::objectFound(msg)){
      std_msgs::String m;
      m.data = object;
      object_publisher.publish(m);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Objects_Detector");
  Detector detector;
  ros::spin();

}
