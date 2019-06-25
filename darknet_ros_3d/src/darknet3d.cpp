#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "robocuphomeeducation_msgs/BoundingBox3d.h"
#include "robocuphomeeducation_msgs/BoundingBoxes3d.h"
#include "sensor_msgs/Image.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include "std_msgs/Int32.h"

class Darknet3d
{
private:
  ros::NodeHandle n;
  /*
  *yolo_sub: Subscriber to bounding boxes provided by darknet_ros
  *depth_sub: subscriber to depth image provided by openni2_launch package
  */
  ros::Subscriber _yolo_sub, _depth_sub;
  ros::Publisher _darknet3d_pub;

  std::vector<darknet_ros_msgs::BoundingBox> _originalBBoxes; //here is saved bounding boxes array received from darknet_ros

public:
  Darknet3d()
  {
    _yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, &Darknet3d::darknetCb, this);
    _depth_sub = n.subscribe("/camera/depth/image_raw", 1, &Darknet3d::depthCb, this);
    _darknet3d_pub = n.advertise<robocuphomeeducation_msgs::BoundingBoxes3d>("/darknet_ros3d/bounding_boxes", 1);
  }

  float getDist(darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg)
  {
    float dist;
    int width, height;
    cv::Mat croppedImage;
    std::vector<float> distVector;
    std_msgs::Int32 size;

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    width = p.xmax - p.xmin;
    height = p.ymax - p.ymin;
    cv::Mat ROI(cv_image->image, cv::Rect(p.xmin, p.ymin,
                width, height));
    ROI.copyTo(croppedImage);
    dist = 0.0;
    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < height; k++)
      {
        dist = (float)croppedImage.at<float>(j, k);
        distVector.push_back(dist);
      }
    }

    std::sort (distVector.begin(), distVector.end());
    float med = distVector[(int)(distVector.size() / 2)];
    return med;
  }

  void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    _originalBBoxes = msg->bounding_boxes;
  }

  void depthCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    std::vector<robocuphomeeducation_msgs::BoundingBox3d> v;
    robocuphomeeducation_msgs::BoundingBox3d data;
    if(_originalBBoxes.size() > 0){
      float d;
      for(int i = 0; i < _originalBBoxes.size(); i++){
        d = getDist(_originalBBoxes[i], msg);
        //Componer el mensaje
        data.Class = _originalBBoxes[i].Class;
        data.probability = _originalBBoxes[i].probability;
        data.xmin = _originalBBoxes[i].xmin;
        data.ymin = _originalBBoxes[i].ymin;
        data.xmax = _originalBBoxes[i].xmax;
        data.ymax = _originalBBoxes[i].ymax;
        data.depth = d;
        //meterlo en el array
        v.push_back(data);
      }
    }
    //Publicar el array
    robocuphomeeducation_msgs::BoundingBoxes3d msg_to_publish;
    msg_to_publish.bounding_boxes = v;
    _darknet3d_pub.publish(msg_to_publish);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darknet_3d");
  Darknet3d darknet3d;
  ros::spin();
  return 0;
}
