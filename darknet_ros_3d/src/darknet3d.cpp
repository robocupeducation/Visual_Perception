#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "visual_perception_msgs/BoundingBox3d.h"
#include "visual_perception_msgs/BoundingBoxes3d.h"
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
  float _distRange; //max difference between nearest pixel and any other pixel.

public:
  Darknet3d()
  {
    _yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, &Darknet3d::darknetCb, this);
    _depth_sub = n.subscribe("/camera/depth/image_raw", 1, &Darknet3d::depthCb, this);
    _darknet3d_pub = n.advertise<visual_perception_msgs::BoundingBoxes3d>("/darknet_ros_3d/bounding_boxes", 1);
    _distRange = 1000.0;
  }

  float mean(std::vector<float> v)
  {
    float sum;
    sum = 0.0;
    for(int i = 0; i < v.size(); i++)
    {
      sum = sum + v[i];
    }
    return sum / (float)v.size();
  }

  float flood(float nearestPixel, darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg)
  {
    float dist;
    int width, height;
    cv::Mat croppedImage;
    std::vector<float> distsArray;

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    width = p.xmax - p.xmin;
    height = p.ymax - p.ymin;
    cv::Mat ROI(cv_image->image, cv::Rect(p.xmin, p.ymin,
                width, height));
    ROI.copyTo(croppedImage);

    for(int j = 0; j < width; j++)
    {
      for(int k = 0; k < height; k++)
      {
        dist = (float)croppedImage.at<float>(j, k);
        if(dist - nearestPixel <= _distRange)
        {
          distsArray.push_back(dist);
        }
      }
    }

    return Darknet3d::mean(distsArray);

  }

  float getMinDist(darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg)
  {
    float dist, mindist;
    int width, height;
    cv::Mat croppedImage;

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    width = p.xmax - p.xmin;
    height = p.ymax - p.ymin;
    cv::Mat ROI(cv_image->image, cv::Rect(p.xmin, p.ymin,
                width, height));
    ROI.copyTo(croppedImage);
    mindist = (float)croppedImage.at<float>(0, 0);
    for(int j = 1; j < width; j++)
    {
      for(int k = 1; k < height; k++)
      {
        dist = (float)croppedImage.at<float>(j, k);
        if(dist < mindist)
        {
          mindist = dist;
        }
      }
    }

    return mindist;
  }

  float getDist(darknet_ros_msgs::BoundingBox p, const sensor_msgs::Image::ConstPtr& msg)
  {
    float nearestPixel, dist; //Distance to the nearest pixel is saved here
    //Get nearest pixel
    nearestPixel = Darknet3d::getMinDist(p, msg);
    //Algorithm to disregard very distant pixels and calculate mean distance
    return Darknet3d::flood(nearestPixel, p, msg);
  }

  void darknetCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
  {
    _originalBBoxes = msg->bounding_boxes;
  }

  void depthCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    std::vector<visual_perception_msgs::BoundingBox3d> v;
    visual_perception_msgs::BoundingBox3d data;
    if(_originalBBoxes.size() > 0)
    {
      float dist;
      for(int i = 0; i < _originalBBoxes.size(); i++)
      {
        dist = Darknet3d::getDist(_originalBBoxes[i], msg);
        //Componer el mensaje
        data.Class = _originalBBoxes[i].Class;
        data.probability = _originalBBoxes[i].probability;
        data.xmin = _originalBBoxes[i].xmin;
        data.ymin = _originalBBoxes[i].ymin;
        data.xmax = _originalBBoxes[i].xmax;
        data.ymax = _originalBBoxes[i].ymax;
        data.depth = dist;
        //meterlo en el array
        v.push_back(data);
      }
      //Publicar el array
      visual_perception_msgs::BoundingBoxes3d msg_to_publish;
      msg_to_publish.bounding_boxes = v;
      _darknet3d_pub.publish(msg_to_publish);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darknet_3d");
  Darknet3d darknet3d;
  ros::spin();
  return 0;
}
