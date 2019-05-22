#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "number_persons_recognition/BoundingBoxPersonArray.h"
#include "number_persons_recognition/BoundingBoxPerson.h"
#include <vector>
#include <algorithm>
#include "follow_person/PersonFollowedData.h"
#include "sensor_msgs/CameraInfo.h"
#include <cmath>
#include <ctime>
#include "std_msgs/String.h"
#include "bica/Component.h"

const float maxDist = 2100.0;
const int MaxDistPixels = 170;

class PersonData: public bica::Component
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_image, sub_persons, info_sub, object_subscriber_;
  ros::Publisher dataPub, talk_publisher;
  std::vector<number_persons_recognition::BoundingBoxPerson> person_arr;
  time_t currentTime = time(NULL);
  time_t prevTime = time(NULL);
  int prevCentralPixel;
  std::string obj;

public:
  PersonData()
  {
    dataPub = n.advertise<follow_person::PersonFollowedData>("/person_followed_data", 1);
    talk_publisher = n.advertise<std_msgs::String>("/talk", 1);
    info_sub = n.subscribe("/camera/rgb/camera_info", 1, &PersonData::cb_info, this);
    sub_image = n.subscribe("/camera/depth/image_raw", 1, &PersonData::cb_image, this);
    sub_persons = n.subscribe("/person_stimate", 1, &PersonData::cb_persons, this);
    object_subscriber_ = n.subscribe("/orders", 1, &PersonData::objectCallback, this);
    obj = "person";
  }
  float getDist(number_persons_recognition::BoundingBoxPerson p, const sensor_msgs::Image::ConstPtr& msg)
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

  int getCentralPixel(number_persons_recognition::BoundingBoxPerson p)
  {
    return (int)((p.xmin + p.xmax) / 2);
  }

  void cb_persons(const number_persons_recognition::BoundingBoxPersonArray::ConstPtr& msg)
  {
    person_arr = msg->persons_array;

  }

  void cb_image(const sensor_msgs::Image::ConstPtr& msg)
  {
    if(isActive()){
      time_t currentTime = time(NULL);
      if(person_arr.size() > 0){
        float dist = getDist(person_arr[0], msg);
        int centralPixel = getCentralPixel(person_arr[0]);
        for(int i = 1; i < person_arr.size(); i++)
        {
          int d = getDist(person_arr[i], msg);
          if(d < dist){
            dist = d;
            centralPixel = getCentralPixel(person_arr[i]);
          }
        }
        if(obj == "person"){
          if(dist < maxDist && abs(centralPixel - prevCentralPixel) <= MaxDistPixels){
            follow_person::PersonFollowedData data;
            prevCentralPixel = centralPixel;
            data.dist = dist;
            data.centralPixel = centralPixel;
            //Publico la distancia y el centro en x
            dataPub.publish(data);
          }
        }else{
          follow_person::PersonFollowedData data;
          prevCentralPixel = centralPixel;
          data.dist = dist;
          data.centralPixel = centralPixel;
          //Publico la distancia y el centro en x
          dataPub.publish(data);
        }
      }
    }
  }

  void cb_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    int width = msg->width;
    prevCentralPixel = (int)(width / 2);
    info_sub.shutdown();
  }

  void objectCallback(const std_msgs::String::ConstPtr& msg)
  {
    obj = msg->data;
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "Person_Followed_Publisher");
  PersonData personData;
  ros::spin();
}
