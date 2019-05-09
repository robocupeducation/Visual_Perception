#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "number_persons_recognition/BoundingBoxPersonArray.h"
#include "number_persons_recognition/BoundingBoxPerson.h"
#include <vector>
#include <algorithm>
#include "follow_person/PersonFollowedData.h"

ros::Publisher pub;
std::vector<number_persons_recognition::BoundingBoxPerson> person_arr;

float getDist(number_persons_recognition::BoundingBoxPerson p, const sensor_msgs::Image::ConstPtr& msg)
{
  float dist;
  int width;
  int height;
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
  if(person_arr.size() > 0){
    float dist = getDist(person_arr[0], msg);
    int centralPixel = getCentralPixel(person_arr[0]);
    for(int i = 1; i < person_arr.size(); i++)
    {
      float d = getDist(person_arr[i], msg);
      if(d < dist){
        dist = d;
        int centralPixel = getCentralPixel(person_arr[i]);
      }
    }
    follow_person::PersonFollowedData data;
    data.dist = dist;
    data.centralPixel = centralPixel;
    //Publico la distancia y el centro en x
    pub.publish(data);
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "Person_Followed_Publisher");
  ros::NodeHandle n;

  pub = n.advertise<follow_person::PersonFollowedData>("/person_followed_data", 1);
  ros::Subscriber sub_image = n.subscribe("/camera/depth/image_raw", 1, cb_image);
  ros::Subscriber sub_persons = n.subscribe("/person_stimate", 1, cb_persons);
  ros::spin();
}
