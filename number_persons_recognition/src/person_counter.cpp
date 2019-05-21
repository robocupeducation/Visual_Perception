#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "number_persons_recognition/BoundingBoxPersonArray.h"
#include "number_persons_recognition/BoundingBoxPerson.h"
#include <vector>
#include <algorithm>

//PONER EN EL .LAUNCH COMO PARAMETER
const float MaxDist = 3000.0; //Max Distance in mm

class Counter
{
private:
  ros::NodeHandle n;
  std::vector<number_persons_recognition::BoundingBoxPerson> person_arr;
  cv_bridge::CvImagePtr cv_image;
  ros::Publisher person_count_publisher;
  ros::Subscriber sub_node_persons, sub_node_image;
public:
  Counter()
  {
    person_count_publisher = n.advertise<std_msgs::Int32>("/person_count", 1);
    sub_node_persons = n.subscribe("/person_stimate", 1, &Counter::cb_stimation, this); //&Init::cb,this
    sub_node_image = n.subscribe("/camera/depth/image_raw", 1, &Counter::cb_person_count, this);
  }


  void cb_stimation(const number_persons_recognition::BoundingBoxPersonArray::ConstPtr& msg)
  {
    person_arr = msg->persons_array;

  }

  void cb_person_count(const sensor_msgs::Image::ConstPtr& msg)
  {

    float dist;
    int width;
    int height;
    cv::Mat croppedImage;
    std::vector<float> distVector;
    std_msgs::Int32 size;
    size.data = 0;
    //if(person_arr.size() > 0){
    for (int i = 0; i < person_arr.size(); i++)
    {
      cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      width = person_arr[i].xmax - person_arr[i].xmin;
      height = person_arr[i].ymax - person_arr[i].ymin;
      cv::Mat ROI(cv_image->image, cv::Rect(person_arr[i].xmin, person_arr[i].ymin,
                  width, height));
      ROI.copyTo(croppedImage);
      //cv::imshow("Image cropped", croppedImage);
      //cv::waitKey(3);
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
      //printf("%f\n", distVector[(int)(distVector.size() / 2)]);
      float med = distVector[(int)(distVector.size() / 2)];
      if (med < MaxDist) {
        size.data = size.data + 1;
      }
    }
    person_count_publisher.publish(size);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "people_stimator_subscriber");
  Counter counter;
  ros::spin();

}
