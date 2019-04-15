#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "number_persons_recognition/BoundingBoxPersonArray.h"
#include "number_persons_recognition/BoundingBoxPerson.h"
#include <vector>

std::vector<number_persons_recognition::BoundingBoxPerson> person_arr;
cv_bridge::CvImagePtr cv_image;
ros::Publisher person_count_publisher;

void cb(const number_persons_recognition::BoundingBoxPersonArray::ConstPtr& msg)
{
  person_arr = msg->persons_array;
  std_msgs::Int32 size;
  size.data = person_arr.size();
  person_count_publisher.publish(size);
}

void cb2(const sensor_msgs::Image::ConstPtr& msg)
{
  if(person_arr.size() > 0){
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat ROI(cv_image->image, cv::Rect(person_arr[0].xmin, person_arr[0].ymin,
                person_arr[0].xmax - person_arr[0].xmin, person_arr[0].ymax - person_arr[0].ymin));
    cv::Mat croppedImage;
    ROI.copyTo(croppedImage);
    cv::imshow("Image cropped", croppedImage);
    cv::waitKey(3);
  }else{
    try{
      cv::destroyWindow("Image cropped");
    }catch (cv::Exception& e){
			return;
		}
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "people_stimator_subscriber");
  ros::NodeHandle n;
  person_count_publisher = n.advertise<std_msgs::Int32>("/person_count", 1);
  ros::Subscriber sub_node_persons = n.subscribe("/person_stimate", 1, cb);
  ros::Subscriber sub_node_image = n.subscribe("/camera/rgb/image_raw", 1, cb2);
  ros::spin();

}
