
#include <ros/ros.h>
#include <ros/console.h>

#include <bica_graph/graph.h>
#include <bica_graph/graph_publisher.h>

#include <bica/Component.h>
#include <bica_graph/graph_handler.h>

class BallDetector: public bica::Component
{
public:

private:
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_detector");

	BallDetector ball_detector;

	ros::Rate loop_rate(5);
	while(ball_detector.ok())
	{
		//ball_detector.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
