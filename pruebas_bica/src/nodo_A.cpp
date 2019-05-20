
#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class Test: public bica::Component
{
public:

	Test()
	{
		addDependency("nodo_B");
	}

	void step()
	{
		if(! isActive()) return;
		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_detector");

	Test test;

	ros::Rate loop_rate(5);
	while(test.ok())
	{
		test.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
