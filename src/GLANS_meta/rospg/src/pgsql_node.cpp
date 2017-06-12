#include "pgsql.h"
#include <ros/ros.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "pgsql_node");
	Pgsql pgsql(argc, argv) ;
        ros::spin();
	return 0;
}
