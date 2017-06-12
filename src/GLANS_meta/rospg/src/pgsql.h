#include <ros/ros.h>

#include <std_msgs/String.h>
#include <rospg/TopologicalNavigationMap.h>
#include <rospg/TopoNavEdgeMsg.h>
#include <rospg/TopoNavNodeMsg.h>


//#include <std_msgs/int.h>
#include <pqxx/pqxx>
//#include <boost/foreach.hpp>
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Path.h>

class Pgsql
{
	public:
		Pgsql(int argc, char**argv);
		~Pgsql();		
	private:
        	ros::NodeHandle n;

                ros::Publisher pub1,pub2;
		ros::Subscriber sub1;
		
			
                //void normalizeMsg();

                int getShortestPoint(int &nid, double &nx, double &ny, double x, double y);
                void sendGoal(const geometry_msgs::PoseStampedConstPtr &posestamped);
                double calcDistance(double x,double y,double point_x,double point_y);
                tf::Pose lookupPoseInMap(tf::Pose input_pose);
};


