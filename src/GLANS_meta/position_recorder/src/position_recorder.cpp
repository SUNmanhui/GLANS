#include <position_recorder/position_recorder.h>




int main(int argc, char **argv) {
  ros::init(argc, argv, "position_recorder");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  double record_frequency;
  std::string map_frame, base_frame, odom_frame;
  int queue_size;//record the message
  std::string topic;
  std::string store_path_;
  FILE* logger_csv_;


  private_nh.param("store_path", store_path_, std::string("."));
  private_nh.param("topic", topic, std::string("odom"));
  
  
  private_nh.param<double>("record_frequency", record_frequency, 0.5);
  private_nh.param<std::string>("odom_frame", odom_frame, "odom");
  private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
  private_nh.param<std::string>("map_frame", map_frame, "map");
  
    
  tf::TransformListener listener;
  std::string tf_prefix = tf::getPrefixParam(private_nh);
	ros::ServiceClient get_gazebopose_servcli_;
  get_gazebopose_servcli_ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");


  store_path_ = "/home/xuefengchang";

  std::string filename, full_path;
  filename = "position_recorder";

  full_path = store_path_ + "/" + filename + ".csv";

  ROS_INFO("Start logging position topic to:\n\t%s",full_path.c_str());

  logger_csv_ = fopen(full_path.c_str(), "w");
  if (!logger_csv_) {
  ROS_ERROR("Couldn't save position topic data to:\n\t%s\nShutting down logger node now", full_path.c_str());
  nh.shutdown();
  }
  fprintf(logger_csv_, "map_x, map_y, gazebo_x, gazebo_y, odom_x, odom_y\n");
	
  
  ros::Rate rate(record_frequency);
  while(nh.ok()) {
    //get the position in map
    tf::StampedTransform transform_m;
    bool tf_ok = true;
    try {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform_m);
    } catch(tf::TransformException ex) {
      //ROS_ERROR("-------> %s", ex.what());
      tf_ok = false;
    }
    
    geometry_msgs::PoseStamped pose_stamped_m;
    if(tf_ok) {
      
      pose_stamped_m.header.stamp = ros::Time::now();
      pose_stamped_m.header.frame_id = "map";
      pose_stamped_m.pose.position.x = transform_m.getOrigin().getX();
      pose_stamped_m.pose.position.y = transform_m.getOrigin().getY();
      pose_stamped_m.pose.position.z = transform_m.getOrigin().getZ();
      
    }
    
		//get the position in odom
    tf::StampedTransform transform_o;
    try {
      listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform_o);
    } catch(tf::TransformException ex) {
      //ROS_ERROR("-------> %s", ex.what());
      tf_ok = false;
    }
    
    geometry_msgs::PoseStamped pose_stamped_o;
    if(tf_ok) {
      
      pose_stamped_o.header.stamp = ros::Time::now();
      pose_stamped_o.header.frame_id = "odom";
      pose_stamped_o.pose.position.x = transform_o.getOrigin().getX();
      pose_stamped_o.pose.position.y = transform_o.getOrigin().getY();
      pose_stamped_o.pose.position.z = transform_o.getOrigin().getZ();
      
    }

    //get the positon in the gazebo 
    geometry_msgs::Pose gazebo_pose;

    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "mobile_base";

	if (!get_gazebopose_servcli_.call(srv)) {
		ROS_WARN("Did not receive a valid response from /gazebo/get_model_state service");
	}

	gazebo_pose.position.x = srv.response.pose.position.x;
	gazebo_pose.position.y = srv.response.pose.position.y;
	gazebo_pose.position.z = srv.response.pose.position.z;
	
	ROS_DEBUG("Gazebo pose (x,y)= (%.4f,%.4f)",
			srv.response.pose.position.x,
			srv.response.pose.position.y);
	
    
  //write it into the file	
	fprintf(logger_csv_, "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
	pose_stamped_m.pose.position.x,
	pose_stamped_m.pose.position.y,
	gazebo_pose.position.x,
	gazebo_pose.position.y,
	pose_stamped_o.pose.position.x,
	pose_stamped_o.pose.position.y);
	



  rate.sleep();
  }


   
  fclose (logger_csv_);

  return 0;
}




