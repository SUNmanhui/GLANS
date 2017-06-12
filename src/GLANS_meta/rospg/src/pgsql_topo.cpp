#include "pgsql.h"
#include <pqxx/pqxx>
#include <iostream>
#include "stdafx.h"
#include <ros/ros.h>


using namespace std;
using namespace pqxx;
using namespace tf;

//geometry_msgs::PoseStamped goal_pose;
rospg::TopologicalNavigationMap msg_map;
rospg::TopoNavEdgeMsg msg_edge;
rospg::TopoNavNodeMsg msg_node;
//int first_node = 0;

Pgsql::Pgsql(int argc, char**argv)
{
        //double frequency;

        //n.param("main_loop_frequency", frequency, double(4.0));

        //ros::Rate rate(frequency);
        ros::NodeHandle private_nh_("~");
        string map_topic = "topological_navigation_mapper/topological_navigation_map";
        pub1 = n.advertise<TopologicalNavigationMap>(map_topic,1000,true);

        //string pose_topic = "current_pose";
    //ROS_INFO("Waiting for the pose at topic: '%s'", pose_topic.c_str());
    //sub1 = n.subscribe("current_pose", 1000, &Pgsql::sendGoal, this);

	
	//get the start positon of the turtlebot from the launch files
	double start_x,start_y;
        private_nh_.param("start_x", start_x, -65.0);
        private_nh_.param("start_y", start_y, 200.0);
	
	double nx, ny; 
	int    nid;    
	int    start_id;
	start_id = getShortestPoint(nid, nx, ny, start_x, start_y);

	
	ROS_INFO("getShortestPath start!");

    //	std::stringstream ss;


    //connect to the database
    connection C("dbname=postgres user=postgres password=123456 \
      hostaddr=127.0.0.1 port=5432");
    if (C.is_open()) {
       ROS_INFO("Opened database successfully: %s",C.dbname());
    } else {
       //cout << "Can't open database" << endl;
    	ROS_INFO("Can't open database!");
    }

    char *sql;

    /* Create SQL statement to ADD column that needed */
    //sql = "ALTER TABLE roadpath ADD COLUMN source integer;ALTER TABLE roadpath ADD COLUMN target integer; ALTER TABLE roadpath ADD COLUMN length double precision;";

    /* Create a transactional object. */
    work W(C);
    /* Execute SQL query */
    //W.exec( sql );
    //W.commit();

    /* Create SQL statement to create topology */
    //sql = "SELECT pgr_createTopology('roadpath',0.001, 'geom', 'gid');";
    //W.exec( sql );
    // W.commit();

    /* Create SQL statement to add index */
    //sql = "CREATE INDEX source_idx1 ON roadpath(source);CREATE INDEX target_idx1 ON roadpath(target);update roadpath set length =st_length(geom);";
    //W.exec( sql );
    //W.commit();

    /* Create SQL statement to add costy */
    //sql = "ALTER TABLE roadpath ADD COLUMN reverse_cost double precision;UPDATE roadpath SET reverse_cost =length;";
    //W.exec( sql );
    //W.commit();

    /* Create SQL statement to get shortestpath by dijkstra
    sql = "SELECT seq, id1 AS node, cost FROM pgr_dijkstra('SELECT gid AS id,source::integer,target::integer,length::double precision AS cost FROM roadpath',2, 60, false, false);";
    W.exec( sql );
    //W.commit();


    sql = "SELECT st_astext(geom) FROM pgr_dijkstra('SELECT gid AS id,source::integer,target::integer,length::double precision AS cost FROM roadpath',1, 80, false, false)as di join roadpath pt on di.id2 = pt.gid;";
    W.exec( sql );*/

    char buf[255] = "";
    sprintf(buf, "SELECT seq,start_x, start_y,cost FROM pgr_dijkstra('SELECT gid AS id, source::integer, target::integer,length::double precision AS cost FROM roadpath',%d, 80,false, false) as di join roadpath pt on di.id2 = pt.gid;", start_id);
    
    sql = buf;
    //W.exec( sql );

    /* Create SQL statement to get result */
    //sql = "SELECT start_x,start_y,cost FROM dijkstra_res;";
    pqxx::result r = W.exec( sql );
    //W.commit();


    // for debug
    ROS_INFO("prepare position_goal");

    
    // set the stamp
    msg_map.header.stamp = ros::Time::now();
    msg_map.header.frame_id = "toponav_map";

    // set other parameters
    int count = 0;
    string data_node0, data_node1;
    string data_cost0, data_cost1;
    string start_x0,start_x1;
    string start_y0,start_y1;
    tf::Pose trans_pose1,trans_pose2;

    int id_inc = 0;
    for (pqxx::result::const_iterator row = r.begin(); row != r.end(); ++row)
    {
        int msg_id = 0;
        for (pqxx::tuple::const_iterator field = row->begin(); field != row->end(); ++field){
                switch(msg_id){
                    case 0:	data_node1 = field->c_str();break;
                    case 1:	start_x1 = field->c_str();break;
                    case 2:	start_y1 = field->c_str();break;
                    case 3:	data_cost1 = field->c_str();break;
                    default: break;
                }
                msg_id++;
        }
        //the pre-process
        if(count==0){
            data_node0 = data_node1;
            data_cost0 = data_cost1;
            start_x0 = start_x1;
            start_y0 = start_y1;
            count =1;
            continue;
        }

        //get all the data for the topomap
       //---------------------------------------------------------------
        // if the cost is larger than the rolling window , split the path
        int id0, id1;
        double x0, y0, x1, y1;
        double cost0, cost1;


        x0    = atof(start_x0.c_str());
        y0    = atof(start_y0.c_str());
        x1    = atof(start_x1.c_str());
        y1    = atof(start_y1.c_str());
        cost0 = atof(data_cost0.c_str());
        cost1 = atof(data_cost1.c_str());
        id0   = atoi((data_node0.c_str()));
        id1   = atoi((data_node1.c_str()));

        id0 = id0 + id_inc;


        int nn = ceil(cost0/10);
        id_inc += nn-1;
        double xc = (x1 - x0) / nn;
        double yc = (y1 - y0) / nn;
        double cc = (cost0)/nn;

        int    *iid   = new int[nn];
        double *xx    = new double[nn];
        double *yy    = new double[nn];
        double *ccost = new double[nn];

        for(int kk = 0; kk<nn; kk++){

            iid[kk]= id0 + kk;
            xx[kk] = x0 + xc*kk;
            yy[kk] = y0 + yc*kk;
            ccost[kk] = cc;


            // current  data are saved in data_node1, data_cost1
            // previous data are saved in data_node0, data_cost0
            stringstream ss;
            ss<<iid[kk];
            msg_edge.edge_id = ss.str();
            msg_edge.last_updated = ros::Time::now();

            msg_edge.start_node_id = iid[kk];
            msg_edge.type = 2;

            msg_edge.end_node_id = iid[kk]+1;
            msg_edge.cost = ccost[kk];

            // define a new node
            msg_node.node_id = iid[kk];
            msg_node.last_updated = ros::Time::now();
            msg_node.last_pose_updated = ros::Time::now();
            msg_node.area_id = 0;
            trans_pose1.setX(xx[kk]);
            trans_pose1.setY(yy[kk]);
            trans_pose1.position.z = 0;
            trans_pose1.orientation.setW(1.0);

            trans_pose2 = lookupPoseInFrame(trans_pose1, "map", "toponav_map");
            poseTFToMsg(trans_pose2, msg_node.pose);
            msg_node.is_door = false;

            // push back and deal with next data
            msg_map.edges.push_back(msg_edge);
            msg_map.nodes.push_back(msg_node);


            ROS_INFO("got position_goal");
        }

        // cache
        data_node0 = data_node1;
        data_cost0 = data_cost1;
        start_x0   = start_x1;
        start_y0   = start_y1;
    }

    pub1.publish(msg_map);
    
    W.commit();
    ROS_INFO("done correctly");
    C.disconnect ();
        
}

Pgsql::~Pgsql()
{

}

/*const tf::Pose Pgsql::getRobotPoseInFrame() const {
  tf::StampedTransform robot_transform_tf_stamped; //stores robots current pose as a stamped transform
  tf::Pose robot_pose; //stores robots current pose as a stamped transform

  try
  {
    tf_listener_.waitForTransform("toponav_map", "map", ros::Time(0), ros::Duration(10));
    tf_listener_.lookupTransform("toponav_map", "map", ros::Time(0), robot_transform_tf_stamped);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }

  robot_pose.setOrigin(robot_transform_tf_stamped.getOrigin());
  robot_pose.setRotation(robot_transform_tf_stamped.getRotation());

  ROS_DEBUG("Pose is x=%f, y=%f, theta=%f in frame %s",
      robot_pose.getOrigin().x(),
      robot_pose.getOrigin().y(),
      tf::getYaw(robot_pose.getRotation()),
      "toponav_map");

  return robot_pose;
}*/

const tf::Pose Pgsql::lookupPoseInFrame(tf::Pose input_pose, std::string source_frame, std::string target_frame) const {
  tf::Stamped<tf::Pose> output_pose_stamped;
  tf::Stamped<tf::Pose> input_pose_stamped(input_pose,ros::Time(0),source_frame);
  tf::Pose output_pose;

  try
  {
    tf_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10));
    tf_listener_.transformPose(target_frame, input_pose_stamped, output_pose_stamped);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }
  output_pose = output_pose_stamped;
  return output_pose;
}

//-----------------------------------------------------------------------------------
//get the shortest point to the position on roadpath
//the roadpath is stored in database
//return the shortest point for further process
int Pgsql::getShortestPoint(int &nid, double &nx, double &ny, double x, double y){
	nx     =  DBL_MAX;
	ny     =  DBL_MAX; 
	nid    =  -1; 
	double nd  = DBL_MAX;
	// 

	int      cid;   // current id
	double   cx,cy; // current x and y
        string   ids,xs,ys;

	 connection C("dbname=postgres user=postgres password=123456 \
      hostaddr=127.0.0.1 port=5432");
    if (C.is_open()) {
       ROS_INFO("Opened database successfully: %s",C.dbname());
    } else {
       //cout << "Can't open database" << endl;
       ROS_INFO("Can't open database!");
    }

    char *sql;

    work W(C);

	sql = "SELECT source,start_x,start_y FROM roadpath;";

    pqxx::result r = W.exec( sql );

        for (pqxx::result::const_iterator row = r.begin(); row != r.end(); ++row)
        {
            int msg_id = 0;
            for (pqxx::tuple::const_iterator field = row->begin(); field != row->end(); ++field){
		        switch(msg_id){
                            case 0:	ids = field->c_str();break;
                            case 1:	xs = field->c_str();break;
                            case 2:	ys = field->c_str();break;
		            
		            default: break;
            }

             cid = atoi(ids.c_str());
             cx = atof(xs.c_str());
             cy = atof(ys.c_str());

             msg_id++;

            // if nearer, replace
            if(((cx-x)*(cx-x) + (cy-y)*(cy-y)) < nd){
                    nx  = cx;
                    ny  = cy;
                    nid = cid;
                    nd  = ((cx-x)*(cx-x) + (cy-y)*(cy-y));
            }
        }
    }

	W.commit();
    //cout << "Table created successfully" << endl;
    C.disconnect ();
	
	return nid;
}

//------------------------------------------------------------------------------------------
//get the positon of the turtlebot and return the proper goal point to it
/*void Pgsql::sendGoal(const geometry_msgs::PoseStampedConstPtr &pose_in){


        geometry_msgs::PoseStamped current_pose;
        current_pose = *pose_in;

        tf::Pose robot_pose = getRobotPoseInFrame();

        int x,y,point_x,point_y;

        int i = 1;

        //ros::Rate rate(0.2);




        while(i<msg_map.nodes.size()){
            x = robot_pose.position.x;
            y = robot_pose.position.y;
            point_x = msg_map.nodes.at(i).pose.position.x;
            point_y = msg_map.nodes.at(i).pose.position.y;


            while(calcDistance(x,y,point_x,point_y) > 1){


                goal_pose.header.stamp = ros::Time();
                goal_pose.header.frame_id = "toponav_map";
                goal_pose.pose.position.x = msg_map.nodes.at(i).pose.position.x;
                goal_pose.pose.position.y = msg_map.nodes.at(i).pose.position.y;
                goal_pose.pose.position.z = 0;

                goal_pose.pose.orientation.w = 1;

                pub1.publish(goal_pose);
                //rate.sleep();
                //i++;
            }
            while(calcDistance(x,y,point_x,point_y) <= 1){

                goal_pose.header.stamp = ros::Time();
                goal_pose.header.frame_id = "toponav_map";
                goal_pose.pose.position.x = msg_map.nodes.at(i+1).pose.position.x;
                goal_pose.pose.position.y = msg_map.nodes.at(i+1).pose.position.y;
                goal_pose.pose.position.z = 0;

                goal_pose.pose.orientation.w = 1;
                pub1.publish(goal_pose);
                //rate.sleep();
                //i++;
            }
            i++;


        }

}


double Pgsql::calcDistance(double x,double y,double point_x,double point_y){
        return sqrt((point_x - x )*(point_x - x )+(point_y - y)*(point_y - y));
}
*/

