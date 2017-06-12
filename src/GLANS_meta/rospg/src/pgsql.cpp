#include "pgsql.h"
#include <pqxx/pqxx>
#include <iostream>
#include "stdafx.h"
#include <ros/ros.h>


using namespace std;
using namespace pqxx;


geometry_msgs::PoseStamped goal_pose;
rospg::TopologicalNavigationMap msg_map;
rospg::TopoNavEdgeMsg msg_edge;
rospg::TopoNavNodeMsg msg_node;


//int first_node = 0;
int node_i = 1;
int flag = 0;

Pgsql::Pgsql(int argc, char**argv)
{
        //double frequency;

        //n.param("main_loop_frequency", frequency, double(4.0));

        //ros::Rate rate(frequency);

        ros::NodeHandle private_nh_("~");
        pub1 = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000,true);

        pub2 = n.advertise<rospg::TopologicalNavigationMap>("TopologicalNavigationMap",1000,true);



	
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
    sprintf(buf, "SELECT seq,start_x, start_y,cost FROM pgr_dijkstra('SELECT gid AS id, source::integer, target::integer,length::double precision AS cost FROM roadpath',%d, 2,false, false) as di join roadpath pt on di.id2 = pt.gid;", start_id);

    //sql = buf;
    //W.exec( sql );

    /* Create SQL statement to get result */
    sql = "SELECT seq,start_x,start_y,cost FROM result_path;";
    //sql = "select * from result_path where seq > 10;";
    pqxx::result r = W.exec( sql );
    //W.commit();


    // for debug
    ROS_INFO("prepare position_goal");


    // set the stamp
    msg_map.header.stamp = ros::Time::now();
    msg_map.header.frame_id = "map";

    // set other parameters
    int count = 0;
    string data_node0, data_node1;
    string data_cost0, data_cost1;
    string start_x0,start_x1;
    string start_y0,start_y1;


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


        int nn = ceil(cost0/4.0);
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
            msg_node.pose.position.x = xx[kk] - start_x;
            msg_node.pose.position.y = yy[kk] - start_y;
            msg_node.pose.position.z = 0;
            msg_node.pose.orientation.w = 0.0;
            //msg_node.is_door = false;

            // push back and deal with next data
            //msg_map.edges.push_back(msg_edge);
            msg_map.nodes.push_back(msg_node);

            ROS_INFO("got position_goal x=%f, y=%f",msg_node.pose.position.x,msg_node.pose.position.y);
            //pub1.publish(msg_map);
        }

        // cache
        data_node0 = data_node1;
        data_cost0 = data_cost1;
        start_x0   = start_x1;
        start_y0   = start_y1;
    }

    /*if(first_node == 0){
        goal_pose.header.stamp = ros::Time();
        goal_pose.header.frame_id = "toponav_map";
        goal_pose.pose.position.x = msg_map.nodes.at(2).pose.position.x;
        goal_pose.pose.position.y = msg_map.nodes.at(2).pose.position.y;
        goal_pose.pose.position.z = 0;

        pub1.publish(goal_pose);
        //pub1.publish(msg_map);
        first_node++;
    }*/

    W.commit();
    ROS_INFO("done correctly");
    C.disconnect ();

    pub2.publish(msg_map);

    string pose_topic = "current_pose";
    ROS_INFO("Waiting for the pose at topic: '%s'", pose_topic.c_str());
    sub1 = n.subscribe("current_pose", 1000, &Pgsql::sendGoal, this);

}

Pgsql::~Pgsql()
{

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
void Pgsql::sendGoal(const geometry_msgs::PoseStampedConstPtr &pose_in){


        geometry_msgs::PoseStamped current_pose;
        current_pose = *pose_in;


        double x,y,point_x,point_y;

        //ros::Rate rate(0.1);

        /*tf::Pose input_pose,output_pose0,output_pose1;
        input_pose.getOrigin().setX(msg_map.nodes.at(node_i).pose.position.x);
        input_pose.getOrigin().setY(msg_map.nodes.at(node_i).pose.position.y);
        output_pose0 = lookupPoseInMap(input_pose);
        input_pose.getOrigin().setX(msg_map.nodes.at(node_i+1).pose.position.x);
        input_pose.getOrigin().setY(msg_map.nodes.at(node_i+1).pose.position.y);
        output_pose1 = lookupPoseInMap(input_pose);*/


        if(node_i<msg_map.nodes.size()){
            x = current_pose.pose.position.x;
            y = current_pose.pose.position.y;
            point_x = msg_map.nodes.at(node_i).pose.position.x;
            point_y = msg_map.nodes.at(node_i).pose.position.y;


            if(calcDistance(x,y,point_x,point_y) > 2.0){

                if(current_pose.pose.orientation.w > 0){

                    if((x-point_x)>2&&x<50&&(x-point_x)<5){

                        goal_pose.header.stamp = ros::Time();
                        goal_pose.header.frame_id = "map";
                        goal_pose.pose.position.x = msg_map.nodes.at(node_i+1).pose.position.x;
                        goal_pose.pose.position.y = msg_map.nodes.at(node_i+1).pose.position.y;
                        //goal_pose.pose.position.x = output_pose0.getOrigin().getX();
                        //goal_pose.pose.position.y = output_pose0.getOrigin().getY();
                        goal_pose.pose.position.z = 0.0;

                        goal_pose.pose.orientation.w = 1.0;

                        pub1.publish(goal_pose);
                        node_i++;
                    }
                    goal_pose.header.stamp = ros::Time();
                    goal_pose.header.frame_id = "map";
                    goal_pose.pose.position.x = msg_map.nodes.at(node_i).pose.position.x;
                    goal_pose.pose.position.y = msg_map.nodes.at(node_i).pose.position.y;
                    //goal_pose.pose.position.x = output_pose0.getOrigin().getX();
                    //goal_pose.pose.position.y = output_pose0.getOrigin().getY();
                    goal_pose.pose.position.z = 0.0;

                    goal_pose.pose.orientation.w = 1.0;

                    pub1.publish(goal_pose);
                    flag++;
                    //rate.sleep();
                    //i++;
                    ROS_INFO("far from the goal");
                    ROS_INFO("at the position x=%f , y=%f  ",x,y);
                    ROS_INFO("the goal is x=%f , y=%f  ",goal_pose.pose.position.x,goal_pose.pose.position.y);
                }

                if(current_pose.pose.orientation.w < 0){
                    if((point_x-x)>2&&x<50&&(point_x-x)<5){

                        goal_pose.header.stamp = ros::Time();
                        goal_pose.header.frame_id = "map";
                        goal_pose.pose.position.x = msg_map.nodes.at(node_i+1).pose.position.x;
                        goal_pose.pose.position.y = msg_map.nodes.at(node_i+1).pose.position.y;
                        //goal_pose.pose.position.x = output_pose0.getOrigin().getX();
                        //goal_pose.pose.position.y = output_pose0.getOrigin().getY();
                        goal_pose.pose.position.z = 0.0;

                        goal_pose.pose.orientation.w = 1.0;

                        pub1.publish(goal_pose);
                        node_i++;
                    }
                    goal_pose.header.stamp = ros::Time();
                    goal_pose.header.frame_id = "map";
                    goal_pose.pose.position.x = msg_map.nodes.at(node_i).pose.position.x;
                    goal_pose.pose.position.y = msg_map.nodes.at(node_i).pose.position.y;
                    //goal_pose.pose.position.x = output_pose0.getOrigin().getX();
                    //goal_pose.pose.position.y = output_pose0.getOrigin().getY();
                    goal_pose.pose.position.z = 0.0;

                    goal_pose.pose.orientation.w = 1.0;

                    pub1.publish(goal_pose);
                    flag++;
                    //rate.sleep();
                    //i++;
                    ROS_INFO("far from the goal");
                    ROS_INFO("at the position x=%f , y=%f  ",x,y);
                    ROS_INFO("the goal is x=%f , y=%f  ",goal_pose.pose.position.x,goal_pose.pose.position.y);
                }

                //ROS_INFO("the goal is x=%f , y=%f  ",output_pose0.getOrigin().getX(),output_pose0.getOrigin().getY());
                /*if(flag > 10){
                    goal_pose.header.stamp = ros::Time();
                    goal_pose.header.frame_id = "map";
                    //goal_pose.pose.position.x = output_pose1.getOrigin().getX();
                    //goal_pose.pose.position.y = output_pose1.getOrigin().getY();
                    goal_pose.pose.position.x = msg_map.nodes.at(node_i+1).pose.position.x;
                    goal_pose.pose.position.y = msg_map.nodes.at(node_i+1).pose.position.y;
                    goal_pose.pose.position.z = 0.0;

                    goal_pose.pose.orientation.w = 1.0;
                    pub1.publish(goal_pose);
                    node_i++;
                    ROS_INFO("near the goal");
                    ROS_INFO("the goal is x=%f , y=%f  ",goal_pose.pose.position.x,goal_pose.pose.position.y);
                    //ROS_INFO("set the next goal x=%f , y=%f  ",output_pose0.getOrigin().getX(),output_pose0.getOrigin().getY());
                    //cout<< node_i <<endl;
                    flag = 0;
                }*/


            }

            if(calcDistance(x,y,point_x,point_y) <= 1.0){

                goal_pose.header.stamp = ros::Time();
                goal_pose.header.frame_id = "map";
                goal_pose.pose.position.x = msg_map.nodes.at(node_i+1).pose.position.x;
                goal_pose.pose.position.y = msg_map.nodes.at(node_i+1).pose.position.y;
                //goal_pose.pose.position.x = output_pose1.getOrigin().getX();
                //goal_pose.pose.position.y = output_pose1.getOrigin().getY();
                goal_pose.pose.position.z = 0.0;

                goal_pose.pose.orientation.w = 1.0;
                pub1.publish(goal_pose);
                //rate.sleep();
                node_i++;
                ROS_INFO("near the goal");
                ROS_INFO("get the next goal x=%f , y=%f  ",goal_pose.pose.position.x,goal_pose.pose.position.y);
                //cout<< node_i <<endl;
                flag = 0;
            }


        }

}


double Pgsql::calcDistance(double x,double y,double point_x,double point_y){
        return sqrt((point_x - x )*(point_x - x )+(point_y - y)*(point_y - y));
}



tf::Pose Pgsql::lookupPoseInMap(tf::Pose input_pose){
    tf::Stamped<tf::Pose> output_pose_stamped;
    tf::Stamped<tf::Pose> input_pose_stamped(input_pose,ros::Time(0),"odom_ground_truth");
    tf::Pose output_pose;
    tf::TransformListener tf_listener_;

    try
    {
    tf_listener_.waitForTransform("map", "odom_ground_truth", ros::Time(0), ros::Duration(10));
    tf_listener_.transformPose("map", input_pose_stamped, output_pose_stamped);
    }
    catch (tf::TransformException &ex)
    {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
    }
  output_pose = output_pose_stamped;
  return output_pose;
}



