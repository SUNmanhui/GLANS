# ROS GLANS
## GIS based Large-scale Autonomous Navigation System
## Setup postgresql
### Installation postgresql
	sudo apt-get install postgresql 
	su - postgres 
	passwd ‘password’ 
	psql postgres 
	\q
### Modify the file 
	gedit /etc/postgresql/9.1/main/pg_hba.conf
### Reboot the service 
	root:# /etc/init.d/postgresql restart
	local	all	all	trust
	host	all	127.0.0.1/32	trust
### Login 
	root:# psql -U postgres -h 127.0.0.1
### Create database 
	psql -d postgres -U postgres -W
### Installation postgis
	sudo apt-get install postgresql-9.1-postgis
### Create database
	sudo su postgres createdb postgres
### Add extension to database
	sudo apt-get install postgresql-9.4-pgrouting
	install pgadminIII(software centre) to connect to database. 
	Right click on database postgres, select”new extension”，input postgis，add it.
	Add postgis_topology，pgrouting as well.
### Add topological roadpath to the database
	Add map.sql in the folder to the database which includes the topological path data. 
	The path data can be updated manually in the database is the corresponding environment changes. 
### Notice
	Make sure the database name is postgres, the username is postgres and the password is 123456.
	The position of the path data in the database has already been transformed from GPS to XYZ. 
	To view the transformation code you can go to cordTrans https://github.com/SUNmanhui/cordTrans.git
	The code of this part of connecting postgresql database is at rospg https://github.com/SUNmanhui/rospg.git


## Setup interface libpqxx
	sudo apt-get install libpqxx-4.0  
	sudo apt-get install libpqxx-dev 
	sudo apt-get install libpqxx-dbg 
	sudo apt-get install libpqxx-doc 
find postgresql installing folder , add the following to pg_hba.conf: <br>
	IPv4 local connections: <br>
	host    all         all         127.0.0.1/32          md5 <br>
Now the database can be connected by c++ code.

## Installation GLANS
This was tested on Indigo.  <br>
#### Create a catkin ws (or use your existing catkin ws): <br>
	mkdir -p ~/catkin_ws/src  
	cd ~/catkin_ws/src  
	catkin_init_workspace  
	cd ~/catkin_ws/  
	catkin_make   
	source devel/setup.bash  //add to .bashrc for convenience  
#### Add and compile the GLANS Metapackage  
	roscd  
	cd ../src  
	wstool init  
	wstool set GLANS_meta --git git://github.com/SUNmanhui/GLANS.git  
	wstool update GLANS_meta  
	rosdep install --from-paths . -i –y  
	cd .. 
	catkin_make  
`Use catkin_make -DCMAKE_BUILD_TYPE=Release for (faster) release build, catkin_make -DCMAKE_BUILD_TYPE=Debug for (slower) debuggable build. `

### Troubleshoot
	A GPU related issue can cause simulated runs to fail. If you experience such issues, please change gpu_ray to ray and libgazebo_ros_gpu_laser.so to libgazebo_ros_laser.so in lemtomap/lemto_description/urdf/lemto_turtle_gazebo.urdf.xacro

### Usage
#### View the simulation surroundings
	rosrun gazebo_ros gazebo GLANS/src/GLANS_meta/lemtomap/src/lemtomap_mata/lemto_gazebo/worlds/campus.world

#### Navigate in topological map
	roslaunch lemto_launchers test.launch  
`If you want to change the goal , you can modify it in the launch file.`

#### Drive around manually
	Use the arrow keys to increment speed, hit space to reset. 
	If you get an error about motors not being powered up, hit 'e'.
Or <br>  
	roslaunch lemto_launchers test_manul.launch <br>
and control it in rviz.

#### Send a metric goal
	Just use the RVIV '2D Nav Goal' tool on any unknown space (should be within the global map however)

#### Send a topological goal
	You can set a topological goal in test.launch file.

#### Load and Store metric maps
	save a map (execute for example from ~/catkin_ws/src/lemto_map_server/maps): <br>
	rosrun map_server map_saver -f mymap

#### view tf
	rosrun tf view_frames
