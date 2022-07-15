// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <upo_actions/Navigate3DAction.h>
#include <upo_actions/LandingAction.h>
#include <upo_actions/TakeOffAction.h>

#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>

//Necessary for use of rotors_simulator
#include <mav_msgs/Actuators.h>


using namespace std;
typedef actionlib::SimpleActionServer<upo_actions::Navigate3DAction> NavigationServer;
typedef actionlib::SimpleActionServer<upo_actions::LandingAction> LandingServer;
typedef actionlib::SimpleActionServer<upo_actions::TakeOffAction> TakeOffServer;

#define FLOAT_SIGN(val) (((val) > 0.0) ? +1.0 : -1.0)

// Global variables
bool fModeActive = false;
bool droneLanded = true;
std::string drone_type;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
double height = -100000.0, landingHeight = -100000.0;
ros::Publisher controlPub, speedMarkerPub, heightAboveTakeoffPub;
double x_ref, y_ref, z_ref, yaw_ref;
double control_factor, arrived_th_xyz, arrived_th_yaw;
tf::TransformListener *tfListener;
double goal_yaw;
double motors_speed[4];
double init_height= 0;

//Used to publish the real distance to the goal when arrived(Mostly debugging purposes)
double x_old, y_old, z_old;
bool refUpdated = false;
double max_vx, max_vy, max_vz, max_ry;
double min_vx, min_vy, min_vz, min_ry;

//Upo actions stuff
//Navigation server
std::unique_ptr<NavigationServer> navigationServer;
upo_actions::Navigate3DFeedback actionFb;
upo_actions::Navigate3DResult actionResult;
upo_actions::Navigate3DGoalConstPtr actionGoal;
//Landing Server
std::unique_ptr<LandingServer> landingServer;
upo_actions::LandingFeedback landingFb;
upo_actions::LandingResult landingResult;
upo_actions::LandingGoalConstPtr landingGoal;
//TakeOff Server
std::unique_ptr<TakeOffServer> takeOffServer;
upo_actions::TakeOffFeedback takeOffFb;
upo_actions::TakeOffResult takeOffResult;
upo_actions::TakeOffGoalConstPtr takeOffGoal;

//Service clients
ros::ServiceClient ctrl_authority_service,drone_task_service;
visualization_msgs::Marker speedMarker,rotMarker;
//Used to set the marker frame
std::string droneFrame, global_frame_id;
ros::Publisher fmode_pub;

//For the watchdog
ros::Time lastT, currentT;
ros::Duration watchdofPeriod;
bool gazebo_sim=false;
bool speed_ref_mode = false;
void configMarker(){

	geometry_msgs::Point p1,p2;
	p1.x=0;
	p1.y=0;
	p1.z=0;

	speedMarker.points.push_back(p1);
	speedMarker.points.push_back(p2);
	speedMarker.header.frame_id = droneFrame;
    speedMarker.header.stamp = ros::Time::now();
    speedMarker.ns = "matrice_traj_tracker_node";
    speedMarker.id = 1;
    speedMarker.type = visualization_msgs::Marker::ARROW;
    speedMarker.action = visualization_msgs::Marker::ADD;
    speedMarker.lifetime = ros::Duration(0.2);
    speedMarker.scale.y = 0.1;
    speedMarker.scale.z = 0.2;
    speedMarker.pose.position.x = 0;
    speedMarker.pose.position.y = 0;
    speedMarker.pose.position.z = 0.3;
    speedMarker.color.a = 1.0;
    speedMarker.color.b = 1.0;
    speedMarker.color.g = 1.0;
    speedMarker.color.r = 0.0;

	rotMarker.header.frame_id = droneFrame;
    rotMarker.header.stamp = ros::Time::now();
    rotMarker.ns = "matrice_traj_tracker_node";
    rotMarker.id = 2;
    rotMarker.type = visualization_msgs::Marker::ARROW;
    rotMarker.action = visualization_msgs::Marker::ADD;
    rotMarker.lifetime = ros::Duration(0.2);
    rotMarker.scale.y = 0.1;
    rotMarker.scale.z = 0.2;
    rotMarker.pose.position.x = 0;
    rotMarker.pose.position.y = 0;
    rotMarker.pose.position.z = 0.3;
    rotMarker.color.a = 1.0;
    rotMarker.color.b = 0.0;
    rotMarker.color.g = 0.0;
    rotMarker.color.r = 1.0;

}

void sendSpeedReference(float vx, float vy, float vz, float ry)
{
	// Build the message 
	sensor_msgs::Joy cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.axes.push_back(vx); // Vel X  
	cmd.axes.push_back(vy); // Vel Y
	cmd.axes.push_back(vz); // Vel Z
	cmd.axes.push_back(ry); // Yaw rate
	cmd.axes.push_back(0x40 | 0x00 | 0x08 | 0x02 | 0x00); // Control flags
								 // Control flags used: 
								 //		0x40 - Command horizontal velocities - Ground/Body - 30 m/s 
								 //     0x00 - Command the vertical speed - Ground - -5 to 5 m/s
								 //     0x08 - Command yaw rate - Ground - 5⁄6π rad/s 
								 //     0x02 - Horizontal command is body_FLU frame 
								 //     0x00 - No active break 
	
	// Publish the message if control is allowed by he remote controller
	if(fModeActive)
		controlPub.publish(cmd);
}

void sendRelPoseYaw(float x, float y, float z, float ry){
// Build the message 
	sensor_msgs::Joy cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.axes.push_back(x); // relative X in body  
	cmd.axes.push_back(y); // relative Y in body
	cmd.axes.push_back(z); // Z in world
	cmd.axes.push_back(ry); // Yaw rate
	cmd.axes.push_back(0x80 | 0x10 | 0x00 | 0x02 | 0x00); // Control flags
								 // Control flags used: 
								 //		0x80 - Command position offsets  
								 //     0x10 - Command altitude - Ground - 0 to 120 m 
								 //     0x00 - Command yaw angle - Ground - -π to π 
								 //     0x02 - Horizontal command is body_FLU frame 
								 //     0x00 - No active break 
	
	// Publish the message if control is allowed by he remote controller
	if(fModeActive)
		controlPub.publish(cmd);
}

void sendRelPoseReference(float x, float y, float z, float ry)
{
	// Build the message 
	sensor_msgs::Joy cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.axes.push_back(x); // relative X in body  
	cmd.axes.push_back(y); // relative Y in body
	cmd.axes.push_back(z); // Z in world
	cmd.axes.push_back(ry); // Yaw rate
	cmd.axes.push_back(0x80 | 0x10 | 0x08 | 0x02 | 0x00); // Control flags
								 // Control flags used: 
								 //		0x80 - Command position offsets  
								 //     0x10 - Command altitude - Ground - 0 to 120 m 
								 //     0x08 - Command yaw rate - Ground - 5⁄6π rad/s 
								 //     0x02 - Horizontal command is body_FLU frame 
								 //     0x00 - No active break 
	
	// Publish the message if control is allowed by he remote controller
	if(fModeActive)
		controlPub.publish(cmd);
}

void MotorSpeedCallback(const mav_msgs::ActuatorsConstPtr& msg_) 
{
	motors_speed[0] = msg_->angular_velocities[0];
	motors_speed[1] = msg_->angular_velocities[1];
	motors_speed[2] = msg_->angular_velocities[2];
	motors_speed[3] = msg_->angular_velocities[3];
}

// RC callback to read if drone is in F mode (allowed for automatic control)
void rc_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
	fModeActive = gazebo_sim;
	if(drone_type=="m210") {
		if(msg->axes[4] > 1000)
			fModeActive = true;
		else
			fModeActive = false;
	} else if(drone_type=="m600") {
		if(msg->axes[4] < -1000)
			fModeActive = true;
		else
			fModeActive = false;
	}
	std_msgs::Bool fmod_msg;
	fmod_msg.data = fModeActive;
	fmode_pub.publish(fmod_msg);
}	

// Drone flight status callback
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	flight_status = msg->data;
}

// Drone display mode callback
void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	display_mode = msg->data;
}

// GPS altitude callback
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	std_msgs::Float64 height_msg; 
	height = msg->altitude;
	if(landingHeight > -1000.0)
	{
		height_msg.data = msg->altitude-landingHeight;
		heightAboveTakeoffPub.publish(height_msg);
	}
}

// Commanded robot trajectory
void input_trajectory_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  //cout << 1 << endl;
  //Okey, if no goal active, forget about 
  if(!navigationServer->isActive())
    return;

  currentT=msg->header.stamp;
  if(currentT-lastT > watchdofPeriod){
    ROS_WARN("Input trajectory timeout...");
    lastT=currentT;
    sendSpeedReference(0, 0, 0, 0);
    return;
  }
  lastT=currentT;

  //cout << 2 << endl;
  // Get the reference position and orientation
  x_ref = msg->points[0].transforms[0].translation.x;
  y_ref = msg->points[0].transforms[0].translation.y;
  z_ref = msg->points[0].transforms[0].translation.z;
  tf::Quaternion q(msg->points[0].transforms[0].rotation.x, 
		   msg->points[0].transforms[0].rotation.y, 
		   msg->points[0].transforms[0].rotation.z, 
		   msg->points[0].transforms[0].rotation.w);
  double r, p, yaw, yaw_ref;
  tf::Matrix3x3 m(q);
  m.getRPY(r, p, yaw_ref);
	
  // PATCH to Fali's mess with orientation of final goal 
  // REMOVE IN THE FUTURE!!!!!
  tf::StampedTransform baseTf;
  try
    {
      tfListener->waitForTransform(global_frame_id, droneFrame, ros::Time(0), ros::Duration(.1));
      tfListener->lookupTransform(global_frame_id, droneFrame, ros::Time(0), baseTf);
    }
  catch (tf::TransformException ex)
    {
      ROS_ERROR("matrice_traj_tracker_node error: %s",ex.what());
      return;
    }
  tf::Matrix3x3 m2(baseTf.getRotation());
  m2.getRPY(r, p, yaw);
  yaw_ref = goal_yaw-yaw;

  // Apply lower bound to the given refence only when last wayoint is used
  if(msg->points.size() == 1)
    {
      if(fabs(x_ref) < arrived_th_xyz){
	x_old=x_ref;
	x_ref = 0.0;
      }
      if(fabs(y_ref) < arrived_th_xyz){
	y_old=y_ref;
	y_ref = 0.0;
      }
      if(fabs(z_ref) < arrived_th_xyz){
	z_old=z_ref;
	z_ref = 0.0;
      }
      if(fabs(yaw_ref) < arrived_th_yaw)
	yaw_ref = 0.0;

      //It means that we reached the goal
      if(x_ref == 0 && y_ref == 0 && z_ref == 0 && yaw_ref == 0 ){
	actionResult.arrived = true;
	actionResult.finalDist.data = sqrt(z_old*z_old+y_old*y_old+x_old*x_old);
	navigationServer->setSucceeded(actionResult,"3D Navigation Goal Reached");
      }
    }
  else
    {
      yaw_ref = 0.0;
    }
  if(speed_ref_mode){
    // Compute commmanded velocities
    double vx, vy, vz, ry;
    if(fabs(x_ref) > 1.0)
      vx = max_vx*FLOAT_SIGN(x_ref);
    else
      vx = max_vx*x_ref;
    if(fabs(y_ref) > 1.0)
      vy = max_vy*FLOAT_SIGN(y_ref);
    else
      vy = max_vy*y_ref;
    if(fabs(z_ref) > 1.0)
      vz = max_vz*FLOAT_SIGN(z_ref);
    else
      vz = max_vz*z_ref;
    if(fabs(yaw_ref) > 1.0)
      ry = max_ry*FLOAT_SIGN(yaw_ref);
    else
      ry = max_ry*yaw_ref;
    if(msg->points.size() == 1)
      {
	vx = control_factor*vx;
	vy = control_factor*vy;
	vz = control_factor*vz;
	ry = control_factor*ry;
      }
    // Command the computed velocities
    sendSpeedReference(vx, vy, vz, ry);
    actionFb.speed.linear.x = vx;
    actionFb.speed.linear.y = vy;
    actionFb.speed.linear.z = vz;
    actionFb.speed.angular.z = ry;
    //TODO: Fill dist2Goal feedback field, not important right now
    navigationServer->publishFeedback(actionFb);

    //Publish markers
    speedMarker.points[1].x = vx;
    speedMarker.points[1].y = vy;
    speedMarker.points[1].z = vz;

    rotMarker.scale.x = ry;

    speedMarkerPub.publish(speedMarker);
    speedMarkerPub.publish(rotMarker);
  }else{
    sendRelPoseYaw(x_ref, y_ref, z_ref+(height-landingHeight),yaw_ref);
  }	
}

// Perform a monitored takeoff 
bool monitoredTakeoff(void)
{  
	ros::Time start_time = ros::Time::now();
	ros::Duration(0.01).sleep();

	// Step 1.1: Spin the motor
	if(!gazebo_sim){
		while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
			display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
			ros::Time::now() - start_time < ros::Duration(5)) 
		{
			ros::Duration(0.01).sleep();
			ros::spinOnce();
		}
		if(ros::Time::now() - start_time > ros::Duration(5)) 
		{
			ROS_ERROR("\tTakeoff failed. Motors are not spinnning!!");
			return false;
		}
		else 
		{
			start_time = ros::Time::now();
			ROS_INFO("\tMotor Spinning ...");
			ros::spinOnce();
		}
	}
	else{
		while ( motors_speed[0] < 300 && motors_speed[1] < 300 && motors_speed[2] < 300 && motors_speed[3] < 300 &&
			ros::Time::now() - start_time < ros::Duration(10)) 
		{
			ros::Duration(0.01).sleep();
			sendSpeedReference(0.0, 0.0, 0.6, 0.0);
			ros::spinOnce();
		}
		if(ros::Time::now() - start_time > ros::Duration(10)) 
		{
			ROS_ERROR("\tTakeoff failed. Motors are not spinnning!!");
			sendSpeedReference(0.0, 0.0, 0.0, 0.0);
			return false;
		}
		else 
		{
			start_time = ros::Time::now();
			ROS_INFO("\tMotor Spinning ...");
			ros::spinOnce();
		}
	}
	// Step 1.2: Get into the air
	if(!gazebo_sim){
		while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
			(display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
			ros::Time::now() - start_time < ros::Duration(20)) 
		{
			ros::Duration(0.01).sleep();
			ros::spinOnce();
		}
		if(ros::Time::now() - start_time > ros::Duration(20)) 
		{
			ROS_ERROR("\tTakeoff failed. Aircraft is still on the ground, but the motors are spinning.");
			return false;
		}
		else 
		{
			start_time = ros::Time::now();
			ROS_INFO("\tAscending...");
			ros::spinOnce();
		}
	}
	else{
		while ( motors_speed[0] < 550 && motors_speed[1] < 550 && motors_speed[2] < 550 && motors_speed[3] < 550 &&
			ros::Time::now() - start_time < ros::Duration(10)) 
		{
			ros::Duration(0.01).sleep();
			sendSpeedReference(0.0, 0.0, 1.0, 0.0);
			ros::spinOnce();
		}
		if(ros::Time::now() - start_time > ros::Duration(10)) 
		{
			ROS_ERROR("\tTakeoff failed. Aircraft is still on the ground, but the motors are spinning.");
			sendSpeedReference(0.0, 0.0, 0.0, 0.0);
			return false;
		}
		else 
		{
			start_time = ros::Time::now();
			ROS_INFO("\tAscending...");
			ros::spinOnce();
		}
	}
	// Step 1.3: Final check: Finished takeoff
	if(!gazebo_sim){
		while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
			ros::Time::now() - start_time < ros::Duration(20)) 
		{
			ros::Duration(0.01).sleep();
			ros::spinOnce();
		}
		if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
		{
			ROS_INFO("\tSuccessful takeoff!");
			start_time = ros::Time::now();
		}
		else
		{
			ROS_ERROR("\tTakeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
			return false;
		}
	}
	else{
		while ( motors_speed[0] < 550 && motors_speed[1] < 550 && motors_speed[2] < 550 && motors_speed[3] < 550 && (height - init_height) > 1.0 &&
			ros::Time::now() - start_time < ros::Duration(20)) 
		{
			ros::Duration(0.01).sleep();
			sendSpeedReference(0.0, 0.0, 1.0, 0.0);
			ros::spinOnce();
		}

		if (ros::Time::now() - start_time < ros::Duration(20))
		{
			ROS_INFO("\tSuccessful takeoff!");
			start_time = ros::Time::now();
		}
		else
		{
			ROS_ERROR("\tTakeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
			sendSpeedReference(0.0, 0.0, 0.0, 0.0);
			return false;
		}
	}

	return true;
}

void navigateGoalCallback(){
  if(droneLanded){
    ROS_ERROR("Not possible to navigate, take off first");
    return;
  }
  else{
    ROS_INFO("Matrice_traj_tracker_node: Receiving new goal to navigate");
  }
  //TODO: Okey if a new goal is while we are navigating to another goal it means that the trajectory has been re-calculated. So what to do?
  actionGoal = navigationServer->acceptNewGoal();
  //As it you don't refuse the first trajectory
  currentT = ros::Time::now();

  // Get yaw from goal
  tf::Quaternion q(actionGoal->global_goal.pose.orientation.x, 
		   actionGoal->global_goal.pose.orientation.y, 
		   actionGoal->global_goal.pose.orientation.z, 
		   actionGoal->global_goal.pose.orientation.w);
  double r, p, yaw, yaw_ref, yaw_goal;

  tf::Matrix3x3 m(q);
  m.getRPY(r, p, yaw_goal);

  tf::StampedTransform baseTf;
  tf::Stamped<tf::Point> global_goal_point, local_goal_point;
  global_goal_point.frame_id_ = global_frame_id;
  global_goal_point.setX(actionGoal->global_goal.pose.position.x);
  global_goal_point.setY(actionGoal->global_goal.pose.position.y);
  global_goal_point.setZ(actionGoal->global_goal.pose.position.z);

  ROS_INFO("Matrice_traj_tracker_node:	goal=[%f %f %f / %f]", global_goal_point.getX(),
	   global_goal_point.getY(),
	   global_goal_point.getZ(), yaw_goal);

  bool achieved_x_ = false, achieved_y_ = false, achieved_z_ = false, achieved_yaw_ = false;
  ros::Time start_time = ros::Time::now();
  while ( !(achieved_x_ && achieved_y_ && achieved_yaw_ && achieved_z_) && ros::ok()) 
    {
      try
	{
	  tfListener->waitForTransform(global_frame_id, droneFrame, ros::Time(0), ros::Duration(.1));
	  tfListener->lookupTransform(global_frame_id, droneFrame, ros::Time(0), baseTf);
	  tfListener->transformPoint(droneFrame, ros::Time(0), global_goal_point, global_frame_id, local_goal_point);
	}
      catch (tf::TransformException ex)
	{
	  ROS_ERROR("matrice_traj_tracker_node error: %s",ex.what());
	  return;
	}
      tf::Matrix3x3 m2(baseTf.getRotation());
      m2.getRPY(r, p, yaw);
		
      tf::Vector3 tf_traslation_; 
      tf_traslation_ = baseTf.getOrigin();

      yaw_ref = yaw_goal - yaw; // Beware of the distances more or less than M_PI
      if (yaw_ref < -M_PI) 
	yaw_ref += 2*M_PI;
      else if (yaw_ref > M_PI)
	yaw_ref -= 2*M_PI;
      x_ref = local_goal_point.getX();
      y_ref = local_goal_point.getY();
      z_ref = local_goal_point.getZ();

      achieved_x_ = achieved_y_ = achieved_z_ = achieved_yaw_ = false;
      if(fabs(x_ref) < arrived_th_xyz)
	achieved_x_ = true;
      if(fabs(y_ref) < arrived_th_xyz)
	achieved_y_ = true;
      if(fabs(z_ref) < arrived_th_xyz)
	achieved_z_ = true;
      if(fabs(yaw_ref) < arrived_th_yaw)
	achieved_yaw_ = true;

      if(speed_ref_mode){
	// Compute commmanded velocitiesx_ref
	double vx, vy, vz, ry;
	vx = vy = vz = ry = 0.0;

	if(fabs(x_ref) > 1.0)
	  vx = max_vx * FLOAT_SIGN(x_ref);
	else
	  vx = (max_vx - min_vx) * x_ref + min_vx * FLOAT_SIGN(x_ref);
	if(fabs(y_ref) > 1.0)
	  vy = max_vy * FLOAT_SIGN(y_ref);
	else
	  vy = (max_vy - min_vy) * y_ref + min_vy * FLOAT_SIGN(y_ref);
	if(fabs(z_ref) > 1.0)
	  vz = max_vz * FLOAT_SIGN(z_ref);
	else
	  vz = (max_vz - min_vz) * z_ref + min_vz * FLOAT_SIGN(z_ref);
	if(fabs(yaw_ref) > 1.0)
	  ry = max_ry*FLOAT_SIGN(yaw_ref);
	else
	  ry = max_ry*yaw_ref;

	printf("error[%.2f %.2f %.2f / %.2f]  Commands: [%.2f %.2f %.2f / %.2f]",
	       x_ref,y_ref,z_ref,yaw_ref, vx, vy, vz, ry);
	printf("\tSpeeds: [%.2f-%.2f %.2f-%.2f %.2f-%.2f/ %.2f-%.2f]            \r",
	       min_vx, max_vx, min_vy, max_vy,
	       min_vz, max_vz, min_ry, max_ry);


	// Command the computed velocities
	sendSpeedReference(vx, vy, vz, ry);
	actionFb.speed.linear.x = vx;
	actionFb.speed.linear.y = vy;
	actionFb.speed.linear.z = vz;
	actionFb.speed.angular.z = ry;
	//TODO: Fill dist2Goal feedback field, not important right now
	navigationServer->publishFeedback(actionFb);

	//Publish markers
	speedMarker.points[1].x = vx * cos(yaw) - vx *sin(yaw);
	speedMarker.points[1].y = vy * cos(yaw) + vx *sin(yaw);
	speedMarker.points[1].z = vz;

	rotMarker.scale.x = ry;

	speedMarkerPub.publish(speedMarker);
	speedMarkerPub.publish(rotMarker);
      } else {
	ROS_ERROR("Relative pose control implemented yet");
	// sendRelPoseYaw(x_ref, y_ref, z_ref+(height-landingHeight),yaw_ref);
      }	
    }
  ROS_INFO("Matrice_traj_tracker_node: Achieved Goal!!!");
  actionResult.arrived = true;
  actionResult.finalDist.data = sqrt(z_ref*z_ref+y_ref*y_ref+x_ref*x_ref);
  navigationServer->setSucceeded(actionResult,"3D Navigation Goal Reached");
  sendSpeedReference(0.0, 0.0, 0.0, 0.0);
}

//Okey, this callback is reached when publishing over Navigate3D/cancel topic a empty message
void navigatePreemptCallback(){

  actionResult.arrived = false;
  actionResult.finalDist.data = sqrt(z_old*z_old+y_old*y_old+x_old*x_old);//TODO: Put the real one
  navigationServer->setPreempted(actionResult,"3D Navigation Goal Preempted Call received");
}

void landingGoalCallback(){

  landingGoal = landingServer->acceptNewGoal();

  if(droneLanded){
    std::string error_msg = "Not possible to land, drone already landed";
    ROS_ERROR("Error %s", error_msg.c_str());
    landingResult.success=false;
    landingResult.extra_info=error_msg;
    landingServer->setAborted(landingResult);
    return;
  }
  if(!fModeActive){
    std::string error_msg = "Drone is not in F-Mode, aborting landing";
    ROS_ERROR("Error %s", error_msg.c_str());
    landingResult.success=false;
    landingResult.extra_info=error_msg;

    landingServer->setAborted(landingResult);
    return;
  }
  ROS_INFO("Requesting landing...");
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
    {
      std::string error_msg {"Drone task control bad response"};
      ROS_ERROR("Error %s", error_msg.c_str());
      landingResult.success=false;
      landingResult.extra_info=error_msg;
      landingServer->setAborted(landingResult);
      return;
    }

  landingResult.success = true;
  landingResult.extra_info = "Landing OK";

  landingServer->setSucceeded(landingResult);
  droneLanded=true;
  ROS_INFO("Drone Landed");
	
}

void takeOffGoalCallback(){

  takeOffGoal = takeOffServer->acceptNewGoal();

  if(!droneLanded)
    {
      std::string error_msg = "Take off done before";
      ROS_ERROR("Error %s", error_msg.c_str());
      takeOffResult.success=false;
      takeOffResult.extra_info=error_msg;
      takeOffServer->setAborted(takeOffResult);
      return;
    }
  // Get takeoff altitude
  ROS_INFO("Checking takeoff altitude ...");
  while(takeOffGoal->takeoff_height.data < -10000.0)
    {
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    } 
  landingHeight = height; 
  ROS_INFO("\tdone!");

  // Check the drone is in F-Mode before starting
  ROS_INFO("Checking drone is in F-Mode ...");

  while(!fModeActive)
    {
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }  
  ROS_INFO("\tdone!");

  // Get control authority
  ROS_INFO("Getting control authority ...");
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  ctrl_authority_service.call(authority);
  if(!authority.response.result && (!gazebo_sim))
    {
      std::string error_msg {"impossible to obtain drone control!"};
      ROS_ERROR("Error %s", error_msg.c_str());

      takeOffResult.success=false;
      takeOffResult.extra_info=error_msg;
      takeOffServer->setAborted(takeOffResult);
      return;
    }
	
  ROS_INFO("\tdone!");

  // Perform automatic take-off
  ROS_INFO("Taking-off ...");
  dji_sdk::DroneTaskControl droneTaskControl;
	
  droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
		
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result && (!gazebo_sim))
    {
      std::string error_msg {"Drone task control bad response"};
      ROS_ERROR("Error %s", error_msg.c_str());
      takeOffResult.success=false;
      takeOffResult.extra_info=error_msg;
      takeOffServer->setAborted(takeOffResult);
      return;
    }
	
  init_height = height;
  if(!monitoredTakeoff()){
    std::string error_msg {"Monitored takeoff error"};
    ROS_ERROR("Error %s", error_msg.c_str());
    takeOffResult.success=false;
    takeOffResult.extra_info=error_msg;
    takeOffServer->setAborted(takeOffResult);
    return;
  }
  ROS_INFO("\tdone!");

  // Fly up until reaching the takeoff altitude
  ROS_INFO("Climbing to takeoff height ...");
  while((height-landingHeight)-takeOffGoal->takeoff_height.data < 0)
    {
      takeOffFb.percent_achieved.data = (height-landingHeight)-takeOffGoal->takeoff_height.data;
      takeOffServer->publishFeedback(takeOffFb);
      sendSpeedReference(0.0, 0.0, max_vz, 0.0);
      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }  
  sendSpeedReference(0.0, 0.0, 0.0, 0.0);
  ros::spinOnce();
  ROS_INFO("\tdone!");
  takeOffResult.success=true;
  takeOffResult.extra_info="TakeOff Done";
	

  /*sendRelPoseReference(0.0, 0.0, takeOffGoal->takeoff_height.data, 0.0);
    ros::spinOnce();
    while(fabs((height-landingHeight)-takeOffGoal->takeoff_height.data) < 0.3)
    {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
    } 
    ROS_INFO("\tdone!");
    takeOffResult.success=true;
    takeOffResult.extra_info="TakeOff Done";
  */
  takeOffServer->setSucceeded(takeOffResult);
  droneLanded = false;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "matrice_traj_tracker_node");
  ros::NodeHandle nh("~");	
	
  //Publish speed arrow marker
  speedMarkerPub = nh.advertise<visualization_msgs::Marker>("speedMarker", 2);
  // Create rc subscriber and control publisher
  ros::Subscriber rcSub = nh.subscribe<sensor_msgs::Joy>("/dji_sdk/rc", 1, &rc_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("/dji_sdk/flight_status", 1, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("/dji_sdk/display_mode", 1, &display_mode_callback);
  ros::Subscriber gps = nh.subscribe("/dji_sdk/gps_position", 1, &gps_callback);
  ros::Subscriber trajectorySub = nh.subscribe("/input_trajectory", 1, &input_trajectory_callback);
  ros::Subscriber cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe("/firefly/command/motor_speed", 1, &MotorSpeedCallback);


  controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 0);    
  heightAboveTakeoffPub = nh.advertise<std_msgs::Float64>("/height_above_takeoff", 0);
	
  // Start TF listener
  tfListener = new tf::TransformListener;

  // Required services
  ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

  //Start action server to communicate with local planner
  navigationServer.reset(new NavigationServer(nh,"/UAVNavigation3D",false));
  navigationServer->registerGoalCallback(boost::function<void()>(navigateGoalCallback));
  navigationServer->registerPreemptCallback(boost::function<void()>(navigatePreemptCallback));
  navigationServer->start();

  //Start action server to receive takeoff requests
  takeOffServer.reset(new TakeOffServer(nh,"/TakeOff",false));
  takeOffServer->registerGoalCallback(boost::function<void()>(takeOffGoalCallback));
  takeOffServer->start();

  //Start action server to receive landing commands
  landingServer.reset(new LandingServer(nh,"/Landing",false));
  landingServer->registerGoalCallback(boost::function<void()>(landingGoalCallback));
  landingServer->start();

  // Fmode publisher
  fmode_pub = nh.advertise<std_msgs::Bool>("fmode", 1, true);
	
  // Read node parameters
  double takeoffHeight;
  double watchdogFreq;
  nh.param("drone_model", drone_type, (std::string)"m210");
  if(!nh.getParam("gazebo_sim", gazebo_sim)){
    gazebo_sim=false;
  }
  fModeActive = gazebo_sim;
  std_msgs::Bool msg;
  msg.data = fModeActive;
  fmode_pub.publish(msg);
  nh.param("global_frame_id", global_frame_id, static_cast<std::string>("world"));
  if(!nh.getParam("drone_frame", droneFrame)){
    droneFrame = "matrice";
  }
  ROS_INFO("Using drone frame %s. Global frame: %s",droneFrame.c_str(), global_frame_id.c_str());
  configMarker();
  if(!nh.getParam("takeoff_height", takeoffHeight))
    takeoffHeight = 3.0;
  if(takeoffHeight < 2.0 || takeoffHeight > 10.0)
    {
      ROS_INFO("takeoff_height must be in range 2.0 to 10.0, setting to 3.0");
      takeoffHeight = 3.0;
    }	
  if(!nh.getParam("max_vx", max_vx))
    max_vx = 1.0;
  if(!nh.getParam("max_vy", max_vy))
    max_vy = 1.0;	
  if(!nh.getParam("max_vz", max_vz))
    max_vz = 0.5;
  if(!nh.getParam("max_ry", max_ry))
    max_ry = 0.2;
  if(!nh.getParam("min_vx", min_vx))
    min_vx = 0.5;
  if(!nh.getParam("min_vy", min_vy))
    min_vy = 0.5;	
  if(!nh.getParam("min_vz", min_vz))
    min_vz = 0.5;
  if(!nh.getParam("min_ry", min_ry))
    min_ry = 0.25;
  if(!nh.getParam("watchdog_freq", watchdogFreq))
    watchdogFreq = 5.0;
  nh.param("arrived_th_xyz", arrived_th_xyz, 0.25);
  nh.param("arrived_th_yaw", arrived_th_yaw, 0.2);
  nh.param("control_factor", control_factor, 0.2);
  nh.param("speed_reference_mode", speed_ref_mode, false);

  watchdofPeriod = ros::Duration(1/watchdogFreq);

  // Spin forever
  ros::spin();
	
  /* Send commands at 10 hz
     ros::Rate r(10);
     vx = 0.0, vy = 0.0, vz = 0.0, ry = 0.0;
     while(!endProgram)
     {
		
		
     // Reset velocities
     vx = 0.0, vy = 0.0, vz = 0.0, ry = 0.0;
		
     // Ros spin and sleep
     ros::spinOnce();
     r.sleep();
     }*/
	
  return 0;
}
