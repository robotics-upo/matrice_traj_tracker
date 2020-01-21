// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>

using namespace std;

#define FLOAT_SIGN(val) (val > 0.0) ? +1.0 : -1.0

// Global variables
bool fModeActive = false;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
float heightAboveTakeoff = -1000.0;
ros::Publisher controlPub;
double x_ref, y_ref, z_ref, yaw_ref;
bool refUpdated = false;
double max_vx, max_vy, max_vz, max_ry;
double min_vx, min_vy, min_vz, min_ry;

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

// RC callback to read if drone is in F mode (allowed for automatic control)
void rc_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
	if(msg->axes[4] < -1000)
		fModeActive = true;
	else
		fModeActive = false;
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

// Height above takeoff callback
void height_above_takeoff_callback(const std_msgs::Float32::ConstPtr& msg)
{
	heightAboveTakeoff = msg->data;
}

// Commanded robot trajectory
void input_trajectory_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
	// Get the reference position and orientation
	x_ref = msg->points[0].transforms[0].translation.x;
	y_ref = msg->points[0].transforms[0].translation.y;
	z_ref = msg->points[0].transforms[0].translation.z;
	tf::Quaternion q(msg->points[0].transforms[0].rotation.x, 
	                 msg->points[0].transforms[0].rotation.y, 
	                 msg->points[0].transforms[0].rotation.z, 
	                 msg->points[0].transforms[0].rotation.w);
	double r, p;
	tf::Matrix3x3 m(q);
	m.getRPY(r, p, yaw_ref);
	
	// Apply lower bound to the given refence
	if(fabs(x_ref) < 0.2)
		x_ref = 0.0;
	if(fabs(y_ref) < 0.2)
		y_ref = 0.0;
	if(fabs(z_ref) < 0.2)
		z_ref = 0.0;
	if(fabs(yaw_ref) < 0.1)
		yaw_ref = 0.0;
		
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
	
	// Command the computed velocities
	sendSpeedReference(vx, vy, vz, ry);
}


// Perform a monitored takeoff 
bool monitoredTakeoff(void)
{  
	ros::Time start_time = ros::Time::now();
	ros::Duration(0.01).sleep();

	// Step 1.1: Spin the motor
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

	// Step 1.2: Get into the air
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

	// Final check: Finished takeoff
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
	
	return true;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "matrice_traj_tracker_node");
	ros::NodeHandle nh("~");	
	
	// Create rc subscriber and control publisher
	ros::Subscriber rcSub = nh.subscribe<sensor_msgs::Joy>("/dji_sdk/rc", 1, &rc_callback);
	ros::Subscriber flightStatusSub = nh.subscribe("/dji_sdk/flight_status", 1, &flight_status_callback);
	ros::Subscriber displayModeSub = nh.subscribe("/dji_sdk/display_mode", 1, &display_mode_callback);
	ros::Subscriber heightAboveTakeoffSub = nh.subscribe("/dji_sdk/height_above_takeoff", 1, &height_above_takeoff_callback);
	ros::Subscriber trajectorySub = nh.subscribe("input_trajectory", 1, &input_trajectory_callback);
	controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 0);    

	// Required services
	ros::ServiceClient ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
	ros::ServiceClient drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

	// Read node parameters
	double takeoffHeight;
	double watchdogFreq;
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
		max_vz = 1.0;
	if(!nh.getParam("max_ry", max_ry))
		max_ry = 0.5;
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
	if(!authority.response.result)
	{
		ROS_ERROR("impossible to obtain drone control!");
		return -1;
	}
	ROS_INFO("\tdone!");

    // Perform automatic take-off
    ROS_INFO("Taking-off ...");
    dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
	drone_task_service.call(droneTaskControl);
	if(!droneTaskControl.response.result)
	{
		ROS_ERROR("takeoff fail!");
		return -1;
	}
	if(!monitoredTakeoff())
		return -1;
	ROS_INFO("\tdone!");

	// Fly up until reaching the takeoff altitude
	ROS_INFO("Climbing to takeoff height ...");
	while(heightAboveTakeoff-takeoffHeight < 0)
	{
		sendSpeedReference(0.0, 0.0, max_vz, 0.0);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}  
	sendSpeedReference(0.0, 0.0, 0.0, 0.0);
	ros::spinOnce();
	ROS_INFO("\tdone!");
	
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

