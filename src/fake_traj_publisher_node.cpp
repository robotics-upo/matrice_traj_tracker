#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "fake_traj_publisher_node");
    ros::NodeHandle lnh("~");
    ros::Publisher traj_pub = lnh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/input_trajectory", 1, false);
    trajectory_msgs::MultiDOFJointTrajectory traj;
    trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point;
    traj_point.transforms.resize(1);
    traj.points.resize(1);
    traj.points[0] = traj_point;
    double dist, or_dist;
    std::string dir;

    if (argc == 3)
    {
        dir = argv[1];
        dist = std::atof(argv[2]);
        or_dist = dist;
        ROS_INFO("Sending trajectory in %s direction of %.2f meters(rad)", dir.c_str(), dist);

        if (dir == "x")
        {
            ROS_INFO("Publishing in x direction");
            traj_point.transforms[0].translation.x = dist;
            traj_point.transforms[0].rotation.w = 1;
        }
        else if (dir == "y")
        {
            ROS_INFO("Publishing in y direction");
            traj_point.transforms[0].translation.y = dist;
            traj_point.transforms[0].rotation.w = 1;
        }
        else if (dir == "z")
        {
            ROS_INFO("Publishing in z direction");
            traj_point.transforms[0].translation.z = dist;
            traj_point.transforms[0].rotation.w = 1;
        }
        else if (dir == "yaw")
        {
            ROS_INFO("Publishing in yaw");
            double r, p, y;
            tf::Quaternion q;
            q.setRPY(0, 0, dist);
            traj_point.transforms[0].rotation.x = q.getX();
            traj_point.transforms[0].rotation.z = q.getY();
            traj_point.transforms[0].rotation.y = q.getZ();
            traj_point.transforms[0].rotation.w = q.getW();
        }
        else
        {
            ROS_WARN("Direction of mouvement should be x y z or yaw");

            exit(0);
        }
        traj.points[0] = traj_point;
    }
    else
    {
        ROS_WARN("You should give the direction of mouvement and the distance rosrun matrice_traj_tracker fake_traj_publisher_node");
        exit(0);
    }
    double rate;
    lnh.param("rate", rate, 10.0);
    ros::Rate loop_r(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        dist -= std::abs(or_dist) / or_dist * 1 / rate;
        ROS_INFO("%.2f", dist);
        if (dir == "x")
        {
            traj_point.transforms[0].translation.x = dist;
        }
        else if (dir == "y")
        {
            traj_point.transforms[0].translation.y = dist;
        }
        else if (dir == "z")
        {
            traj_point.transforms[0].translation.z = dist;
        }
        else if (dir == "yaw")
        {
            tf::Quaternion q;
            q.setRPY(0, 0, dist);
            traj_point.transforms[0].rotation.x = q.getX();
            traj_point.transforms[0].rotation.z = q.getY();
            traj_point.transforms[0].rotation.y = q.getZ();
            traj_point.transforms[0].rotation.w = q.getW();
        }

        if (dist*or_dist < 0)
        {
            break;
        }

        traj.points[0] = traj_point;
        traj_pub.publish(traj);
        loop_r.sleep();
    }
}