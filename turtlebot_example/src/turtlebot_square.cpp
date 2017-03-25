//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

const double pi = 3.141592;
const double pi2 = 2.0*pi;

const double max_pos_cmd = 0.5;
const double max_ang_cmd = 0.15;


double est_x;
double est_y;
double est_yaw;

bool initialized = false;
bool have_waypoints = false;

ros::Time last_time;

ros::Publisher goal_pub;

struct Waypoint
{
	double x;
	double y;

	double tol;
};
const int num_waypoints = 6;
std::vector<Waypoint> waypoints;

void PlanCallback(geometry_msgs::PoseArray msg)
{
    have_waypoints = false;
    waypoints.clear();

    for(int i=0; i < msg.poses.size(); i++)
    {
        Waypoint w;

        w.x = msg.poses.at(i).position.x;
        w.y = msg.poses.at(i).position.y;
        w.tol = 0.05;

        waypoints.push_back(w);
    }
    have_waypoints = true;
}



double SmoothAvg(double x, double last)
{
	double alpha = 1;
	double beta = 1 - alpha;

	return alpha*x + beta*last;
}

double Deg2Rad(double x)
{
	return x * pi / 180.0;
}

double Rad2Deg(double x)
{
	return x * 180.0 / pi;
}

class PID
{
  private:
      double Kp;
      double Kd;
      double Ki;
      
      double I;
      
      double iSat;
      double sat;
      
      bool been_used;
      double last_val;
      
  public:
      PID(double s)
      {
        sat = s;
        been_used = false;
      }
      
      
      void SetConstants(double p, double i, double d, double isat)
      {
        Kp = p;
        Kd = d;
        Ki = i;
        iSat = isat;
      }
      
      double Iterate(double val, double dt)
      {
          // Avoid derivative kick
          double dx;
          if (!been_used)
          {
            dx = 0;
          }
          else
          {
            dx = (val - last_val) / dt;
          }
          
          I = I + val;
          I = Saturate(I, iSat);
          
          last_val = val;
          been_used = true;
          
          double u = Kp*val + Kd*dx + Ki*I;
          
          u = Saturate(u, sat);
          ROS_INFO_STREAM("PID VAL " << val << ", CMD " <<u << ", dt " << dt);
          return u;
      }
      
      void Reset()
      {
          been_used = false;
          I = 0;
      }
      
    private:
      
      double Saturate(double val, double limit)
      {
          if (val > limit)
          {
              return limit;
          }
          if (val < -limit)
          {
              return -limit;
          }
          return val;
      }
};

double DistWaypoint(int i)
{
	double dx = waypoints.at(i).x - est_x;
	double dy = waypoints.at(i).y - est_y;
	return sqrt(dx*dx + dy*dy);
}

bool CloseToWaypoint(int i)
{
	double d = DistWaypoint(i);
	return d < waypoints[i].tol;
}

double AngleLimit(double x)
{
	return 2.0 / (1.0 + exp(0.01*x*x));
}

double ProcessAngle(double x)
{// Expect x in degrees

	// Make x positive
	x = fmod((x + 360),360);

	double px = x;

	double nx = x - 360;

	if (fabs(px) < fabs(nx))
	{
		return px;
	}
	else if (fabs(px) > fabs(nx))
	{
		return nx;
	}
	return 0;
}

//Callback function for the Position topic
void pose_callback(geometry_msgs::PoseWithCovarianceStamped msg)
//void pose_callback(const nav_msgs::Odometry& msg)
{
	//This function is called when a new pose message is received
	/*
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
	*/
    geometry_msgs::PoseWithCovariance p = msg.pose;

		initialized = true;
		est_x = p.pose.position.x;
		est_y = p.pose.position.y;
		est_yaw = Rad2Deg(tf::getYaw(p.pose.orientation));

	last_time = ros::Time::now();
}

const int num_goals = 3;
int goal_index = 0;
double goal_x[num_goals] = {4, 8, 8};
double goal_y[num_goals] = {0, -4, 0};

void SendGoals(int i)
{
    geometry_msgs::PoseArray pa;


    geometry_msgs::Pose p;

    p.position.x = est_x;
    p.position.y = est_y;
    pa.poses.push_back(p);

    p.position.x = goal_x[i];
    p.position.y = goal_y[i];
    pa.poses.push_back(p);

    goal_pub.publish(pa);
}



int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;
	ROS_INFO("NODE RUNNINGx");

	//Subscribe to the desired topics and assign callbacks
	//ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 5, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 5, pose_callback);

	//Setup topics to Publish from this node
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 5);
    
    goal_pub = n.advertise<geometry_msgs::PoseArray>("/goal", 5);

    ros::Subscriber plan_sub = n.subscribe("/plan_array", 5, PlanCallback);

	//Velocity control variable
	geometry_msgs::Twist vel;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;

	//Set the loop rate
	const int rate = 10;
	ros::Rate loop_rate(rate);    //20Hz update rate

	PID angle_pid = PID(max_ang_cmd);
	angle_pid.SetConstants(0.5, 0.1, 0.0, 0.05);

	PID vel_pid = PID(max_pos_cmd);
	vel_pid.SetConstants(0.1, 0.05, 0.0001, 0.05);

	while(!initialized && ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce(); 
	}
    
	int current_waypoint = 0;

    have_waypoints = false;
    SendGoals(0);

  ROS_INFO_STREAM("Starting Loop");
	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
    
    //ROS_INFO_STREAM("Checking Waypoint");
		if(have_waypoints && CloseToWaypoint(current_waypoint))
		{
      //ROS_INFO_STREAM("Grabbing New Waypoint");
			current_waypoint++;

            if (current_waypoint > waypoints.size())
            {
                have_waypoints = false;
                goal_index = goal_index + 1;
                SendGoals(goal_index);
                current_waypoint = 0;
            }
		}

        if (have_waypoints)
        {
          //ROS_INFO_STREAM("Have WP; Processing");
    		ros::Time now = ros::Time::now();
    		double dt = 0.1;

    		double goal_x = waypoints[current_waypoint].x;
    		double goal_y = waypoints[current_waypoint].y;

    		double err_x = goal_x - est_x;
    		double err_y = goal_y - est_y;

    		double angle = Rad2Deg(atan2(err_y, err_x));
    		double err_angle = angle - est_yaw;
    		err_angle = ProcessAngle(err_angle);

    		double d = DistWaypoint(current_waypoint);

    		double cmd_angle = angle_pid.Iterate(err_angle, dt);

    		double cmd_vel = vel_pid.Iterate(d, dt);
    		cmd_vel = cmd_vel * AngleLimit(err_angle);

    		vel.linear.x = cmd_vel;
    		vel.angular.z = cmd_angle;

    		
    		ROS_INFO("\n");
    		ROS_INFO("Goal, dt = %i, %f", current_waypoint, dt);
    		ROS_INFO("Est: (%f %f %f)", est_x, est_y, est_yaw);
    		ROS_INFO("Goal %i: (%f %f)", current_waypoint, goal_x, goal_y);
    		ROS_INFO("Err: (%f %f %f)", err_x, err_y, err_angle);
    		ROS_INFO("Dist: %f", d);
    		ROS_INFO("Cmd: (%f %f)", cmd_vel, cmd_angle);
    		
    		velocity_publisher.publish(vel);
        }
        else
        {
          ROS_INFO_STREAM("No Waypoints");
        }
	}

	return 0;
}
