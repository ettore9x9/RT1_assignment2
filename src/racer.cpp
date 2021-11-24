/* rosrun stage_ros stageros $(rospack find RT1_assignment2)/world/my_world.world */

#include "ros/ros.h"
#include "algorithm"
#include "cmath"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

void scanSectors( float * ranges, float * sectors );
void logic( float * sectors, geometry_msgs::Twist my_vel );
double integral( float values[], int start, int end );
void drive( float straight, float turn, geometry_msgs::Twist my_vel );

float d_br = 1.2; // float: Alert distance for avoiding obstacles, distance break.
float a_th = 20;
float speed;

int nsect = 51; // int: Number of sectors in which the space around the robot is divided.
int front = std::floor( nsect / 2 );
int sector_nelem = std::floor( 721/nsect );

ros::Publisher pub; // Global, so I can use it also in the Callback.

void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg) {

	float sectors[nsect];
	float ranges[721];
	fflush(stdout);
	std::fill_n(sectors, nsect, 10);

	geometry_msgs::Twist my_vel;

	for (int i = 0; i < 721; i++) {
		ranges[i] = msg->ranges[i];
	}

	scanSectors(ranges, sectors);

	logic(sectors, my_vel);
}

void scanSectors(float * ranges, float * sectors) {
    /*
    Function to search for the closest gold token in each sector.

    Return:
    dist_scan: float array, every element j is the smallest distance from a golden token
               detected in j-th sector.
    */
    for ( int i = 1; i <= nsect; i++ ) {
    	for ( int j = 0; j < sector_nelem; j++) {

    		// If it finds a closest token, than update the dist_scan array.
    		if ( ranges[i*sector_nelem+j] < sectors[i]) {
    		    sectors[i] = ranges[i*sector_nelem+j];
    		}

    	}

    }
    ROS_INFO("front distance : %f", sectors[front]);

}

void logic( float * sectors, geometry_msgs::Twist my_vel ) {

	double right_area, left_area;

	if ( sectors[front] > d_br ) {

		right_area = integral( sectors, front - 10, front );
		left_area = integral( sectors, front, front + 10 );

		ROS_INFO("left-side area : %.2f", left_area);
		ROS_INFO("rigt-side area : %.2f", right_area);



		if ( (left_area >= a_th) && (right_area >= a_th) ) {

			if ( sectors[front-1] < d_br ) {
				drive( speed/2, 0.8, my_vel );
				ROS_INFO("right sector control obstacle");

			} else if ( sectors[front+1] < d_br ) {
				drive( speed/2, -0.8, my_vel );
				ROS_INFO("left sector control obstacle");

			} else {
				drive( speed, 0.0, my_vel );
			}

		} else if ( left_area > right_area ) {
			drive( speed/3, 0.4, my_vel );
			ROS_INFO("right area control obstacle");

		} else {
			drive( speed/3, -0.4, my_vel );
			ROS_INFO("left area control obstacle");
		}

	} else {

		right_area = integral( sectors, 0, front );
		left_area = integral( sectors, front, nsect );

		ROS_INFO("OBSTACLE, left-side area : %.2f", left_area);
		ROS_INFO("OBSTACLE, rigt-side area : %.2f", right_area);

		if ( right_area > left_area ) {
			drive( 0.2, -8.0, my_vel );

		} else if ( left_area > right_area ){
			drive( 0.2, 8.0, my_vel );

		} else {
			drive( 0.0, 1.0, my_vel );
		}
	}
}

double integral( float * values, int start, int end ) {

	double result = 0;
	for ( int i = start; i < end; i++ ) {
		result = result + ( values[i] + values[i+1]) / 2;
	}
	return result;
}

void drive( float straight, float turn, geometry_msgs::Twist my_vel ){

	my_vel.linear.x = straight;
	my_vel.angular.z = turn;
	pub.publish(my_vel);
}

int main ( int argc, char ** argv ) {

	ros::init(argc, argv, "racer");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("base_scan", 1, functionCallback);

	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	speed = 5;

	// ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

	// turtlesim::Spawn srv1;
	// srv1.request.x = 1.0;
	// srv1.request.y = 5.0;
	// srv1.request.theta = 0.0;
	// srv1.request.name = "rt1_turtle";

	// client1.waitForExistence();
	// client1.call(srv1);
	
	ros::spin();

	return 0;
}