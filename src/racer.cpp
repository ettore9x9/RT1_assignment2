/* Run with: $ roslaunch rt1_assignment2 ass2.launch */

/* LIBRARIES */
#include "ros/ros.h"
#include "algorithm"
#include "cmath"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "rt1_assignment2/Command.h"

/* FUNCTION HEADERS */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg );
void scanSectors( float * ranges, float * sectors );
int logic( float * sectors );
void integral_logic( float * ranges );
double integral( float * values, int start, int end );
void drive( float straight, float turn );
bool server_response( rt1_assignment2::Command::Request &req, rt1_assignment2::Command::Response &res );

/* GLOBAL VARIABLES */
float d_br;                                   // Alert distance for avoiding obstacles, distance break.
float speed;								  // Speed of the robot.
int nsect = 9; 								  // Number of sectors.
int front = std::floor( nsect / 2 );          // Index of the frontal sector.
int sector_nelem = std::floor( 720/nsect );   // Number of laser surveys per sector.
ros::Publisher pub;                           // Publisher on cmd_vel.

/* FUNCTIONS */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg ) {
	/*
	Function callback for the base_ scan subscriber, it is executed each time something is published 
	in this specific topic.
	*/

	/* Preallocates variables. */
	float sectors[nsect];             // Closest distance from obstacle in each sector.
	float ranges[720];                // Copy of the laser_scan ranges.

	std::fill_n(sectors, nsect, 10);  // Initializes the values in the sectors float array to 10.

	/* Copies the ranges values in the base_scan message into an array. */
	for (int i = 0; i < 720; i++) {
		ranges[i] = msg -> ranges[i];
	}

	/* Calls the function scanSector to fill the sectors array. */
	scanSectors( ranges, sectors );

	/* Call the function logic to decide what to do. If it can not take a decision, then call the function
	   integral_logic. */
	if ( !logic( sectors ) ) {

		integral_logic( ranges );
	}
}

void scanSectors( float * ranges, float * sectors ) {
    /*
    Function to search for the closest obstacle in each sector.
    */
    for ( int i = 1; i <= nsect; i++ ) {           // For all sectors.
    	for ( int j = 0; j < sector_nelem; j++) {  // For all elements in each sector.

    		/* If it finds a closest obstacle, than update the sectors array. */
    		if ( ranges[i*sector_nelem+j] < sectors[i]) {
    		    sectors[i] = ranges[i*sector_nelem+j];
    		}

    	}

    }
}

int logic( float * sectors ) {
	/*
    Function that rapresents the robot's behaviour. It makes choices based on the values into the sectors array.

    Returns: 1 choice made
    		 0 choice not made
    */

	if ( sectors[front] > d_br ) { // The frontal sector is obstacle-free.

		/* Searchs in the front-right and in the front-left sectors if there are obstacles, to line up with
		   the track. */
		if ( (sectors[front+1] <= 0.8 * d_br) && (sectors[front-1] >= d_br) ) {
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn right", sectors[front], speed);
            drive( speed, -1 );

		} else if ( (sectors[front-1] <= 0.8 * d_br) && (sectors[front+1] >= d_br) ) {
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn left", sectors[front], speed);
            drive( speed, 1 );

		} else {
			ROS_INFO("dist: %.2f, speed: %.2f, free road", sectors[front], speed);
		    drive( speed, 0 );
		}

		return 1;

	} else { // There is an obstacle in the frontal sector.

		/* Searchs if there is an obstacle-free sector. */
		for ( int j = 1; j <= ( front - 1 ); j++ ) { // Looks in all sectors without the frontal one.

		    if ( (sectors[front+j] >= d_br ) && ( sectors[front+j] >= sectors[front-j] ) ){ // First looks left.
		    	ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn left", sectors[front], 0.2);
		        drive( 0.2, 4 );
		        return 1;

		    } else if ( ( sectors[front-j] >= d_br ) && ( sectors[front-j] >= sectors[front+j] ) ) { // Then looks right.
		        drive( 0.2, -4 );
		        ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn right", sectors[front], 0.2);
		        return 1;
		    }
		}
	}
	/* If there is not one obstacle-free sector, then it can not make any choice.*/
	return 0;
}

void integral_logic( float * ranges ) {
	/*
    Function to decide where to go when there are obstales all around the robot.
    The choice made is based on the free area swept by the laser scan.
    */

	double right_area = integral( ranges, 0, 360 );   // Right-side free area.
	double left_area = integral( ranges, 360, 720 );  // Left-side free area.

		if ( right_area > left_area ) {
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn right", right_area, 0.2);
			drive( 0.2, -8 );

		} else {
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn left", left_area, 0.2);
			drive( 0.2, 8 );

		}
}

double integral( float * values, int start, int end ) {
	/*
    Function to perform a discrete integral with the trapezium method.
    */

	double result = 0;
	for ( int i = start; i < end; i++ ) {
		result = result + ( values[i] + values[i+1]) / 2;
	}
	return result;
}

void drive( float straight, float turn ) {
	/*
    Function to drive the robot, filling the geometry message and publishing it on the topic cmd_vel.
    */

	geometry_msgs::Twist my_vel;
	my_vel.linear.x = straight;
	my_vel.angular.z = turn;
	pub.publish(my_vel);
}

bool server_response( rt1_assignment2::Command::Request &req, rt1_assignment2::Command::Response &res ) {
	/*
    Function callback to the command service, it sets the speed of the motor depending on the message 
    received and replies with the updated velocity.
    */

	if ( req.command == 'a' && speed >= 0.1 ) {
		ROS_INFO("Decrease speed");
		speed = speed - 0.1;
	}
	if ( req.command == 's' ) {
		ROS_INFO("Increase speed");
		speed = speed + 0.1;
	}
	if ( req.command == 'r' ) {
		ROS_INFO("Resetted");
		speed = 0.0;
	}
	if ( req.command == 'q' ) {
		ROS_INFO("Exit");
		sleep(2);
		exit(0);
	}

	/* The distance break depends on the velocity, when the robot has an higer speed it increases the d_br variable. */
	d_br = 1.2 + speed/18;

	res.max_speed = speed;

	return true;
}

/* MAIN */
int main ( int argc, char ** argv ) {
  	
  	/* Init the ros node. */
	ros::init(argc, argv, "racer");
	ros::NodeHandle nh;

	/* Initialises the values of the global variables speed and d_br. */
	speed = 0;
	d_br = 1.2;

	/* Subscribes to the topic base_scan to receive informations about the distance from obstacles from the 
	   laser scanner on the robot. */
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, functionCallback);

	/* Creates a service for the keyboard_pilot_node to control the speed of the robot, to reset the position, 
	   and to exit from the simulation. */
	ros::ServiceServer service = nh.advertiseService("/command", server_response);

	/* Creates a publisher to the cmd_vel topic, to guide the robot in real-time controlling its velocity. */
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::spin();

	return 0;
}