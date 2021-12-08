/* Run with: $ roslaunch rt1_assignment2 ass2.launch */

/* LIBRARIES */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "rt1_assignment2/Command.h"
#include "std_srvs/Empty.h"
#include "stdio.h"
#include "unistd.h"
#include "termios.h"

/* Defines a raw string literal to print the title of the console. */
const char * title = R"(

   ##################################################################

    _              _                         _         _ _       _   
   | |            | |                       | |       (_) |     | |  
   | | _____ _   _| |__   ___   __ _ _ __ __| |  _ __  _| | ___ | |_ 
   | |/ / _ \ | | | '_ \ / _ \ / _` | '__/ _` | | '_ \| | |/ _ \| __|
   |   <  __/ |_| | |_) | (_) | (_| | | | (_| | | |_) | | | (_) | |_ 
   |_|\_\___|\__, |_.__/ \___/ \__,_|_|  \__,_| | .__/|_|_|\___/ \__|
              __/ |                             | |                  
             |___/                              |_|                 


   ##################################################################

             
   s : increase max speed of the robot.
   a : decrease max speed of the robot.
   r : put the robot in the initial position.
   q : quit the keyboard pilot.


)";

char getch( void ) {
   /* Function to acquire a character from keyboard input, setting the attributes of the terminal to avoid pressing 
      enter every time. 
      It returns the character typed from the user. */

   struct termios oldt, newt; // Structs for termilal attributes.

   tcgetattr( STDIN_FILENO, &oldt ); // Gets the old attributes.
   newt = oldt;

   /* Changes terminal attributes. */
   newt.c_lflag &= ~(ICANON | ECHO);
   newt.c_lflag |= IGNBRK;
   newt.c_lflag &= ~(INLCR | ICRNL | IXON | IXOFF);
   newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
   newt.c_cc[VMIN] = 1;
   newt.c_cc[VTIME] = 0;

   tcsetattr( STDIN_FILENO, TCSANOW, &newt ); // Sets new attributes.

   char ch = getchar(); // Gets the input character.

   tcsetattr( STDIN_FILENO, TCSANOW, &oldt ); // Sets old attributes

   return ch;

}

/* MAIN */
int main ( int argc, char ** argv ) {

   /* Init the ros node. */
   ros::init(argc, argv, "pilot");
   ros::NodeHandle nh;

   /* Creates a client for command service to control the speed of the robot, to reset the position, 
      and to exit from the simulation. */
   ros::ServiceClient client_com = nh.serviceClient<rt1_assignment2::Command>("/command");

   /* Creates a client for reset_position to reset the position of the robot in the world. */
   ros::ServiceClient client_res = nh.serviceClient<std_srvs::Empty>("/reset_positions");

   rt1_assignment2::Command srv_com;
   std_srvs::Empty srv_res;

   printf( "%s", title );
   printf( "\rSpeed: 0.00 | Awaiting command...       \r" );

   char key = ' '; // Initialize the variable key.

   /* While the user does not press the q and the roscore is running, executes this loop. */
   while ( key != 'q' && ros::ok() ) {

      key = getch(); // Gets the user command.

      if ( key == 'a' || key == 's' || key == 'r' || key == 'q' ) { // If the command is valid.

         srv_com.request.command = key;

         client_com.waitForExistence();

         if ( client_com.call(srv_com) ) { // Calls the command service.

            if ( key == 'r' ) { // If the reset key is pressed.
               client_res.waitForExistence();
               client_res.call(srv_res); // Calls the reset_position service.
            }
            printf( "\rSpeed: %.2f | Last command: %c            \r", srv_com.response.max_speed, key );
         }
      } else { // The command is not valid.
         printf( "\rCommand %c NOT valid                                                  \r", key );
      }
   }

   printf("\n \n");
   printf("Press ctrl C to exit from the world.\n \n");

   return 0;
}