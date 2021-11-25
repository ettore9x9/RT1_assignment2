/* rosrun stage_ros stageros $(rospack find rt1_assignment2)/world/my_world.world 
   rosrun rt1_assignment2 racer_node 
   rosrun rt1_assignment2 pilot_node */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "rt1_assignment2/Command.h"
#include "stdio.h"
#include "unistd.h"
#include "termios.h"

const char * reminder = R"(

   s : increase max speed of the robot.
   a : decrease max speed of the robot.
   r : put the robot in the initial position.
   q : quit.


)";

char getch( void ) {

   int ch;
   struct termios oldt, newt;

   tcgetattr( STDIN_FILENO, &oldt );
   newt = oldt;

   newt.c_lflag &= ~(ICANON | ECHO);
   newt.c_lflag |= IGNBRK;
   newt.c_lflag &= ~(INLCR | ICRNL | IXON | IXOFF);
   newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
   newt.c_cc[VMIN] = 1;
   newt.c_cc[VTIME] = 0;

   tcsetattr( STDIN_FILENO, TCSANOW, &newt );

   ch = getchar();

   tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

   return ch;

}

int main ( int argc, char ** argv ) {

   ros::init(argc, argv, "pilot");

   ros::NodeHandle nh;

   ros::ServiceClient client = nh.serviceClient<rt1_assignment2::Command>("/command");

   rt1_assignment2::Command srv;

   printf( "%s", reminder );
   printf( "\rMax speed: 0.00 | Current speed: ... | Awaiting command...       \r" );

   char key = ' ';

   while ( key != 'q') {

      key = getch();

      srv.request.command = key;

      client.waitForExistence();

      if ( client.call(srv) ) {
         printf( "\rMax speed: %.2f | Current speed: ... | Last command: %c            \r", srv.response.max_speed, key );
      }
   }

   printf("\n \n");

   return 0;
}