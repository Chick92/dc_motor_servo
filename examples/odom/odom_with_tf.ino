/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


// if using this file on a new machine, you need to make changes to the arduino ros lib - 
//gedit ~/Arduino/libraries/ros_lib/ros/node_handle.h
//change OUTPUT_SIZE as to - 
/* Node Handle 
template<class Hardware,
   int MAX_SUBSCRIBERS = 15,
         int MAX_PUBLISHERS = 15,
         int INPUT_SIZE = 512,
         int OUTPUT_SIZE = 1024>
*/

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom_frame[] = "/odom";

void setup()
{
  nh.initNode();
  broadcaster.init(nh);

  nh.advertise(odom_pub);


}

void loop()
{  
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom_frame;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);


  odom.header.frame_id = base_link;
  odom.child_frame_id  = odom_frame;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

  odom.twist.twist.linear.x  = dx;
  odom.twist.twist.angular.z = dtheta;

  odom.header.stamp = nh.now();;
  odom_pub.publish(&odom);






  
  nh.spinOnce();
  
  delay(10);
}










