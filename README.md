# dc_motor_servo
An arduino library for turnin a DC motor (with encoder) into a servo, giving position and velocity control with PI

The ROS info output text is a nod to my favourite book - Alien, The Cold Forge.


If using the ROS versions of this software, particularly with odom, you need to make changes to the arduino ros lib - 
gedit ~/Arduino/libraries/ros_lib/ros/node_handle.h
change OUTPUT_SIZE as to - 
Node Handle 
template<class Hardware,
   int MAX_SUBSCRIBERS = 15,
         int MAX_PUBLISHERS = 15,
         int INPUT_SIZE = 512,
         int OUTPUT_SIZE = 1024>
This is becuase the default output size isn't big enough, so the odom messages get dropped. 
