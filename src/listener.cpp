#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "math.h"

float last_left_encoder = 0;
float last_right_encoder = 0;

float left_encoder = 0;
float right_encoder = 0;

float wheel_radius = 0.17;
float distance_between_wheels = 0.55;
float N = 5.80;

float Delta_ticks_left;
float Delta_ticks_right;

float Dc;

float theta = 0;
float last_theta_prima = 0;
float theta_prima = 0;

float last_x = 0;
float x = 0;
float last_y;
float y = 0;

int id = 0;

void Callback(const sensor_msgs::JointState & msg)
{
  
  last_left_encoder = left_encoder;
  last_right_encoder = right_encoder;

  left_encoder = msg.position[0];
  right_encoder = msg.position[1];

  Delta_ticks_left = 2*M_PI*wheel_radius*((left_encoder-last_left_encoder)/N);
  Delta_ticks_right = 2*M_PI*wheel_radius*((right_encoder-last_right_encoder)/N);

  Dc = (Delta_ticks_left + Delta_ticks_right)/2;

  theta = (Delta_ticks_right - Delta_ticks_left)/distance_between_wheels;
  last_theta_prima = theta_prima;
  theta_prima = last_theta_prima + (theta/2);

  last_x = x;
  x = last_x + Dc * cos(last_theta_prima);
  last_y = y;
  y = last_y + Dc * sin(last_theta_prima);

  //std::cout << x << "\n";
  //std::cout << y << "\n";

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  id = id + 1;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 1;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;


  chatter_pub.publish(marker);
  ros::spin();

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "joint_states");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   //ros::Subscriber sub = n.subscribe("chatter", 1000, Callback);
   ros::Subscriber sub = n.subscribe("joint_states", 1000, Callback);   

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}