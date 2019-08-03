#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_cicle");
    ros::NodeHandle n;

    // create publisher
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    
    // frequency
    ros::Rate loop_rate(10);


        
    ROS_INFO("%s", "Start!"); 

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 5;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 2;        

        // sent msg
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
