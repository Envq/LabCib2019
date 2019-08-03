#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    //inizializza ros
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    
    // crea publisher con topic "chatter" con queue di max 1000 msg
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // frequenza del loop
    ros::Rate loop_rate(10);
  
    int count = 0;
    while (ros::ok())
    {
        // creo messaggio e lo inizializzo con lo stream di string
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        // equivalente del printf/cout per ros
        ROS_INFO("%s", msg.data.c_str());

        //pubblico il messagio
        chatter_pub.publish(msg);

        //fuzione per il callback
        ros::spinOnce();
        
        // aspetta in modo da rispettare la frequenza selezionata
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
