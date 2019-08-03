#include "ros/ros.h"
#include "std_msgs/String.h"


// funzione di callback invocata alla ricezione di un messaggio
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // inizializza ros
    ros::init(argc, argv, "listener");
    
    // crea l'handle del nodo
    ros::NodeHandle n;
    
    // subscribe(a,b,c) si sottoscrive al Master chiedendo di riceve messaggi del topic "a" i quali vengono passati alla funzione di callback "c". "b" è la grandezza della queue dei messaggi
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // spin() entra in un loop dove attende e risponde coi callback. Uscirà dal loop quando ros::ok() nel publisher restituirà false
    ros::spin();

    return 0;
}
