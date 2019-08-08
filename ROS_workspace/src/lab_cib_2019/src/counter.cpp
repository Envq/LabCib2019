#include "ros/ros.h"
#include "lab_cib_2019/board_status.h"
#include "std_msgs/Int64.h"


ros::Publisher pub;
long unsigned int counter = 0;


// funzione di callback invocata alla ricezione di un messaggio
void counterCallBack(const lab_cib_2019::board_status::ConstPtr& msg) {
    // update counter if button was pressed
    if (msg->button_pressed == true) {
        counter++;
        ROS_INFO("Count: %li", counter);
    }

    std_msgs::Int64 new_msg;
    new_msg.data = counter;
    pub.publish(new_msg);
}



int main(int argc, char** argv) {

    // inizializzo ros
    ros::init(argc, argv, "Counter");

    // creo l'handle del nodo
    ros::NodeHandle node;

    // creo il subscriber.
    // Esso si sottoscrive al Master chiedendo di riceve messaggi del lab_cib_topic
    ros::Subscriber sub = node.subscribe<lab_cib_2019::board_status>("lab_cib_topic", 1000, counterCallBack);
    
    // creo il publisher.
    // Esso pubblica sul topic counter_topic
    pub = node.advertise<std_msgs::Int64>("counter_topic", 1000);



    // setto il rate del loop
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
   }    

    return 0;
}