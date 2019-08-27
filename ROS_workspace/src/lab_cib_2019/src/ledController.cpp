#include "ros/ros.h"
#include "lab_cib_2019/board_status.h"
#include "std_msgs/Int64.h"
#include "Serial.h"

#include <iostream>


ros::Publisher pub;
Serial *serial;

void intensity_main(int argc, char** argv);
void frequency_main(int argc, char** argv);


//  MAIN  ####################################################################
int main(int argc, char** argv) {
	string serialName = "/dev/ttyACM";
	char input;

	// open Serial
	std::cout << "Insert the serial port of the board: " << serialName << ": ";
	std::cin >> input;
	serialName += input;
	serial = new Serial(serialName.c_str(), 0, 1);
	std::cout << "Connected..." << std::endl;


	// select Mode
	std::cout << "Select mode (intensity/frequency) [i/f]: ";
	std::cin >> input;


	// start
	switch (input) {
	case 'i':
		intensity_main(argc,argv);
		break;

	case 'f':
		frequency_main(argc,argv);
		break;

	default:
		std::cout << "Wrong Mode!!!" << std::endl;
		break;
	}

	return 0;
}


//  WORKSPACE INTENSITY  #####################################################
const double i_factor_scale = 0.10;
double intensity = 0;

void intensityCallBack(const std_msgs::Int64::ConstPtr& msg) {
    // get intensity
    intensity = msg->data * i_factor_scale;
    ROS_INFO("intensity: %f", intensity);
    
    // create the string for the board from the msg
	char buffer[256];
    sprintf(buffer,"%i,%i,%f", 1, 0, intensity);
		
	// send to the board the status
	serial->writeWithIdentifier(buffer, 's');
}

void intensity_main(int argc, char** argv) {
    ros::init(argc, argv, "intensity");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe<std_msgs::Int64>("counter_topic", 1000, intensityCallBack);
	
	// send started intensity	
	char buffer[256];
    sprintf(buffer,"%i,%i,%f", 1, 0, intensity);
	serial->writeWithIdentifier(buffer, 's');	
	ROS_INFO("starter intensity: %f", intensity);

	// loop
	ros::spin();
}


//  WORKSPACE FREQUENCY  #####################################################
double frequency = 10;
int led = 1;

void do_frequency() {
	char buffer[256];

    // send command  
	sprintf(buffer,"%i,%i,%f", led, 0, 1.0);
	serial->writeWithIdentifier(buffer, 's');

	// update led
	led = (led + 1) % 2;
}


void frequencyCallBack(const std_msgs::Int64::ConstPtr& msg) {
    // get frequency
    frequency = 1.0 / msg->data;		//msg-data = seconds
    ROS_INFO("frequency: %f hz", frequency);
}


//  MAIN FREQUENCY  ##########################################################
void frequency_main(int argc, char** argv) {
    ros::init(argc, argv, "frequency");    
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe<std_msgs::Int64>("counter_topic", 1000, frequencyCallBack);
	ROS_INFO("starter frequency: %f hz", frequency);

    // setto il rate del loop
    ros::Rate loop_rate(frequency);

	// loop
    while(ros::ok()) {
        loop_rate = ros::Rate(frequency);
        do_frequency();
        ros::spinOnce();
        loop_rate.sleep();
   }   
}