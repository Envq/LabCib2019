#include <ros/ros.h>
#include <iostream>

#include "lab_cib_2019/board_status.h"
#include "Serial.h"

using namespace std;

ros::Publisher robot_pub;
lab_cib_2019::board_status message;
Serial *serial;

void publishMessage();
void master(int argc, char** argv);
void slave(int argc, char** argv);

int main(int argc, char** argv)
{
	char car;

	string serialName = "/dev/ttyACM";
	cout << "Insert the serial port of the board: " << serialName << ": ";
	cin >> car;
	serialName += car;

	serial = new Serial(serialName.c_str(),0,1);
	
	cout << "Connected.\n";

	cout << "Select the mode (master/slave) [m/s]: ";
	cin >> car;
	
	if(car == 'm')
		master(argc,argv);
	else if (car == 's')
		slave(argc,argv);

	return 0;
}

void boardStatusCallback(const lab_cib_2019::board_status::ConstPtr& msg) {
	char buf[256];
    
    // create the string for the board from the msg
    sprintf(buf,"%i,%i,%f",msg->led_on,msg->button_pressed,msg->potentiometer_value);
	
	printf("[SLAVE] %s\n",buf);
	
	// send to the board the status
	serial->writeWithIdentifier(buf,'s');
}

void publishMessage() {
    // update the id message
    message.id++;

    // publish the message
    robot_pub.publish(message);
    //ros::spinOnce();
}

void master(int argc, char** argv) {
	char car, buf[4096];
	char separator[2] = ",";
	int count=0;
	
	ros::init(argc, argv, "lab_cib_main_master");
	ros::NodeHandle n;
	robot_pub = n.advertise<lab_cib_2019::board_status>("lab_cib_topic", 10);
	
	do {
		// reading from the board
		do{
			car = serial->readChar();
			if(car=='s') {
				count = 0;
			}
			buf[count++] = car;
		}while(car!='\n');
		buf[count] = '\0';
		
		// it's the board status
		if(buf[0]=='s') {
			char *temp = "s";
			serial->writeWithout(temp);

			printf("[MASTER] %s",buf);
			char *token = strtok(buf,separator);
                
            token = strtok(NULL,separator);
            message.led_on  = atoi(token);
            token = strtok(NULL,separator);
            message.button_pressed  = atoi(token);
            token = strtok(NULL,separator);
            message.potentiometer_value  = atof(token);
            
            publishMessage();
		}
		count = 0;
	}while(true);
}

void slave(int argc, char** argv) {
	ros::init(argc, argv, "lab_cib_main_slave");
	ros::NodeHandle n;
	ros::Subscriber robot_sub = n.subscribe("lab_cib_topic", 10, boardStatusCallback);
	
	ros::spin();
}


