#include "SerialAck.h"
#include "mbed.h"

// serial communication with PC
SerialAck pc(USBTX, USBRX, 921600); // tx, rx, speed

bool led_on = false;
bool button_before = false;
bool button_now = false;
bool button_pressed = false;
float potentiometer_value = 0.0f;

// LED
PwmOut led(LED2);

// user button
DigitalIn userButton(USER_BUTTON);

// potentiometer
AnalogIn potentiometer(PB_1);

void master() {
    char buf[256];
    
    do{
        // read potentiometer value
        potentiometer_value = potentiometer.read();

        // read button_now value
        button_now = !userButton.read();

        // update button and led status
        if (button_now == true && button_before == false) {
            button_pressed = true;
            led_on = !led_on;
        }

        // update led value
        if (led_on) {
            led.write(potentiometer_value);
        } else {
            led.write(0);
        }


        // update serial
        sprintf(buf,"%i,%i,%f\n", led_on, button_pressed, potentiometer_value);
        pc.writeWithIdentifier(buf,'s');


        // update variables
        button_before = button_now;
        button_pressed = false;



        wait(0.01f);

    }while(true);
}

void slave() {
    char buf[256];
    char *token;
    char separator[2] = ",";


    do{
        // read from the serial the board status
        pc.readWithIdentifier(buf,'s');
        token = strtok(buf,separator);

        token = strtok(NULL,separator);
        led_on = atoi(token);
        token = strtok(NULL,separator);
        button_pressed = atoi(token);
        token = strtok(NULL,separator);
        potentiometer_value = atof(token);

        // set the led status equal to the master
        if(led_on) {
            led.write(potentiometer_value);
        } else {
            led.write(0);
        }
    }while(true);
}

// This is the main function
int main() {

    // Code to print float values on serial
    asm(".global _printf_float");


    master();
    // slave();

    return 0;
}

