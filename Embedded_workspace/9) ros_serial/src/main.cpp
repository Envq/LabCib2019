#include "SerialAck.h"
#include "mbed.h"

// serial communication with PC
SerialAck pc(USBTX, USBRX, 921600); // tx, rx, speed

bool led_on = false;
bool button_pressed = true;
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
        
        // if(button_pressed!=userButton.read()) {
        //     button_pressed = userButton.read();
        //     led_on = !led_on;
        // }

        // pulsante premuto == 0
        if(userButton.read() == 0) {
            led_on = 1;
            button_pressed = 1;
        } else {
            led_on = 0;
            button_pressed = 0;
        }

        potentiometer_value = potentiometer.read();
        if(led_on==false) {
            led.write(0);
        } else {
            led.write(potentiometer_value);
        }

        sprintf(buf,"%i,%i,%f\n",led_on,button_pressed,potentiometer_value);
        pc.writeWithIdentifier(buf,'s');
        wait(0.01f);
        //wait(1);

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

