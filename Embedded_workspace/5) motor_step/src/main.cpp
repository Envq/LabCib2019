#include "mbed.h"

Serial pc(USBTX,USBRX,115200);

PwmOut uh_1(PA_8);
PwmOut vh_2(PA_9);
PwmOut wh_3(PA_10);

DigitalOut en_1(PC_10);
DigitalOut en_2(PC_11);
DigitalOut en_3(PC_12);

DigitalOut en_chip(PA_6);

float duty_cicle = 0.0f;
float pwm = 1.0f;
float gnd = 0.0f;


// funzione per l'attivazione della fase selezionata
void active_phase(int phase) {
/*
    N -> 1
    S -> 0

         u   
    _         _
    w    N    v       
         |   
    v    S    w
         _
         u
*/
    switch(phase){
        case 0:
            uh_1.write(0.0f);
            vh_2.write(gnd);
            wh_3.write(pwm);
            en_1 = 0;
            en_2 = 1;
            en_3 = 1;
            break;
        case 1:
            uh_1.write(gnd);
            vh_2.write(0.0f);
            wh_3.write(pwm);
            en_1 = 1;
            en_2 = 0;
            en_3 = 1;
            break;
        case 2:
            uh_1.write(gnd);
            vh_2.write(pwm);
            wh_3.write(0.0f);
            en_1 = 1;
            en_2 = 1;
            en_3 = 0;
            break;
        case 3:
            uh_1.write(0.0f);
            vh_2.write(pwm);
            wh_3.write(gnd);
            en_1 = 0;
            en_2 = 1;
            en_3 = 1;
            break;
        case 4:
            uh_1.write(pwm);
            vh_2.write(0.0f);
            wh_3.write(gnd);
            en_1 = 1;
            en_2 = 0;
            en_3 = 1;
            break;
        case 5:
            uh_1.write(pwm);
            vh_2.write(gnd);
            wh_3.write(0.0f);
            en_1 = 1;
            en_2 = 1;
            en_3 = 0;
            break;
    }
}


int main() {

  en_chip = 1;

  uh_1.period(0.0001f);
  wh_2.period(0.0001f);
  vh_3.period(0.0001f);

  int step_number = 0;

  while(1) {

    printf("%i\n",step_number);
	
	active_phase(step_number);

    wait(0.1f);

    step_number = (step_number + 1) % 6;


}
