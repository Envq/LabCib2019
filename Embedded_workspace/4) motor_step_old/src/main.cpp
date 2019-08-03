#include "mbed.h"

Serial pc(USBTX,USBRX,115200);

PwmOut u_h(PA_8);
PwmOut v_h(PA_9);
PwmOut w_h(PA_10);

DigitalOut en1(PC_10);
DigitalOut en2(PC_11);
DigitalOut en3(PC_12);

DigitalOut en(PA_6);

float duty_cicle = 0.0f;
float pwm = 1.0f;
float gnd = 0.0f;


int main(){

en = 1;

u_h.period(0.0001f);
w_h.period(0.0001f);
v_h.period(0.0001f);

  int step_number = 0;

  while(1){

    printf("%i\n",step_number);

      switch(step_number){
        
        case 0:
          u_h.write(pwm);
          v_h.write(0.0f);
          w_h.write(gnd);
          en1 = 1;
          en2 = 0;
          en3 = 1;
          break;
        case 1:
          u_h.write(0.0f);
          v_h.write(pwm);
          w_h.write(gnd);
          en1 = 0;
          en2 = 1;
          en3 = 1;
          break;
        case 2:
          u_h.write(gnd);
          v_h.write(pwm);
          w_h.write(0.0f);
          en1 = 1;
          en2 = 1;
          en3 = 0;
          break;
        case 3:
          u_h.write(gnd);
          v_h.write(0.0f);
          w_h.write(pwm);
          en1 = 1;
          en2 = 0;
          en3 = 1;
          break;
        case 4:
          u_h.write(0.0f);
          v_h.write(gnd);
          w_h.write(pwm);
          en1 = 0;
          en2 = 1;
          en3 = 1;
          break;
        case 5:
          u_h.write(pwm);
          v_h.write(gnd);
          w_h.write(0.0f);
          en1 = 1;
          en2 = 1;
          en3 = 0;
          break;
      }

    wait(0.1f);

  step_number = (step_number + 1) % 6;

  }

}