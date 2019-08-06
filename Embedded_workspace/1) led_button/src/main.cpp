#include <mbed.h>


Serial pc(USBTX,USBRX,115200);

PwmOut led(LED2);

DigitalIn input(USER_BUTTON);



int main() {

  float val = 0.0;

  while(1) { 
  
    val = input.read();

    led.write(val);
    printf("%f\n", val);

  }
}