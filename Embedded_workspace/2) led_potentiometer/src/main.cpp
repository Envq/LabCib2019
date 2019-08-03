#include <mbed.h>

// definisco il seriale per settare il baud rate di scrittura
// settare in platformio.ini il baud rate di lettura
Serial pc(USBTX,USBRX,115200);
// definisco l'input in cui leggere il valore di un potenziometro
AnalogIn input(A0);
// definisco l'output con cui controllare un led
PwmOut led(LED2);



int main() {

  while(1) { 
    // leggo il valore del potenziomentro
    float val = input.read();

    // tronco il valore
    val = (float)(((int) (val * 100))) / 100;


    // stampo il valore sullo stdout
    printf("%f\n", val);

    // uso l'input per gestire il led
    led.write(val);
  }
}