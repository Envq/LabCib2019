#include "mbed.h"
#include "DigitalEncoderAB.h"
#include "DigitalEncoderAS5601.h"
#include "DigitalEncoderPIC.h"
#include "math.h"


// creo un oggetto per la comunicazione seriale
Serial pc(SERIAL_TX, SERIAL_RX, 921600);


// attivo l'encoder
// DigitalEncoderPIC encoder(I2C_SDA,I2C_SCL);      // PIC encoder
DigitalEncoderAS5601 encoder(PB_9, PB_8);           // AS5601 encoder
// DigitalEncoderAB encoder(48.0f);                 // AB encoder (PA0 = A, PA1 = B)


// potenziometro
AnalogIn potentiometer(PB_1);


// gestione fasi motore
PwmOut uh_1(PA_8);
PwmOut vh_2(PA_9);
PwmOut wh_3(PA_10);

DigitalOut en_1(PC_10);
DigitalOut en_2(PC_11);
DigitalOut en_3(PC_12);

DigitalOut en_chip(PA_6);

float pwm = 0.8f;
float gnd = 0.0f;


// funzione per l'attivazione della fase "step"
void active_phase(int step) {
    switch(step){
        case 0:
            uh_1.write(pwm);
            vh_2.write(0.0f);
            wh_3.write(gnd);
            en_1 = 1;
            en_2 = 0;
            en_3 = 1;
            break;
        case 1:
            uh_1.write(0.0f);
            vh_2.write(pwm);
            wh_3.write(gnd);
            en_1 = 0;
            en_2 = 1;
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
            uh_1.write(gnd);
            vh_2.write(0.0f);
            wh_3.write(pwm);
            en_1 = 1;
            en_2 = 0;
            en_3 = 1;
            break;
        case 4:
            uh_1.write(0.0f);
            vh_2.write(gnd);
            wh_3.write(pwm);
            en_1 = 0;
            en_2 = 1;
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
    // per stampare valori float
    asm(".global _printf_float");

    // lunghezza in gradi di una fase = 360° / (numero_magneti/2 * numero_bobine/2)
    // quindi per passare da una fase ad un altra ci saranno circa 9°
    // quindi i 6 step saranno ripetuti 7 volte per fare 360°
    float phase_length = 360.0f / 7.0f / 6.0f;
    // offeset gestito dall'utente attraverso un potenziometro
    float phase_offset = 0.0;
    float phase_overlap;

    // abilito la shield per il motore
    en_chip = 1;
    // aumenta la precisione dei pwm
    uh_1.period(0.00001f);
    vh_2.period(0.00001f);
    wh_3.period(0.00001f);

    // step attuale
    int step = 0;
    // angolo in gradi
    float angle_deg = 0.0;
    // posizione nella fase
    float pos_in_round = 0.0;
    // numero di fasi passate
    int phase_num = 0;
    
    while(1) {
        
        // leggo il valore del potenziometro per aggiustare l'offset dalla phase
        // il quale è moltiplicato per 6 per avere più precisione
        phase_offset = potentiometer.read() * phase_length * 6;

        // leggo il valore attuale dell'angolo
        angle_deg = encoder.getAngleDeg();


        // METODO 1:
        // trovo la posizione nella fase:
        // 360/7 è la lunghezza di round cioè il numero di gradi necessari a fare un giro
        // angle_deg % round mi dice in che posizione sono all'interno del round
        pos_in_round =  fmod(angle_deg,(360.0f / 7));

        // aggiungo l'offset per allineare la posizione nella fase
        pos_in_round = pos_in_round + phase_offset;

        // calcolo lo step: 
        // pos_in_round / 6 è lo step e varia ogni 6 posizioni
        // % 6 per farsi che siano comprese in [0, 5]
        step =  fmod(floor(pos_in_round / phase_length), 6.0f);

        //METODO 2:
        // aggiusto l'angolo con l'offset per cercare di alinearlo alla fase
        // calcolo quante fasi sono passate 
        // phase_num = (int) ((angle_deg + phase_offset) / phase_length);
        // calcolo in che step siamo ogni 6 fasi passate
        // step = phase_num % 6;



        // monitoraggio
        // pc.printf("angle: %f\nphase: %d\nstep: %d\n----------\n", angle_deg, phase_num, step);
        // pc.printf("%f\n", angle_deg);

        // attivo la phase scelta
        active_phase(step);     
        // step = (step + 1) % 6;
   
        //...?
        wait(0.0001);
    }
}
