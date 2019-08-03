#include "mbed.h"
#include "AS5601.h"
//#include "I2C.h"

//#include "DigitalEncoderAB.h"
//#include "DigitalEncoderPIC.h"
//#include <cmath>


// IMPOSTAZIONI
float angle_ref = 90.0f;
float POT_PHASE = 0.722857;



// attivo l'encoder giusto
// DigitalEncoderPIC encoder(I2C_SDA,I2C_SCL);      // PIC encoder
// DigitalEncoderAB encoder(48.0f);                 // AB encoder (PA0 = A, PA1 = B)
AS5601 encoder(PB_9, PB_8);           // AS5601 encoder

// creo un oggetto per la comunicazione seriale
Serial pc(SERIAL_TX, SERIAL_RX, 921600);

// abilito l'uso del potenziometro della board
AnalogIn potentiometer(PB_1);

// gestione fasi motore
PwmOut uh_1(PA_8);
PwmOut vh_2(PA_9);
PwmOut wh_3(PA_10);
DigitalOut en_1(PC_10);
DigitalOut en_2(PC_11);
DigitalOut en_3(PC_12);

// abilito l'uso del chip per la gestione del motore
DigitalOut en_chip(PA_6);

// potenziale da usare nel circuito del motore
float pwm = 1.0f;
float gnd = 0.0f;
float duty_cycle = pwm;



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
    // per stampare valori float
    asm(".global _printf_float");

    // lunghezza in gradi di una fase = 360° / (numero_magneti/2 * numero_fasi*2)
    // numero_magneti/2 sono i magneti che compongono il nord (e anche il sud)
    // numero_fasi*2 sono le fasi per compiere tutte le combinazioni con uvw
    // quindi per passare da una fase ad un altra ci saranno circa 9°
    // quindi le 6 fasi saranno ripetute 7 volte per fare 360°
    float phase_length = 360.0 / 7 / 6;
    // offeset gestito dall'utente attraverso un potenziometro
    float phase_offset = 0.0;
    // fase attuale
    int phase = 0;
    // angolo in gradi
    float angle_deg = 0.0;
    // numero di fasi passate
    int phase_num = 0;


    // abilito la shield per il motore
    en_chip = 1;
    // aumenta la precisione dei pwm in quanto aumentando il periodo...
    // il segnale del pwm diventa quasi continuo
    uh_1.period(0.00001f);
    vh_2.period(0.00001f);
    wh_3.period(0.00001f);



    // PARAMETRI per controllo posizione
    // angolo precedente
    float err_last = 0.0f;
    // dichiaro un timer
    Timer t;
    // tempo passato dalle letture degli angoli
    float time_read_angle = 0.0f;
    // errore = angolo_voluto - angolo_attuale
    float err = 0.0;
    // costante di smorzamento
    float d = 0.05f;
    // costante elastica
    float k = 0.05f;
    // velocità
    float derr = 0.0f;

    float sum = 0.0;

    for(int i = 0; i < 10; i++) {
        sum += potentiometer.read();
    }
    pc.printf("%f\n", sum/10.0f);




    // inizio a contare il tempo
    t.start();

    while(1) {

        // mi segno il precedente valore dell'angolo
        // angle_last = angle_deg;


        angle_ref = potentiometer.read() * 360.0f;

        // leggo il valore attuale dell'angolo
        angle_deg = encoder.getAngleDegCompensated();

        // mi segno il tempo passato dalla lettura dell'angolo
        t.stop();                       //smetto di cronometrare e mi salvo il tempo
        time_read_angle = t.read();
        t.reset();
        t.start();                      //riparto a cronometrare il tempo


        // CONTROLLO DI POSIZIONE
        // errore = angolo_voluto - angolo_attuale
        err = angle_ref - angle_deg;

        // calcolo la velocità
        derr = (err - err_last ) / time_read_angle;

        // calcolo il valore del duty_cycle come tau = k*err - d*vel
        duty_cycle = k * err;// + d * derr;
        // duty_cycle = k * err + d * vel;

        // standardizzo tau
        if (duty_cycle > 1)
            duty_cycle = 1;
        if (duty_cycle < -1)
            duty_cycle = -1;

        // assegno tau
        if (duty_cycle > 0) {
            // gira in senso orario
            pwm = duty_cycle;
            gnd = 0.0f;
        } else {
            // gira in senso inverso ed evenutalmente fermati
            // girare in senso inverso vuol dire scambiare i valori di pwm e gnd
            pwm = 0.0f;
            gnd = - duty_cycle;
        }

        err_last = err;






        // ATTIVAZIONE MOTORE
        // leggo il valore del potenziometro per aggiustare l'offset dalla fase
        // lo moltiplico per phase_lenght * 6 che è circa 51,4°
        // in questo modo sono in grado di avanzare fino a 51,4°
        // supponendo che che 0° letti dall'encoder siano davvero 0° se:
        //                                          u  v    direzione
        //  phase_offset <= phase_length * 1 -->    -  1     -->
        //  phase_offset <= phase_length * 2 -->    1  1     <->
        //  phase_offset <= phase_length * 3 -->    1  -     <--
        //  phase_offset <= phase_length * 4 -->    -  0     <--
        //  phase_offset <= phase_length * 5 -->    0  0     <->
        //  phase_offset <= phase_length * 6 -->    0  -     -->
        phase_offset = POT_PHASE * phase_length * 6;


        // METODO 1:
        // // trovo la posizione nella fase:
        // // 360/7 è la lunghezza di round cioè il numero di gradi necessari a fare un giro
        // // angle_deg % round mi dice in che posizione sono all'interno del round
        // float pos_in_round =  fmod(angle_deg,(360.0f / 7));
        // // aggiungo l'offset per allineare la posizione nella fase
        // pos_in_round = pos_in_round + phase_offset;
        // // calcolo la fase:
        // // pos_in_round / phase_lenght mi dice in che fase sono
        // // % 6 per farsì che le fasi siano comprese in [0, 5]
        // phase =  fmod(floor(pos_in_round / phase_length), 6.0f);

        //METODO 2:
        // aggiusto l'angolo con l'offset e poi calcolo quante fasi sono passate
        phase_num = (int) ((angle_deg + phase_offset) / phase_length);
        // % 6 per farsì che le fasi siano comprese in [0, 5]
        phase = phase_num % 6;


        // attivo la fase scelta
        active_phase(phase);

        // versione step:
        // phase = (phase + 1) % 6;




        // MONITORAGGIO
        // pc.printf("angle: %f\nphase: %d\nfase: %d\n----------\n", angle_deg, phase_num, phase);
        pc.printf("%f\n", angle_deg);
        // pc.printf("%f\n", potentiometer.read());



    }
}
