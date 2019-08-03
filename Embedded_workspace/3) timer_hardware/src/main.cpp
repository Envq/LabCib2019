#include <mbed.h>

// definizione timer hardware. Per la scheda F446RE usare TIM3 o TIM7 
#define TIM_USR TIM3
#define TIM_USR_IRQ TIM3_IRQn


// impostazioni seriale
Serial pc(USBTX,USBRX,921600);

// flag di notifica usato per identifiare l'attivazione dell'interrupt
volatile char flag_time = 0;

// struttura timer handler
TIM_HandleTypeDef mTimUserHandle;

// funzione da azionare all'attivazione dell'interrupt
extern "C"
void M_TIM_USR_Handler(void) {
  if(__HAL_TIM_GET_FLAG(&mTimUserHandle,TIM_FLAG_UPDATE) == SET) {
    // pulisco il flag del timer per poter ricevere un altro interrupt
    __HAL_TIM_CLEAR_FLAG(&mTimUserHandle, TIM_FLAG_UPDATE);
    // attivo il flag per poter temporizzare il ciclo while nel main
    flag_time = 1;
  }
}


int main() {
  // abilito il clock relativo al timer hardware TIM3
  __HAL_RCC_TIM3_CLK_ENABLE();

  // configuro la struttura per usare il timer hardware
  mTimUserHandle.Instance = TIM_USR;
  mTimUserHandle.Init.Prescaler = 200;
  mTimUserHandle.Init.ClockDivision = TIM_COUNTERMODE_UP;
  mTimUserHandle.Init.Period = 90;
  mTimUserHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  // NB: registri a 16 bit -> max val = 65536
  //    1hz = 90M / 10000 / 9000
  //  100hz = 90M / 10000 / 90
  // 1000hz = 90M / 1000 / 90
  //10000hz = 90M / 100 / 90
  // 5000hz = 90M / 200 / 90

  // inizializzo il timer hardware con la struttura configurata
  HAL_TIM_Base_Init(&mTimUserHandle);

  // faccio partire il timer hardware
  HAL_TIM_Base_Start_IT(&mTimUserHandle);

  // abilito il servizio di interrupt relativo al timer hardware
  NVIC_SetVector(TIM_USR_IRQ, (uint32_t)M_TIM_USR_Handler);
  NVIC_EnableIRQ(TIM_USR_IRQ);



  // timer software usato per controllare la frequenza
  //Timer t;
  // frequenza precedente
  float prev_t = 0;

  while(1) {
    //t.start();

    if(flag_time){
      flag_time=0;  
    
      float actual_t = (float) us_ticker_read();
      printf("%f sec\n", 1/(actual_t - prev_t));
      prev_t = actual_t;
      
      // il timer funziona bene per intervalli tra msec e sec
      //t.stop();
      //printf("%f sec\n", t.read());
      //t.reset();

    }
  }
}
