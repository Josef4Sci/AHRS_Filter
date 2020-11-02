#include <Wire.h>
#include "BMX055_Minimal_Lib.h"
#include "Justa_AHRS_Filter.h"

BMX055 bmx055;
AHRS AHRSlib;

volatile unsigned long timerCount=0;
unsigned long timerCountMem=0;

ISR(TIMER1_COMPA_vect){
  timerCount++;
}

void setupTimerInterrupt(){
  
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 99;// = (16*10^6) / (10000*8) - 1 
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

#define Addr_Mag 0x10
void setup() {

  setupTimerInterrupt();
  
  Serial.begin(115200);
  Serial.println("Started...");
  
  bmx055.init();
  AHRSlib=AHRS();
  Serial.println("Initialized...");
  Serial.println("You have 10 secs to calibrate magnetometer...");
}


int printPart=0;
void loop() {
  
  byte data[18];
  float dataF[9];
  
  bmx055.getSensorData(data);
  
  if(timerCount<100000){
    
    AHRSlib.updateMagnetCalib(data);
    timerCountMem=timerCount;
    
  }else{
    
    AHRSlib.convertData(data, dataF);    
   
    AHRSlib.updateRotationEtimation(dataF, 0.005, 0.005, (timerCount-timerCountMem)/10000.0);

    timerCountMem=timerCount;
    
    if(printPart%5==0){
      char buffer1[200];
      sprintf(buffer1, "<<%s,%s,%s,%s>>\r\n", 
      String(AHRSlib.quaternion[0],3).c_str(),
      String(AHRSlib.quaternion[1],3).c_str(),
      String(AHRSlib.quaternion[2],3).c_str(),
      String(AHRSlib.quaternion[3],3).c_str());  
      Serial.print (buffer1);
    }    
    printPart++;
  }

  delay(1);
}
