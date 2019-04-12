#include "HC-SR04-non-Blocking.h"

USensor sensorUS1(9,8);
USensor sensorUS2(15,14);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("Comienza prueba de libreria USensor");
  
  sensorUS1.startMeasure();
  //sensorUS2.startMeasure();
}

void loop() {
  // put your main code here, to run repeatedly:
 while(!sensorUS1.isMeasureReady());
      Serial.print("US1: ");
      Serial.println(sensorUS1.lastMeasure()); 
      sensorUS2.startMeasure();
      
  

  while(sensorUS2.isMeasureReady());
      Serial.print("US2: ");
      Serial.println(sensorUS2.lastMeasure()); 
      sensorUS1.startMeasure();
  

  
}
