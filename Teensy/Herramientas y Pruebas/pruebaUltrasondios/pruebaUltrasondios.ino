#include <HCSR04.h>

#define PROMEDIADO true
#define MUESTRAS 10

//Definicion de ultrasonidos (Trigger, Echo)
UltraSonicDistanceSensor der(1, 2);
UltraSonicDistanceSensor atr(3, 4);
UltraSonicDistanceSensor izq(14, 15);
UltraSonicDistanceSensor adel(24, 25);

/*Variables para ultrasonidos*/
float dist_adel = 0;
float dist_izq = 0;
float dist_der = 0;
float dist_atr = 0;
int muestras_us = 0;    //Cantidad de muestras de US a promediar

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   Serial.println("Comienza prueba de US:");
}

void loop() {
  // put your main code here, to run repeatedly:
  #if PROMEDIADO
  {
    dist_adel = 0;
    dist_izq = 0;
    dist_der = 0;
    dist_atr = 0;
    
    for (int i=0; i<MUESTRAS; i++){
      dist_adel += adel.measureDistanceCm();
      dist_izq += izq.measureDistanceCm();
      dist_der += der.measureDistanceCm();
      dist_atr += atr.measureDistanceCm();
      muestras_us++;
    }

    if(muestras_us >= MUESTRAS){
      dist_adel /= muestras_us;
    dist_izq /= muestras_us;
    dist_der /= muestras_us;
    dist_atr /= muestras_us; 
    Serial.println("PROMEDIADO:");
    Serial.print("Adel:");
    Serial.println(dist_adel); 
    Serial.print("Izq:");
    Serial.println(dist_izq); 
    Serial.print("Atr:");
    Serial.println(dist_atr); 
    Serial.print("der:");
    Serial.println(dist_der); 
    muestras_us=0;
    }
    
    
  }
  #else
  {
    dist_adel = adel.measureDistanceCm();
    dist_izq = izq.measureDistanceCm();
    dist_der = der.measureDistanceCm();
    dist_atr = atr.measureDistanceCm();
    Serial.print("Adel:");
    Serial.println(dist_adel); 
    Serial.print("Izq:");
    Serial.println(dist_izq); 
    Serial.print("Atr:");
    Serial.println(dist_atr); 
    Serial.print("der:");
    Serial.println(dist_der); 
    }
  #endif
}
