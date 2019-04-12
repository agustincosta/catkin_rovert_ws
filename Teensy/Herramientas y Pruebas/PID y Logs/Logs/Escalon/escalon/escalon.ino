#include <PID_v1.h>
#include <Encoder.h>

#define ADELANTE true

/*Motores*/
//define pin name driver 1
#define pwm_1 30
#define dir_1 31
#define pwm_2 29
#define dir_2 28
//define pin name dirver 2
#define pwm_3 35
#define dir_3 34
#define pwm_4 38
#define dir_4 39

#define pwm_pin pwm_1
#define dir_pin dir_1

int input=0;
unsigned long changeTime = 0;
double w_medida = 0;
int sentidoDelaProgresion =1;
float VEL_ANG_MAX_RUEDAS = 8.507;
unsigned long now, aux;

//Definicion de encoders (A, B)
Encoder rueda1(6,5);
Encoder rueda2(7,8);
Encoder rueda3(9,10);
Encoder rueda4(12,11);

Encoder *rueda = &rueda1;

void setup() {
  // put your setup code here, to run once:
   
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
 

  
  changeTime = 0;
  Serial.begin(115200);
  delay(10000);
  Serial.print("EXPERIMENTO Resp en frecuencia 1, CON DIRECCION: ");
  #if ADELANTE
      Serial.println ("ADELANTE");
    #else
      Serial.println ("ATRAS");
    #endif
    Serial.println("input   wi  wo");

    now=millis();
  
}



void loop() {
  // put your main code here, to run repeatedly:
  if(millis()>changeTime)
  {
    #if ADELANTE
      digitalWrite(dir_pin, HIGH);
    #else
      digitalWrite(dir_pin,LOW);
    #endif

    if(w_medida >=0){
      input =100;
      digitalWrite(dir_pin,LOW);
      analogWrite(pwm_pin, input);
    }else{
      input = 0;
      digitalWrite(dir_pin, HIGH);
      analogWrite(pwm_pin, -input);
    }
    
    changeTime+=2000;
  }


  aux = now;
  now = millis();

  unsigned long  Ts = (now - aux);
   long encoderArray = 0;
  lecturaEncoders(encoderArray);
  //Serial.println(Ts);
   w_medida = encoderArray*2.0*PI/(float)(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400

   Serial.print(input);
   Serial.print(" ");
   Serial.print((input*VEL_ANG_MAX_RUEDAS/255));
   Serial.print(" ");
   Serial.print(w_medida);
   Serial.println();
   delay(10); //Si no la cuenta de la velocidad de la rueda pira
}


void lecturaEncoders(long &encoders){

  encoders = -rueda->read();


  //Serial.println("Encoders:");
  //Serial.println(encoders[1]);



  //resetea el contador del objeto

  rueda->write(0);

  
  //return printPostionsArray;
}
