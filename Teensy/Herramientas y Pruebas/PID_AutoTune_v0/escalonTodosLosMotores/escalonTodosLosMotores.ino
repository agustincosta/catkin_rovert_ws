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
bool arriba;
unsigned long changeTime = 0;
double w_medida1 = 0,w_medida2 = 0,w_medida3 = 0,w_medida4 = 0;
int sentidoDelaProgresion =1;
float VEL_ANG_MAX_RUEDAS = 8.507;
unsigned long now, aux;

//Definicion de encoders (A, B)
Encoder rueda1(6,5);
Encoder rueda2(7,8);
Encoder rueda3(9,10);
Encoder rueda4(12,11);


void setup() {
  // put your setup code here, to run once:
   
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_3, OUTPUT);
  pinMode(dir_3, OUTPUT);
  pinMode(pwm_4, OUTPUT);
  pinMode(dir_4, OUTPUT);
 

  
  changeTime = 0;
  Serial.begin(115200);
  delay(10000);
  Serial.print("EXPERIMENTO Resp en frecuencia 1, CON DIRECCION: ");

  //ACA HAY QUE CAMBIAR PARA TODAS LAS RUEDAS GIREN EN EL MISMO SENTIDO, O SEA QUE TODAS APORTEN A IR PARA ADELANTE O ATRAS
  #if ADELANTE
      Serial.println ("ADELANTE");
      digitalWrite(dir_1, LOW);
      digitalWrite(dir_2, HIGH);
      digitalWrite(dir_3, LOW);
      digitalWrite(dir_4, HIGH);
    #else
      Serial.println ("ATRAS");
      digitalWrite(dir_1, HIGH);
      digitalWrite(dir_2, LOW);
      digitalWrite(dir_3, HIGH);
      digitalWrite(dir_4, LOW);
    #endif
    Serial.println("inputPWM   wi  wo1  wo2  w3o wo4");

    now=millis();
  
}



void loop() {
  // put your main code here, to run repeatedly:
  if(millis()>changeTime)
  {
   
    
    if(arriba == false){
      input =100;

      analogWrite(pwm_1, input);
      analogWrite(pwm_2, input);
      analogWrite(pwm_3, input);
      analogWrite(pwm_4, input);
      arriba =true;
    }else{
      input = 0;
      analogWrite(pwm_1, input);
      analogWrite(pwm_2, input);
      analogWrite(pwm_3, input);
      analogWrite(pwm_4, input);
      arriba = false;
    }
    
    changeTime+=2000;
  }


  aux = now;
  now = millis();

  unsigned long  Ts = (now - aux);
   long encoderArray[4] ;
  lecturaEncoders(encoderArray);
  //Serial.println(Ts);
   w_medida1 = encoderArray[0]*2.0*PI/(float)(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400
   w_medida2 = encoderArray[1]*2.0*PI/(float)(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400
   w_medida3 = encoderArray[2]*2.0*PI/(float)(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400
   w_medida4 = encoderArray[3]*2.0*PI/(float)(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400
   Serial.print(input);
   Serial.print(" ");
   Serial.print((input*VEL_ANG_MAX_RUEDAS/255));
   Serial.print(" ");
   Serial.print(w_medida1);
    Serial.print(" ");
   Serial.print(w_medida2);
   Serial.print(" ");
   Serial.print(w_medida3);
    Serial.print(" ");
   Serial.print(w_medida4);
   Serial.println();
   delay(10); //Si no la cuenta de la velocidad de la rueda pira
}


void lecturaEncoders(long *encoders){
  encoders[0] = rueda1.read();
  encoders[1] = rueda2.read();
  encoders[2] = rueda3.read();
  encoders[3] = rueda4.read();

  //Serial.println("Encoders:");
  //Serial.println(encoders[1]);

  //resetea el contador del objeto
  rueda1.write(0);
  rueda2.write(0);
  rueda3.write(0);
  rueda4.write(0);

  //resetea el contador del objeto


  
  //return printPostionsArray;
}
