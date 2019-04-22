#include <PID_v1.h>
#include <Encoder.h>

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

int pulsosAdelante = 5;                      //Cantidad de escalones de velocidad que recibirá el robot para avanzar
float duracionPulso = 2000;                  //Duración de los escalones de velocidad
int velocidadPWM[5] = {50,100,150,200,250};  //5 velocidades para experimento
int indice;

float VEL_ANG_MAX_RUEDAS = 8.507;

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
 
  Serial.begin(115200);
  delay(10000);
  Serial.print("EXPERIMENTO: Deslizamiento por pulsos");
  
}


void loop() {
  
  while (Serial.available() == 0)         //Esperar a que haya un byte para leer del serial
  { 
    Serial.println("Esperando comando de inicio");
    delay(1000);
  }

  for (int j=0; j<5; j++)
  {
    digitalWrite(dir_1, LOW);              //Seteo de pines para movimiento hacia adelante
    digitalWrite(dir_2, HIGH);
    digitalWrite(dir_3, LOW);
    digitalWrite(dir_4, HIGH);
  
    //Movimiento hacia adelante
    for (int i=0; i<pulsosAdelante; i++)
    {
      analogWrite(pwm_1, velocidadPWM[j]);     //Movimiento de todas las ruedas a la velocidad seteada
      analogWrite(pwm_2, velocidadPWM[j]);
      analogWrite(pwm_3, velocidadPWM[j]);
      analogWrite(pwm_4, velocidadPWM[j]);
  
      delay(duracionPulso);                 //Delay de duracion del pulso
      
      analogWrite(pwm_1, 0);                //Frenado
      analogWrite(pwm_2, 0);
      analogWrite(pwm_3, 0);
      analogWrite(pwm_4, 0);
  
      delay(1000);                          //Delay entre pulsos
    }
  
    //Movimiento hacia atras
    digitalWrite(dir_1, HIGH);               //Seteo de pines para movimiento hacia atras
    digitalWrite(dir_2, LOW);
    digitalWrite(dir_3, HIGH);
    digitalWrite(dir_4, LOW);
    
    analogWrite(pwm_1, velocidadPWM[j]);       //Movimento hacia atras
    analogWrite(pwm_2, velocidadPWM[j]);
    analogWrite(pwm_3, velocidadPWM[j]);
    analogWrite(pwm_4, velocidadPWM[j]);
  
    delay(duracionPulso*pulsosAdelante);    //Duracion del movimiento hacia atras
  
    analogWrite(pwm_1, 0);                //Frenado
    analogWrite(pwm_2, 0);
    analogWrite(pwm_3, 0);
    analogWrite(pwm_4, 0);

    delay(5000);
  }
}
