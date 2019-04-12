#include <PID_v1.h>
#include <Encoder.h>


//Con esta variable se setea el control solo por pid o feed foward si esta en true
#define controlActivo true
#define feedfowardcontrol false

#define ESCALON true

#if ESCALON
  #define SINUSOIDAL false
#else
  #define SINUSOIDAL true
#endif

#define ploter true
#define conComando false
/* wseteada wmedida wcomando*/

#define pwm_2 29
#define dir_2 28

#define dir_pin dir_2
#define pwm_pin pwm_2

double w2 = 6;
double w_medida = 0;
double w2_comando = 0;

float VEL_ANG_MAX_RUEDAS = 8.507;

Encoder rueda2(7,8);

int izqAdelante[]= {dir_2,pwm_2};


byte ATuneModeRemember=2;
//double input=80, output=50, setpoint=180;


//Estos valores surgieron de experimentar.
//double kp=0.5,ki=2.6,kd=0.0; //Test con grafica 22/3/19 sin feed foward
//double kp= 0.597, ki =514614.9557 ,kd = 0 ;
double kp= 0.69, ki =27.1119 ,kd = 0.00 ;// Meentira

//double kp= 1.0945, ki= 20.3967 ,kd = 0.0 ;  el que gusto menos

double outputStart=5;
double aTuneStep=0.1, aTuneNoise=0.01, aTuneStartValue=5;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime, changeTime;

PID myPID(&w_medida, &w2_comando, &w2,kp,ki,kd, DIRECT);
//PID_ATune aTune(&w2_medida, &w2_comando);

//set to false to connect to the real world
boolean useSimulation =false;

void setup()
{

  pinMode(pwm_2, OUTPUT);
  pinMode(dir_2, OUTPUT);
  
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  
  serialTime = 0;
  Serial.begin(115200);
  myPID.SetOutputLimits(-VEL_ANG_MAX_RUEDAS*0.95,VEL_ANG_MAX_RUEDAS*0.95);
  myPID.SetSampleTime(50);
  delay(6000);

}

unsigned long now, aux;

void loop()
{


  #if ESCALON
    if(millis()>changeTime)
  {

    if (w2<=0){
        w2 = 6;
    }else{
        w2=0;
    }
    changeTime+=5000;
    
  }
  #endif

  /*
  while(Serial.available())
  {
    char comando = Serial.read();
    switch(comando) {

   case 'a':
      kp-=0.01;
      break;  //optional 
  
   case 'q' :
      kp+=0.01;
      break; //optional 
   case 's':
      ki-=0.01;
      break; optional 
  
   case 'w' :
      ki+=0.01;
      break;  optional 

   case 'd':
      kd-=0.01;
      break; 
  
   case 'e' :
      kd+=0.01;
      break; 

   case 'f':
      kp-=0.1;
      break; 
  
   case 'r' :
      kp+=0.1;
      break;
   case 'g':
      ki-=0.1;
      break; 
  
   case 't' :
      ki+=0.1;
      break; 

   case 'h':
      kd-=0.1;
      break;
   case 'y' :
      kd+=0.1;
      break; 

   case 'z':
      w2-=0.1;
      break; 
  
   case 'x' :
      w2+=0.1;
      break; 

   case 'c' :
      w2=0;
      break; 
   case 'v' :
      w2=6;
    break; 
     
  
   /* you can have any number of case statements 
   default : /* Optional 
          Serial.println("Comando no valido");
   }
  }
  if(kp<0){
    kp=0;
  }
  if(ki<0){
    ki=0;
  }
  if(kd<0){
    kd=0;
  }*/
  
  //myPID.SetTunings(kp, ki, kd);
  aux = now;
  now = millis();

  unsigned long  Ts = (now - aux); //Se multiplica por mil para que quede en segundos, mas parecido a lo que usamos en nuestro sketch
  //Serial.print("Ts: ");
  //Serial.println(Ts); 
  long encoderArray[4] = {0,0,0,0};
 
  lecturaEncoders(encoderArray);
 
   w_medida = encoderArray[1]*2*PI/(8.4*Ts); //Ts esta en milis por eso 8.4 en vez de 8400
  myPID.Compute();
  #if controlActivo
    #if feedfoward
      w2_comando = w2_comando + w2; // Se suma la velocidad pasada por pid y el target de velocidad     
    #endif
  #else 
    w2_comando = w2;
  #endif
  
   if(w2_comando<0){
      digitalWrite(dir_2, LOW); //controls the direction the motor
      analogWrite(pwm_2, -w2_comando/VEL_ANG_MAX_RUEDAS*255);
   }else{
      digitalWrite(dir_2, HIGH);
      analogWrite(pwm_2, w2_comando/VEL_ANG_MAX_RUEDAS*255);
   }
       //digitalWrite(dir_2, HIGH);
       
        //analogWrite(pwm_2, 40000);
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=10;
    
  }


  

  delay(10);
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    w2_comando=aTuneStartValue;
//    aTune.SetNoiseBand(aTuneNoise);
//    aTune.SetOutputStep(aTuneStep);
//    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
//    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  #if ploter
  #if conComando
  Serial.print(w2); Serial.print(" ");
  Serial.print(w_medida);
  Serial.print(" ");
  Serial.println(w2_comando);
  #else
  Serial.print(w2); Serial.print(" ");
  Serial.println(w_medida);
  #endif
  
  
  #else
  
  Serial.print("setpoint: ");Serial.print(w2); Serial.print(" ");
  Serial.print("input: ");Serial.print(w2_medida); Serial.print(" ");
  Serial.print("output: ");Serial.print(w2_comando); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
  #endif
}



void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}


void lecturaEncoders(long *encoders){

  encoders[1] = rueda2.read();


  //Serial.println("Encoders:");
  //Serial.println(encoders[1]);



  //resetea el contador del objeto

  rueda2.write(0);

  
  //return printPostionsArray;
}
