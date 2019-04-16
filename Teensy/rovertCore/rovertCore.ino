#include "nucleo_rovert.h"

#define WITH_IMU true
#define WITH_ULTRASOUND true
#define CONTROL_ACTIVO true

#define ODOMETRY_SAMPLE_RATE                   1000 //hz
#define CONTROL_MOTOR_SPEED_FREQUENCY          800   //hz
#define IMU_SAMPLE_RATE                        100  //hz
#define ULTRASONIDO_SAMPLE_RATE                5    //hz

#define ODOMETRY_PUBLISH_FREQUENCY             100  //hz
#define IMU_PUBLISH_FREQUENCY                  60  //hz
#define ULTRASONIDO_PUBLISH_FREQUENCY          5    //hz

#define BAUDRATE 230400


static uint32_t tTime[7]; //Timer por componente para asegurar rates
/************************************************************************
 * El orden del array de Timers es: 
 * 
 * IMU_SAMPLE_RATE                 
 * IMU_PUBLISH_FREQUENCY
 * 
 * ULTRASONIDO_SAMPLE_RATE     
 * ULTRASONIDO_PUBLISH_FREQUENCY 
 * 
 * ODOMETRY_SAMPLE_RATE                  
 * ODOMETRY_PUBLISH_FREQUENCY                       
 *         
 * CONTROL_MOTOR_SPEED_FREQUENCY  
 * 
 * ***********************************************************************/

/*******************************************************************************
********************************** SETUP ***************************************
*******************************************************************************/

void setup()
{ 
  Serial.begin(BAUDRATE);
  while(!Serial){};

  
  /********************* Inicilizacion y calibracion IMU ************************/
  {
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  #if WITH_IMU
  {
    initIMU();
  }
  #endif
  }

  /*************** Creacion de nodo ROS y publishers/subscirber *****************/
  {
  initROSnodeWithMessages(BAUDRATE);
  }

  /**************** Definicion de PINES para control de motores *****************/
  {
  setPinModes();
  }

 /************************************* PID *************************************/
 {
  #if CONTROL_ACTIVO
    initPID();
    
  #endif
 }
}

/*******************************************************************************
********************************** LOOP ****************************************
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  
  /************************************* IMU ***********************************/
  #if WITH_IMU 
  {
    if ((t-tTime[0]) >= (1000 / IMU_SAMPLE_RATE))
    {
      medirIMUyMAG();
      tTime[0]=t;
    }
    if ((t-tTime[1]) >= (1000 / IMU_PUBLISH_FREQUENCY))
    {
      envioMensajeIMUyMAG(); 
      tTime[1]=t;
    }
  }
  #endif

  /********************************* ULTRASONIDOS ******************************/
  #if WITH_ULTRASOUND
  {
    if ((t-tTime[2]) >= (1000 / ULTRASONIDO_SAMPLE_RATE))
    {
      medirUltrasonidos();
      tTime[2]=t;
    }
    if ((t-tTime[3]) >= (1000 / ULTRASONIDO_PUBLISH_FREQUENCY))
    {
      envioMensajeUltraSonidos(); 
      tTime[3]=t;
    }
  }
  #endif
  
  /***************************** ENCODERS & ODOMETRIA **************************/
  {
  if((t-tTime[4]) >= (1000/ODOMETRY_SAMPLE_RATE)){
    long Ts = (t-tTime[4]);
    encodersUpdateyCalculoOdometria(Ts);
    tTime[4]=t;
  }
  
  if((t-tTime[5]) >= (1000/ODOMETRY_PUBLISH_FREQUENCY)){
    envioMensajeOdometria();
    staticTransforms();
  }
  
  }
  
  /****************************** CALCULO DE CONTROL ***************************/
  {
  controlDeRuedas(); //Esta seccion lleva el control de rate adentro del PID, si no se usa pid habria que poner un samp rate como el resto
  }
 
  /****************************** COMANDO DE MOTORES ***************************/
  {
    if((t-tTime[6]) >= (1000/CONTROL_MOTOR_SPEED_FREQUENCY)){
        actualizacionDeVelocidadMotores();
    }
  }

  /************************************* ROS ***********************************/
  {
    cerrandoElLoopROS();
    checkROSconnection();
   }
  
  //delay(1);
}
