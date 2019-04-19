#include "nucleo_rovert.h"

int derAdelante[]= {dir_1,pwm_1};
int izqAdelante[]= {dir_2,pwm_2};
int izqAtras[]= {dir_3,pwm_3};
int derAtras[]= {dir_4,pwm_4};


// Pin definitions
int intPin = 26;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

  // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

//double encoderArray[4] = {0,0,0,0};
long encoderArray[4] = {0,0,0,0};

//Parametros Rover
float A = 0.16;
float B = 0.0975;
float R = 0.05;
float VEL_ANG_MAX_RUEDAS = 8.507;

  float radio1 = 0.05;   //Radio de las ruedas
  float a1 = 0.16;      //Mitad del ancho del carro
  float b1 = 0.0975;     //Mitad del largo del carro
  //Todo en metros

#if WITH_IMU
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
#endif

//Definicion de ultrasonidos (Trigger, Echo)
USensor der(1, 2);
USensor atr(3, 4);
USensor izq(14, 15);
USensor adel(24, 25);



//Puse segun A y B
Encoder rueda1(6,5);
Encoder rueda2(7,8);
Encoder rueda3(9,10);
Encoder rueda4(12,11);

double w1 = 0;
double w2 = 0;
double w3 = 0;
double w4 = 0;

double w1_medida = 0;
double w2_medida = 0;
double w3_medida = 0;
double w4_medida = 0;

double w1_comando = 0;
double w2_comando = 0;
double w3_comando = 0;
double w4_comando = 0;




/*Variables para ultrasonidos*/
float dist_adel = 0;
float dist_izq = 0;
float dist_der = 0;
float dist_atr = 0;
int muestras_us = 5;    //Cantidad de muestras de US a promediar
float us_max_range = 2.00;

/*Variable de conexión a rosserial*/
bool rosserial_online = false;

/**
   * PID
   */
#if CONTROL_ACTIVO

  #if FEEDFOWARD_CONTROL
  double Kp=1.5, Ki=6, Kd=0.06;
  #else
  //double Kp=0.28, Ki=2.6, Kd=0.0; //A partir de test realizados el 23/3
  //double Kp= 0.69, Ki =27.1119 ,Kd = 0.00 ;// Bastante bien PIDSampleTime 50ms

  //double Kp= 2.092, Ki =29.934 ,Kd = 0.009 ;// Bastante bien PIDSampleTime 50ms
  double Kp=1.068, Ki =17.225 ,Kd = 0.00 ;// Bastante bien PIDSampleTime 50ms
  //double Kp= 0.215, Ki =4.032 ,Kd = 0.00 ;// Bastante bien PIDSampleTime 50ms
  
  #endif
  PID rueda1PID(&w1_medida,&w1_comando,&w1,Kp, Ki, Kd, DIRECT);
  PID rueda2PID(&w2_medida,&w2_comando,&w2,Kp, Ki, Kd, DIRECT);
  PID rueda3PID(&w3_medida,&w3_comando,&w3,Kp, Ki, Kd, DIRECT);
  PID rueda4PID(&w4_medida,&w4_comando,&w4,Kp, Ki, Kd, DIRECT);
  
#endif

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

float vel_x=0, vel_y=0, vel_ang=0;

double pos[6] = {0,0,0,0,0,0};



//Crear quaternion manualmente por simplicidad y porque tf::createQuaternionMsgFromYaw no anda
double qx = 0;
double qy = 0;
double qz = 0;
double qw = 0;

double mag = 0;

ros::NodeHandle  nh;

/**************************** Subscriber **************************************/
void velocity_cb(const geometry_msgs::Twist& vel);
//Instancia de subscriber
ros::Subscriber<geometry_msgs::Twist> sub_velocity("cmd_vel", velocity_cb);

/**************************** Publishers **************************************/
/*IMU*/
#if WITH_IMU
//Defino mensaje de tipo Imu para MPU9250
sensor_msgs::Imu imu_msg;
//Obtengo objeto Publisher
ros::Publisher pub_imu( "/imu", &imu_msg);


/*Magentometro*/
//Defino mensaje de tipo MagneticField para AK8963
sensor_msgs::MagneticField mag_msg;
//Obtengo objeto Publisher
ros::Publisher pub_mag( "/mag", &mag_msg);
char frameid_imu[] = "/imu";
char frameid_mag[] = "/mag";
#endif

/*Odometria*/
//Defino mensaje de tipo nav_msgs::Odometry para encoders
nav_msgs::Odometry odom_msg;
//Obtengo objeto Publisher
ros::Publisher pub_odom( "/odom", &odom_msg);
char frameid_odom[] = "/odom";



/*Ultrasonido adelante*/
//Defino mensaje de tipo sensor_msgs::Range para utlrasonidos
sensor_msgs::Range us_adel_msg;
//Obtengo objeto Publisher
ros::Publisher pub_us_adel("/us_adel", &us_adel_msg);
char frameid_us_adel[] = "/us_adel";

/*Ultrasonido izquierda*/
//Defino mensaje de tipo sensor_msgs::Range para utlrasonidos
sensor_msgs::Range us_izq_msg;
//Obtengo objeto Publisher
ros::Publisher pub_us_izq("/us_izq", &us_izq_msg);
char frameid_us_izq[] = "/us_izq";

/*Ultrasonido derecha*/
//Defino mensaje de tipo sensor_msgs::Range para utlrasonidos
sensor_msgs::Range us_der_msg;
//Obtengo objeto Publisher
ros::Publisher pub_us_der("/us_der", &us_der_msg);
char frameid_us_der[] = "/us_der";

/*Ultrasonido atras*/
//Defino mensaje de tipo sensor_msgs::Range para utlrasonidos
sensor_msgs::Range us_atr_msg;
//Obtengo objeto Publisher
ros::Publisher pub_us_atr("/us_atr", &us_atr_msg);
char frameid_us_atr[] = "/us_atr";

/***************************** TF Broadcasters ********************************/
//Defino mensaje de tipo geometry_msgs::TransformStamped para movimiento de sistemas de referencia
geometry_msgs::TransformStamped odom_trans;
//Obtengo Broadcaster
tf::TransformBroadcaster odom_broadcaster;


char frameid_parent[] = "/odom";
char frameid_child[] = "/base_link";

//Defino mensaje de tipo geometry_msgs::TransformStamped para movimiento de sistemas de referencia
geometry_msgs::TransformStamped laser_trans;
//Obtengo Broadcaster
tf::TransformBroadcaster laser_broadcaster;


/******************************** ROS Time ************************************/
ros::Time current_time;
ros::Time last_time;

void initIMU(){
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    
    if (!rosserial_online)
    {
      Serial.println("MPU9250 initialized for active data mode....");
    }

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    /*myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    delay(2000); // Add delay to see results before serial spew of data*/

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void initROSnodeWithMessages(int baudrate){
  nh.initNode();
  nh.getHardware()->setBaud(baudrate);

  /*Velocidad*/
  nh.subscribe(sub_velocity);
  
  /*IMU*/
  #if WITH_IMU
  nh.advertise(pub_imu);
  nh.advertise(pub_mag);
  
  imu_msg.header.seq = 0;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = frameid_imu;
  
  mag_msg.header.seq = 0;
  mag_msg.header.stamp = nh.now();
  mag_msg.header.frame_id = frameid_mag;
  #endif

  /*ENCODERS*/
  odom_broadcaster.init(nh);
  nh.advertise(pub_odom);
  
  odom_msg.child_frame_id = frameid_child;
  odom_msg.header.frame_id = frameid_parent;

    /*ULTRASONIDOS*/
  nh.advertise(pub_us_adel);
  us_adel_msg.header.seq = 0;
  us_adel_msg.header.frame_id = frameid_us_adel;
  us_adel_msg.radiation_type = 0;    //Ultrasound
  us_adel_msg.min_range = 0.04;
  us_adel_msg.max_range = us_max_range;
  us_adel_msg.field_of_view = 0.349; //20° pero en rad
  
  nh.advertise(pub_us_izq);
  us_izq_msg.header.seq = 0;
  us_izq_msg.header.frame_id = frameid_us_izq;
  us_izq_msg.radiation_type = 0;    //Ultrasound
  us_izq_msg.min_range = 0.05;
  us_izq_msg.max_range = us_max_range;
  us_izq_msg.field_of_view = 0.349; //20° pero en rad
  
  nh.advertise(pub_us_der);
  us_der_msg.header.seq = 0;
  us_der_msg.header.frame_id = frameid_us_der;
  us_der_msg.radiation_type = 0;    //Ultrasound
  us_der_msg.min_range = 0.05;
  us_der_msg.max_range = us_max_range;
  us_der_msg.field_of_view = 0.349; //20° pero en rad
  
  nh.advertise(pub_us_atr);
  us_atr_msg.header.seq = 0;
  us_atr_msg.header.frame_id = frameid_us_atr;
  us_atr_msg.radiation_type = 0;    //Ultrasound
  us_atr_msg.min_range = 0.05;
  us_atr_msg.max_range = us_max_range;
  us_atr_msg.field_of_view = 0.349; //20° pero en rad
}

void initPID(){
  #if CONTROL_ACTIVO
  rueda1PID.SetMode(AUTOMATIC);
  rueda2PID.SetMode(AUTOMATIC);
  rueda3PID.SetMode(AUTOMATIC);
  rueda4PID.SetMode(AUTOMATIC);

   //Se ajusta el rango de salida porque de lo contrario solo funciona para el rango 0 - 255
  rueda1PID.SetOutputLimits(-VEL_ANG_MAX_RUEDAS*0.98,VEL_ANG_MAX_RUEDAS*0.98);
  rueda2PID.SetOutputLimits(-VEL_ANG_MAX_RUEDAS*0.98,VEL_ANG_MAX_RUEDAS*0.98);
  rueda3PID.SetOutputLimits(-VEL_ANG_MAX_RUEDAS*0.98,VEL_ANG_MAX_RUEDAS*0.98);
  rueda4PID.SetOutputLimits(-VEL_ANG_MAX_RUEDAS*0.98,VEL_ANG_MAX_RUEDAS*0.98);

  rueda1PID.SetSampleTime((1000/(float)CONTROL_SAMPLE_RATE));
  rueda2PID.SetSampleTime((1000/(float)CONTROL_SAMPLE_RATE));
  rueda3PID.SetSampleTime((1000/(float)CONTROL_SAMPLE_RATE));
  rueda4PID.SetSampleTime((1000/(float)CONTROL_SAMPLE_RATE));
  #endif
}

void initUS(){
  
}

void setPinModes(){
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(dir_2, OUTPUT);


  pinMode(pwm_3, OUTPUT);
  pinMode(dir_3, OUTPUT);
  pinMode(pwm_4, OUTPUT);
  pinMode(dir_4, OUTPUT);

  /*Encendido de led para mostrar actividad*/
  pinMode(pin_led, OUTPUT);
  digitalWrite(pin_led,HIGH);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
}


void desplazamiento(double x, double y, double th, double w1, double w2, double w3, double w4, double Ts, float radio, float a, float b, double pos[6]){
  
  double dt = Ts;

  
  //Ecuacion de velocidad relativa a partir de velocidades de ruedas
  double vx = (radio/4)*(-w1+w2-w3+w4);
  double vy = (radio/4)*(+w1+w2+w3+w4);
  double vth = (radio/(4*(a+b)))*(w1-w2-w3+w4);
  
  double dx = (vx * cos(th) - vy*sin(th))*dt;
  double dy = (vx * sin(th) + vy*cos(th))*dt;
  double dth = vth*dt;

  pos[0] = x+dx;
  pos[1] = y+dy;
  pos[2] = th+dth;
  pos[3] = vx;
  pos[4] = vy;
  pos[5] = vth;
  
}

void lecturaEncoders(long *encoders){

  encoders[0] = rueda1.read();
  encoders[1] = rueda2.read();
  encoders[2] = rueda3.read();
  encoders[3] = rueda4.read();

  if (!rosserial_online)
  {
    Serial.println("Encoders:");
    Serial.println(encoders[0]);
    Serial.println(encoders[1]);
    Serial.println(encoders[2]);
    Serial.println(encoders[3]);
  }

  //resetea el contador del objeto
  rueda1.write(0);
  rueda2.write(0);
  rueda3.write(0);
  rueda4.write(0);
  //return printPostionsArray;
}

void velocity_cb(const geometry_msgs::Twist& vel){
     vel_x = vel.linear.x ;
     vel_y = vel.linear.y;
     vel_ang = vel.angular.z;

     //Se calcula la velocidad que debe tener cada rueda
     actualizacionSetPointDeVelocidadMotores();
}

void actualizacionSetPointDeVelocidadMotores(){
    //Estas ecuaciones pasan de las velocidades relativas a las velocidades de cada rueda.
    w1 = (1/R)*(- vel_x - vel_y + -(A+B)*vel_ang) ;
    w2 = (1/R)*(+ vel_x - vel_y + (-A-B)*vel_ang) ;
    w3 = (1/R)*(- vel_x - vel_y + -(-A-B)*vel_ang) ;
    w4 = (1/R)*(+ vel_x - vel_y + (A+B)*vel_ang) ;

    //Se invirtieron los signos de la velocidad en y

    //Esta seccion tara la velocidad requerida a la maxima aguantada por el robot para luego poder linealizarla.
    if(w1>VEL_ANG_MAX_RUEDAS){
      w1 = VEL_ANG_MAX_RUEDAS;
    }
    
    if(w2>VEL_ANG_MAX_RUEDAS){
      w2 = VEL_ANG_MAX_RUEDAS;
    }
    
    if(w3>VEL_ANG_MAX_RUEDAS){
      w3 = VEL_ANG_MAX_RUEDAS;
    }
    
    if(w4>VEL_ANG_MAX_RUEDAS){
      w4 = VEL_ANG_MAX_RUEDAS;
    }

  
       
 }

void comandoMotor(int direccion, int pwmPin, float velocidad){

  if(velocidad<0){
      velocidad= -velocidad ;
      digitalWrite(direccion, LOW); //controls the direction the motor
   }else{
      digitalWrite(direccion, HIGH);
   }

   if(velocidad <= VELOCIDAD_MINIMA_EN_RAD_S){
    velocidad = 0;
   }

  analogWrite(pwmPin, velocidad/VEL_ANG_MAX_RUEDAS*255);
      
 }

void actualizacionDeVelocidadMotores(){
  //El cambio de signo esta dado para acomodar el sentido de las ruedas con relacion a como fue conectado el motor al driver.
    #if FEEDFOWARD_CONTROL
    comandoMotor(derAdelante[0],derAdelante[1],w1_comando+w1);
    comandoMotor(izqAdelante[0],izqAdelante[1],w2_comando+w2);
    comandoMotor(izqAtras[0],izqAtras[1],w3_comando+w3);
    comandoMotor(derAtras[0],derAtras[1],w4_comando+w4); 
    #else
    comandoMotor(derAdelante[0],derAdelante[1],w1_comando);
    comandoMotor(izqAdelante[0],izqAdelante[1],w2_comando);
    comandoMotor(izqAtras[0],izqAtras[1],w3_comando);
    comandoMotor(derAtras[0],derAtras[1],w4_comando); 
    #endif
  }

void medirUltrasonidos(){
 
    if(!adel.isMeasureReady() && !izq.isMeasureReady() && !der.isMeasureReady() && !atr.isMeasureReady()){
        adel.startMeasure();
        izq.startMeasure();
        der.startMeasure();
        atr.startMeasure();
    }
}

void envioMensajeUltraSonidos(){
 

    /*  //ESTO HAY QUE CAMBIARLO *********************************************
    dist_adel = 0;
    dist_izq = 0;
    dist_der = 0;
    dist_atr = 0;

    float medida_adel = 0;
    float medida_izq = 0;
    float medida_der = 0;
    float medida_atr = 0;

    for (int i=0; i<muestras_us; i++){
      
      medida_adel = adel.measureDistanceCm()/100;
      medida_izq = izq.measureDistanceCm()/100;
      medida_der = der.measureDistanceCm()/100;
      medida_atr = atr.measureDistanceCm()/100;
  
      if (medida_adel<= 0 || medida_adel > us_max_range) {medida_adel = us_max_range;}
      if (medida_izq <= 0 || medida_izq  > us_max_range) {medida_izq = us_max_range;}
      if (medida_der <= 0 || medida_der  > us_max_range) {medida_der = us_max_range;}
      if (medida_atr <= 0 || medida_atr  > us_max_range) {medida_atr = us_max_range;}

      dist_adel += medida_adel;
      dist_izq += medida_izq;
      dist_der += medida_der;
      dist_atr += medida_atr;
    }

    dist_adel /= muestras_us;
    dist_izq /= muestras_us;
    dist_der /= muestras_us;
    dist_atr /= muestras_us; 
    */
    
   if(adel.isMeasureReady() && izq.isMeasureReady() && der.isMeasureReady() && atr.isMeasureReady()){
        dist_adel = adel.lastMeasure()/100;
        dist_izq = izq.lastMeasure()/100;
        dist_der = der.lastMeasure()/100;
        dist_atr = atr.lastMeasure()/100;
        
    		//Adelante
    		us_adel_msg.range = dist_adel;
    		us_adel_msg.header.seq++;
    		us_adel_msg.header.stamp = nh.now();
    		pub_us_adel.publish(&us_adel_msg);
    
    		//Izquierda
    		us_izq_msg.range = dist_izq;
    		us_izq_msg.header.seq++;
    		us_izq_msg.header.stamp = nh.now();
    		pub_us_izq.publish(&us_izq_msg);
    
    		//Derecha
    		us_der_msg.range = dist_der;
    		us_der_msg.header.seq++;
    		us_der_msg.header.stamp = nh.now();
    		pub_us_der.publish(&us_der_msg);
    
    		//Atras
    		us_atr_msg.range = dist_atr;
    		us_atr_msg.header.seq++;
    		us_atr_msg.header.stamp = nh.now();
    		pub_us_atr.publish(&us_atr_msg);
   }
}

void medirIMUyMAG(){
  
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
    
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  myIMU.updateTime();
  myIMU.count = millis();

  
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug && !rosserial_online)
      {
        
        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);

      }
      myIMU.sumCount = 0;
      myIMU.sum = 0;

  }
}

void encodersUpdateyCalculoOdometria(long Ts){
  
  //Revisar. 
  lecturaEncoders(encoderArray);
  
  w1_medida = encoderArray[0]*2.0*PI/(8.4*Ts);
  w2_medida = encoderArray[1]*2.0*PI/(8.4*Ts);
  w3_medida = encoderArray[2]*2.0*PI/(8.4*Ts);
  w4_medida = encoderArray[3]*2.0*PI/(8.4*Ts);
  //Las velocidades angulares de las ruedas en rad/s

  desplazamiento(x, y, th, w1_medida, w2_medida, w3_medida, w4_medida, (float)Ts/1000, R, A, B, pos);

  x = pos[0];
  y = pos[1];
  th = pos[2];
  vx = pos[3];
  vy = pos[4];
  vth = pos[5];
  
  if (!rosserial_online)
  {
    Serial.println((String)"Pos x = " + pos[0]);
    Serial.println((String)"Pos y = " + pos[1]);
    Serial.println((String)"Ori = " + pos[2]);
    Serial.println((String)"Vel x = " + pos[3]);
    Serial.println((String)"Vel y = " + pos[4]);
    Serial.println((String)"Vel ang = " + pos[5]);
  }
}

void checkROSconnection(){
  if (nh.connected())
  {
    x = pos[0];
    y = pos[1];
    th = pos[2];
    vx = pos[3];
    vy = pos[4];
    vth = pos[5];
    rosserial_online = true;
  }
  else
  {
    x = 0;
    y = 0;
    th = 0;
    vx = 0;
    vy = 0;
    vth = 0;
    rosserial_online = false;
    vel_x = 0.0;
    vel_y = 0.0;
    vel_ang = 0.0;
  }
  
}

void envioMensajeOdometria(){
  qx = 0;
  qy = 0;
  qz = sin(th/2);
  qw = cos(th/2);
  
  mag = sqrt(qz*qz + qw*qw);

  qz = qz/mag;
  qw = qw/mag;
   
  odom_msg.pose.pose.position.x = y;//x;
  odom_msg.pose.pose.position.y = -x;//y;
  odom_msg.pose.pose.position.z = 0.0;
  
  odom_msg.pose.pose.orientation.x = qx;
  odom_msg.pose.pose.orientation.y = qy;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  odom_msg.pose.covariance[0] = 8.47e-6;
  odom_msg.pose.covariance[7] = 8.47e-6;
  odom_msg.pose.covariance[14]= 99999;
  odom_msg.pose.covariance[21]= 99999;
  odom_msg.pose.covariance[28]= 99999;
  odom_msg.pose.covariance[35]= 1.30e-4;
  
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;

  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vth;

  odom_msg.twist.covariance[0] = 6.35e-6;
  odom_msg.twist.covariance[7] = 6.35e-6;
  odom_msg.twist.covariance[14]= 99999;
  odom_msg.twist.covariance[21]= 99999;
  odom_msg.twist.covariance[28]= 99999;
  odom_msg.twist.covariance[35]= 9.76e-5;
 
  odom_msg.header.stamp = nh.now();
  pub_odom.publish(&odom_msg);

#if BROADCAST_TF
  //Transformacion
  odom_trans.header.frame_id = frameid_parent;
  odom_trans.child_frame_id = frameid_child;
  
  odom_trans.transform.translation.x = y;//x;
  
  odom_trans.transform.translation.y = -x;//y;
  odom_trans.transform.translation.z = 0.0;
  
  odom_trans.transform.rotation.x = qx;
  odom_trans.transform.rotation.y = qy;
  odom_trans.transform.rotation.z = qz;
  odom_trans.transform.rotation.w = qw;

  odom_trans.header.stamp = nh.now();
  odom_broadcaster.sendTransform(odom_trans);
#endif
}

void envioMensajeIMUyMAG(){
  
  imu_msg.linear_acceleration.x = myIMU.ax/g;
  imu_msg.linear_acceleration.y = myIMU.ay/g;
  imu_msg.linear_acceleration.z = myIMU.az/g;

  imu_msg.linear_acceleration_covariance[0] = 1.57e-5;
  imu_msg.linear_acceleration_covariance[4] = 1.39e-5;
  imu_msg.linear_acceleration_covariance[8] = 0.98e-5;

  imu_msg.angular_velocity.x = myIMU.gx*2*PI/360;
  imu_msg.angular_velocity.y = myIMU.gy*2*PI/360;
  imu_msg.angular_velocity.z = myIMU.gz*2*PI/360;

  imu_msg.angular_velocity_covariance[0] = 1.30e-4;
  imu_msg.angular_velocity_covariance[4] = 1.02e-4;
  imu_msg.angular_velocity_covariance[8] = 3.97e-5;
     
  imu_msg.header.stamp = nh.now();
  imu_msg.header.seq ++;
  pub_imu.publish(&imu_msg);

  mag_msg.magnetic_field.x = myIMU.mx;
  mag_msg.magnetic_field.y = myIMU.my;
  mag_msg.header.seq++;
  mag_msg.header.stamp = nh.now();
  pub_mag.publish(&mag_msg);
}

void staticTransforms(){
  //Transformacion del frame del lidar al centro del robot
  laser_trans.header.frame_id = "/base_link";
  laser_trans.child_frame_id = "/base_laser";
  
  laser_trans.transform.translation.x = 0.0;
  laser_trans.transform.translation.y = 0.0;
  laser_trans.transform.translation.z = 0.130;
  
  laser_trans.transform.rotation.x = 0;
  laser_trans.transform.rotation.y = 0;
  laser_trans.transform.rotation.z = 0.105;
  laser_trans.transform.rotation.w = 0.995;

  laser_trans.header.stamp = nh.now();
  odom_broadcaster.sendTransform(laser_trans);
}

void controlDeRuedas(){
  //Control de ruedas
  #if CONTROL_ACTIVO
  
   /**
   * PID: actualizacion
   */
   //Esto quiere corregir el mismatch entre la odometria y el movimiento del comando
  
  w1_medida = -w1_medida;
  w3_medida = -w3_medida;
  rueda1PID.Compute();
  rueda2PID.Compute();
  rueda3PID.Compute();
  rueda4PID.Compute();

  if (!rosserial_online)
  {
    Serial.println("W setpoint:");
    Serial.println(w1);
    Serial.println(w2);
    Serial.println(w3);
    Serial.println(w4);
    
    Serial.println("W comando:");
    Serial.println(w1_comando);
    Serial.println(w2_comando);
    Serial.println(w3_comando);
    Serial.println(w4_comando);
  }
  
  #else
  
  w1_comando = w1;
  w2_comando = w2;
  w3_comando = w3;
  w4_comando = w4;
 
  
  #endif 
}

void cerrandoElLoopROS(){
  nh.spinOnce();
 // last_time = current_time;
}
