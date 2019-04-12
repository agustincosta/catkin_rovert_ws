#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <Encoder.h>
#include <HCSR04.h>
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <PID_v1.h>

#define WITH_IMU true
#define CONTROL_ACTIVO true
#define WITH_ULTRASOUND false
#define FEEDFOWARD_CONTROL false
#define PROMEDIADO false
#define PID_DEBUG true
#define BROADCAST_TF false


#define CONTROL_SAMPLE_RATE                    500  //hz  PI SAMPLE RATE SETEADA POR FUNCION, NO TIENE IF en el LOOP



//Este es un fix para evitar que al frenar qude un motor consumiendo por el PID
#define VELOCIDAD_MINIMA_EN_RAD_S 0.65

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

#define pin_led 13

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 

#define USE_TEENSY_HW_SERIAL


#define SerialDebug true  // Set to true to get Serial output for debugging

#define g 9.8



void desplazamiento(double x, double y, double th, double w1, double w2, double w3, double w4, double Ts, int radio, int a, int b, double pos[6]) ;

void actualizacionDeVelocidadMotores();

void actualizacionSetPointDeVelocidadMotores();

void medirUltrasonidos();

void encodersUpdateyCalculoOdometria(long Ts);

void checkROSconnection();

void envioMensajeOdometria();

void staticTransforms();

void controlDeRuedas();

void envioMensajeIMUyMAG();

void envioMensajeUltraSonidos();


void initIMU();

void initROSnodeWithMessages(int baudrate);

void initPID();

void setPinModes();

void medirIMUyMAG();

void cerrandoElLoopROS();
