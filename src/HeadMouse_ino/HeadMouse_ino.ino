#include <Mouse.h>
//Carrega a biblioteca Wire
#include <Wire.h>
// #include "filters.h"
#include "MahonyAHRS.h"
#include"mpu6050.h"


//---------------------------------------------------------------------------------------------------
//Definitions
//Mouse
#define SENSITIVITY  35;
//
#define MPU6050_ACC_GAIN 16384.0
#define MPU6050_GYRO_GAIN 131.072 
/*********************************************************************
 * Global variables
 */
float axR, ayR, azR, gxR, gyR, gzR;
float axg, ayg, azg, gxrs, gyrs, gzrs;
//float ax_filtro, ay_filtro, az_filtro, gx_filtro, gy_filtro, gz_filtro;
bool conectado = false;
//float yaw_filtro, pitch_filtro, roll_filtro;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
/*********************************************************************
 * External variables
 */
extern int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
extern float yaw_mahony,pitch_mahony,roll_mahony;
//Objects
// BleMouse bleMouse;
// filters pbax,pbay,pbaz,pbgx,pbgy,pbgz;

#define INTERRUPT_PIN 7 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 8 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


void filtraIMU()
{
  //Filtro 2 ordem
  // ax_filtro= filtro_2PB10Hz(AcX, pbax);
  // ay_filtro= filtro_2PB10Hz(AcY, pbay);
  // az_filtro= filtro_2PB10Hz(AcZ, pbaz);
  // gx_filtro= filtro_2PB10Hz(GyX, pbgx);
  // gy_filtro= filtro_2PB10Hz(GyY, pbgy);
  // gz_filtro= filtro_2PB10Hz(GyZ, pbgz);
  
  //Converte
  axg = (float)(AcX /*- LSM6DSM_AXOFFSET*/) / MPU6050_ACC_GAIN;
  ayg = (float)(AcY /*- LSM6DSM_AYOFFSET*/) / MPU6050_ACC_GAIN;
  azg = (float)(AcZ /*- LSM6DSM_AZOFFSET*/) / MPU6050_ACC_GAIN;
  gxrs = (float)(GyX - (-143)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
  gyrs = (float)(GyY - (245)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
  gzrs = (float)(GyZ - (25)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}
float corrigeYaw(float sinal)
{
  static float valorDeriv = 0, zero = 0;
  static float sinalCorrigido=0, offset = 0;
  static float  sinalAnterior = 0;
  static int s=1,d=-1;
  
  valorDeriv = (sinal - zero);
  zero = sinal;
  
  if(valorDeriv <= -180)
  {
      offset += +360.0f;
  }
  else if(valorDeriv >= 180)
  {
      offset += -360.0f;
  }
  sinalCorrigido = sinal;
  sinalCorrigido += offset;
  
  return sinalCorrigido;
}
uint8_t mouseHoriz(void)
{
  static float horzZero =0.0f;
  static float horzValue = 0.0f;  // Stores current analog output of each axis
  static float yaw_corrigido = 0.0f;
  yaw_corrigido = corrigePitch(yaw_mahony);

  horzValue = (yaw_corrigido - horzZero)*SENSITIVITY;
  horzZero = yaw_corrigido;
  
  return horzValue;
}
float corrigePitch(float sinal)
{
  static float valorDeriv = 0, zero = 0;
  static float sinalCorrigido=0, offset = 0;
  static float  sinalAnterior = 0;
  static int s=1,d=-1;
  
  valorDeriv = (sinal - zero);
  zero = sinal;

  if(valorDeriv <= -180)
  {
      offset += +360.0f;
  }
  else if(valorDeriv >= 180)
  {
      offset += -360.0f;
  }
  sinalCorrigido = sinal;
  sinalCorrigido += offset;
  
  return sinalCorrigido;
}
uint8_t mouseVert(void)
{
  static float vertZero =0.0f;
  static float vertValue = 0.0f;  // Stores current analog output of each axis
  static float pitch_corrigido = 0.0f;
  pitch_corrigido = corrigePitch(pitch_mahony);
  vertValue = (pitch_corrigido - vertZero)*SENSITIVITY;
  vertZero = pitch_corrigido;

  return vertValue;
}
void setup() 
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  MPU6050_Init();
  // pinMode(ACIONADOR,INPUT);
}

void loop() 
{
  static int printDivider = 10;
  static int estado = 0;
  uint8_t xchg = 0,ychg = 0;
  bool estadoAcionador = false;
  static int contador = 0;

  mpu6050_GetData();
  filtraIMU();
  MahonyAHRSupdateIMU( gxrs,  gyrs,  gzrs , axg,  ayg,  azg);
  getRollPitchYaw_mahony();
  xchg = mouseHoriz()*2;
  ychg = mouseVert()*2;

  Mouse.move(xchg, ychg, 0);                                      // move mouse on x axis
  
  
}
