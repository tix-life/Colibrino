// This file is part of the Colibrino project.

// Colibrino is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Colibrino is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with Foobar.  If not, see <https://www.gnu.org/licenses/>.

#include "Mouse.h"
#include <Wire.h>
#include "MahonyAHRS.h"
#include "mpu6050.h"
#include "blink.h"
#include "mouseIMU.h"

//---------------------------------------------------------------------------------------------------
//Definitions
//Mouse

/*********************************************************************
 * Global variables
 */

//float ax_filtro, ay_filtro, az_filtro, gx_filtro, gy_filtro, gz_filtro;
bool conectado = false;
//float yaw_filtro, pitch_filtro, roll_filtro;
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
/*********************************************************************
 * External ariables
 */
extern int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
extern float yaw_mahony, pitch_mahony, roll_mahony;
extern float axR, ayR, azR, gxR, gyR, gzR;
extern float axg, ayg, azg, gxrs, gyrs, gzrs;
extern bool g_novaPiscada;
//Objects
// BleMouse bleMouse;
// filters pbax,pbay,pbaz,pbgx,pbgy,pbgz;

#define INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 8        // (Arduino is 13, Teensy is 11, Teensy++ is 6)

extern int g_clique;

void setup() 
{
  Wire.begin();
  Wire.setClock(100000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  delay(100);
  MPU6050_Init();
  // pinMode(ACIONADOR,INPUT);
  eyeBlinkSetup();
}

int8_t gesto = INVALIDO;

void loop() 
{
  static int printDivider = 10;
  static int estado = 0;
  int8_t xchg = 0, ychg = 0;
  int scroll = 0;
  bool estadoAcionador = false;
  static int contador = 0;
  static int counter = 0;
  static int subcounter = 10;


  eyeBlinkRefresh();
  mpu6050_GetData();
  filtraIMU();
  if(counter<3000)
  {
    if(--subcounter == 0)
    {
      subcounter = 10;
      digitalWrite(16, HIGH);
    }
    digitalWrite(16, LOW);

  }
  else if(IMU_calibration())
  {
    //MahonyAHRSupdateIMU( gxrs,  gyrs,  gzrs , axg,  ayg,  azg);
    MahonyAHRSupdateIMU(gyrs, gzrs, gxrs, ayg, azg, axg);
    getRollPitchYaw_mahony();
    xchg = mouseHoriz();
    ychg = mouseVert();

    //gesto = maquinaGestos_v2(derivaYaw(yaw_mahony), derivaPitch(pitch_mahony), g_clique);
    // atividade = interpretaGestos(gesto);
    scroll = scrollDetector();
    // if(atividade == false)
    // {
    //   xchg = 0;
    //   ychg = 0;
    //   scroll = 0;
    // }
    //dwellClick(xchg, ychg, scroll);
    Mouse.move(xchg, ychg, scroll);  // move mouse on x axis
    if (g_novaPiscada)
    {
       Mouse.click();
    }
  }
  counter ++;
  // Serial.print(gyrs);
  // Serial.print(" ");
  // Serial.print(gzrs);
  // Serial.print(" ");
  // Serial.print(gxrs);
  // // Serial.print(ayg);
  // // Serial.print(" ");
  // // Serial.print(azg);
  // // Serial.print(" ");
  // // Serial.print(axg);
  // Serial.print(" ");
  // Serial.println();


}

