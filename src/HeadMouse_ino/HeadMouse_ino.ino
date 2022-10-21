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
//Carrega a biblioteca Wire
#include <Wire.h>
// #include "filters.h"
#include "MahonyAHRS.h"
#include"mpu6050.h"

//---------------------------------------------------------------------------------------------------
//Definitions
//Mouse
#define SENSIBILIDADE  30
#define TEMPO_DWELL_CICK_MS 1000 
#define DWELL_CICK false
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

#define ON true
#define OFF false
int g_clique = 0;
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
  gxrs = (float)(GyX - (0)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
  gyrs = (float)(GyY - (0)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
  gzrs = (float)(GyZ - (0)) / MPU6050_GYRO_GAIN * 0.01745329; //degree to radians
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
int8_t mouseHoriz(void)
{
  static float horzZero =0.0f;
  static float horzValue = 0.0f;  // Stores current analog output of each axis
  static float yaw_corrigido = 0.0f;
  static int amostragem = 5;
  yaw_corrigido = corrigePitch(yaw_mahony);
  
  if(--amostragem == 0)
  {
    amostragem = 5;
    horzValue = (yaw_corrigido - horzZero)*SENSIBILIDADE;
    horzZero = yaw_corrigido;
  }
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
int8_t mouseVert(void)
{
  static float vertZero =0.0f;
  static float vertValue = 0.0f;  // Stores current analog output of each axis
  static float pitch_corrigido = 0.0f;
  static int amostragem = 5;
  pitch_corrigido = corrigePitch(pitch_mahony);
  if(--amostragem == 0)
  {
    amostragem = 5;
    vertValue = (pitch_corrigido - vertZero)*SENSIBILIDADE;
    vertZero = pitch_corrigido;
  }
  return vertValue;
}


/*********************************************************************
 * @fn      scrollDetector
 * 
 * @brief   Ativa ou reativa o Colibri de acordo com a 
 *          sequencia de movimentos feita
 *
 * @param   horz - derivada do Yaw
 * @param   verti - derivada do Pitch
 *
 * @return  ligado - flag para ativar/desativar o colibri.
 */
#define RMEDIA 0.001f
#define RMARGEM 4
int scrollDetector() {
  static float rmedia = 90;
  static uint32_t timer1 = 0;
  int diferenca;
  
  diferenca = roll_mahony - rmedia;
  
  
            //SendInt(diferenca);
         //SendLineEnd();
  
  if (abs(diferenca) < RMARGEM) {
    rmedia = ((1 - RMEDIA) * rmedia) + (RMEDIA * roll_mahony);
    timer1 = millis();
    return 0;
  }
  else if (abs(diferenca) > 2.5*RMARGEM) { // Ignora roll muito grande
    if (millis() - timer1 > 5000) {       // Reseta média se ficar muito tempo com roll alto
      rmedia = roll_mahony;
      timer1 = millis();
    }
    return 0;
  }
  else if (millis() - timer1 > 200) {
    timer1 = millis();
    if (diferenca > 0) {
      return 1 ;//+ (2 * diferenca / RMARGEM);
    }
    else {
      return -1;// + (2 * diferenca / RMARGEM);
    }
  }
  return 0;
}
int tabelaGestos[6][2]={0,0,0,0,0,0};
int tabelaMOV[6][2]={0,0,0,0,0,0};
int8_t VG[4]={0,0,0};
#define NADA        0
#define ESQUERDA    1
#define DIREITA     2
#define CIMA        3
#define BAIXO       4
#define SCROLL_ESQ  5
#define SCROLL_DIR  6
#define DESLIGA     ESQUERDA
#define LIGA        CIMA
#define SCROLL      SCROLL_ESQ  
#define TIMEOUT_MOVE 110 //Tempo entre movimentos
bool g_atividade = 0;
bool atividade = true;

#define INVALIDO    0
#define SIM         1
#define NAO         2
enum
{
  Estado_movimentov2 = 0,
  Estado_passaNaZm,
};
int8_t coletaMovimentos(float horz, float vert)
{
  int8_t movimento = 0;
  static int estado = 0, tipo = 0,tipo_1=0,tipo_2=0;
  switch (estado)
  {
    case Estado_movimentov2:
      movimento = 0;
      tipo =0;
      tipo_1=0;tipo_2=0;
      if(horz> 1.0f)
      {
        tipo =  DIREITA;
        estado = Estado_passaNaZm;
      }
      else if(horz< -1.0f)
      {
        tipo =  ESQUERDA;
        estado = Estado_passaNaZm;
      }
      else if(vert > 0.75f)
      {
        tipo =  CIMA;
        estado = Estado_passaNaZm;
      }
      else if(vert< -0.75f)
      {
        tipo =  BAIXO;  
        estado = Estado_passaNaZm;
      }

      // if(tipo_1!= 0 || tipo_2 !=0)
      // {
      //   if(abs_float(horz) >= abs_float(vert) )
      //   {
      //     tipo = tipo_1 ;
      //   }
      //   if(abs_float(vert) >= abs_float(horz) )
      //   {
      //     tipo = tipo_2 ;
      //   }

      //   estado = Estado_passaNaZm;
      // }
      break;
    case Estado_passaNaZm:
      
      if(tipo == DIREITA)
      {
        if(horz< 1.0f)
        {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      }
      else if(tipo == ESQUERDA)
      {
        if(horz> (-1.0f))
        {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      }
      else if(tipo == CIMA)
      {
        if(vert < 0.75f)
        {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      }
      else if(tipo == BAIXO)
      {
        if(vert > (-0.75f))
        {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      }
      
      break;
    
    default:
      break;
  }
  return movimento;
}

int8_t regrideIndice(int8_t indice, int8_t casas)
{
  int8_t resultado = 0;
  resultado =  indice;
  for (int i = 0; i < casas; i++)
  {
    if(resultado == 0)
    {
        resultado = 5;
    }
    else
    {
        resultado--;
    }
  }
  
  return resultado;
}
int8_t progrideIndice(int8_t indice)
{
  int8_t resultado = 0;
  resultado =  indice;

  if(resultado == 5)
  {
    resultado = 0;
  }
  else
  {
    resultado++;
  }
  
  return resultado;
}

int calculaTempo(int valorIntervalo)
{
  int tempoCerto =0;
  tempoCerto = (int)(valorIntervalo);
  return tempoCerto;
}
int8_t analisaVetor()
{
  int8_t resultado = INVALIDO;
  if(VG[0]==ESQUERDA && VG[1]==DIREITA && VG[2]==ESQUERDA && VG[3]==DIREITA )
  {
    resultado = NAO;
  }
  else if(VG[0]==DIREITA && VG[1]==ESQUERDA && VG[2]==DIREITA && VG[3]==ESQUERDA )
  {
    resultado = NAO;
  }
  else if(VG[0]==CIMA && VG[1]==BAIXO && VG[2]==CIMA )
  {
    resultado = SIM;
  }
  else if(VG[0]==BAIXO && VG[1]==CIMA && VG[2]==BAIXO )
  {
    resultado = SIM;
  }
  else
  { 
    resultado = INVALIDO;
  }
  return resultado;
}
enum
{
  Estado_novoGesto = 0,
  Estado_reduzIndice,
  Estado_calculaIntervalo_1,
  Estado_calculaIntervalo_2,
  Estado_calculaIntervalo_3,
  Estado_calculaIntervalo_Total,
  Estado_analisaVetor,
};
int8_t moveBackwards(int8_t indice,int8_t clique)
{
  /*********************************************************************
   * Requisitos:
   * 1.Gesto diferente de 0 //feito
   * 2.Intervalo 1 e dois dentro do timeout //feito //
   * 3.Intervalo total dentro do timeout //
   */
  int8_t estado = 0;
  int8_t copiaIndice = 0;//, i = 0;
  int8_t gestos = 0;
  unsigned long intervalo_1 = 0,intervalo_2 = 0,intervalo_3=0, intervalo_total = 0;
  bool emProcesso = true;
  int8_t resultadoVetor = 0;
  int8_t tipoGesto =INVALIDO;
  
  do
  {
    if(clique && g_atividade == true )
    {
      emProcesso= false;
      VG[0]=0;VG[1]=0;VG[2]=0;
      tabelaGestos[0][1]=0;tabelaGestos[1][1]=0;tabelaGestos[2][1]=0;tabelaGestos[3][1]=0;tabelaGestos[4][1]=0;tabelaGestos[5][1]=0;
    }
    switch (estado)
    {
      case Estado_novoGesto:
        VG[0]=0;VG[1]=0;VG[2]=0;
        copiaIndice = indice;
        estado = Estado_reduzIndice; 
        break;
      case Estado_reduzIndice:
       
        indice = regrideIndice(indice,1);
        gestos ++;
        //Se contem zero na posição anterior - sai
        if(tabelaGestos[indice][1]==0)
        {
          if(gestos < 5 )
            emProcesso = false;
        }

        if(gestos==3)
        {
            estado = Estado_calculaIntervalo_1; 
        }
        else if(gestos==4)
        {
            estado = Estado_calculaIntervalo_2; 
        }
        else if(gestos==5)
        {
            estado = Estado_calculaIntervalo_3; 
        }

        break;
      case Estado_calculaIntervalo_1:
      
        intervalo_1 = tabelaGestos[regrideIndice(copiaIndice,1)][0]-tabelaGestos[regrideIndice(copiaIndice,2)][0];
        VG[0]=tabelaGestos[regrideIndice(copiaIndice,1)][1];
        VG[1]=tabelaGestos[regrideIndice(copiaIndice,2)][1];
        
        
        if(intervalo_1 > calculaTempo(700))
        {
            emProcesso= false;
        }
        else
            estado = Estado_reduzIndice;
        break;
      case Estado_calculaIntervalo_2:
        VG[2]=tabelaGestos[regrideIndice(copiaIndice,3)][1];
        
        intervalo_2 = tabelaGestos[regrideIndice(copiaIndice,2)][0]-tabelaGestos[progrideIndice(indice)][0];
        if(intervalo_2 > calculaTempo(300))
            emProcesso = false;
        else
        {
          resultadoVetor = analisaVetor();
          switch (resultadoVetor)
          {
            case SIM:
              estado = Estado_calculaIntervalo_Total;
              break;
            
            default:
              estado = Estado_reduzIndice;
              break;
          }

        }
        break;
   case Estado_calculaIntervalo_3:
        VG[3]=tabelaGestos[regrideIndice(copiaIndice,4)][1];
        
        intervalo_3 = tabelaGestos[regrideIndice(copiaIndice,3)][0]-tabelaGestos[progrideIndice(indice)][0];
        if(intervalo_3 > calculaTempo(300))
            emProcesso = false;
        else
        {
          resultadoVetor=analisaVetor();
            switch (resultadoVetor)
            {
              case NAO:
                estado = Estado_calculaIntervalo_Total;
                break;
              
              default:
                emProcesso = false;
                break;
            }
        }
        break;
    case Estado_calculaIntervalo_Total:
      
        intervalo_total = tabelaGestos[regrideIndice((gestos-1),1)][0]-tabelaGestos[regrideIndice(copiaIndice,3)][0];
        if(intervalo_total > calculaTempo(1500))
        {
          tipoGesto = INVALIDO;
        }
        else
        {
          tipoGesto = analisaVetor();
        }
        emProcesso = false;
        break;
      default:
        break;
    }   
    

  }while (emProcesso);

  return tipoGesto;
}
/*********************************************************************
 * @fn      maquinaGestos_v2
 * 
 * @brief   Ativa ou reativa o Colibri de acordo com a 
 *          sequencia de movimentos feita com janela de tempo
 *
 * @param   horz - derivada do Yaw
 *
 * @return  ligado - flag para ativar/desativar o colibri.
 */

int8_t maquinaGestos_v2(float horz,float vert, int8_t clique)
{
  static int8_t indiceG = 0;
  static int8_t encontraPadrao = 0;
  static unsigned long contadorGestosM=0;
  int8_t movimento = 0;
  movimento = coletaMovimentos(horz,vert);
  contadorGestosM=millis();
  //Prenche tabela
  if(movimento!= 0)
  {
      if(indiceG > 5)
      {
          indiceG = 0;
      }
      tabelaGestos[indiceG][0] = contadorGestosM; //Tempo
      tabelaGestos[indiceG][1] = movimento;
      
      
      //Analise vetor
      indiceG ++;
      encontraPadrao = moveBackwards(indiceG, clique);
      
  
  }
  
  
}
enum
{
  Estado_inicializa=0,
  Estado_dorme ,
  Estado_AcordaDes,  
};
bool interpretaGestos(int gestos)
{
  static int8_t estado = 0;
  static bool saida = true;
  
  switch (estado)
  {
    case Estado_inicializa:
      if(gestos == NAO)
      {
        estado = Estado_dorme;
      }
      break;
    case Estado_dorme:
      saida = false;
      if(gestos == SIM  )
      {
        estado = Estado_AcordaDes;
      }
      break;
    case Estado_AcordaDes:
      saida = true;
      if(gestos == NAO)
      {
        estado = Estado_dorme;
      }
      break;
    default:
      break;
  }

  return saida;
}


float derivaYaw(float sinal)
{
  static float valorDeriv = 0;
  static float sinalCorrigido=0, zero = 0;
  //static float zero = 0;
  sinalCorrigido = corrigeYaw(sinal);
  valorDeriv =(-1)*(sinalCorrigido -zero);
  zero = sinalCorrigido;
  return valorDeriv;
}
float derivaPitch(float sinal)
{
  static float valorDeriv = 0;
  static float sinalCorrigido=0, zero = 0;
  //static float zero = 0;
  sinalCorrigido = corrigePitch(sinal);
  valorDeriv = (sinalCorrigido -zero);
  zero = sinalCorrigido;
  return valorDeriv;
}



void setup() 
{
  Wire.begin();
  Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
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
  int8_t xchg = 0,ychg = 0;
  int scroll =0;
  bool estadoAcionador = false;
  static int contador = 0;

  eyeBlinkRefresh();

  mpu6050_GetData();
  filtraIMU();
  //MahonyAHRSupdateIMU( gxrs,  gyrs,  gzrs , axg,  ayg,  azg);
  MahonyAHRSupdateIMU( gyrs,  gzrs,  gxrs , ayg,  azg,  axg);
  getRollPitchYaw_mahony();
  xchg = mouseHoriz();
  ychg = mouseVert();
  gesto = maquinaGestos_v2( derivaYaw(yaw_mahony),derivaPitch(pitch_mahony) ,g_clique );
  // atividade = interpretaGestos(gesto);
  scroll = scrollDetector();
  // if(atividade == false)
  // {
  //   xchg = 0;
  //   ychg = 0;
  //   scroll = 0;
  // }
  dwellClick(xchg, ychg, scroll);
  Mouse.move(xchg, ychg, scroll);                                      // move mouse on x axis
  
  //if (g_novaPiscada) Mouse.click();
  
}


#include "TimerOne.h"

// Duração típica de uma piscada.
#define EYES_BLINK_DURATION 30 // ms

int sensorPin  = 0;        // IR Sensor
int ledPin = 16;           // Visualization
int relePin = 9;          // Pino do rele
int lastLevel = 0;         // Previous IR level
int sensorLastRefValue = 0;
int lastChange = 0;        // Change in IR level
int changeThreshold = 5;  // How hard a rising edge do we need?
int pino_buzzer = 14;
int timer = 0;
int i = 0;

int pino_sample = 15;

int sensorValue = 0;
int sensorValueAnterior = 0;
unsigned long mediaJanelaAtual = 0;
unsigned long mediaJanelaAnterior = 0;
unsigned long eyes_blink_timer = 0;
unsigned long tempoAnterior = 0;
boolean edge_detected = false;
boolean sistema_inicializado = false;

boolean bordaDeSubida = false;
boolean espera = false;
boolean horaDoPrint = false;
boolean piscada = false;

unsigned long tendenciaDescida = 0;
unsigned long tendenciaSubida = 0;

#define MIN_QUANT_LEITURAS_ASCENDENTES  40
#define MIN_QUANT_LEITURAS_DESCENDENTES  5

#define PERIODO_ENTRE_LEITURAS_MS  3
#define NUM_LEITURAS_JANELA 50
#define NUM_MEDIAS_JANELA 50

unsigned int vetorLeituras[NUM_LEITURAS_JANELA];
unsigned long vetorMedias[NUM_MEDIAS_JANELA];
unsigned long tickMs = 0;

int indexLeituras = 0;
int indexMedias = 0;
long derivada = 0;
int contLiberaPiscada = 0;


#define LIMIAR_BORDA_SUBIDA    10
#define LIMIAR_BORDA_DESCIDA  8

//visualization
int duration = 100;        // Length of visualization
float lastStart = 0;       // Last start of visualization

int fase = 1;
int estado = 0;
int estadoPiscadas = 0;
int estadoMaquinaAcao = 0;

#define NUM_CICLOS_INICIAIS 2*NUM_LEITURAS_JANELA
int ciclosIniciais = 0;

// Para 5 fases:

// Duração de cada fase fase: 200 us
// Duração total do ciclo: 1000 us (1 ms)
// Tempo de medição (analogRead()): 110 us
// Tempo total com LED ON: 200 us + 110 us = 310 us
// Duty cycle: 310/1000 = 31% (menos de 1/3 da potência do LED)
#define NUMERO_DE_FASES 5

#define MAX_PISCADAS_PARA_CALIBRAR 4

// DEFINIÇÕES DE TIPO
typedef struct
{
  int picoP;
  int picoN;
  int duracao;

}
infoPiscada;

// VARIÁVEIS GLOBAIS
infoPiscada g_bufferPiscadas [MAX_PISCADAS_PARA_CALIBRAR];
infoPiscada g_piscadaMediaPadrao, g_maiorPiscada, g_menorPiscada, g_piscadaAtual;

volatile int16_t sensorValueOn = 0;
volatile int16_t sensorValueOff = 0;

int g_quantPiscadasNoBuffer = 0;
int  g_toleranciaPicoP;
int  g_toleranciaPicoN;
int  g_toleranciaDuracao;

int g_deltaMedioPiscada = 0;
float g_valorBaseLine = 0.0;
bool g_calibrado = true;
bool g_novaPiscada = false;
bool g_releAtuando = false;
bool g_releLigado = false;
bool g_resetMaquinaBordas = false;
bool g_movimentoAcontecendo = false;
bool g_avisandoCalibracao = false;
bool g_piscadaBloqueada = false;

float g_mediaJanelaAtual = 0;

//boolean g_novaPiscada = false;

// CABEÇALHOS DAS FUNÇÕES
void MaquinaBordas (void);
void MaquinaPiscadas(void);
void MaquinaAcao(void);
unsigned long CalculaMedia (void);
long CalculaDerivada (void);
void SalvarPiscada (int valorPicoDerivadaP, int valorPicoDerivadaN, int tempoEntrePicos);
void CalibrarPiscada (void);

void dwellClick(int x, int y, int s)
{
  static uint32_t timestamp = 0;
  static bool clickReady = false;

  if (!DWELL_CICK) return;
  
  if (x == 0 && y == 0 && s == 0) {
    if ( g_clique == 0 && clickReady) {
      if (millis() - timestamp > TEMPO_DWELL_CICK_MS)
      {
        clickReady = false;
        if (millis() > 5000) Mouse.press();
        g_clique = 1;
        digitalWrite(ledPin, HIGH);
      }
    }
  }
  else {
    Mouse.release();
    digitalWrite(ledPin, LOW);
    g_clique = 0;
    timestamp = millis();
    clickReady = true;
  }
  
  if (g_clique && (millis() - timestamp > TEMPO_DWELL_CICK_MS + 50)) // Holds click for 50ms
  {
    Mouse.release();
    digitalWrite(ledPin, LOW);
    g_clique = 0;
  }
}

void eyeBlinkSetup() {

  delay(100);

  Serial.begin(115200);      // Debug constructor
  pinMode(ledPin, OUTPUT);     // Visualizatio constructor
  //pinMode(pino_buzzer, OUTPUT); // <--- DESCOMENTE PRO BARULHO AUMENTAR!
  pinMode(pino_sample, OUTPUT);
  pinMode(relePin, OUTPUT);
  digitalWrite(relePin, LOW);
  tempoAnterior = 0;


  Timer1.initialize(1000 / NUMERO_DE_FASES); // Inicializa o Timer1 e configura para um período de (1000/NUMERO_DE_FASES) microssegundos
  Timer1.attachInterrupt(callback); // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1
}

void callback()
{

  	static int8_t fase = 1;
	
	switch (fase)
	{
		case 1:
		{
			// Liga o IR
			digitalWrite(pino_sample, HIGH);

			horaDoPrint = true;

			// Um milissegundo a mais
			tickMs++;
		}
		break;

		case 2:
		{
			// O LED j� ligou e a sa�da do transistor j� estabilizou. Hora de medir a sa�da e desligar o Led.
      sensorValueOn = analogRead(sensorPin);
      digitalWrite(pino_sample, LOW);
			//AcionadorPiscada_acionamentoLEDIR(false);
			
		}
		break;

		case 3:
		{
			// o LED j� desligou e a sa�da do transistor j� estabilizou. Hora de medir novamente a sa�da.
			sensorValueOff = analogRead(sensorPin);
		}
		break;

		case 4:
		{
			// faz nada
		}
		break;

		case 5:
		{

		}
		break;

		default:
		{
			// Faz nada. S� gasta o tempo da fase.
		}
		break;
	}

  if (fase < NUMERO_DE_FASES)
  {
    fase++;
  }
  else
  {
    fase = 1;
  }
}


void eyeBlinkRefresh() {

  //MaquinaBordas ();
  //MaquinaPiscadas();
  //MaquinaAcao();
  AcionadorPiscada_refresh();

}

////////////////////////////////////////////////////////////////////
//
//	MaquinaBordas
//	
//	Descri��o: 
// 
//  Parametro de entrada: - nenhum.
// 
//	Retorno: -nenhum.
//
////////////////////////////////////////////////////////////////////
enum
{
  ESTADO_INICIALIZACAO = 0,
  ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA,
  ESTADO_DETECTANDO_BORDA,
  ESTADO_DETECTANDO_FIM_PISCADA,
  ESTADO_DETECTANDO_PICOS,
  ESTADO_AGUARDANDO_PICO_POSITIVO,
  ESTADO_AGUARDANDO_PICO_NEGATIVO,
  EVENTO_DETECTADO,
  ESTADO_AGUARDANDO_REPOUSO,


  ESTADO_AGUARDANDO_INICIO_PISCADA,
  ESTADO_AGUARDANDO_CONFIRMAR_INICIO_PISCADA,
  ESTADO_AGUARDANDO_FINAL_PISCADA
};

void MaquinaBordas (void)
{
  static int mediaAntesDaBorda;
  static int limiarSuperior;
  static int limiarInferior;

  static int valorPicoDerivadaP;
  static int valorPicoDerivadaN;
  static int valorInicioMovimento;
  static int valorFimMovimento;
  static int tempoPicoP;
  static int tempoPicoN;
  static int etapaDetectEvento;
  static int numCiclosAguardandoPico;
  static float mediaDurantePiscada;
  static bool s_led;
  static unsigned long tempoInicioBorda;
  static unsigned long tempoInicioMovimento;
  static unsigned long tempoFimMovimento;
  static unsigned long tempoInicioRepouso;
  static unsigned long tempoInicioPiscada;
  static unsigned long tempoParaResetBaselineRepouso;
  static float limiarDeltaBaseline;

  int tempoEntrePicos;



#define DERIVADA_LIMIAR_BORDA_SUBIDA  4//2.5
#define DERIVADA_LIMIAR_BORDA_DESCIDA  -3.4//-2.5
#define DERIVADA_LIMIAR_BORDA_SUBIDA_2  1.0 //0.4
#define DERIVADA_LIMIAR_BORDA_DESCIDA_2  -1.0 //0.4
#define DERIVADA_LIMIAR_SUPERIOR_BASELINE 0.25
#define DERIVADA_LIMIAR_INFERIOR_BASELINE -0.25

#define NUM_CICLOS_LIBERAR_PISCADA 150
#define DELTA_MINIMO_PARA_CONFIRMAR_PISCADA 10
#define MAX_CICLOS_AGUARDANDO_PICO  1
#define TEMPO_MINIMO_REPOUSO_ENTRE_BORDAS_MS  50

  // m�quina de estados para interpreta��o de piscadas
  switch (estado)
  {
    case ESTADO_INICIALIZACAO:
      {
        s_led = false;
        //LEDs_Acionamento(ACIONADOR_LED_RELE_STATUS,false);
        numCiclosAguardandoPico = 0;
        etapaDetectEvento = 0;

        // Deixa correr uma quantidade m�nima de ciclos iniciais para que as m�dias estabilizem
        if (ciclosIniciais <= NUM_CICLOS_INICIAIS)
        {
          ciclosIniciais++;
          //printf("%f\r\n",g_mediaJanelaAtual);
        }
        else
        {
          // Inicializa��o conclu�da!
          tempoInicioRepouso = tickMs;
          estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
        }
      }
      break;

    case ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA:
      {
        // Este estado exige um tempo de repouso no qual n�o deve haver movimento brusco, de forma a criar
        // um intervalo for�ado entre as piscadas, evitando tamb�m que uma piscada forte e lenta seja confundida com
        // um trem de v�rias piscadas r�pidas
#define TEMPO_MINIMO_REPOUSO_ANTES_DE_BORDA_MS 20 //50

        // cessar movimento
        g_movimentoAcontecendo = false;

        if ((tickMs - tempoInicioRepouso) > TEMPO_MINIMO_REPOUSO_ANTES_DE_BORDA_MS)
        {
#define MAX_DELTA_BASELINE  25

					// TODO: Ajuste do delta base line pelo potenciometro.
          //limiarDeltaBaseline =  (float) analogRead(pinoAjusteDeltaBaseline) / 1023 * MAX_DELTA_BASELINE;

#ifdef SEM_POT_AJUSTE_SENSIBILIDADE
          limiarDeltaBaseline = DELTA_BASELINE_PADRAO;
#endif

          //printf("#\r\n");

          // Libera novas piscadas.
          g_piscadaBloqueada = false;
          estado = ESTADO_DETECTANDO_BORDA;

          g_valorBaseLine = g_mediaJanelaAtual;

          tempoParaResetBaselineRepouso = tickMs;

          // Bipezinho super curto.
			// REMOVENDO BIPES LEGADOS AcionadorPiscada_acionamentoBuzzer (ON);	// Barulho alto.
					//AcionadorPiscada_acionamentoBuzzer(INPUT,HIGH);		// Barulho baixo.
        }
      }
      break;

    case ESTADO_DETECTANDO_BORDA:
      {
        if (g_movimentoAcontecendo == false)
        {
          // Desliga bipezinho super curto.
		  // REMOVENDO BIPES LEGADOS AcionadorPiscada_acionamentoBuzzer (OFF);
        }

#define DELTA_BASELINE_UP 12
#define DELTA_BASELINE_DOWN 5

        if ((derivada < DERIVADA_LIMIAR_BORDA_DESCIDA) || (derivada > DERIVADA_LIMIAR_BORDA_SUBIDA))
        {

          //if (g_mediaJanelaAtual > (g_valorBaseLine + DELTA_BASELINE_UP))
          if (g_mediaJanelaAtual > (g_valorBaseLine + limiarDeltaBaseline))
          {
            AcionadorPiscada_acionamentoRele(true);
			AcionadorPiscada_acionamentoBuzzer (ON);
			  
            g_movimentoAcontecendo = true;

			 	
            //printf("ON!\r\n");

						//printf("BASELINE REPOUSO: ");
						//printf("%.1f\r\n",g_valorBaseLine);

						//printf(" LIMIAR DELTA BASELINE: ");
						//printf("%.1f\r\n",limiarDeltaBaseline);            

            tempoInicioPiscada = tickMs;

            estado = ESTADO_DETECTANDO_FIM_PISCADA;
          }
          else
          {
            //printf("*** MOVIMENTO INSUFICIENTE ***\r\n");

          }

          // Reinicia a contagem do prazo para reset do Baseline.
          tempoParaResetBaselineRepouso = tickMs;
        }
        else
        {
          // Se ficarmos um tempo em repouso, redefine o baseline.

#define TIMEOUT_REDEFINICAO_DE_BASELINE_REPOUSO_MS  100

          // se tivermos repouso por uma certa quantidade de tempo, redefine o baseline
          if ( (tickMs - tempoParaResetBaselineRepouso) > TIMEOUT_REDEFINICAO_DE_BASELINE_REPOUSO_MS)
          {
            tempoInicioRepouso = tickMs;
            estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
          }
        }
      }
      break;

    case ESTADO_DETECTANDO_FIM_PISCADA:
      {
        //printf("%f\r\n",derivada);
        //printf("%f\r\n",g_mediaJanelaAtual);

        //if (g_mediaJanelaAtual < (g_valorBaseLine + DELTA_BASELINE_DOWN))
        if (g_mediaJanelaAtual < (g_valorBaseLine + 0.7 * limiarDeltaBaseline)) // a histerese para fim da piscada � de baseline + 70% do limiar do delta para in�cio da piscada
        {
			AcionadorPiscada_acionamentoRele(false);
			AcionadorPiscada_acionamentoBuzzer (OFF);

          g_movimentoAcontecendo = false;

          //printf("OFF!\r\n");

          tempoInicioRepouso = tickMs;

          estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
        }

#define TIMEOUT_ESTABILIZACAO_PISCADA_MS  1000

        // Segunda condi��o de sa�da: se a piscada n�o estiver est�vel ap�s um tempo, for�a a sa�da
        if ( (tickMs - tempoInicioPiscada) > TIMEOUT_ESTABILIZACAO_PISCADA_MS)
        {
          if ((derivada < DERIVADA_LIMIAR_BORDA_DESCIDA) || (derivada > DERIVADA_LIMIAR_BORDA_SUBIDA))
          {
				AcionadorPiscada_acionamentoRele(false);
				AcionadorPiscada_acionamentoBuzzer (OFF);

            g_movimentoAcontecendo = false;

			// REMOVENDO BIPES LEGADOS AcionadorPiscada_acionamentoBuzzer(OFF);

            //printf("OFF NA MARRETA!\r\n");

            tempoInicioRepouso = tickMs;

            estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
          }
        }
      }
      break;

    case ESTADO_DETECTANDO_PICOS:
      {
        // REMOVENDO BIPES LEGADOS AcionadorPiscada_acionamentoBuzzer(OFF);
				
        //digitalWrite(ledPin, LOW);

        if (numCiclosAguardandoPico < MAX_CICLOS_AGUARDANDO_PICO)
        {
          // s� aguarda
          numCiclosAguardandoPico++;
        }
        else
        {
          //printf("%d\r\n"mediaAntesDaBorda);

          // Usa essas vari�veis para salvar o maior e o menor valor do movimento:
          if (mediaAntesDaBorda > valorPicoDerivadaP)
          {
            valorPicoDerivadaP = mediaAntesDaBorda;
          }
          if (mediaAntesDaBorda < valorPicoDerivadaN)
          {
            valorPicoDerivadaN = mediaAntesDaBorda;
          }

          // Marca o tempo em que O MOVIMENTO pode estar acabando
          tempoFimMovimento = tickMs;

          // Salva o que pode ser o �ltimo valor registrado durante o movimento
          valorFimMovimento = mediaAntesDaBorda;

          // Vai ver se o movimento vai acabar por aqui
          estado = ESTADO_AGUARDANDO_REPOUSO;
        }
      }
      break;

    case ESTADO_AGUARDANDO_REPOUSO:
      {

        // J� temos um bom tempo sem bordas detectadas?
        //if ((tickMs - tempoInicioBorda) > TEMPO_MINIMO_REPOUSO_ENTRE_BORDAS_MS)

#define TEMPO_MAXIMO_PISCADA_PILOTO_MS   1000

        if ((tickMs - tempoInicioBorda) > TEMPO_MAXIMO_PISCADA_PILOTO_MS)
        {
          // Fim do movimento: calcula o tempo que levou.
          tempoEntrePicos = tempoFimMovimento - tempoInicioMovimento;

          // Volta para o estado inicial de detec��o de repouso antes da pr�xima borda.
          g_piscadaBloqueada = true;
          tempoInicioRepouso = tickMs;
          estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
        }
        else
        {
          //printf("%f\r\n",g_mediaJanelaAtual);

          // Usa essas vari�veis para salvar o maior e o menor valor do movimento:
          if (g_mediaJanelaAtual > valorPicoDerivadaP)
          {
            valorPicoDerivadaP = g_mediaJanelaAtual;
          }
          if (g_mediaJanelaAtual < valorPicoDerivadaN)
          {
            valorPicoDerivadaN = g_mediaJanelaAtual;
          }

        }
      }
      break;
  }
}



enum
{
  ESTADO_PISCADAS_INICIAL = 0,
  ESTADO_PISCADAS_AGUARDAR_PISCADAS_CALIBRACAO,
};



#define TEMPO_MAXIMO_ENTRE_PISCADAS_CALIBRACAO_MS 1000

void MaquinaPiscadas(void)
{
  static int quantPiscadasAnterior;
  static long tempoPiscada;
  int i;

  switch (estadoPiscadas)
  {
    case ESTADO_PISCADAS_INICIAL:
      {
        g_quantPiscadasNoBuffer = 0;
        quantPiscadasAnterior = 0;
        digitalWrite(ledPin, LOW);

        estadoPiscadas = ESTADO_PISCADAS_AGUARDAR_PISCADAS_CALIBRACAO;
      }
      break;

    case ESTADO_PISCADAS_AGUARDAR_PISCADAS_CALIBRACAO:
      {
        if (g_quantPiscadasNoBuffer > 0)
        {
          if (g_quantPiscadasNoBuffer > quantPiscadasAnterior)
          {
            // guarda o tempo de início:
            tempoPiscada = tickMs;

            quantPiscadasAnterior = g_quantPiscadasNoBuffer;

            // acende o LED para aguardar as demais piscadas
            digitalWrite(ledPin, HIGH);

            if (g_quantPiscadasNoBuffer == MAX_PISCADAS_PARA_CALIBRAR)
            {
              // faz as contas para determinar se vai calibrar

              //printão:
              for (i = 0; i < MAX_PISCADAS_PARA_CALIBRAR; i++)
              {
                Serial.print(g_bufferPiscadas [i].picoP);
                Serial.print(";");
                Serial.print(g_bufferPiscadas [i].picoN);
                Serial.print(";");
                Serial.println(g_bufferPiscadas [i].duracao);
              }

              CalibrarPiscada();

              estadoPiscadas = ESTADO_PISCADAS_INICIAL;
            }
          }
          else
          {
            // verifica se já expirou o tempo máximo entre duas piscadas para calibração
            if ((tickMs - tempoPiscada) > TEMPO_MAXIMO_ENTRE_PISCADAS_CALIBRACAO_MS)
            {
              // volta pro começo
              estadoPiscadas = ESTADO_PISCADAS_INICIAL;
            }
          }
        }
        else
        {
          // faz nada. Continua aguardando as piscadas
        }
      }
      break;

  }
}

enum
{
  ESTADO_MAQUINA_ACAO_INICIAL = 0,
  ESTADO_MAQUINA_ACAO_DESLIGAR_RELE,
};



// Calcula a média das últimas 50 leituras e salva no buffer circular de médias calculadas
unsigned long CalculaMedia ()
{
  int i;
  unsigned long media = 0;

  for (i = 0; i < NUM_LEITURAS_JANELA; i++)
  {
    media += vetorLeituras[i];
  }

  media = media / NUM_LEITURAS_JANELA;

  // atualiza o buffer circular de médias
  vetorMedias[indexMedias] = media;

  //atualiza índice do buffer circular de médias
  indexMedias++;

  if (indexMedias == NUM_MEDIAS_JANELA)
  {
    indexMedias = 0;
  }

  return media;
}

long CalculaDerivada ()
{
  long derivada = 0;

  if (indexMedias > 0)
  {
    derivada = vetorMedias[indexMedias - 1] - vetorMedias[indexMedias];
  }
  else
  {
    derivada = vetorMedias[NUM_MEDIAS_JANELA - 1] - vetorMedias[0];
  }

  return derivada;
}


void SalvarPiscada (int valorPicoDerivadaP, int valorPicoDerivadaN, int tempoEntrePicos)
{
  infoPiscada piscada;

  // monta a estrutura da piscada
  piscada.picoP = valorPicoDerivadaP;
  piscada.picoN = valorPicoDerivadaN;
  piscada.duracao = tempoEntrePicos;

  // Salva a piscada no buffer, se houver espaço
  if (g_quantPiscadasNoBuffer < MAX_PISCADAS_PARA_CALIBRAR)
  {
    g_bufferPiscadas [g_quantPiscadasNoBuffer] = piscada;
    g_quantPiscadasNoBuffer++;
  }

  g_piscadaAtual = piscada;

  g_novaPiscada = true;
}

void AcionadorPiscada_acionamentoRele(bool on_off)
{
  if (!DWELL_CICK) {
    if (on_off) {
      Mouse.press();
      g_clique = 1;
      digitalWrite(ledPin, HIGH);
    } else {
      Mouse.release();
      digitalWrite(ledPin, LOW);
      g_clique = 0;
    }
  }
}

void AcionadorPiscada_acionamentoBuzzer(bool on_off)
{

}

void AcionadorPiscada_refresh()
{
	
  if(horaDoPrint)
  {
    // Faz a correção da leitura do sensor para desconsiderar a luz ambiente
    sensorValue = abs(sensorValueOff - sensorValueOn);
    Serial.println(sensorValue);

    // M�quinas de estado que interessam:
    MaquinaMedias ();
    MaquinaBordas ();

    g_novaPiscada = false;
    horaDoPrint = false;
  }
}

void MaquinaMedias (void)
{
  // Armazena a leitura mais recente
  vetorLeituras[indexLeituras] = sensorValue;

  // Calcula a m�dia das �ltimas 50 leituras e atualiza o buffer circular de m�dias
  g_mediaJanelaAtual = CalculaMedia();

  // Calcula a derivada das �ltimas 50 m�dias armazenadas
  derivada = CalculaDerivada();

  // Atualiza o �ndice do buffer circular de leituras do sensor
  indexLeituras++;
  if (indexLeituras == NUM_LEITURAS_JANELA)
  {
    indexLeituras = 0;
  }
}
