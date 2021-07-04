#include <Mouse.h>
//Carrega a biblioteca Wire
#include <Wire.h>
// #include "filters.h"
#include "MahonyAHRS.h"
#include"mpu6050.h"


//---------------------------------------------------------------------------------------------------
//Definitions
//Mouse
#define SENSIBILIDADE  30;
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

boolean g_novaPiscada = false;

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
uint8_t mouseHoriz(void)
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
uint8_t mouseVert(void)
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
void setup() 
{
  Wire.begin();
  Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  MPU6050_Init();
  // pinMode(ACIONADOR,INPUT);
  eyeBlinkSetup();
  
}

void loop() 
{
  static int printDivider = 10;
  static int estado = 0;
  uint8_t xchg = 0,ychg = 0;
  bool estadoAcionador = false;
  static int contador = 0;

 eyeBlinkRefresh();

  mpu6050_GetData();
  filtraIMU();
  MahonyAHRSupdateIMU( gxrs,  gyrs,  gzrs , axg,  ayg,  azg);
  getRollPitchYaw_mahony();
  xchg = mouseHoriz();
  ychg = mouseVert();

  Mouse.move(xchg, ychg, 0);                                      // move mouse on x axis
  
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

#define MIN_QUANT_LEITURAS_ASCENDENTES  20
#define MIN_QUANT_LEITURAS_DESCENDENTES  10

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
#define LIMIAR_BORDA_DESCIDA  10

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
int g_quantPiscadasNoBuffer = 0;
int  g_toleranciaPicoP;
int  g_toleranciaPicoN;
int  g_toleranciaDuracao;
boolean g_calibrado = false;
//boolean g_novaPiscada = false;

// CABEÇALHOS DAS FUNÇÕES
void MaquinaBordas (void);
void MaquinaPiscadas(void);
void MaquinaAcao(void);
unsigned long CalculaMedia (void);
long CalculaDerivada (void);
void SalvarPiscada (int valorPicoDerivadaP, int valorPicoDerivadaN, int tempoEntrePicos);
void CalibrarPiscada (void);


void eyeBlinkSetup() {
  Serial.begin(115200);      // Debug constructor
  pinMode(13, OUTPUT);     // Visualizatio constructor
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
  switch (fase)
  {
    case 1:
      {
        digitalWrite(pino_sample, HIGH);
        //sensorValue = analogRead(sensorPin);
        //digitalWrite(pino_sample, LOW);
        //Serial.println(sensorValue);

        horaDoPrint = true;

        // um milissegundo a mais
        tickMs++;
      }
      break;

    case 2:
      {
        // o LED já ligou e a saída do transistor já estabilizou. Hora de medir a saída e desligar o Led.
        sensorValue = analogRead(sensorPin);
        digitalWrite(pino_sample, LOW);
      }
      break;

    default:
      {
        // Faz nada. Só gasta o tempo da fase.
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

  MaquinaBordas ();
  MaquinaPiscadas();
  MaquinaAcao();

}

enum
{
  ESTADO_INICIALIZACAO = 0,
  ESTADO_DETECTANDO_BORDA,
  ESTADO_DETECTANDO_PICOS,
  ESTADO_AGUARDANDO_PICO_POSITIVO,
  ESTADO_AGUARDANDO_PICO_NEGATIVO,
  EVENTO_DETECTADO,


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
  static int tempoPicoP;
  static int tempoPicoN;
  static int etapaDetectEvento;
  static int numCiclosAguardandoPico;
  static boolean led;

  int tempoEntrePicos;

  // Vamos entrar aqui uma vez a cada leitura do sensor (1 ms)
  if (horaDoPrint)
  {
    horaDoPrint = false;


    // Armazena a leitura mais recente
    vetorLeituras[indexLeituras] = sensorValue;

    // Calcula a média das últimas 50 leituras e atualiza o buffer circular de médias
    mediaJanelaAtual = CalculaMedia();

    // Calcula a derivada das últimas 50 médias armazenadas
    derivada = CalculaDerivada();

    // Atualiza o índice do buffer circular de leituras do sensor
    indexLeituras++;
    if (indexLeituras == NUM_LEITURAS_JANELA)
    {
      indexLeituras = 0;
    }

    // Imprime em formato CSV
    //Serial.print(sensorValue);
    //Serial.print(";");
    //Serial.print(mediaJanelaAtual);
    //Serial.print(";");
    //Serial.println(derivada);

#define DERIVADA_LIMIAR_BORDA_SUBIDA  3
#define DERIVADA_LIMIAR_BORDA_DESCIDA  -3
#define NUM_CICLOS_LIBERAR_PISCADA 150
#define DELTA_MINIMO_PARA_CONFIRMAR_PISCADA 10
#define MAX_CICLOS_AGUARDANDO_PICO  500 // máximo 125 ms aguardando pico

    // máquina de estados para interpretação de piscadas
    switch (estado)
    {
      case ESTADO_INICIALIZACAO:
        {
          led = false;
          digitalWrite(ledPin, LOW);
          numCiclosAguardandoPico = 0;
          etapaDetectEvento = 0;

          // Deixa correr uma quantidade mínima de ciclos iniciais para que as médias estabilizem
          if (ciclosIniciais <= NUM_CICLOS_INICIAIS)
          {
            ciclosIniciais++;
            Serial.println(mediaJanelaAtual);
          }
          else
          {
            // inicialização concluída!
            estado = ESTADO_DETECTANDO_BORDA;
          }
        }
        break;

      case ESTADO_DETECTANDO_BORDA:
        {
          if ((derivada < DERIVADA_LIMIAR_BORDA_DESCIDA) || (derivada > DERIVADA_LIMIAR_BORDA_SUBIDA))
          {
            estado = ESTADO_DETECTANDO_PICOS;
            numCiclosAguardandoPico = 0;
            valorPicoDerivadaN = 0;
            valorPicoDerivadaP = 0;

            digitalWrite(pino_buzzer, HIGH);
          }
        }
        break;

      case ESTADO_DETECTANDO_PICOS:
        {
          // Vamos analisar as derivadas durante uma janela de tempo
          if (numCiclosAguardandoPico < MAX_CICLOS_AGUARDANDO_PICO)
          {
            numCiclosAguardandoPico++;

            // Vamos armazenar os valores de pico positivo e negativo durante a janela
            if (derivada >= 0)
            {
              // Armazena o valor do pico e o momento em que ele ocorreu dentro da janela
              if (derivada > valorPicoDerivadaP)
              {
                valorPicoDerivadaP = derivada;
                tempoPicoP = numCiclosAguardandoPico;
              }
            }
            else
            {
              // Armazena o valor do pico e o momento em que ele ocorreu dentro da janela
              if (derivada < valorPicoDerivadaN)
              {
                valorPicoDerivadaN = derivada;
                tempoPicoN = numCiclosAguardandoPico;
              }
            }

            //Serial.println(derivada);
          }
          else
          {
            // Calcula a duração entre os picos
            tempoEntrePicos = tempoPicoP - tempoPicoN;
            if (tempoEntrePicos < 0)
            {
              tempoEntrePicos = tempoPicoN - tempoPicoP;
            }

            estado = ESTADO_DETECTANDO_BORDA;
            digitalWrite(pino_buzzer, LOW);

            SalvarPiscada (valorPicoDerivadaP, valorPicoDerivadaN, tempoEntrePicos);

            /*
              //Print
              Serial.print(valorPicoDerivadaP);
              Serial.print(";");
              Serial.print(valorPicoDerivadaN);
              Serial.print(";");
              Serial.println(tempoEntrePicos);
            */

          }
        }
        break;
    }
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

void MaquinaAcao(void)
{
  static int tempoRele;

  switch (estadoMaquinaAcao)
  {
    case ESTADO_MAQUINA_ACAO_INICIAL:
      {
        if (g_novaPiscada)
        {
               
          Serial.print ("PISCADA ATUAL: ");
          Serial.print(g_piscadaAtual.picoP);
          Serial.print(";");
          Serial.print(g_piscadaAtual.picoN);
          Serial.print(";");
          Serial.println(g_piscadaAtual.duracao);

          Serial.print ("menor piscada: ");
          Serial.print(g_menorPiscada.picoP);
          Serial.print ("; ");
          Serial.print(g_menorPiscada.picoN);
          Serial.print ("; ");
          Serial.println(g_menorPiscada.duracao);

          Serial.print ("maior piscada: ");
          Serial.print(g_maiorPiscada.picoP);
          Serial.print ("; ");
          Serial.print(g_maiorPiscada.picoN);
          Serial.print ("; ");
          Serial.println(g_maiorPiscada.duracao);
          Mouse.click();

        }


        if ((g_calibrado) && (g_novaPiscada))
        {
                
          // compara a piscada atual com os parâmetros salvos:

          // Se o pico positivo da piscada atual está dentro da tolerância...
          if ((g_piscadaAtual.picoP >= g_menorPiscada.picoP) && (g_piscadaAtual.picoP <= g_maiorPiscada.picoP))
          {
            // ...e se o pico negativo da piscada atual está dentro da tolerância...
            if ((g_piscadaAtual.picoN <= g_menorPiscada.picoN) && (g_piscadaAtual.picoN >= g_maiorPiscada.picoN))
            {
              // ...e se a duração da piscada atual está dentro da tolerância...
              if ((g_piscadaAtual.duracao >= g_menorPiscada.duracao) && (g_piscadaAtual.duracao <= g_maiorPiscada.duracao))
              {
                // PISCADA ACEITA!
                tempoRele = tickMs;
         
                Serial.println("==== PISCADA ACEITA ====");

                estadoMaquinaAcao = ESTADO_MAQUINA_ACAO_DESLIGAR_RELE;
              }
            }
          }
        }

        g_novaPiscada = false;
      }
      break;

    case ESTADO_MAQUINA_ACAO_DESLIGAR_RELE:
      {
        /*
        #define TEMPO_MINIMO_RELE_MS  100
     
        if ((tickMs - tempoRele) >= TEMPO_MINIMO_RELE_MS)
        {        
          estadoMaquinaAcao = ESTADO_MAQUINA_ACAO_INICIAL;
        }
        */

        estadoMaquinaAcao = ESTADO_MAQUINA_ACAO_INICIAL;
      }
      break;
  }
}

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

void CalibrarPiscada (void)
{
  int i;
  infoPiscada piscadaMedia;

  // Inicializa as variáveis
  int menorPicoP = 1000;
  int maiorPicoP = 0;
  int somaPicoP = 0;

  int menorPicoN = -1000;
  int maiorPicoN = 0;
  int somaPicoN = 0;

  int menorDuracao = 1000;
  int maiorDuracao = 0;
  int somaDuracao = 0;


  // Calcula o maior e o menor Pico Positivo dentre as piscadas salvas
  for (i = 0; i < MAX_PISCADAS_PARA_CALIBRAR; i++)
  {
    if (g_bufferPiscadas[i].picoP > maiorPicoP)
    {
      maiorPicoP = g_bufferPiscadas[i].picoP;
    }

    if (g_bufferPiscadas[i].picoP < menorPicoP)
    {
      menorPicoP = g_bufferPiscadas[i].picoP;
    }

    // vai somando para calcular a média
    somaPicoP += g_bufferPiscadas[i].picoP;
  }
  /*
    Serial.println ("picoP:");
    Serial.println(maiorPicoP);
    Serial.println(menorPicoP);
  */


  // Calcula o maior e o menor Pico Negativo dentre as piscadas salvas
  // (Fazendo as contas com números positivos pra facilitar)
  for (i = 0; i < MAX_PISCADAS_PARA_CALIBRAR; i++)
  {
    if (g_bufferPiscadas[i].picoN < maiorPicoN)
    {
      maiorPicoN = g_bufferPiscadas[i].picoN;
    }

    if (g_bufferPiscadas[i].picoN > menorPicoN)
    {
      menorPicoN = g_bufferPiscadas[i].picoN;
    }

    // vai somando para calcular a média
    somaPicoN += g_bufferPiscadas[i].picoN;
  }
  /*
    Serial.println ("picoN:");
    Serial.println(maiorPicoN);
    Serial.println(menorPicoN);
  */


  // Calcula a maior e a menor duração entre as piscadas salvas
  for (i = 0; i < MAX_PISCADAS_PARA_CALIBRAR; i++)
  {
    if (g_bufferPiscadas[i].duracao > maiorDuracao)
    {
      maiorDuracao = g_bufferPiscadas[i].duracao;
    }

    if (g_bufferPiscadas[i].duracao < menorDuracao)
    {
      menorDuracao = g_bufferPiscadas[i].duracao;
    }

    // vai somando para calcular a média
    somaDuracao += g_bufferPiscadas[i].duracao;
  }
  /*
    Serial.println ("duracao");
    Serial.println(maiorDuracao);
    Serial.println(menorDuracao);
  */

  // Calcula as médias
  piscadaMedia.picoP = somaPicoP / MAX_PISCADAS_PARA_CALIBRAR;
  piscadaMedia.picoN = somaPicoN / MAX_PISCADAS_PARA_CALIBRAR;
  piscadaMedia.duracao = somaDuracao / MAX_PISCADAS_PARA_CALIBRAR;

  // Salva a piscada média que será considerada a partir de agora
  g_piscadaMediaPadrao = piscadaMedia;

  // Calcula as tolerâncias
  g_toleranciaPicoP = (maiorPicoP - menorPicoP) / 2;
  g_toleranciaPicoN = (maiorPicoN - menorPicoN) / 2;
  g_toleranciaDuracao = (maiorDuracao - menorDuracao) / 2;

  // Calcula a menor piscada tolerável
  g_menorPiscada.picoP = menorPicoP - g_toleranciaPicoP;
  g_menorPiscada.picoN = menorPicoN - g_toleranciaPicoN;
  g_menorPiscada.duracao = menorDuracao - g_toleranciaDuracao;

  // Calcula a maior piscada tolerável
  g_maiorPiscada.picoP = maiorPicoP + g_toleranciaPicoP;
  g_maiorPiscada.picoN = maiorPicoN + g_toleranciaPicoN;
  g_maiorPiscada.duracao = maiorDuracao + g_toleranciaDuracao;

  Serial.print ("menor piscada: ");
  Serial.print(g_menorPiscada.picoP);
  Serial.print ("; ");
  Serial.print(g_menorPiscada.picoN);
  Serial.print ("; ");
  Serial.println(g_menorPiscada.duracao);

  Serial.print ("maior piscada: ");
  Serial.print(g_maiorPiscada.picoP);
  Serial.print ("; ");
  Serial.print(g_maiorPiscada.picoN);
  Serial.print ("; ");
  Serial.println(g_maiorPiscada.duracao);

  // informa que agora temos um valor calibrado
  g_calibrado = true;

}
