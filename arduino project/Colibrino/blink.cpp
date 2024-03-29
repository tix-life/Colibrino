#include "blink.h"

// Duração típica de uma piscada.
#define EYES_BLINK_DURATION 30  // ms

extern int g_clique;
int sensorPin = 0;  // IR Sensor
int ledPin = 16;    // Visualization
int relePin = 9;    // Pino do rele
int lastLevel = 0;  // Previous IR level
int sensorLastRefValue = 0;
int lastChange = 0;       // Change in IR level
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

#define MIN_QUANT_LEITURAS_ASCENDENTES 40
#define MIN_QUANT_LEITURAS_DESCENDENTES 5

#define PERIODO_ENTRE_LEITURAS_MS 3
#define NUM_LEITURAS_JANELA 50
#define NUM_MEDIAS_JANELA 50

unsigned int vetorLeituras[NUM_LEITURAS_JANELA];
unsigned long vetorMedias[NUM_MEDIAS_JANELA];
unsigned long tickMs = 0;

int indexLeituras = 0;
int indexMedias = 0;
long derivada = 0;
int contLiberaPiscada = 0;


#define LIMIAR_BORDA_SUBIDA 10
#define LIMIAR_BORDA_DESCIDA 8

//visualization
int duration = 100;   // Length of visualization
float lastStart = 0;  // Last start of visualization

int fase = 1;
int estado = 0;
int estadoPiscadas = 0;
int estadoMaquinaAcao = 0;

#define NUM_CICLOS_INICIAIS 2 * NUM_LEITURAS_JANELA
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

} infoPiscada;

// VARIÁVEIS GLOBAIS
infoPiscada g_bufferPiscadas[MAX_PISCADAS_PARA_CALIBRAR];
infoPiscada g_piscadaMediaPadrao, g_maiorPiscada, g_menorPiscada, g_piscadaAtual;

volatile int16_t sensorValueOn = 0;
volatile int16_t sensorValueOff = 0;

int g_quantPiscadasNoBuffer = 0;
int g_toleranciaPicoP;
int g_toleranciaPicoN;
int g_toleranciaDuracao;

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


void dwellClick(int x, int y, int s) {
  static uint32_t timestamp = 0;
  static bool clickReady = false;

  if (!DWELL_CICK) return;

  if (x == 0 && y == 0 && s == 0) {
    if (g_clique == 0 && clickReady) {
      if (millis() - timestamp > TEMPO_DWELL_CICK_MS) {
        clickReady = false;
        if (millis() > 5000) Mouse.press();
        g_clique = 1;
        digitalWrite(ledPin, HIGH);
      }
    }
  } else {
    Mouse.release();
    digitalWrite(ledPin, LOW);
    g_clique = 0;
    timestamp = millis();
    clickReady = true;
  }

  if (g_clique && (millis() - timestamp > TEMPO_DWELL_CICK_MS + 50))  // Holds click for 50ms
  {
    Mouse.release();
    digitalWrite(ledPin, LOW);
    g_clique = 0;
  }
}

void eyeBlinkSetup() {

  delay(100);

  Serial.begin(115200);     // Debug constructor
  pinMode(ledPin, OUTPUT);  // Visualizatio constructor
  //pinMode(pino_buzzer, OUTPUT); // <--- DESCOMENTE PRO BARULHO AUMENTAR!
  pinMode(pino_sample, OUTPUT);
  pinMode(relePin, OUTPUT);
  digitalWrite(relePin, LOW);
  tempoAnterior = 0;


  Timer1.initialize(1000 / NUMERO_DE_FASES);  // Inicializa o Timer1 e configura para um período de (1000/NUMERO_DE_FASES) microssegundos
  Timer1.attachInterrupt(callback);           // Configura a função callback() como a função para ser chamada a cada interrupção do Timer1
}

void callback() {

  static int8_t fase = 1;

  switch (fase) {
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

  if (fase < NUMERO_DE_FASES) {
    fase++;
  } else {
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
enum {
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

void MaquinaBordas(void) {
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



#define DERIVADA_LIMIAR_BORDA_SUBIDA 4        //2.5
#define DERIVADA_LIMIAR_BORDA_DESCIDA -3.4    //-2.5
#define DERIVADA_LIMIAR_BORDA_SUBIDA_2 1.0    //0.4
#define DERIVADA_LIMIAR_BORDA_DESCIDA_2 -1.0  //0.4
#define DERIVADA_LIMIAR_SUPERIOR_BASELINE 0.25
#define DERIVADA_LIMIAR_INFERIOR_BASELINE -0.25

#define NUM_CICLOS_LIBERAR_PISCADA 150
#define DELTA_MINIMO_PARA_CONFIRMAR_PISCADA 10
#define MAX_CICLOS_AGUARDANDO_PICO 1
#define TEMPO_MINIMO_REPOUSO_ENTRE_BORDAS_MS 50

  // m�quina de estados para interpreta��o de piscadas
  switch (estado) {
    case ESTADO_INICIALIZACAO:
      {
        s_led = false;
        //LEDs_Acionamento(ACIONADOR_LED_RELE_STATUS,false);
        numCiclosAguardandoPico = 0;
        etapaDetectEvento = 0;

        // Deixa correr uma quantidade m�nima de ciclos iniciais para que as m�dias estabilizem
        if (ciclosIniciais <= NUM_CICLOS_INICIAIS) {
          ciclosIniciais++;
          //printf("%f\r\n",g_mediaJanelaAtual);
        } else {
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
#define TEMPO_MINIMO_REPOUSO_ANTES_DE_BORDA_MS 20  //50

        // cessar movimento
        g_movimentoAcontecendo = false;

        if ((tickMs - tempoInicioRepouso) > TEMPO_MINIMO_REPOUSO_ANTES_DE_BORDA_MS) {
#define MAX_DELTA_BASELINE 25

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
        if (g_movimentoAcontecendo == false) {
          // Desliga bipezinho super curto.
          // REMOVENDO BIPES LEGADOS AcionadorPiscada_acionamentoBuzzer (OFF);
        }

#define DELTA_BASELINE_UP 12
#define DELTA_BASELINE_DOWN 5

        if ((derivada < DERIVADA_LIMIAR_BORDA_DESCIDA) || (derivada > DERIVADA_LIMIAR_BORDA_SUBIDA)) {

          //if (g_mediaJanelaAtual > (g_valorBaseLine + DELTA_BASELINE_UP))
          if (g_mediaJanelaAtual > (g_valorBaseLine + limiarDeltaBaseline)) {
            AcionadorPiscada_acionamentoRele(true);
            AcionadorPiscada_acionamentoBuzzer(ON);

            g_movimentoAcontecendo = true;


            //printf("ON!\r\n");

            //printf("BASELINE REPOUSO: ");
            //printf("%.1f\r\n",g_valorBaseLine);

            //printf(" LIMIAR DELTA BASELINE: ");
            //printf("%.1f\r\n",limiarDeltaBaseline);

            tempoInicioPiscada = tickMs;

            estado = ESTADO_DETECTANDO_FIM_PISCADA;
          } else {
            //printf("*** MOVIMENTO INSUFICIENTE ***\r\n");
          }

          // Reinicia a contagem do prazo para reset do Baseline.
          tempoParaResetBaselineRepouso = tickMs;
        } else {
          // Se ficarmos um tempo em repouso, redefine o baseline.

#define TIMEOUT_REDEFINICAO_DE_BASELINE_REPOUSO_MS 100

          // se tivermos repouso por uma certa quantidade de tempo, redefine o baseline
          if ((tickMs - tempoParaResetBaselineRepouso) > TIMEOUT_REDEFINICAO_DE_BASELINE_REPOUSO_MS) {
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
        if (g_mediaJanelaAtual < (g_valorBaseLine + 0.7 * limiarDeltaBaseline))  // a histerese para fim da piscada � de baseline + 70% do limiar do delta para in�cio da piscada
        {
          AcionadorPiscada_acionamentoRele(false);
          AcionadorPiscada_acionamentoBuzzer(OFF);

          g_movimentoAcontecendo = false;

          //printf("OFF!\r\n");

          tempoInicioRepouso = tickMs;

          estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
        }

#define TIMEOUT_ESTABILIZACAO_PISCADA_MS 1000

        // Segunda condi��o de sa�da: se a piscada n�o estiver est�vel ap�s um tempo, for�a a sa�da
        if ((tickMs - tempoInicioPiscada) > TIMEOUT_ESTABILIZACAO_PISCADA_MS) {
          if ((derivada < DERIVADA_LIMIAR_BORDA_DESCIDA) || (derivada > DERIVADA_LIMIAR_BORDA_SUBIDA)) {
            AcionadorPiscada_acionamentoRele(false);
            AcionadorPiscada_acionamentoBuzzer(OFF);

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

        if (numCiclosAguardandoPico < MAX_CICLOS_AGUARDANDO_PICO) {
          // s� aguarda
          numCiclosAguardandoPico++;
        } else {
          //printf("%d\r\n"mediaAntesDaBorda);

          // Usa essas vari�veis para salvar o maior e o menor valor do movimento:
          if (mediaAntesDaBorda > valorPicoDerivadaP) {
            valorPicoDerivadaP = mediaAntesDaBorda;
          }
          if (mediaAntesDaBorda < valorPicoDerivadaN) {
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

#define TEMPO_MAXIMO_PISCADA_PILOTO_MS 1000

        if ((tickMs - tempoInicioBorda) > TEMPO_MAXIMO_PISCADA_PILOTO_MS) {
          // Fim do movimento: calcula o tempo que levou.
          tempoEntrePicos = tempoFimMovimento - tempoInicioMovimento;

          // Volta para o estado inicial de detec��o de repouso antes da pr�xima borda.
          g_piscadaBloqueada = true;
          tempoInicioRepouso = tickMs;
          estado = ESTADO_DETECTANDO_REPOUSO_ANTES_DE_BORDA;
        } else {
          //printf("%f\r\n",g_mediaJanelaAtual);

          // Usa essas vari�veis para salvar o maior e o menor valor do movimento:
          if (g_mediaJanelaAtual > valorPicoDerivadaP) {
            valorPicoDerivadaP = g_mediaJanelaAtual;
          }
          if (g_mediaJanelaAtual < valorPicoDerivadaN) {
            valorPicoDerivadaN = g_mediaJanelaAtual;
          }
        }
      }
      break;
  }
}



enum {
  ESTADO_PISCADAS_INICIAL = 0,
  ESTADO_PISCADAS_AGUARDAR_PISCADAS_CALIBRACAO,
};



#define TEMPO_MAXIMO_ENTRE_PISCADAS_CALIBRACAO_MS 1000

void MaquinaPiscadas(void) {
  static int quantPiscadasAnterior;
  static long tempoPiscada;
  int i;

  switch (estadoPiscadas) {
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
        if (g_quantPiscadasNoBuffer > 0) {
          if (g_quantPiscadasNoBuffer > quantPiscadasAnterior) {
            // guarda o tempo de início:
            tempoPiscada = tickMs;

            quantPiscadasAnterior = g_quantPiscadasNoBuffer;

            // acende o LED para aguardar as demais piscadas
            digitalWrite(ledPin, HIGH);

            if (g_quantPiscadasNoBuffer == MAX_PISCADAS_PARA_CALIBRAR) {
              // faz as contas para determinar se vai calibrar

              //printão:
              for (i = 0; i < MAX_PISCADAS_PARA_CALIBRAR; i++) {
                // Serial.print(g_bufferPiscadas[i].picoP);
                // Serial.print(";");
                // Serial.print(g_bufferPiscadas[i].picoN);
                // Serial.print(";");
                // Serial.println(g_bufferPiscadas[i].duracao);
              }

              CalibrarPiscada();

              estadoPiscadas = ESTADO_PISCADAS_INICIAL;
            }
          } else {
            // verifica se já expirou o tempo máximo entre duas piscadas para calibração
            if ((tickMs - tempoPiscada) > TEMPO_MAXIMO_ENTRE_PISCADAS_CALIBRACAO_MS) {
              // volta pro começo
              estadoPiscadas = ESTADO_PISCADAS_INICIAL;
            }
          }
        } else {
          // faz nada. Continua aguardando as piscadas
        }
      }
      break;
  }
}

enum {
  ESTADO_MAQUINA_ACAO_INICIAL = 0,
  ESTADO_MAQUINA_ACAO_DESLIGAR_RELE,
};



// Calcula a média das últimas 50 leituras e salva no buffer circular de médias calculadas
unsigned long CalculaMedia() {
  int i;
  unsigned long media = 0;

  for (i = 0; i < NUM_LEITURAS_JANELA; i++) {
    media += vetorLeituras[i];
  }

  media = media / NUM_LEITURAS_JANELA;

  // atualiza o buffer circular de médias
  vetorMedias[indexMedias] = media;

  //atualiza índice do buffer circular de médias
  indexMedias++;

  if (indexMedias == NUM_MEDIAS_JANELA) {
    indexMedias = 0;
  }

  return media;
}

long CalculaDerivada() {
  long derivada = 0;

  if (indexMedias > 0) {
    derivada = vetorMedias[indexMedias - 1] - vetorMedias[indexMedias];
  } else {
    derivada = vetorMedias[NUM_MEDIAS_JANELA - 1] - vetorMedias[0];
  }

  return derivada;
}


void SalvarPiscada(int valorPicoDerivadaP, int valorPicoDerivadaN, int tempoEntrePicos) {
  infoPiscada piscada;

  // monta a estrutura da piscada
  piscada.picoP = valorPicoDerivadaP;
  piscada.picoN = valorPicoDerivadaN;
  piscada.duracao = tempoEntrePicos;

  // Salva a piscada no buffer, se houver espaço
  if (g_quantPiscadasNoBuffer < MAX_PISCADAS_PARA_CALIBRAR) {
    g_bufferPiscadas[g_quantPiscadasNoBuffer] = piscada;
    g_quantPiscadasNoBuffer++;
  }

  g_piscadaAtual = piscada;

  g_novaPiscada = true;
}

void AcionadorPiscada_acionamentoRele(bool on_off) {
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

void AcionadorPiscada_acionamentoBuzzer(bool on_off) {
}

void AcionadorPiscada_refresh() {

  if (horaDoPrint) {
    // Faz a correção da leitura do sensor para desconsiderar a luz ambiente
    sensorValue = abs(sensorValueOff - sensorValueOn);
    // Serial.println(sensorValue);

    // M�quinas de estado que interessam:
    MaquinaMedias();
    MaquinaBordas();

    g_novaPiscada = false;
    horaDoPrint = false;
  }
}

void MaquinaMedias(void) {
  // Armazena a leitura mais recente
  vetorLeituras[indexLeituras] = sensorValue;

  // Calcula a m�dia das �ltimas 50 leituras e atualiza o buffer circular de m�dias
  g_mediaJanelaAtual = CalculaMedia();

  // Calcula a derivada das �ltimas 50 m�dias armazenadas
  derivada = CalculaDerivada();

  // Atualiza o �ndice do buffer circular de leituras do sensor
  indexLeituras++;
  if (indexLeituras == NUM_LEITURAS_JANELA) {
    indexLeituras = 0;
  }
}