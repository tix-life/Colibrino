#include "mouseIMU.h"
extern float yaw_mahony, pitch_mahony, roll_mahony;

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axR, ayR, azR, gxR, gyR, gzR;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float GyX_offset, GyY_offset, GyZ_offset;

int g_clique = 0;

void filtraIMU() {
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
  gxrs = (float)(GyX - (GyX_offset)) / MPU6050_GYRO_GAIN * 0.01745329;  //degree to radians
  gyrs = (float)(GyY - (GyY_offset)) / MPU6050_GYRO_GAIN * 0.01745329;  //degree to radians
  gzrs = (float)(GyZ - (GyZ_offset)) / MPU6050_GYRO_GAIN * 0.01745329;  //degree to radians
  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}
enum
{
  state_gx = 0,
  state_gy,
  state_gz,
};
#define SAMPLES 100
bool IMU_calibration()
{
    static bool calibrated = false;
    static int state = 0;
    static int32_t counter = 0;
    static int32_t samples_x = 0;
    static int32_t samples_y = 0;
    static int32_t samples_z = 0;

    if(calibrated== false)
    {           
        digitalWrite(16, HIGH);
        if(counter < SAMPLES)
        {
          samples_x+=GyX;
          samples_y+=GyY;
          samples_z+=GyZ;
        }
        else
        {
            GyX_offset = samples_x/SAMPLES;
            GyY_offset = samples_y/SAMPLES;
            GyZ_offset = samples_z/SAMPLES;
            calibrated = true;
        }
        counter ++;
    }
    else
    {
      digitalWrite(16, LOW);
    }

    return calibrated;
}


float corrigeYaw(float sinal) {
  static float valorDeriv = 0, zero = 0;
  static float sinalCorrigido = 0, offset = 0;
  static float sinalAnterior = 0;
  static int s = 1, d = -1;

  valorDeriv = (sinal - zero);
  zero = sinal;

  if (valorDeriv <= -180) {
    offset += +360.0f;
  } else if (valorDeriv >= 180) {
    offset += -360.0f;
  }
  sinalCorrigido = sinal;
  sinalCorrigido += offset;

  return sinalCorrigido;
}
int8_t mouseHoriz(void) {
  static float horzZero = 0.0f;
  static float horzValue = 0.0f;  // Stores current analog output of each axis
  static float yaw_corrigido = 0.0f;
  static int amostragem = 5;
  yaw_corrigido = corrigePitch(yaw_mahony);

  if (--amostragem == 0) {
    amostragem = 5;
    horzValue = (yaw_corrigido - horzZero) * SENSIBILIDADE;
    horzZero = yaw_corrigido;
  }
  return horzValue;
}
float corrigePitch(float sinal) {
  static float valorDeriv = 0, zero = 0;
  static float sinalCorrigido = 0, offset = 0;
  static float sinalAnterior = 0;
  static int s = 1, d = -1;

  valorDeriv = (sinal - zero);
  zero = sinal;

  if (valorDeriv <= -180) {
    offset += +360.0f;
  } else if (valorDeriv >= 180) {
    offset += -360.0f;
  }
  sinalCorrigido = sinal;
  sinalCorrigido += offset;

  return sinalCorrigido;
}
int8_t mouseVert(void) {
  static float vertZero = 0.0f;
  static float vertValue = 0.0f;  // Stores current analog output of each axis
  static float pitch_corrigido = 0.0f;
  static int amostragem = 5;
  pitch_corrigido = corrigePitch(pitch_mahony);
  if (--amostragem == 0) {
    amostragem = 5;
    vertValue = (pitch_corrigido - vertZero) * SENSIBILIDADE;
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
  } else if (abs(diferenca) > 2.5 * RMARGEM) {  // Ignora roll muito grande
    if (millis() - timer1 > 5000) {             // Reseta média se ficar muito tempo com roll alto
      rmedia = roll_mahony;
      timer1 = millis();
    }
    return 0;
  } else if (millis() - timer1 > 200) {
    timer1 = millis();
    if (diferenca > 0) {
      return 1;  //+ (2 * diferenca / RMARGEM);
    } else {
      return -1;  // + (2 * diferenca / RMARGEM);
    }
  }
  return 0;
}
int tabelaGestos[6][2] = { 0, 0, 0, 0, 0, 0 };
int tabelaMOV[6][2] = { 0, 0, 0, 0, 0, 0 };
int8_t VG[4] = { 0, 0, 0 };
#define NADA 0
#define ESQUERDA 1
#define DIREITA 2
#define CIMA 3
#define BAIXO 4
#define SCROLL_ESQ 5
#define SCROLL_DIR 6
#define DESLIGA ESQUERDA
#define LIGA CIMA
#define SCROLL SCROLL_ESQ
#define TIMEOUT_MOVE 110  //Tempo entre movimentos
bool g_atividade = 0;
bool atividade = true;


enum {
  Estado_movimentov2 = 0,
  Estado_passaNaZm,
};
int8_t coletaMovimentos(float horz, float vert) {
  int8_t movimento = 0;
  static int estado = 0, tipo = 0, tipo_1 = 0, tipo_2 = 0;
  switch (estado) {
    case Estado_movimentov2:
      movimento = 0;
      tipo = 0;
      tipo_1 = 0;
      tipo_2 = 0;
      if (horz > 1.0f) {
        tipo = DIREITA;
        estado = Estado_passaNaZm;
      } else if (horz < -1.0f) {
        tipo = ESQUERDA;
        estado = Estado_passaNaZm;
      } else if (vert > 0.75f) {
        tipo = CIMA;
        estado = Estado_passaNaZm;
      } else if (vert < -0.75f) {
        tipo = BAIXO;
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

      if (tipo == DIREITA) {
        if (horz < 1.0f) {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      } else if (tipo == ESQUERDA) {
        if (horz > (-1.0f)) {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      } else if (tipo == CIMA) {
        if (vert < 0.75f) {
          estado = Estado_movimentov2;
          movimento = tipo;
        }
      } else if (tipo == BAIXO) {
        if (vert > (-0.75f)) {
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

int8_t regrideIndice(int8_t indice, int8_t casas) {
  int8_t resultado = 0;
  resultado = indice;
  for (int i = 0; i < casas; i++) {
    if (resultado == 0) {
      resultado = 5;
    } else {
      resultado--;
    }
  }

  return resultado;
}
int8_t progrideIndice(int8_t indice) {
  int8_t resultado = 0;
  resultado = indice;

  if (resultado == 5) {
    resultado = 0;
  } else {
    resultado++;
  }

  return resultado;
}

int calculaTempo(int valorIntervalo) {
  int tempoCerto = 0;
  tempoCerto = (int)(valorIntervalo);
  return tempoCerto;
}
int8_t analisaVetor() {
  int8_t resultado = INVALIDO;
  if (VG[0] == ESQUERDA && VG[1] == DIREITA && VG[2] == ESQUERDA && VG[3] == DIREITA) {
    resultado = NAO;
  } else if (VG[0] == DIREITA && VG[1] == ESQUERDA && VG[2] == DIREITA && VG[3] == ESQUERDA) {
    resultado = NAO;
  } else if (VG[0] == CIMA && VG[1] == BAIXO && VG[2] == CIMA) {
    resultado = SIM;
  } else if (VG[0] == BAIXO && VG[1] == CIMA && VG[2] == BAIXO) {
    resultado = SIM;
  } else {
    resultado = INVALIDO;
  }
  return resultado;
}
enum {
  Estado_novoGesto = 0,
  Estado_reduzIndice,
  Estado_calculaIntervalo_1,
  Estado_calculaIntervalo_2,
  Estado_calculaIntervalo_3,
  Estado_calculaIntervalo_Total,
  Estado_analisaVetor,
};
int8_t moveBackwards(int8_t indice, int8_t clique) {
  /*********************************************************************
   * Requisitos:
   * 1.Gesto diferente de 0 //feito
   * 2.Intervalo 1 e dois dentro do timeout //feito //
   * 3.Intervalo total dentro do timeout //
   */
  int8_t estado = 0;
  int8_t copiaIndice = 0;  //, i = 0;
  int8_t gestos = 0;
  unsigned long intervalo_1 = 0, intervalo_2 = 0, intervalo_3 = 0, intervalo_total = 0;
  bool emProcesso = true;
  int8_t resultadoVetor = 0;
  int8_t tipoGesto = INVALIDO;

  do {
    if (clique && g_atividade == true) {
      emProcesso = false;
      VG[0] = 0;
      VG[1] = 0;
      VG[2] = 0;
      tabelaGestos[0][1] = 0;
      tabelaGestos[1][1] = 0;
      tabelaGestos[2][1] = 0;
      tabelaGestos[3][1] = 0;
      tabelaGestos[4][1] = 0;
      tabelaGestos[5][1] = 0;
    }
    switch (estado) {
      case Estado_novoGesto:
        VG[0] = 0;
        VG[1] = 0;
        VG[2] = 0;
        copiaIndice = indice;
        estado = Estado_reduzIndice;
        break;
      case Estado_reduzIndice:

        indice = regrideIndice(indice, 1);
        gestos++;
        //Se contem zero na posição anterior - sai
        if (tabelaGestos[indice][1] == 0) {
          if (gestos < 5)
            emProcesso = false;
        }

        if (gestos == 3) {
          estado = Estado_calculaIntervalo_1;
        } else if (gestos == 4) {
          estado = Estado_calculaIntervalo_2;
        } else if (gestos == 5) {
          estado = Estado_calculaIntervalo_3;
        }

        break;
      case Estado_calculaIntervalo_1:

        intervalo_1 = tabelaGestos[regrideIndice(copiaIndice, 1)][0] - tabelaGestos[regrideIndice(copiaIndice, 2)][0];
        VG[0] = tabelaGestos[regrideIndice(copiaIndice, 1)][1];
        VG[1] = tabelaGestos[regrideIndice(copiaIndice, 2)][1];


        if (intervalo_1 > calculaTempo(700)) {
          emProcesso = false;
        } else
          estado = Estado_reduzIndice;
        break;
      case Estado_calculaIntervalo_2:
        VG[2] = tabelaGestos[regrideIndice(copiaIndice, 3)][1];

        intervalo_2 = tabelaGestos[regrideIndice(copiaIndice, 2)][0] - tabelaGestos[progrideIndice(indice)][0];
        if (intervalo_2 > calculaTempo(300))
          emProcesso = false;
        else {
          resultadoVetor = analisaVetor();
          switch (resultadoVetor) {
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
        VG[3] = tabelaGestos[regrideIndice(copiaIndice, 4)][1];

        intervalo_3 = tabelaGestos[regrideIndice(copiaIndice, 3)][0] - tabelaGestos[progrideIndice(indice)][0];
        if (intervalo_3 > calculaTempo(300))
          emProcesso = false;
        else {
          resultadoVetor = analisaVetor();
          switch (resultadoVetor) {
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

        intervalo_total = tabelaGestos[regrideIndice((gestos - 1), 1)][0] - tabelaGestos[regrideIndice(copiaIndice, 3)][0];
        if (intervalo_total > calculaTempo(1500)) {
          tipoGesto = INVALIDO;
        } else {
          tipoGesto = analisaVetor();
        }
        emProcesso = false;
        break;
      default:
        break;
    }


  } while (emProcesso);

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

int8_t maquinaGestos_v2(float horz, float vert, int8_t clique)
{
  static int8_t indiceG = 0;
  static int8_t encontraPadrao = 0;
  static unsigned long contadorGestosM = 0;
  int8_t movimento = 0;
  movimento = coletaMovimentos(horz, vert);
  contadorGestosM = millis();
  //Prenche tabela
  if (movimento != 0) {
    if (indiceG > 5) {
      indiceG = 0;
    }
    tabelaGestos[indiceG][0] = contadorGestosM;  //Tempo
    tabelaGestos[indiceG][1] = movimento;


    //Analise vetor
    indiceG++;
    encontraPadrao = moveBackwards(indiceG, clique);
  }
}
enum {
  Estado_inicializa = 0,
  Estado_dorme,
  Estado_AcordaDes,
};
bool interpretaGestos(int gestos) {
  static int8_t estado = 0;
  static bool saida = true;

  switch (estado) {
    case Estado_inicializa:
      if (gestos == NAO) {
        estado = Estado_dorme;
      }
      break;
    case Estado_dorme:
      saida = false;
      if (gestos == SIM) {
        estado = Estado_AcordaDes;
      }
      break;
    case Estado_AcordaDes:
      saida = true;
      if (gestos == NAO) {
        estado = Estado_dorme;
      }
      break;
    default:
      break;
  }

  return saida;
}


float derivaYaw(float sinal) {
  static float valorDeriv = 0;
  static float sinalCorrigido = 0, zero = 0;
  //static float zero = 0;
  sinalCorrigido = corrigeYaw(sinal);
  valorDeriv = (-1) * (sinalCorrigido - zero);
  zero = sinalCorrigido;
  return valorDeriv;
}
float derivaPitch(float sinal) {
  static float valorDeriv = 0;
  static float sinalCorrigido = 0, zero = 0;
  //static float zero = 0;
  sinalCorrigido = corrigePitch(sinal);
  valorDeriv = (sinalCorrigido - zero);
  zero = sinalCorrigido;
  return valorDeriv;
}

