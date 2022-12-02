#include "Mouse.h"
#include "blink.h"

#define SENSIBILIDADE 30
#define INVALIDO 0
#define SIM 1
#define NAO 2
#define MPU6050_ACC_GAIN 16384.0
#define MPU6050_GYRO_GAIN 131.072
void filtraIMU();
bool IMU_calibration();
float corrigeYaw(float sinal) ;
int8_t mouseHoriz(void) ;
float corrigePitch(float sinal) ;
int8_t mouseVert(void) ;
int scrollDetector() ;
int8_t coletaMovimentos(float horz, float vert) ;
int8_t regrideIndice(int8_t indice, int8_t casas) ;
int8_t progrideIndice(int8_t indice) ;
int calculaTempo(int valorIntervalo) ;
int8_t analisaVetor() ;
int8_t moveBackwards(int8_t indice, int8_t clique) ;
int8_t maquinaGestos_v2(float horz, float vert, int8_t clique) ;
bool interpretaGestos(int gestos);
float derivaYaw(float sinal);
float derivaPitch(float sinal) ;
