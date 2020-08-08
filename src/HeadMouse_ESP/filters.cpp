
#include "filters.h"

float filtro_2PB4Hz(float sinal,  filters filter)
{  
  float filtered;
  static float xv[NZEROS+1], yv[NPOLES+1];

  filter.xv[0] = filter.xv[1]; filter.xv[1] = filter.xv[2]; 
    filter.xv[2] = sinal / GAIN;
    filter.yv[0] = filter.yv[1]; filter.yv[1] = filter.yv[2];
    filter.yv[2] =   (filter.xv[0] + filter.xv[2]) + 2 * filter.xv[1]
                 + ( -0.2039806062 * filter.yv[0]) + (  0.4269374068 * filter.yv[1]);
        filtered = filter.yv[2];//-->Lowpass filter]
  return filtered;
}
