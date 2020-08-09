#ifndef FILTERS_H
#define FILTERS_H

#define NZEROS 2
#define NPOLES 2
#define GAIN   1.058546241e+03

class filters
{
  private:
    
  public:
    float xv[NZEROS+1];
    float yv[NPOLES+1];
  
};

float filtro_2PB10Hz(float sinal,  filters filter);

#endif /*FILTERS_H*/
