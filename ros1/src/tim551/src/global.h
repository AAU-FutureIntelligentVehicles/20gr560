#ifndef TIM551
#define TIM551

struct scan{
  float ranges[180];
  float len = 14;
  float limit = 1;
};
struct results{
  float sum;
  float avg;
};
#endif
