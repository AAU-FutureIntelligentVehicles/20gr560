#ifndef TIM551
#define TIM551

struct scan{
  float ranges[180];
  int pipi[181];
  float range;
  float angle_min;
  float angle_max;
  float len = 14;
  float limit = 0.3;
};
struct results{
  float sum;
  float avg;
  float lrange[181];
};
#endif