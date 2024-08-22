#define IMU_HEADER {'$', 'I', 'M', 'U'}

typedef struct {
  char header[4] = {'$', 'I', 'M', 'U'};
  uint32_t t_sec;
  uint32_t t_usec;
  int16_t acc[3];
  int16_t rate[3];
} imuPackage;