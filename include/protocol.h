#define IMU_HEADER {'$', 'I', 'M', 'U'}
#define GNSS_HEADER {'$', 'G', 'N', 'S', 'S'}
#define STATUS_HEADER {'$', 'S', 'T'}


typedef struct {
  char header[4];
  uint32_t t_sec;
  uint32_t t_usec;
  int16_t acc[3];
  int16_t rate[3];
} imuPackage;


typedef struct {
  char header[5];
  uint32_t t_sec;
  uint32_t t_usec;
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
} gnssPackage;


typedef struct {
  char header[3];
  uint32_t ip;
  uint32_t t_sec;
  uint32_t t_usec;
  uint32_t age;
  uint32_t ntp_interval;
  int32_t ntp_offset;
  bool ptp_active;
  uint32_t ptp_interval;
  bool imu_active;
  uint8_t imu_sr; 
  bool gnss_active;
  uint8_t gnss_sr;
} arduinoStatus;