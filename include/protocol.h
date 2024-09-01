#define IMU_HEADER {'$', 'I', 'M', 'U'}
#define GNSS_HEADER {'$', 'G', 'N', 'S', 'S'}
#define STATUS_HEADER {'$', 'S', 'T'}
#define CMD_HEADER  {'$', 'C', 'M', 'D'}


#pragma pack(1)
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
  char header[3] = {'$', 'S', 'T'};
  uint32_t ip;
  uint32_t t_sec;
  uint32_t t_usec;
  uint32_t age; // Local time (arduino clock)
  int32_t sync_offset;
  uint16_t imu_id;
  uint8_t imu_rate; 
  float imu_temp;
  uint32_t ptp_interval;
} arduinoStatus;

typedef struct {
  char header[4] = CMD_HEADER;
  bool reset;
  uint32_t ntp_interval;
  bool ptp_active;
  uint32_t ptp_interval;
  bool imu_active;
  uint8_t imu_sr; 
  bool gnss_active;
  uint8_t gnss_sr;
} arduinoCommand;