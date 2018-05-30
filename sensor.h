#ifndef __SENSOR_H__
#define __SENSOR_H__

//	user include
#include "type.h"
#define RF_SERIAL Serial3

#include "SHT31.h"
#include "MPU6050.h"
#include "uHMC5883L.h"
#include "BMP180.h"
#include "solP_ctr.h"

// sys_data
typedef struct SYS_DATA{
  float heating_panel_temp = 0.0;
  float inside_temp = 0.0;
  float inside_humi = 0.0;
  float posture_x = 0.00;
  float posture_y = 0.0;
  float posture_z = 0.0;
  float accelerated = 0.0;
  float heading = 0.0;
  float gps_latitude = 0.0;
  float gps_longitude = 0.0;
  uint8_t gps_satellites = 0;
  float gps_altitude = 0.0;
  float pressure = 0.0;
  float baro_altitude = 0.0;
  float current_speed = 0.0;
  uint16_t gps_distance = 0;
  uint16_t pm25 = 0;
  
  bool is_solar_panel_on = false;
}SENSOR_Type;

enum communicate {
  COMM_SOH = 0x01,
  COMM_EOT = 0x04,
  COMM_ACK = 0x06,
  COMM_NAK = 0x15,
  COMM_CAN = 0x18,
  COMM_FAK = 0xFF,

  // command
  COMM_REQUEST_SAT_DATA = 254,
  COMM_OPEN_SOLAR_PANEL = 253,
  COMM_CLOSE_SOLAR_PANEL = 252,
  COMM_RESET_SYS = 251,
  COMM_TURN_ON_HEATER = 250,
  COMM_TURN_OFF_HEATER = 249,
  COMM_RESET = 244
};

enum Option_Index {
  // comm execute index
  opt_Turn_On_Heater = 3,
  opt_Turn_Off_Heater = 4,
  opt_Open_Solar_Panel = 5,
  opt_Close_Solar_Panel = 6,
  opt_Request_Sat_Data = 7,
  opt_Reset = 8,
  opt_default
};

extern bool sht31Flag;
extern SENSOR_Type sys_data;
extern volatile uint8_t operation_index;
extern bool is_opt_busy;

void UHF_Init(long buad);
void Sensor_Init(void);
void getAccel_Data(void);
bool RF_data_update(void);
bool Serial_data_update(void);
void comm_Handle(void);
void comm_run(HardwareSerial* port);

#endif



