#include "sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <advancedSerial.h>
#include <SimpleKalmanFilter.h>

//#define RF_SERIAL Serial3

SENSOR_Type sys_data;

bool sht31Flag = false;
bool is_opt_busy = false;
volatile uint8_t operation_index = 0;
static float temp_Speed = 0;

void UHF_Init(long buad) {
  RF_SERIAL.begin(buad);
  aSerial.setPrinter(RF_SERIAL);
}

//mpu6050
MPU6050 accelgyro;

void Sensor_Init(void) {  
	accelgyro.initialize();
  Serial.println("MPU6050 boot OK!");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setRate(0x07); // 7+1 = 8KHz  Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  accelgyro.setDLPFMode(0x00); //Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  Serial.println("MPU6050 init done!");
  Serial.println();
  
  operation_index = opt_default;
  sys_data.is_solar_panel_on = false;
}

#define FILTER_N 20
float filter_buf[FILTER_N + 1];
float shiftwinFilter(float val) {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = val;
  for(i = 0; i < FILTER_N; i++) {
  filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
  filter_sum += filter_buf[i];
  }
  return (float)(filter_sum / FILTER_N);
}
/*    SimpleKalmanFilter(e_mea, e_est, q);
      e_mea: Measurement Uncertainty 
      e_est: Estimation Uncertainty 
      q: Process Noise  */
SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.01);//(2, 2, 0.01)
#define CONSTANTS_ONE_G 9.8015f    /* m/s^2    */
float Axyz[3], Gxyz[3];
long long sum_ax,sum_ay,sum_az;
int16_t offset_ax = 183;
int16_t offset_ay = -60;
int16_t offset_az = 8390;
void getAccel_Data(void) {
  int16_t ax, ay, az, gx, gy, gz;
//  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.getAcceleration(&ax, &ay, &az);
//  static int cnt = 0;
//  if(cnt > 100) { cnt = 0;
//    char strBuffer[200];
//    offset_ax = sum_ax / 100; offset_ay = sum_ay / 100; offset_az = sum_az / 100;
//    sprintf(strBuffer, "%d,%d,%d\r\n", offset_ax, offset_ay, offset_az);Serial.print(strBuffer);
//    sum_ax = sum_ay = sum_az = 0; 
//    } else {  sum_ax += ax; sum_ay += ay; sum_az += az; cnt++;  }
  //4g * ax / 32768;  -->8192
  Axyz[0] = (double) (ax - offset_ax) / 8192; 
  Axyz[1] = (double) (ay - offset_ay) / 8192;
  Axyz[2] = (double) (az - offset_az) / 8192;
//  Gxyz[0] = (double) gx * 250 / 32768;
//  Gxyz[1] = (double) gy * 250 / 32768;
//  Gxyz[2] = (double) gz * 250 / 32768;
  sys_data.posture_x = Axyz[0];
  sys_data.posture_y = Axyz[1];
  sys_data.posture_z = Axyz[2];
  
  // compass heading
  if (compass_found)  sys_data.heading = getHeading();
  
  sys_data.accelerated = sqrt( Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]);
  
//  float estimated_value = simpleKalmanFilter.updateEstimate(sys_data.accelerated);
//  sys_data.accelerated = estimated_value + CONSTANTS_ONE_G;
  sys_data.accelerated = shiftwinFilter(sys_data.accelerated) + CONSTANTS_ONE_G;
}

bool RF_data_update(void) {
  aSerial.p("{");
  aSerial.p("\"OUT_TP\":").p("{").p("\"v\":").p(sys_data.heating_panel_temp).p("}");
  aSerial.p(",");
  aSerial.p("\"IN_TP\":").p("{").p("\"v\":").p(sys_data.inside_temp).p("}");
  aSerial.p(",");
  aSerial.p("\"IN_HM\":").p("{").p("\"v\":").p(sys_data.inside_humi).p("}");
  aSerial.p(",");
  aSerial.p("\"ROL\":").p("{").p("\"v\":").p(sys_data.posture_x).p("}");
  aSerial.p(",");
  aSerial.p("\"PIT\":").p("{").p("\"v\":").p(sys_data.posture_y).p("}");
  aSerial.p(",");
  aSerial.p("\"YAW\":").p("{").p("\"v\":").p(sys_data.posture_z).p("}");
  aSerial.p(",");
  aSerial.p("\"HEAD\":").p("{").p("\"v\":").p(sys_data.heading).p("}");
  aSerial.p(",");
  aSerial.p("\"G_SAT\":").p("{").p("\"v\":").p(sys_data.gps_satellites).p("}");
  aSerial.p(",");
  aSerial.p("\"LAT\":").p("{").p("\"v\":");RF_SERIAL.print(sys_data.gps_latitude, 5);aSerial.p("}");
  aSerial.p(",");
  aSerial.p("\"LON\":").p("{").p("\"v\":");RF_SERIAL.print(sys_data.gps_longitude, 5);aSerial.p("}");
  aSerial.p(",");
  aSerial.p("\"G_ALT\":").p("{").p("\"v\":").p(sys_data.gps_altitude).p("}");
  aSerial.p(",");
  aSerial.p("\"B_ALT\":").p("{").p("\"v\":").p(sys_data.baro_altitude).p("}");
  aSerial.p(",");
  aSerial.p("\"BARO\":").p("{").p("\"v\":").p(sys_data.pressure).p("}");
  aSerial.p(",");
  aSerial.p("\"G_SPD\":").p("{").p("\"v\":").p(sys_data.current_speed).p("}");
  aSerial.p(",");
  aSerial.p("\"ACC_G\":").p("{").p("\"v\":").p(sys_data.accelerated).p("}");
  aSerial.p("}");
  aSerial.pln();
  return true;
}

void serialEvent3() {
  if (RF_SERIAL.available()) {
    comm_run(&RF_SERIAL);
  }
}

void comm_Handle(void) {
  switch (operation_index) {		
    case opt_Open_Solar_Panel:
      operation_index = opt_default;
      is_opt_busy = true;
      RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL);
      set_solar_panel_up();
      is_opt_busy = false;
      break;

    case opt_Close_Solar_Panel:
      operation_index = opt_default;
      is_opt_busy = true;
      RF_SERIAL.write(COMM_CLOSE_SOLAR_PANEL);
      set_solar_panel_down();
      is_opt_busy = false;
      break;

    case opt_Request_Sat_Data:
//      Serial.println("opt_Request_Sat_Data");
      operation_index = opt_default;
      RF_data_update();                     //根据上位机指令才更新，不是自动更新
      break;

    case opt_Reset:
      operation_index = opt_default;
      RF_SERIAL.write(COMM_RESET);
      break;
    default: break;
  }
}

void comm_run(HardwareSerial* port) {
	char data;
  if (port->available()) {
    data = port->read();
    if (is_opt_busy == true) {
      return;
    }
    switch ((uint8_t)data) {
      case COMM_SOH:
        operation_index = opt_default;
        break;
      case COMM_EOT:  // End of transmit
        break;
      case COMM_ACK:
        break;
      case COMM_NAK:
        break;
      case COMM_CAN:  // Cancle anyway
        break;
      case COMM_REQUEST_SAT_DATA:
        operation_index = opt_Request_Sat_Data;
        break;
      case COMM_OPEN_SOLAR_PANEL:
        operation_index = opt_Open_Solar_Panel;
        break;
      case COMM_CLOSE_SOLAR_PANEL:
        operation_index = opt_Close_Solar_Panel;
        break;
      case COMM_RESET:
        operation_index = opt_Reset;
        break;
      default: break;
    }
  }
}


