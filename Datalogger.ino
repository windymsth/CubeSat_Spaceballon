/*
  SD card datalogger

 This Task is log data from ALL sensors
 to an SD card using the SD library.

 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 5 (for CubeSatV3_OBC SD: SDCARD_SS_PIN)

 created 29 May 2018
 modified 29 May 2018
 by Windymsth

 */

#include <SPI.h>
#include <SD.h>
#include <stdio.h>

const int chipSelect = 5;

File myFile;

void SDlogger_init(void) {
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.\r\n");
  Serial.println("DNL checks for datalog.txt files...");
  if (SD.exists("datalog.txt")) {
    Serial.println("datalog.txt exists.");
    // delete the file:
    Serial.println("Removing datalog.txt...");
    SD.remove("datalog.txt");
  } else {
    Serial.println("datalog.txt doesn't exist.");
  }
  // creat a new file and immediately close it:
  Serial.println("\nCreating new datalog.txt...");
  myFile = SD.open("datalog.txt", FILE_WRITE);
  myFile.close();
}

void SDlogger_handle(void) {
  char logBuf[1000];
  char heating_panel_temp_sBuf[20];
  char inside_temp_sBuf[20];
  char inside_humi_sBuf[20];
  char posture_x_sBuf[20];
  char posture_y_sBuf[20];
  char posture_z_sBuf[20];
  char heading_sBuf[20];
  char gps_latitude_sBuf[20];
  char gps_longitude_sBuf[20];
  char gps_altitude_sBuf[20];
  char baro_altitude_sBuf[20];
  char pressure_sBuf[20];
  char current_speed_sBuf[20];
  char accelerated_sBuf[20]; 
  int j = 0;
  // open the file. note that only one file can be open at a time.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dtostrf(sys_data.heating_panel_temp, 1, 1, heating_panel_temp_sBuf);
    dtostrf(sys_data.inside_temp, 1, 1, inside_temp_sBuf);
    dtostrf(sys_data.inside_humi, 1, 1, inside_humi_sBuf);
    dtostrf(sys_data.posture_x, 1, 1, posture_x_sBuf);
    dtostrf(sys_data.posture_y, 1, 1, posture_y_sBuf);
    dtostrf(sys_data.posture_z, 1, 1, posture_z_sBuf);
    dtostrf(sys_data.heading, 1, 1, heading_sBuf);
    dtostrf(sys_data.gps_latitude, 1, 4, gps_latitude_sBuf);
    dtostrf(sys_data.gps_longitude,1, 4, gps_longitude_sBuf);
    dtostrf(sys_data.gps_altitude, 1, 1, gps_altitude_sBuf);
    dtostrf(sys_data.baro_altitude, 1, 1, baro_altitude_sBuf);
    dtostrf(sys_data.pressure, 1, 3, pressure_sBuf);
    dtostrf(sys_data.current_speed, 1, 2, current_speed_sBuf);
    dtostrf(sys_data.accelerated, 1, 2, accelerated_sBuf);
    j = sprintf(logBuf, "{");
    j += sprintf(logBuf+j, "\"OUT_TP\":{\"v\":%s},", heating_panel_temp_sBuf);
    j += sprintf(logBuf+j, "\"IN_TP\":{\"v\":%s},", inside_temp_sBuf);
    j += sprintf(logBuf+j, "\"IN_HM\":{\"v\":%s},", inside_humi_sBuf);
    j += sprintf(logBuf+j, "\"ROL\":{\"v\":%s},", posture_x_sBuf);
    j += sprintf(logBuf+j, "\"PIT\":{\"v\":%s},", posture_y_sBuf);
    j += sprintf(logBuf+j, "\"YAW\":{\"v\":%s},", posture_z_sBuf);
    j += sprintf(logBuf+j, "\"HEAD\":{\"v\":%s},", heading_sBuf);
    j += sprintf(logBuf+j, "\"G_SAT\":{\"v\":%d},", sys_data.gps_satellites);
    j += sprintf(logBuf+j, "\"LAT\":{\"v\":%s},", gps_latitude_sBuf);
    j += sprintf(logBuf+j, "\"LON\":{\"v\":%s},", gps_longitude_sBuf);
    j += sprintf(logBuf+j, "\"G_ALT\":{\"v\":%s},", gps_altitude_sBuf);
    j += sprintf(logBuf+j, "\"B_ALT\":{\"v\":%s},", baro_altitude_sBuf);
    j += sprintf(logBuf+j, "\"BARO\":{\"v\":%s},", pressure_sBuf);
    j += sprintf(logBuf+j, "\"G_SPD\":{\"v\":%s},", current_speed_sBuf);
    j += sprintf(logBuf+j, "\"ACC_G\":{\"v\":%s},", accelerated_sBuf);
    j += sprintf(logBuf+j, "\"G_DIS\":{\"v\":%d},", sys_data.gps_distance);
    j += sprintf(logBuf+j, "}");
    dataFile.println(logBuf);
    dataFile.close();
    Serial.println(logBuf);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

