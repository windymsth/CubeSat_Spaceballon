#include <Adafruit_GPS.h>

SoftwareSerial GPSSerial(A14, A15);//RX TX

Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

unsigned char  engga[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x00,0x00,0x01,0x00,0x00,0xFE,0x18 // 使能GGA  
};  
unsigned char  disgga[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0xFD,0x15 // 取消GGA  
};  
unsigned char  engll[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x01,0x00,0x01,0x00,0x00,0xFF,0x1D // 使能GLL  
};  
unsigned char  disgll[14]={  
0xB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0xFE,0x1A // 取消GLL  
};  
unsigned char  engsa[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x02,0x00,0x01,0x00,0x00,0x00,0x22 // 使能GSA  
};  
unsigned char  disgsa[14]={  
0xB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0xFF,0x1F // 取消GSA  
};  
unsigned char  engsv[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x03,0x00,0x01,0x00,0x00,0x01,0x27 // 使能GSV  
};  
unsigned char  disgsv[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x24 // 取消GSV  
};  
unsigned char  enrmc[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x04,0x00,0x01,0x00,0x00,0x02,0x2C // 使能RMC  
};  
unsigned char  disrmc[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x01,0x29 // 取消RMC   
};  
unsigned char  envtg[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x05,0x00,0x01,0x00,0x00,0x03,0x31 // 使能VTG  
};  
unsigned char  disvtg[14]={  
0xB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x02,0x2E // 取消VTG  
};  
unsigned char  engrs[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x06,0x00,0x01,0x00,0x00,0x04,0x36 // 使能GRS  
};  
unsigned char  disgrs[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x06,0x00,0x00,0x00,0x00,0x03,0x33 // 取消GRS  
};  
unsigned char  engst[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x07,0x00,0x01,0x00,0x00,0x05,0x3B // 使能GST 
};  
unsigned char  disgst[14]={  
0xB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x07,0x00,0x00,0x00,0x00,0x04,0x38 // 取消GST  
};  
unsigned char  enzda[14]={  
0xB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x08,0x00,0x01,0x00,0x00,0x06,0x40 // 使能ZDA 
};  
unsigned char  diszda[14]={  
0XB5,0x62,0x06,0x01,0x06,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x05,0x3D // 取消ZDA  
};  
unsigned char  setbud960[28]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
0x80,0x25,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xA2,0xB5 // 设置波特率为9600
};
unsigned char  setbud192[28]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
0x00,0x4B,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x48,0x57 // 设置波特率为19200
};
unsigned char  setbud576[28]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
0x00,0xE1,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDE,0xC9 // 设置波特率为57600
};
unsigned char  setbud115[28]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E // 设置波特率为115200
};
unsigned char  set10hz[14]={
0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12 // 设置输出频率为10Hz
};

uint32_t timer = millis();

void GPS_init(void) {
  GPSSerial.begin(9600);
  Serial.println("GPS Serial Start...\r\n");
  GPSSerial.write(disgsa, 14); //DIS GSA
  GPSSerial.write(disgsv, 14); //DIS GSV
  GPSSerial.write(disgll, 14); //DIS GLL
  GPSSerial.write(disvtg, 14); //DIS VTG
  GPSSerial.write(enrmc, 14);  // EN RMC
  GPSSerial.write(engga, 14);  // EN GGA
  GPSSerial.write(setbud960, 28);  // SET buad: 9600
  delay(1000);
  GPSSerial.begin(9600);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void GPS_Parsing(void) {
  // read data from the GPS in the 'main loop'
  GPSSerial.listen();
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
//    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    sys_data.gps_satellites = (unsigned char)GPS.satellites;
    if( GPS.fix == true && GPS.fixquality == true ) {
      sys_data.current_speed = GPS.speed;
      sys_data.gps_latitude = GPS.latitude;
      sys_data.gps_longitude = GPS.longitude;
      sys_data.gps_altitude = GPS.altitude;
    }
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//    }
  }
}



void task_saftyHandle(void) {
  
}

void task_balloon_separation(void) {
  task_saftyHandle();
  //
  if( GPS.fix == true && GPS.fixquality == true && F_takeoff == false) {
    if( F_GetAlt == false ) {
      takeoff_gps_alt = sys_data.gps_altitude;
      takeoff_brao_alt = sys_data.baro_altitude;
      RF_SERIAL.println("Get nowtime ground GPS_Altitude and Baro_Altitude !");
      RF_SERIAL.print("Ground GPS_Altitude: ");RF_SERIAL.println(takeoff_gps_alt);  
      RF_SERIAL.print("Ground Baro_Altitude: ");RF_SERIAL.println(takeoff_brao_alt);
      F_GetAlt = true;
    }
    if( ((sys_data.gps_altitude - takeoff_gps_alt) > 50 || (sys_data.baro_altitude - takeoff_brao_alt) > 50) 
        && F_takeoff == false) {
      RF_SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
      RF_SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
      RF_SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
      F_takeoff = true;
      takeoff_monment_time_sec = millis() / 1000;
    }
  }

  // 进入飞行上升阶段执行程序
  if(F_takeoff == true) {
    static volatile long nowtime = 0;
    static volatile long lasttime = 0;
    static volatile long offset_baroAlt = 0;
      nowtime = millis();
      // 飞行阶段一：飞行上升到海拔9000米高度，校准气压高度偏差值；每10ms检查一次
      if( nowtime - lasttime >= 10 && F_9000m_Baro_getOffset == false ) { 
        lasttime = nowtime;

        // 当GPS海拔高度到达9000，并且还没有补偿气压高度。同时距离释放时间超过30秒
        if( sys_data.gps_altitude >= 9000 && (nowtime - takeoff_monment_time_sec) >= 30 ) {
          offset_baroAlt = sys_data.gps_altitude - sys_data.baro_altitude;
          Safty_9000m_timer = millis();
          F_9000m_Baro_getOffset = true;
          RF_SERIAL.println("ATTENTION ! SPACEBALLON RISE TO 9000m !");
        }
      }
      
    //飞行阶段二：继续上升到海拔12000米高度，准备释放动作
    if( F_9000m_Baro_getOffset ==true ) {
      
      //  Safty_9000m_timer：9000m时启动的安全计时器
      if( (millis() - Safty_9000m_timer) > 10*1000 ) {
          RF_SERIAL.println("WARNING ! SAFTY_TIMER_A ACTION !");
          
          Action_separation();

          RF_SERIAL.println("ATTENTION ! SEPARATION ACTION !");
      }
      
      //  释放策略A：如果GPS可靠，依赖GPS高程执行释放
      if( sys_data.gps_altitude > 11000 ) {
        
        if( sys_data.gps_altitude > 12000 ) {
          
          Action_separation();
          
          RF_SERIAL.println("ATTENTION ! SEPARATION ACTION !");
        }
      }
      
      //  释放策略B：如果GPS不可靠，依赖BARO高程执行释放
      else {

        if( sys_data.baro_altitude > 12000 ) {
          
          Action_separation();
          
          RF_SERIAL.println("ATTENTION ! SEPARATION ACTION !");
        }
      }
    }// *** end ->  "if( F_9000m_Baro_getOffset ==true )"

  }// *** end ->  "if(F_takeoff == true)"
}

