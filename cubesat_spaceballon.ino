#include <Arduino.h>
#include "type.h"
#include "sensor.h"

//***** 2018-05-29
//#define PROCESSING_SERIAL_DEBUG   true
//#define PROCESSING_SERIAL_DEBUG   flase
/*-------Ver Config--------------*/
volatile uint32_t timeSlice = 0;
volatile uint32_t nowTimes = 0;
void setup()
{
  Serial.begin(250000);
  Serial.println("Serial Debug Port Init OK!");
  Wire.begin();
  Serial.println("IIC Bus Enable...");
	solar_Servo_Init();
  setup_watchdog(2);// 32ms 
  GPS_init();
  Sensor_Init();
	SHT31_Init();
	Compass_hmc5883l_Init();
  BMP180_Init();
  UHF_Init(9600);
  SDlogger_init();
  Serial.println("Periheral Init finished!");Serial.println();
  timeSlice = millis();
}

volatile bool F_takeoff = false;
volatile bool F_GetAlt = false;
volatile bool F_9000m_Baro_getOffset = false;
volatile float takeoff_gps_alt = 0.0;
volatile float takeoff_brao_alt = 0.0;
volatile long takeoff_monment_time_sec = 0;

void task_balloon_separation(void) {
  //
  if( GPS.fix == true && GPS.fixquality == true && F_takeoff == false) {
    if( F_GetAlt == false ) {
      takeoff_gps_alt = sys_data.gps_altitude;
      takeoff_brao_alt = sys_data.baro_altitude;
      RF.SERIAL.println("Get nowtime ground GPS_Altitude and Baro_Altitude !");
      RF.SERIAL.print("Ground GPS_Altitude: ");RF.SERIAL.println(takeoff_gps_alt);  
      RF.SERIAL.print("Ground Baro_Altitude: ");RF.SERIAL.println(takeoff_brao_alt);
      F_GetAlt = true;
    }
    if( ((sys_data.gps_altitude - takeoff_gps_alt) > 50 || (sys_data.baro_altitude - takeoff_brao_alt) > 50) 
        && F_takeoff == false) {
      RF.SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
      RF.SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
      RF.SERIAL.println("ATTENTION ! SPACEBALLON TAKEOFF !");
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
        }
      }
      
    //飞行阶段二：继续上升到海拔12000米高度，准备释放动作
    if( F_9000m_Baro_getOffset ==true ) {
      
      //  Safty_9000m_timer：9000m时启动的安全计时器
      if( (millis() - Safty_9000m_timer) > 10*1000 ) {
          action_separation();
      }
      
      //  释放策略A：如果GPS可靠，依赖GPS高程执行释放
      if( sys_data.gps_altitude > 11000 ) {
        
        if( sys_data.gps_altitude > 12000 ) {
          action_separation();
        }
      }
      
      //  释放策略B：如果GPS不可靠，依赖BARO高程执行释放
      else {

        if( sys_data.baro_altitude > 12000 ) {
          action_separation();
        }
      }
    }// *** end ->  "if( F_9000m_Baro_getOffset ==true )"

  }// *** end ->  "if(F_takeoff == true)"
}

uint8_t sliceType = pm2_5Type;
void loop()
{
  static uint8_t task_comm = 0;
  static uint8_t task_SHT31 = 0;
  static uint8_t task_getAccel = 0;
  static uint8_t task_Bmp180 = 0;
  static uint8_t task_GPS = 0;
  static uint8_t task_G5 = 0;

  // 探空气球分离任务执行程序
  task_balloon_separation();
  
  nowTimes = millis();
  if ( nowTimes - timeSlice >= 10 ) {
	  #if PROCESSING_SERIAL_DEBUG
  			Serial.print("ST: ");
  			Serial.println( nowTimes - timeSlice, DEC); 
    #endif 
       
    timeSlice = nowTimes; 
    if( ++task_comm >= 100) { task_comm = 0;
      #if PROCESSING_SERIAL_DEBUG
      Serial.print("RF->");    
      #endif
      
      RF_data_update();
      #if PROCESSING_SERIAL_DEBUG   
      Serial.print("OUT:");   
      #endif
      }		      
//--------------------------------------------------------------------------		
      switch (sliceType) {  // have 5 tasks, one loop neads 50ms;
				case pm2_5Type: 
				  if ( ++task_G5 >= 20) { task_G5 = 0; 
				  }
					break;
					
				case sht31Type: 
					if ( ++task_SHT31 >= 10) {  task_SHT31 = 0;
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("SHT->");    
            #endif
            
						SHT31_Handle();
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("OUT:");    
            #endif
					}
					break;
					
				case mpuType:   
					if ( ++task_getAccel >= 1) {  task_getAccel = 0;
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("IMU->");    
            #endif
            
					  getAccel_Data();
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("OUT:");    
            #endif
					  }
					break;
					
        case bmp180Type:
					if ( ++task_Bmp180 >= 10) { task_Bmp180 = 0;
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("BARO->");   
            #endif
            
					  Bmp180_Handle();
            #if PROCESSING_SERIAL_DEBUG   
            Serial.print("OUT:");   
            #endif
					  }
          break;
					
				case gpsType:   
					if ( ++task_GPS >= 40) {  task_GPS = 0;
            SDlogger_handle();
						}
					break;
			} //>>end switch...
			
    if (++ sliceType > gpsType) sliceType = pm2_5Type;
    comm_Handle();
  }// end: if ( millis() - timeSlice >= 10 )
//--------------------------------------------------------------------------  
  GPS_Parsing();
}// end: loop()



