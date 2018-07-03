#include <Arduino.h>
#include "type.h"
#include "sensor.h"

//***** 2018-05-29
//#define PROCESSING_SERIAL_DEBUG   true
//#define PROCESSING_SERIAL_DEBUG   flase
/*-------Ver Config--------------*/

uint8_t sliceType = pm2_5Type;
volatile uint32_t timeSlice = 0;
volatile uint32_t nowTimes = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Debug Port Init OK!");
  Wire.begin();
  Serial.println("IIC Bus Enable...\r\n");
	solar_Servo_Init();
  setup_watchdog(2);// 32ms 
  GPS_init();
  Sensor_Init();
	SHT31_Init();
	Compass_hmc5883l_Init();
  BMP180_Init();
  UHF_Init(115200);
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
volatile long Safty_9000m_timer = 0;

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



