#include <Arduino.h>
//#include <Servo.h>
#include "solP_ctr.h"

extern volatile char L_servo_current_deg = L_SERVOR_DOWN;    //***modify by wmy
extern volatile char R_servo_current_deg = R_SERVOR_DOWN;    //***modify by wmy

//Servo myservoR, myservoL;
Adafruit_TiCoServo myservoR, myservoL;    //***modify by wmy
//    Servo parameters. These have limited configurability. Compatible pins,
//    for example, are determined by the specific hardware. On Arduino Uno,
//    Diecimila and Adafruit Pro Trinket, pins 9 and 10 are supported by the
//    library. On Arduino Leonardo: pins 5, 9, 10 and 11. Adafruit Flora:
//    pins D9, D10. PJRC Teensy 2.0 (not Teensy++ or 3.X): pins 4, 9, 14, 15.
// Arduino Mega: 2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45, 46.
//    Servo position can be specified in degrees or in microseconds; library
//    can distinguish between the two. The #defines below are reasonable
//    min/max pulse durations (in microseconds) for many servos, but for
//    maximum control you'll probably need to do some calibration to find
//    the optimal range for your specific servos.

void solar_Servo_Init()
{
  Serial.println("Servo attach!");
	myservoL.attach(SERVO_PIN_L, SERVO_MIN, SERVO_MAX);
	myservoR.attach(SERVO_PIN_R, SERVO_MIN, SERVO_MAX);
	myservoL.write(L_SERVOR_DOWN);
	myservoR.write(R_SERVOR_DOWN);
  Serial.println("Servo attach finished!");
	delay(1000);
  Serial.println("Servo Actived!");
  Serial.println();
}
void set_solar_panel_up() {
		Serial.println("act up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg < L_SERVOR_UP) 
				myservoL.write( L_servo_current_deg++ );
			if( R_servo_current_deg > R_SERVOR_UP) 
				myservoR.write( R_servo_current_deg-- );
			delay(30);
		}
}

void set_solar_panel_down() {
	Serial.println("act down");
  if (sys_data.is_solar_panel_on == true) {
    sys_data.is_solar_panel_on = false;			
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg > L_SERVOR_DOWN) 
				myservoL.write( L_servo_current_deg-- );
			if( R_servo_current_deg < R_SERVOR_DOWN) 
				myservoR.write( R_servo_current_deg++ );
			delay(30);
		}
  }
}

void set_solar_panel_left_up() {
		Serial.println("act left up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg < L_SERVOR_UP )
				myservoL.write( L_servo_current_deg++ );
			else break;
			delay(30);
		}
}

void set_solar_panel_right_up() {
		Serial.println("act right up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( R_servo_current_deg > R_SERVOR_UP ) 
				myservoR.write( R_servo_current_deg-- );
			else break;
			delay(30);
		}
}



