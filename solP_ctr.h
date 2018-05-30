#ifndef	__SOLP_CTR_H_
#define	__SOLP_CTR_H_
#include <Adafruit_TiCoServo.h>         //***modify by wmy
#include "sensor.h"

//***** User define ******//
//#define SERVO_PIN_R    8  //R 8
//#define SERVO_PIN_L    7  //L 7

#define SERVO_PIN_R     8
#define SERVO_PIN_L     7
#define SERVO_MIN  500 // 0.5 ms pulse
#define SERVO_MAX 2500 // 2.5 ms pulse

#define L_SERVOR_DOWN   50  //L
#define L_SERVOR_UP    110  //L
#define R_SERVOR_DOWN  120  //R
#define R_SERVOR_UP     60  //R

extern Adafruit_TiCoServo myservoR, myservoL;

//***** User function ******//
void solar_Servo_Init(void);
void action_separation(void);
void set_solar_panel_up(void);
void set_solar_panel_down(void);
void set_solar_panel_left_up(void);
void set_solar_panel_right_up(void);

#endif

