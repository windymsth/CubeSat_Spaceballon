#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

ISR(WDT_vect) {  
    static unsigned int heartCnt = 0;
    Serial.print("Heart count: ");
    Serial.println(++heartCnt, DEC);
}

void OFF_ACD_ADC(void) {
  ACSR |=_BV(ACD);  // OFF ACD
  ADCSRA = 0; //OFF ADC
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(char psl) {
  byte bb; 
  if (psl > 9 ) psl=9;
  bb=psl & 7;
  if (psl > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
   
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

void Sleep_avr()//睡眠模式
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN  ); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
}

