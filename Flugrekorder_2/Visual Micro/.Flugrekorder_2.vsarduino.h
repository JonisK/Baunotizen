/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Pro or Pro Mini w/ ATmega328 (3.3V, 8 MHz), Platform=avr, Package=arduino
*/

#define __AVR_ATmega328P__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 8000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

inline uint8_t ringAvailable();
inline uint8_t ringNext(uint8_t ht);
uint16_t Power(uint8_t base, uint8_t pow);
void shutdown();
//
//

#include "G:\Programme\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h" 
#include "G:\Programme\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include "G:\Eigene Dateien\Arduino\Flugrekorder_2\Flugrekorder_2.ino"
