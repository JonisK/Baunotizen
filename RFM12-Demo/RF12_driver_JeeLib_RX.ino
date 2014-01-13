#include <JeeLib.h>

int counter;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  // node 10, group 212, 433 MHz
  rf12_initialize(10, RF12_433MHZ, 212);
}

void loop() {
  if (rf12_recvDone()){    
    if (rf12_crc == 0) {
      Serial.print("empfangen: ");
			Serial.println(counter,DEC);
    }
  }
}
