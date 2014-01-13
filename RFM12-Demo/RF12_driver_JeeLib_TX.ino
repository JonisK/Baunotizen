#include <JeeLib.h>

int counter = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  // node 10, group 212, 433 MHz
  rf12_initialize(10, RF12_433MHZ, 212);
}

void loop() {
  ++counter;
		
	while (!rf12_canSend()) {
		rf12_recvDone();
	}
	rf12_sendStart(0, &counter, sizeof counter);
		 
	Serial.print("gesendet: ");
	Serial.println(counter, DEC);

	delay(100);
	}
}


