/*  Auslesen der Geschwindigkeit eines Potentiometers mit einem Hochpass
    und einem Operationsverstaerker.
    Jonis Kiesbye, 15.11.13
    baunotizen.wordpress.com
*/

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Kann ausgelassen werden, wenn du nicht Megunolink benutzt

int output_opamp = A0;
int reading = 0;
int minimum = 520;
int maximum = 490;
int index = 0;
int interval = 50;

void setup()
{
	Serial.begin(115200); //Wenn der Serialmonitor Muell ausliest, passe die Baudrate an.
}

void loop()
{
	reading = analogRead(output_opamp);
	
	Serial << "{TIMEPLOT|data|analogRead|T|" << reading << "}\n"; //Sollte ausgelassen werden, wenn du nicht Megunolink benutzt
	Serial.println(reading);
	
	if (reading > maximum ) maximum = reading;
	else if ( reading < minimum ) minimum = reading;
  if (index == interval) {		
		Serial.print(minimum);
		Serial.print("  -  ");
		Serial.println(maximum);
		
		index = 0;
	}
	index++;
	delay(10); //Für hoehere Messrate, verkleinere das delay.

}