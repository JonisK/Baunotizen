template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
	
#include <Wire.h>
#include <SdFat.h>
#include <SdFatUtil.h> // define FreeRam()
#define SD_CHIP_SELECT  SS  // SD chip select pin
#define LOG_INTERVAL  1000  // mills between entries
//#define ECHO_TO_SERIAL   1  // echo data to serial port if nonzero
#define WAIT_TO_START    1  // Wait for serial input in setup()

//#define DEBUG
#define DEBUG2

//Variablen fuer SD-Card, SD
// file system object
SdFat sd;

// text file for logging
ofstream logfile;

#ifdef DEBUG
// Serial print stream
ArduinoOutStream cout(Serial);
#endif

// buffer to format data - makes it eaiser to echo to Serial
char buf[100];

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

//Konstanten für Beschleunigungssensor 1, ACC1
const byte ACC1_ADDRESS = B0011000;
const byte ACC1_CTRL1 = B00100000;
const byte ACC1_CTRL4 = B00100011;
//const byte ACC1_READ = B00110001; //Adresse + Read Bit
//const byte ACC1_WRITE = B00110000; //Adresse + Write Bit

//Konstanten fuer Beschleunigungssensor 2, ACC2
const byte ACC2_ADDRESS = B0011001;
const byte ACC2_CTRL1 = 0x20;
//const byte ACC2_CTRL2 = 0x21;
//const byte ACC2_CTRL3 = 0x22;
const byte ACC2_CTRL4 = 0x23;
//const byte ACC2_CTRL5 = 0x24;
//const byte ACC2_CTRL6 = 0x25;

//Konstanten fuer ACC1 und ACC2
const byte ACC_CONT = B10000000; //Automatisches Inkrementieren der Registeradresse
const byte ACC_STATUS = 0x27;
const byte ACC_X_L = 0x28;
//const byte OUT_X_H = 0x29;
//const byte OUT_Y_L = 0x2A;
//const byte OUT_Y_H = 0x2B;
//const byte OUT_Z_L = 0x2C;
//const byte OUT_Z_H = 0x2D;

//Variablen fuer ACC1 und ACC2
byte ACC_stat = 0;
byte ACC_low = 0;
int ACC_raw[3] = {0,0,0};


//Konstanten fuer Magnetometer, MAG
const byte MAG_ADDRESS = B0011110;
const byte MAG_CRA = 0x00;
const byte MAG_CRB = 0x01;
const byte MAG_MR = 0x02;
const byte MAG_X_H = 0x03;
const byte MAG_SR = 0x09;

//Variablen fuer MAG
byte MAG_high = 0;
int MAG_raw[3] = {0,0,0};

//Konstanten fuer Temperatursensor, THM
const byte MAG_THM_H = 0x31;
const byte MAG_THM_L = 0x32;

//Variablen fuer THM
byte THM_high = 0;
unsigned int THM_raw = 0;
unsigned int THM_last = 0;

//Allgemeine Konstanten
const byte readByteCount = 6;
const byte THM_readByteCount = 2;
const byte singleByte = 1;
const byte rTrue = 1;
const byte rFalse = 1;
const byte LED = 8;

//Allgemeine Variablen
byte i=0;

//Temp

void setup() {
	#ifdef DEBUG
	Serial.begin(115200);
	#endif
	#ifdef DEBUG2
	Serial.begin(115200);
	#endif
	//LED
	pinMode(LED, OUTPUT);
	//SD Karte initialisieren
	#ifdef DEBUG
	// pstr stores strings in flash to save RAM
	cout << endl << pstr("FreeRam: ") << FreeRam() << endl;
	#endif
	// initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
	if (!sd.begin(SD_CHIP_SELECT, SPI_HALF_SPEED)) sd.initErrorHalt();
	
	// create a new file in root, the current working directory
	char name[] = "LOGGER00.CSV";
	for (uint8_t i = 0; i < 100; i++) {
		name[6] = i/10 + '0';
		name[7] = i%10 + '0';
		if (sd.exists(name)) continue;
		logfile.open(name);
		break;
	}
	if (!logfile.is_open()) error("file.open");
	
	#ifdef DEBUG
	cout << pstr("Logging to: ") << name << endl;
	cout << pstr("Type any character to stop\n\n");
	#endif
	
	// format header in buffer
	obufstream bout(buf, sizeof(buf));
	bout << pstr("millis");
	bout << pstr(": Logger wurde gestartet.");
	logfile << buf << endl;
	
	#ifdef ECHO_TO_SERIAL
	cout << buf << endl;
	#endif  // ECHO_TO_SERIAL
	
	//I2C Bus starten
	Wire.begin();
	
	//Ersten Beschleunigungssensor initialisieren
	#ifdef DEBUG
		Serial.println(F("Ersten Beschleunigungssensor initialisieren"));
	#endif
	Wire.beginTransmission(ACC1_ADDRESS);
	Wire.write(ACC1_CTRL1);
	Wire.write(B00110111);  //Normal, 400Hz, XYZ enabled
	Wire.endTransmission(rTrue);
	Wire.beginTransmission(ACC1_ADDRESS);
	Wire.write(ACC1_CTRL4);
	Wire.write(B00110000);  //Wertebereich +-24g
	Wire.endTransmission(rTrue);
	
	//Zweiten Beschleunigungssensor initialisieren
	#ifdef DEBUG
		Serial.println(F("Zweiten Beschleunigungssensor initialisieren"));
	#endif
	Wire.beginTransmission(ACC2_ADDRESS);
	Wire.write(ACC2_CTRL1);
	Wire.write(B01110111);  //Normal, 400Hz, XYZ enabled
	Wire.endTransmission(rTrue);
	Wire.beginTransmission(ACC2_ADDRESS);
	Wire.write(ACC2_CTRL4);
	Wire.write(B00111000); //Wertebereich +-16g
	Wire.endTransmission(rTrue);
	
	//Magnetometer initialisieren
	#ifdef DEBUG
		Serial.println(F("Magnetometer initialisieren"));
	#endif
	Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(MAG_CRA);
	Wire.write(B10011100);  //Temp enabled, 220 Hz
	Wire.endTransmission(rTrue);
	Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(MAG_MR);
	Wire.write(B00000000); //Continuous Conversion mode
	Wire.endTransmission(rTrue);
	
	#ifdef DEBUG
	Serial.println(F("Initialisierung abgeschlossen"));
	#endif
}

void loop() {

	// use buffer stream to format line
	obufstream bout(buf, sizeof(buf));
	
	//Daten einlesen
	//ACC1
	if(ACC_dataAvailable(ACC1_ADDRESS)) {
		PINB = 1; //LED blinken lassen
		bout << "ACC1," << millis();  //belegt 13 Eintraege in buf
		#ifdef DEBUG
			Serial.println(F("ACC1 hat neue Daten"));
		#endif
		if(ACC_dataRead(ACC1_ADDRESS)) {
			for (i=0; i<3; i++) {  //belegt 9 Eintraege in buf
				bout << "," << ACC_raw[i];
				#ifdef DEBUG2
					Serial << "{TIMEPLOT|data|ACC1_" << i << "|T|" << ACC_raw[i] << "}\n";
				#endif
			}
		}
		bout << "," << ACC_dataLost() << endl;  //belegt 3 Eintraege
		
		#ifdef ECHO_TO_SERIAL
		cout << buf;
		#endif  // ECHO_TO_SERIAL
		
		// log data and flush to SD
		//logfile << buf << flush;
	}
	
	//ACC2
	if(ACC_dataAvailable(ACC2_ADDRESS)) {
		PINB = 1; //LED blinken lassen
		bout << "ACC2," << millis();  //belegt 13 Eintraege in buf
		#ifdef DEBUG
			Serial.println(F("ACC2 hat neue Daten"));
		#endif
		if(ACC_dataRead(ACC2_ADDRESS)) {
			for (i=0; i<3; i++) {  //belegt 9 Eintraege
				bout << "," << ACC_raw[i];
				#ifdef DEBUG2
					Serial << "{TIMEPLOT|data|ACC2_" << i << "|T|" << ACC_raw[i] << "}\n";
				#endif
			}
		}
		bout << "," << ACC_dataLost() << endl; //belegt 3 Eintraege
		
		#ifdef ECHO_TO_SERIAL
		cout << buf << endl;
		#endif  // ECHO_TO_SERIAL
		
		// log data and flush to SD
		//logfile << buf << flush;
	}
	
	//MAG und THM
	if(MAG_dataAvailable()) {
		PINB = 1; //LED blinken lassen
		bout << "MAG," << millis();  //belegt 12 Eintraege in buf
		#ifdef DEBUG
			Serial.println(F("MAG hat neue Daten"));
		#endif
		if(MAG_dataRead()) {
			for (i=0; i<3; i++) {  //belegt 9 Eintraege
				bout << "," << MAG_raw[i];
				#ifdef DEBUG2
					Serial << "{TIMEPLOT|data|MAG_" << i << "|T|" << MAG_raw[i] << "}\n";
				#endif
			}
			bout << endl << "THM" << "," << THM_raw << endl;  //belegt 8 Eintraege
			#ifdef DEBUG2
				Serial << "{TIMEPLOT|data|THM|T|" << THM_raw << "}\n";
			#endif
		}
		
		#ifdef ECHO_TO_SERIAL
		cout << buf << endl;
		#endif  // ECHO_TO_SERIAL
		
		// log data and flush to SD
		//logfile << buf << flush;
	}
	

	
	
	// log data and flush to SD
	logfile << buf << flush;
	
	#ifdef DEBUG
	// check for error
	if (!logfile) error("write data failed");
	#endif
	
	#ifdef ECHO_TO_SERIAL
	cout << buf;
	#endif  // ECHO_TO_SERIAL
	
	#ifdef DEBUG
	if (Serial.available()) {
		logfile.close();
		cout << pstr("Done!");
		while(1);
	}
	#endif
}
	
byte ACC_dataAvailable(byte ACC_ADDRESS) {
	Wire.beginTransmission(ACC_ADDRESS);
	Wire.write(ACC_STATUS);
	Wire.endTransmission(false);
	Wire.requestFrom(ACC_ADDRESS, singleByte, rTrue);
	ACC_stat = Wire.read();
	return bitRead(ACC_stat,3);
}

byte ACC_dataRead(byte ACC_ADDRESS) {
	Wire.beginTransmission(ACC_ADDRESS);
	Wire.write(ACC_CONT | ACC_X_L);
	Wire.endTransmission(rFalse);
	Wire.requestFrom(ACC_ADDRESS, readByteCount, rTrue);
	i = 0;
	while(Wire.available() > 1){
		ACC_low = Wire.read();
		ACC_raw[i] = (Wire.read()<<8) | ACC_low;
		i++;
	}
	return 1;
}

byte ACC_dataLost() {
	return (ACC_stat >> 4);
}

byte MAG_dataAvailable() {
	Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(MAG_SR);
	Wire.endTransmission(rFalse);
	Wire.requestFrom(MAG_ADDRESS, singleByte, rTrue);
	if (Wire.available()) {
		return bitRead(Wire.read(),0);
	}
	else return 0;
}

byte MAG_dataRead() {
	Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(MAG_X_H);
	Wire.endTransmission(rFalse);
	Wire.requestFrom(MAG_ADDRESS, readByteCount, rTrue);
	i = 0;
	while(Wire.available() > 1){
		MAG_high = Wire.read();
		MAG_raw[i] = Wire.read() | (MAG_high<<8);
		i++;
	}
	Wire.beginTransmission(MAG_ADDRESS);
	Wire.write(MAG_THM_H);
	Wire.endTransmission(false);
	Wire.requestFrom(MAG_ADDRESS, THM_readByteCount, rTrue);
	while(Wire.available()) {
		THM_high = Wire.read();
		THM_raw = Wire.read() | (THM_high << 8);
		if (THM_raw > 1200 | THM_raw < 0)	{
			THM_raw = THM_last;
		}
		else THM_last = THM_raw;
	}
	return 1;
}
