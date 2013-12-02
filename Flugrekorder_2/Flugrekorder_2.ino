// 2013/12/01  by Jonis Kiesbye
// 
// Flugrekorder 2
// 
// Log data of I2C devices to an SD Card at high sample frequencies (600 Hz for 
// a 3 axis 16bit sensor) as nicely formatted CSV files. Uses the fantastic
// libraries SdFat and Buffered Writer by fat16lib: 
// https://code.google.com/p/beta-lib/downloads/list
// and I2C by Wayne Truchsess: 
// http://dsscircuits.com/articles/arduino-i2c-master-library.html
// 
// An interrupt service routine (ISR) is called in a fixed interval that you
// can set. It uses the I2C library to read out the sensor's readings. Wire would
// not work here since it relies on interrupts which are blocked while staying in
// the ISR. The readings are stored in (several) ring buffers.
// Those buffers are read out via the main loop, formatted to CSV and written to
// the SD card by the Buffered Writer.
// 
// When selecting the sampling interval keep in mind to give the loop enough time
// so your buffer has time to get written to the SD card.
// If you have multiple sensors you can read them out at different sampling rates.
// Currently the dividers between the sampling rates have to be powers of 2, e.g.
// 1,2,4,8,...
// 
// Tutorials and examples for possible uses are available on the blog
// http://baunotizen.wordpress.com/ (unfortunately only in german)
// 
// I hope you can make some good use of this!

// This example logs an Accelerometer at 400 Hz and a Magnetometer at 200 Hz

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

#include <I2C.h>
#include <BufferedWriter.h>
#include <SdFat.h>
//#include <SdFatUtil.h> // define FreeRam()
#define SD_CHIP_SELECT  SS  // SD chip select pin
// #define LOG_INTERVAL  5000  // mills between entries
// #define ECHO_TO_SERIAL   1  // echo data to serial port if nonzero
// #define WAIT_TO_START    1  // Wait for serial input in setup()
// interval between timer interrupts in microseconds
// const uint16_t TICK_TIME_USEC = 1000;
// // number of ticks between data points
// const uint16_t TICK_LOG_COUNT = 5;
// // interval in usec between data points.
// const uint32_t LOG_INTERVAL_USEC = (uint32_t)TICK_TIME_USEC * TICK_LOG_COUNT;
// // Maximum time between sync() calls in milliseconds.  If MAX_SYNC_TIME_MSEC is
// set to zero, sync will only be called when a character is typed to stop the
// program.  This allows the fastest possible rate.
const uint32_t MAX_SYNC_TIME_MSEC = 1000;
// To get the fastest rate set LOG_DATA_IN_HEX true and MAX_SYNC_TIME_MSEC zero.
//const bool LOG_DATA_IN_HEX = false;

//#include <TimerTwo.h>



// #define DEBUG
// #define DEBUG2

//Variablen fuer SD-Card, SD
// file system object
SdFat sd;
// file for logging data
SdFile file;
// fast text formatter
BufferedWriter bw;
//------------------------------------------------------------------------------
// store error strings in flash
#define error(s) sd.errorHalt_P(PSTR(s));
//------------------------------------------------------------------------------



//Konstanten für Beschleunigungssensor, ACC
const uint8_t ACC_ADDRESS = B0011000;
const uint8_t ACC_CTRL1 = B00100000;
const uint8_t ACC_CTRL4 = B00100011;
const uint8_t ACC_X_L_CONT = B10101000; // address of the first data register with a leading 1 so the address gets incremented automatically for the next reads

//Variablen fuer ACC
uint8_t ACC_raw[6] = {0,0,0,0,0,0};

//Konstanten fuer Magnetometer, MAG
const uint8_t MAG_ADDRESS = B0011110;
const uint8_t MAG_CRA = 0x00;
const uint8_t MAG_MR = 0x02;
const uint8_t MAG_X_H = 0x03;

//Variablen fuer MAG
uint8_t MAG_raw[6] = {0,0,0,0,0,0};
volatile uint8_t read_MAG = 0;

//Allgemeine Konstanten
const uint8_t LED = 8;
const uint8_t div_power = 1;
const uint16_t RING_DIM = 70;

//Allgemeine Variablen
volatile uint16_t overflow = 0;
//------------------------------------------------------------------------------
// ring buffer for binary ADC data
// 328 cpu
// the ring buffers take up a fourth of the total memory on the 328p!
volatile uint32_t ring_time[RING_DIM];
volatile int16_t ring_ACC[RING_DIM][3];
volatile int16_t ring_MAG[(RING_DIM >> div_power)][3];
volatile uint16_t ring_overflow[RING_DIM];
volatile uint8_t head = 0;
volatile uint8_t tail = 0;


//Temp
// volatile uint32_t cycle_time = 0;
// volatile uint32_t cycle_time_pre = 0;
// volatile uint32_t ACC_time = 0;
// volatile uint32_t ACC_time_pre = 0;

// number of points in the ring buffer
inline uint8_t ringAvailable() {
	return (head >= tail ? 0 : RING_DIM) + head - tail;
}
//------------------------------------------------------------------------------
// next value for head/tail
inline uint8_t ringNext(uint8_t ht) {
	return ht < (RING_DIM - 1) ? ht + 1 : 0;
}
//------------------------------------------------------------------------------
// integer power function, posted by krak on 
// http://cboard.cprogramming.com/c-programming/66465-integer-power.html#post471637
uint16_t Power(uint8_t base, uint8_t pow){
	//Special Cases:
	if (pow == 0) {return 1;}
	
	uint16_t result=1;
	for(int i=0;i<pow;i++){
		result *= base;
	}
	return result;
}

//------------------------------------------------------------------------------
// interrupt routine for ADC read.
ISR(TIMER2_COMPA_vect) {
// 	cycle_time = micros() - cycle_time_pre;
// 	cycle_time_pre = micros();
	
// 	// ticks until time to log a data point
// 	static uint16_t ticks = 0;
// 	
// 	// return if not time to log data
// 	if (ticks-- > 1) return;
// 	
// 	// reset tick count
// 	ticks = TICK_LOG_COUNT;
	
	// check for ring full
	uint8_t next = ringNext(head);
	if (next != tail) {
		// log data
		// cycle_time_pre = micros();
		ring_time[head] = micros();
// 		ring[head][0] = (this_time & 0xFFFF0000) >> 16;
// 		ring[head][1] = this_time & 0x0000FFFF;
		I2c.read(ACC_ADDRESS, ACC_X_L_CONT, (uint8_t) 6, ACC_raw);
		ring_ACC[head][0] = ACC_raw[0] | (ACC_raw[1] << 8);	//Einlesen ohne for, um ein paar ns zu sparen
		ring_ACC[head][1] = ACC_raw[2] | (ACC_raw[3] << 8);
		ring_ACC[head][2] = ACC_raw[4] | (ACC_raw[5] << 8);
		// only read out the magnetometer every 2nd pass
		// its max sample rate is only 220 Hz vs 1000 Hz for the accelerometer
		if ((head % Power(2, div_power)) == 0)	{
			I2c.read(MAG_ADDRESS, MAG_X_H , (uint8_t) 6, MAG_raw);
			ring_MAG[(head >> div_power)][0] = (MAG_raw[0] << 8) | MAG_raw[1];
			ring_MAG[(head >> div_power)][1] = (MAG_raw[2] << 8) | MAG_raw[3];
			ring_MAG[(head >> div_power)][2] = (MAG_raw[4] << 8) | MAG_raw[5];
		}
		ring_overflow[head] = overflow;
		
		if (head % 20 == 0) PINB = 1; //LED blinken lassen
		// cycle_time = micros() - cycle_time_pre;
		
		head = next;
	} 
	else {
		// do something about these overruns!
		// I have no idea what to do with them, sir
		// then you should keep track of them at least
		overflow++;
	}
	
}

void shutdown() {
	// Flight Recording Termination Jumper has been put into place
	// Cease operation and close the log file
	// TimerTwo::stop();
	TIMSK2 = 0;
	bw.writeBuf();
	if (!file.close()) error("file.close");
	digitalWrite(LED, LOW);
	cli();
	#ifdef DEBUG	
		PgmPrintln("Stopped!");
	#endif
	while (1);
}

void setup() {
	#ifdef DEBUG
	Serial.begin(115200);
	#endif
	#ifdef DEBUG2
	//Serial.begin(115200);
	#endif
	//LED
	pinMode(LED, OUTPUT); // turn pin 8 into an output. An attached LED will blink during reads
	digitalWrite(LED, LOW);
	//Ausschalter
	pinMode(3, INPUT_PULLUP); // enable Pullup on pin 3. Connecting it to GND will close the file and stop the program
	pinMode(4, OUTPUT); // set pin 4 LOW so we can connect it via a jumper to pin 3 and close the file
	digitalWrite(4, LOW);

	#ifdef DEBUG
		PgmPrint("FreeRam: ");
		Serial.println(FreeRam());
// 		PgmPrint("Log Interval: ");
// 		Serial.print(LOG_INTERVAL_USEC);
// 		PgmPrintln(" usec"	
	#endif
	
	//SD Karte initialisieren
	if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) sd.initErrorHalt();
	
	// create a new file
	char name[13];
	strcpy_P(name, PSTR("B2FAST00.CSV"));
	for (uint8_t n = 0; n < 100; n++) {
		name[6] = '0' + n / 10;
		name[7] = '0' + n % 10;
		if (file.open(name, O_WRITE | O_CREAT | O_EXCL)) break;
	}
	if (!file.isOpen()) error("file.open");
	
// 	file.write_P(PSTR("Log Interval usec: "));
// 	file.println(LOG_INTERVAL_USEC);
	
	bw.init(&file);
	
	#ifdef DEBUG
		PgmPrint("Logging to: ");
		Serial.println(name);
		PgmPrintln("Put the flight recording termination jumper in place to stop.");
	#endif
	
	//I2C Bus starten
	I2c.begin();
	I2c.setSpeed(1);
	I2c.pullup(1);
	
	//Ersten Beschleunigungssensor initialisieren
	#ifdef DEBUG
		Serial.println(F("Ersten Beschleunigungssensor initialisieren"));
	#endif
	I2c.write(ACC_ADDRESS, ACC_CTRL1, (uint8_t) B00110111); // normal mode, 400Hz, XYZ enabled
	I2c.write(ACC_ADDRESS, ACC_CTRL4, (uint8_t) B00110000); // +-24g
	
	
	//Magnetometer initialisieren
	#ifdef DEBUG
		Serial.println(F("Magnetometer initialisieren"));
	#endif
	I2c.write(MAG_ADDRESS, MAG_CRA, (uint8_t) B00011100); // temp disabled, 220 Hz
	I2c.write(MAG_ADDRESS, MAG_MR, (uint8_t) B00000000); // continuous conversion mode
	
	#ifdef DEBUG
		Serial.println(F("Initialisierung abgeschlossen"));
		delay(10); // interrupts might start soon. give serial some time
	#endif
	
	//Timer und Interrupts konfigurieren
	// set tick time
		
	// 	if (!TimerTwo::init(TICK_TIME_USEC)
	// 	|| TICK_TIME_USEC != TimerTwo::period()) {
	// 		// TICK_TIME_USEC is too large or period rounds to a different value
	// 		#ifdef DEBUG
	// 			error("TimerTwo::init");
	// 		#endif
	//	}
	// configure timer2 and interrupts
	// first disable interrupts
	cli();
	TCCR2A = (1 << WGM21); // CTC mode. Counter returns to zero after compare match
	TCCR2B = 0;
	TCNT2  = 0;	//start to count from zero
	OCR2A = 156;              // 2.5 ms, 400 Hz, 8 Mhz, Prescaler 128
	TCCR2B |= (1 << CS22);    // 128 prescaler
	//TCCR2B |= (1 << CS21);
	TCCR2B |= (1 << CS20);
	TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
		
	attachInterrupt(1, shutdown, LOW);
	sei();
	
	// start calls to ISR
	// TimerTwo::start();
}

// cluster for last sync
uint32_t syncCluster = 0;

// time of last sync
uint32_t syncTime = 0;

void loop() {
	cli();
	uint8_t n = ringAvailable();
  sei();
	
	// write a new line to the file
  for (uint8_t i = 0; i < n; i++) {
		// start with the time. It was split up in the ISR and we put it back together here
		//uint32_t time_current = (ring[tail][0] << 16) | ring[tail][1];
		uint32_t time_current = ring_time[tail];
		bw.putNum(time_current); // putNum takes everything from byte to long
		bw.putChar(','); // separate every value with a comma
		
		// insert the sensor readings
		bw.putNum(ring_ACC[tail][0]);
		bw.putChar(',');
		bw.putNum(ring_ACC[tail][1]);
		bw.putChar(',');
		bw.putNum(ring_ACC[tail][2]);
		bw.putChar(',');
		
		//divided_tail = (tail % 2) ? ((tail-1)/2) : (tail/2);
		// even if there are no new readings we will store the old ones multiple times
		// at least we won't get inconsistencies that way
		bw.putNum(ring_MAG[(tail >> div_power)][0]);
		bw.putChar(',');
		bw.putNum(ring_MAG[(tail >> div_power)][1]);
		bw.putChar(',');
		bw.putNum(ring_MAG[(tail >> div_power)][2]);
		bw.putChar(',');
// 		for (uint8_t m = 0; m < (SAMPLE_LEN - 1); m++) {
// 			// get data point. They were stored as unsigned ints but there is a sign in them and we don't want to misinterpret that
// 			//int16_t d = ring[tail][m];
// 			
// 			// format the data point
// 			bw.putNum(ring_ACC[tail][0]);
// 			bw.putChar(',');
// 		}
//		uint16_t d = ring_overflow[tail][6]; // overflow value
		bw.putNum(ring_overflow[tail]);
		
		bw.putCRLF(); // next run will start in a new line of the output file
		tail = ringNext(tail); // and the next run will also start in a new position of the ring buffer
		
		// enabling DEBUG with Serial calls will significantly slow down the code
		// also the interrupts might cause Serial to output garbage characters 
		#ifdef DEBUG
			Serial.println(FreeRam());
			//delay(2);
		#endif
  }
  // check for write error
  /*if (file.writeError) error("write");*/

  // never sync if zero
  if (MAX_SYNC_TIME_MSEC == 0) return;
  
  if (syncCluster == file.curCluster()
    && (millis() - syncTime) < MAX_SYNC_TIME_MSEC) return;
    
  if (!file.sync()) error("file.sync");
  syncCluster = file.curCluster();
  syncTime = millis();	
}
