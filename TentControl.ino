/************************************************************************************************
 *	
 *	Target board is Arduino UNO, but this version will work on the ATMega without modification
 *
 *	
 *
 * The temperature sensors (18B20) each have a unique address. Must change this address in the code to get other sensors to work
 *
 *
 * NOT USED IN THIS FILE   is used to remove everything from tent.ino that is not used in this file.
 *
 * Created: 1/10/2018 8:01:07 PM
 * Author: Many. Not me. I assembled this code, but didn't write the good stuff
 *
 * Uses DallasTemperature and OneWire to read temperature probes. 18B20 probes have specific addresses !
 * Uses DHT to read humidity using DHT11 detector.
 ********************************************************************************************/ 


/**********************************************************************************************
*   18B20 TEMPERATURE SENSOR DATA
*	sensor DevID's were captured one at a time using DallasTemperature for new probes.
*
*	The purchased sensors are marked with 1,2,3,4 and 5 stripes. Two sensor boards that I built combine 18B20 and DHT11 
*	humidity sensor. These are labeled 0442 and 04F6
*
*	DevID 28FFC30D8114021B	probe1			0x28,0xFF,0xC3,0x0D,0x81,0x14,0x02,0x1B
*	DevID 28FF7D658114027A	probe2
*	DevID 28FFD25C30170462	probe3
*	DevID 28FFDB578114029A	probe4
*	DevID 28FF7BF680140224	probe5
*	DevID 28FF552C33170337	probeX			0x28,0xFF,0x55,0x2C,0x33,0x17,0x03,0x37		device used for testing
*	DevID 28FF2F8D33170442	probe0442		0x28,0xFF,0x2F,0x8D,0x33,0x17,0x04,0x42		assembled 18B20 + DHT11 labeled 0442
*	DevID 28FF2EAB331704F6	probe04F6		0x28.0xFF,0x2E,0xAB,0x33,0x17,0x04,0xF6		assembled 18B20 + DHT11 labeled 04F6
**********************************************************************************************/

/* PINS USED Arduino Uno   ******************************************************************
*
*	2		>> One Wire bus for multipul 18B20 temp sensors
*	3		>> DHT11 Humidity Sensor
*	6		>> Humidifier
*	7		>> Vent
*	8		>> Heater
*	
*	13		>> used for indication during testing. DO NOT use otherwise !!

**********************************************************************************************/

#include <OneWire.h>					      // for communication to 18B20 Temp Sensors
#include <DallasTemperature.h>			// for control of 18B20 temp sensors
#include <DHT.h>						        // for control of DHT11 humidity sensor
#include <Adafruit_Sensor.h>


  // init the timers
unsigned long TEMP_current_millis;
unsigned long LED_current_millis;



// 18B20 Temperature Sensor>> 
#define ONE_WIRE_BUS 2					    // create a OneWire on pin 2... many devices can be read on this single pin
#define TEMPERATURE_PRECISION_12 12		// for setting the precision of the 18B20
//#define TEMPERATURE_PRECISION_9 9
		
// DeviceAddress defined in DallasTemperature.h as typedef uint8_t DeviceAddress[8];
// each of the following devices have had their address read in other software...
// NOT USED IN THIS FILE   DeviceAddress probe1			{0x28,0xFF,0xC3,0x0D,0x81,0x14,0x02,0x1B};			//device encoded address
// NOT USED IN THIS FILE   DeviceAddress probe2			{0x28,0xFF,0x7D,0x65,0x81,0x14,0x02,0x7A};			//address is physically burned into the component
// NOT USED IN THIS FILE   DeviceAddress probe3			{0x28,0xFF,0xD2,0x5C,0x30,0x17,0x04,0x62};
// NOT USED IN THIS FILE   DeviceAddress probe4			{0x28,0xFF,0xDB,0x57,0x81,0x14,0x02,0x9A};
// NOT USED IN THIS FILE   DeviceAddress probe5			{0x28,0xFF,0x7B,0xF6,0x80,0x14,0x02,0x24};
//DeviceAddress probeX		{0x28,0xFF,0x55,0x2C,0x33,0x17,0x03,0x37};
DeviceAddress probe0442	{0x28,0xFF,0x2F,0x8D,0x33,0x17,0x04,0x42};										// my probe 0442
DeviceAddress probe04F6	{0x28,0xFF,0x2E,0xAB,0x33,0x17,0x04,0xF6};											// my probe 04F6  labeled Small Tent

// DHT11 Humidity Sensor >> Define the Arduino pin used
#define DHTpin	3						// Humidity sensor used int the small tent on Pin 3
// NOT USED IN THIS FILE   #define DHTlargeTentPin 4						// Humidity sensor used int the large tent on Pin 4
// Uncomment whatever type DHT11 you're using!
#define DHTTYPE DHT11   // DHT 11			// this is the device I am using
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


// OneWire Setup >>  Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// DallasTemperature Setup >>  // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);		

// DHT11 Setup >>  two DHT objects (humidity sensors)
//
DHT DHTsensor(DHTpin, DHTTYPE);		//  (pin3,DHT11)
// NOT USED IN THIS FILE   DHT DHTLargeTent(DHTlargeTentPin, DHTTYPE);		// pin 4


//***************    Pin assignments   ***************
#define LEDpin			13
#define heaterPin		8			// relay 1  outlet 1
#define ventPin			A0			// relay 2  outlet 2
#define waterPin	  6			// relay 3  outlet 3
#define switchPin   9     // shouldn't be used by other libraries


// logic for the relay module HIGH = OFF (active LOW inputs)
#define OFF HIGH
#define ON	LOW

// GLOBAL VARIABLES >>*****************************************
float			tentTemperature;
float			tentHumidity;

bool			tentWARNflag = false;				// default startup

//  Constants >>  ***********************************************
const int		tentMinTemp=			  40;			// heaters ON
const int		tentNominalTemp=		70;			// Vent off
const int		tentMaxTemp=			  75;			// Vent ON		NOTE the tentMaxTemp MUST be at least tentNominalTemp + 3... hysteresis
const int		tentWARNtemp=			  93;			// ALARM >> NOT USED yet

const int		tentMinHumidity=		40;			// humidifier ON
const int		tentMaxHumidity=		45;			// humidifier OFF

// for the timer and time
#define TEMP 0x00
unsigned long TEMP_int = 2000;             //  second temp probe read interval

#define LED 0x02
unsigned long LED_int = 500;           // 5 sec



void setup(void)
{
	// define pins
	// heater pin 8
	pinMode(heaterPin, OUTPUT);
	digitalWrite(heaterPin, OFF);
	
	// vent pin 7
	pinMode(ventPin, OUTPUT);
	digitalWrite(ventPin, OFF);

  // switchPin 9
  pinMode(switchPin, INPUT_PULLUP);
	
	// For testing ONLY using on-board LED
	pinMode(LEDpin,OUTPUT);
 digitalWrite(LEDpin, OFF);
	
	// start serial port
	Serial.begin(9600);
	Serial.println();
	
	// start DHT11 Humidity Sensor
	DHTsensor.begin();
	
	// Start the 18B20 temp sensors
	sensors.begin();
	
	// locate the 18B20 devices on the bus
	// this will indicate the presence of devices and the number of them.
	Serial.print("Locating devices...");
	Serial.print("Found ");
	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(" 18B20 sensors.");


	// set the resolution to 12(9 default) bit
	// NOT USED IN THIS FILE   sensors.setResolution(largeTentProbe, TEMPERATURE_PRECISION_12);
	sensors.setResolution(probe04F6, TEMPERATURE_PRECISION_12);
	
	// delay 2sec before starting loop

// init the timers. used to schedule function calls at interval
TEMP_current_millis    = millis();

LED_current_millis  = millis();

	//delay(2000);
	
}

void loop(void)
{
  if (timer_lapsed(TEMP) == true){     // 1 second interval
    getTemps();
    ventTent();
    printTemps();
    }

  if (timer_lapsed(LED) == true){
    digitalWrite(LEDpin, !digitalRead(LEDpin));
  }
    
//delay(1000);
	
}

void getTemps(void){
  // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
  // THIS is REQUIRED before any request is made to get the temperature of the sensors !!!
  sensors.requestTemperatures();
  // get the temperatures
  tentTemperature = (sensors.getTempF(probe04F6));    // returns degF
  
  // get humidity data NOT USED IN THIS PROGRAM
  //tentHumidity = DHTsensor.readHumidity();
}


void ventTent(void){
  // Control the Vent, Heater and Humidifier inside of the tent
  // First turn on the Vent is the temp is too high. If YES then turn off both heater and humidifier OFF
  if (tentTemperature >= tentMaxTemp){
    //digitalWrite(heaterPin, OFF);     //turn off heater.. shouldn't have been on anyway but
    digitalWrite(ventPin, ON);        //turn ON venting
  }
  if (tentTemperature <= tentNominalTemp){
    //digitalWrite(heaterPin, OFF);     //turn off heater
    digitalWrite(ventPin, OFF);        //turn ON venting
  } 
}


bool timer_lapsed(uint8_t PID){                             // timer. used for short interval scheduling 1sec- a few minutes
  if (PID == TEMP){
    if ((millis() - TEMP_current_millis) >= TEMP_int){        // set to 5 sec
      TEMP_current_millis = millis();
      return true;
    }
    else {return false;}
    }

  if (PID == LED){
    if ((millis() - LED_current_millis) >= LED_int){    // set to 5 sec
      LED_current_millis = millis();
      return true;
    }
    else {return false;}
    }
}

void  printTemps(void){
  
    for (int x = 0; x < 60; x++){
      delay(5);
      Serial.print(".");
    }
    
    Serial.println();
    Serial.print("Tent:  temperature deg F=  ");
    Serial.print(tentTemperature);
    Serial.println();
    
    return;
  
}
