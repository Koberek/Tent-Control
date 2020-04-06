/************************************************************************************************
**********************************************************************************************/

/* PINS USED Arduino Uno   ******************************************************************
*
*	2		>> One Wire bus for multipul 18B20 temp sensors
* A0	>> Heater relay 1  Used WITH Heater relay 2 in parallel
* A1  >> Heater relay 2
* A2  >> Vent
* A3  >> EXTRA
*	13		>> used for indication during testing. DO NOT use otherwise !!

**********************************************************************************************/

#include <OneWire.h>					      // for communication to 18B20 Temp Sensors
#include <DallasTemperature.h>			// for control of 18B20 temp sensors
//#include <Adafruit_Sensor.h>


  // init the timers
unsigned long TEMP_current_millis;
unsigned long LED_current_millis;



// 18B20 Temperature Sensor>> 
#define ONE_WIRE_BUS 2					    // create a OneWire on pin 2... many devices can be read on this single pin
#define TEMPERATURE_PRECISION_12 12		// for setting the precision of the 18B20
//#define TEMPERATURE_PRECISION_9 9

// OneWire Setup >>  Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// DallasTemperature Setup >>  // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);		

DeviceAddress probe1 = { 0x28, 0xAA, 0xBA, 0x81, 0x53, 0x14, 0x01, 0x3F };

//***************    Pin assignments   ***************
#define LEDpin			13
#define heaterPin1	A0			// relay 1  outlet HEAT 1/2
#define heaterPin2  A1      // relay 2  outlet HEAT 1/2... both relay 1 and 2 in parallel
#define ventPin			A2		  // relay 3  outlet VENT
#define extraPin    A3
#define switchPin   3       // unused

// logic for the relay module HIGH = OFF (active LOW inputs)
#define OFF HIGH
#define ON	LOW

// GLOBAL VARIABLES >>*****************************************
int			tentTemperature;

//  Constants >>  ***********************************************
const int		heaterONtemp = 			  45;
const int		heaterOFFtemp=    		60;
const int		ventONtemp=			      70;
const int		ventOFFtemp=			    65;

// for the timer and time
#define TEMP 0x00
unsigned long TEMP_int = 2000;             //  second temp probe read interval
#define LED 0x01
unsigned long LED_int = 500;           // 5 sec



void setup(void)
{
	// define pins
	// heater pins A0 and A1
	pinMode(heaterPin1, OUTPUT);
	digitalWrite(heaterPin1, OFF);
  pinMode(heaterPin2, OUTPUT);
  digitalWrite(heaterPin2, OFF);
	
	// vent pin A2
	pinMode(ventPin, OUTPUT);
	digitalWrite(ventPin, OFF);

  // extraPin
  pinMode(extraPin, OUTPUT);
  digitalWrite(extraPin, OFF);

  // switchPin 3
  pinMode(switchPin, INPUT_PULLUP);
	
	// For testing ONLY using on-board LED
	pinMode(LEDpin,OUTPUT);
  digitalWrite(LEDpin, OFF);
	
	// start serial port
	Serial.begin(9600);
	Serial.println();
	
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
	sensors.setResolution(probe1, TEMPERATURE_PRECISION_12);
	
	// delay 2sec before starting loop

// init the timers. used to schedule function calls at interval
TEMP_current_millis    = millis();

LED_current_millis  = millis();
	
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
	
}

void getTemps(void){
  float temp;
  // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
  // THIS is REQUIRED before any request is made to get the temperature of the sensors !!!
  sensors.requestTemperatures();
  // get the temperatures
  temp = (sensors.getTempF(probe1));    // returns degF
  tentTemperature = (int) temp;
}


void ventTent(void){
  // Control the Vent, Heater and Humidifier inside of the tent
  // First turn on the Vent is the temp is too high. If YES then turn off both heater and humidifier OFF
  if (tentTemperature >= ventONtemp){
    digitalWrite(heaterPin1, OFF);     //turn off heater.. shouldn't have been on anyway bu
    digitalWrite(heaterPin2, OFF);     //turn off heater.. shouldn't have been on anyway but
    digitalWrite(ventPin, ON);        //turn ON venting
  }
  if (tentTemperature <= ventOFFtemp){
    digitalWrite(ventPin, OFF);        //turn ON venting
  }
  if (tentTemperature <= heaterONtemp){
    digitalWrite(ventPin, OFF);
    digitalWrite(heaterPin1, ON);
    digitalWrite(heaterPin2, ON);
  }
  if (tentTemperature >= heaterOFFtemp){
    digitalWrite(heaterPin1, OFF);
    digitalWrite(heaterPin2, OFF);
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
   
    Serial.println();
    Serial.print("Tent:  temperature deg F=  ");
    Serial.print(tentTemperature);
    Serial.println();
    
    return;
  
}
