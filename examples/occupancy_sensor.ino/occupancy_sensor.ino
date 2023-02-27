#include "Arduino.h"
#include "HomeSpan.h"
#include "liteRadar.h"

#define CONTROL_PIN			9   			// pin for the ontrol button
#define STATUS_PIN			10				// pin for the status led


#include <HardwareSerial.h>
HardwareSerial UART(0);

//////////////////////////////////
//   DEVICE-SPECIFIC SERVICES   //
//////////////////////////////////

struct DEV_Identify : Service::AccessoryInformation {

  int nBlinks;                    // number of times to blink built-in LED in identify routine
  SpanCharacteristic *identify;   // reference to the Identify Characteristic
  
  DEV_Identify(const char *name, const char *manu, const char *sn, const char *model, const char *version, int nBlinks) : Service::AccessoryInformation(){
	
	new Characteristic::Name(name);                   // create all the required Characteristics with values set based on above arguments
	new Characteristic::Manufacturer(manu);
	new Characteristic::SerialNumber(sn);    
	new Characteristic::Model(model);
	new Characteristic::FirmwareRevision(version);
	identify=new Characteristic::Identify();          // store a reference to the Identify Characteristic for use below

	this->nBlinks=nBlinks;                            // store the number of times to blink the LED

	//pinMode(2,OUTPUT);          // make sure LED is set for output
  }

  boolean update(){
	LOG1("Identify called");   
	return(true);                               // return true
	
  } // update
  
};

struct DEV_OccupancySensor : Service::OccupancySensor {                       // Motion sensor

	SpanCharacteristic *occupancy;                                         // reference to the MotionDetected Characteristic
	Radar radar = Radar(&UART);

	DEV_OccupancySensor() : Service::OccupancySensor() {
	Serial.println("start of radar initialzation");

	// initialize the radar 
	if (!radar.resetRadar()) Serial.println("reset failed");
	else Serial.println("reset succeeds");

	if(!radar.openCustomMode(MODE_1)) Serial.println("open custom mode failed");
	else Serial.println("open custom mode succeeds");

	if (!radar.setPresenceThreshold((unsigned int)0x1E)) Serial.println("set presence threshold failed");
	else Serial.println("set presence threshold succeeds");
	Serial.printf("current presence threshold = %02X\n", radar.getPresenceThreshold());

	if (!radar.setPresenceRange((unsigned int)0x09)) Serial.println("set presence range failed");
	else Serial.println("set presence range succeeds");
	Serial.printf("current presence range = %02X\n", radar.getPresenceRange());

	if (!radar.setStationaryValidTime(10000)) Serial.println("set stationary valid time failed");
	else Serial.println("set stationary valid time succeeds");
	Serial.printf("current stationary valid time = %d\n", radar.getStationaryValidTime());

	if (!radar.setMotionThreshold(0x0E)) Serial.println("set presence threshold failed");
	else Serial.println("set motion threshold succeeds");
	Serial.printf("current motion threshold = %02X\n", radar.getMotionThreshold());

	if (!radar.setMotionRange(0x09)) Serial.println("set motion range failed");
	else Serial.println("set motion range succeeds");
	Serial.printf("current motion range = %02X\n", radar.getMotionRange());

	if (!radar.setMotionValidTime(3000)) Serial.println("set motion valid time failed");
	else Serial.println("set motion valid time succeeds");
	Serial.printf("current motion valid time = %d\n", radar.getMotionValidTime());

	if(!radar.exitCustomMode()) Serial.println("exit custom mode failed");
	else Serial.println("exit custom mode succeeds");

	if (!radar.resetRadar()) Serial.println("reset failed");
	else Serial.println("reset succeeds");
	LOG1("radar setup complete\n\n");

	// done with radar setup

		occupancy = new Characteristic::OccupancyDetected(false);
		
	}  // end of constructor	

	void loop() {
		if (radar.updateStatus()) {
			boolean occupied = radar.isPresent();
			if (occupied != occupancy->getVal()) {
				LOG1("occupied = %d\n", occupied);
				occupancy->setVal(occupied);
				if (occupied == true) {
					LOG1("occupancy detected\n");
				}
			}
		}
	}	// update routine
};


void setup() {
	Serial.begin(115200);
	UART.begin(115200, SERIAL_8N1, D7, D6);
	delay(3000);

	homeSpan.setLogLevel(1);
	homeSpan.setStatusPin(STATUS_PIN);
	homeSpan.setControlPin(CONTROL_PIN);
	homeSpan.setPairingCode("11122333");
	homeSpan.setQRID("ANYT");
	homeSpan.enableOTA();

	homeSpan.begin(Category::Sensors, "Occupancy Sensor");

	new SpanAccessory();
		new DEV_Identify("Occupancy Sensor", "Billy", "000001", "OS1", "0.0.1", 3);
		new Service::HAPProtocolInformation();
			new Characteristic::Version("1.1.0"); 
		new DEV_OccupancySensor();
}

void loop() {
	homeSpan.poll();
}