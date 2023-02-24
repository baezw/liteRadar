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

		// initialize the radar 
		if (!radar.resetRadar()) LOG1("reset failed\n");
		delay(1000);

		if(!radar.openCustomMode(MODE_1)) LOG1("open custom mode failed\n");
		delay(1000);
		if (!radar.setPresenceThreshold(0x1E)) LOG1("set presence threshold failed\n");
		delay(1000);
		if (!radar.setTimeOfAbsence(0x01)) LOG1("set time of absence trigger failed\n");
		delay(1000);
		if (!radar.setMotionThreshold(0x0E)) LOG1("set presence threshold failed\n");
		delay(1000);
		if(!radar.exitCustomMode()) LOG1("exit custom mode failed\n");
		delay(1000);
		if (!radar.resetRadar()) LOG1("reset failed\n");
		delay(1000);
		// done with radar setup
		LOG1("radar setup complete\n");
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

	homeSpan.setLogLevel(1);
	homeSpan.setStatusPin(STATUS_PIN);
	homeSpan.setControlPin(CONTROL_PIN);
	homeSpan.setPairingCode("11122333");
	homeSpan.setQRID("WHAT");
	homeSpan.enableOTA();

	homeSpan.begin(Category::Sensors, "Occupancy Sensor");

	new SpanAccessory();
		new DEV_Identify("Occupancy Sensor", "XXXX", "000000", "OS1", "0.0.1", 3);
		new Service::HAPProtocolInformation();
			new Characteristic::Version("1.1.0"); 
		new DEV_OccupancySensor();
}

void loop() {
	homeSpan.poll();
}