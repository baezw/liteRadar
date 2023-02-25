/*!
 * @headerfile liteRadar.h
 * @details	header file Arduino radr library for using Seeed 24ghz mmWave lite module
 */

#include "Arduino.h"

#ifndef simpleRadar_h
#define simpleRadar_h

#define HEAD1		  				0x53        //Data frame header1
#define HEAD2   					0x59        //Data frame header2
#define END1    					0x54        //End1 of data frame
#define END2    					0x43        //End2 of data frame

#define CONTROL         			2           // control byte
#define COMMAND         			3           // command byte
#define DATA            			6           // data byte

#define SET_SCENARIO				0x07		// set scenario command
#define GET_SCENARIO				0x87		// get scenario command 
#define LIVING_ROOM     			0x01        // living room scenario
#define AREA_DETECTION  			0x02        // area detection scenario
#define BATHROOM        			0x03        // bathroom scenario
#define BEDROOM         			0x04        // bedroom scenario

#define SET_PRESENCE_THRESHOLD		0x08		// set presence threshold command
#define SET_PRESENCE_RANGE			0x0A		// set range for presence detection
#define SET_TIME_OF_ABSENCE			0x0A		//	set time of absence command
#define SET_MOTION_THRESHOLD		0x09		// set motion threshold command
#define SET_MOTION_RANGE			0x0B		// set range for motion detection
#define SET_SENSITIVITY				0x08		// set sensitivity command

#define OPEN_CUSTOM					0x09		// open custom mode command
#define EXIT_CUSTOM					0x0A		// exit custom mode
#define MODE_1          			0x01        // custom mode 1
#define MODE_2          			0x02        // custom mode 2
#define MODE_3          			0x03        // custom mode 3
#define MODE_4          			0x04        // custom mode 4

#define SYSTEM          			0x01        // heartbeat packet 
#define WORKING_STATUS  			0x05        // working status packet
#define CUSTOM						0x08		// settings for custom modes
#define HUMAN_STATUS    			0x80        // human status packet

#define PRESENCE         			0x01        // human present
#define MOTION          			0x02        // somoene moving
#define AMPLITUDE_DATA				0x03		// amplitude data
#define POSITION_EVENT				0x0B 		// report on position

#define RESET						0x02		//reset command
#define ZERO_F						0x0F		// convienience
#define INIT_COMPLETE				0x01		// init complete return command

/*!
 * @struct		Frame
 * @param		msg		buffer to hold the frame
 * @param		l		length of the frame
 */

struct Frame {
	unsigned char msg[20];
	int l;
};

/*!
 * @class class structure for the radar device
 *
 */

class Radar {
	private:
		Stream *stream;
		bool presence;
		byte motion;
		void putFrame(Frame* frame);
		bool getFrame(Frame* frame);
		void printFrame(Frame* frame);
		void calculateChecksum(Frame* frame);
		bool setParam(byte control, byte command, byte value);
	public:
		Radar(Stream *s);
		bool resetRadar();
		bool setScenario(byte scenario);
		bool setSensitivity(byte sensitivity);
		bool openCustomMode(byte mode);
		bool exitCustomMode();
		bool setPresenceThreshold(byte threshold);
		bool setTimeOfAbsence(byte threshold);
		bool setPresenceRange(byte range);
		bool setMotionThreshold(byte threshold);
		bool setMotionRange(byte range);
		bool updateStatus();
		bool isPresent();
		bool isMoving();
};

#endif
