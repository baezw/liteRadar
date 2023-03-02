/*!
 * @headerfile liteRadar.h
 * @details	header file Arduino radr library for using Seeed 24ghz mmWave lite module
 */

#include "Arduino.h"

#ifndef simpleRadar_h
#define simpleRadar_h

//  frame headers and enders
#define HEAD1		  				0x53        //Data frame header1
#define HEAD2   					0x59        //Data frame header2
#define END1    					0x54        //End1 of data frame
#define END2    					0x43        //End2 of data frame

//  convenient byte positions
#define CONTROL         			2           // control byte
#define COMMAND         			3           // command byte
#define DATA            			6           // first data byte

// mostly stuff for built in scenarios
#define WORKING_STATUS  			0x05        // working status packet
#define INIT_COMPLETE				0x01		// init complete return command
#define SET_SENSITIVITY				0x08		// set sensitivity command
#define GET_SENSITIVITY				0x88		// get sensitivity command
#define SET_SCENARIO				0x07		// set scenario command
#define LIVING_ROOM     			0x01        // living room scenario
#define AREA_DETECTION  			0x02        // area detection scenario
#define BATHROOM        			0x03        // bathroom scenario
#define BEDROOM         			0x04        // bedroom scenario
#define GET_SCENARIO				0x87		// get scenario command 
#define OPEN_CUSTOM					0x09		// open custom mode command
#define MODE_1          			0x01        // custom mode 1
#define MODE_2          			0x02        // custom mode 2
#define MODE_3          			0x03        // custom mode 3
#define MODE_4          			0x04        // custom mode 4
#define EXIT_CUSTOM					0x0A		// exit custom mode

#define WORKING_STATUS_RANGE		0x07		// controls ror range in canned scenarios
#define SET_MAX_ACTIVE_RANGE		0x01		// command byte to set maximum active detection range
#define SET_MAX_STATIONARY_RANGE	0x04		// command byte to set maximum stationary detectio range
#define GET_MAX_ACTIVE_RANGE		0x81		// command to get maximum active detection range
#define GET_MAX_STATIONARY_RANGE	0X84		// command to get maximum stationary detection range

// stuff for custom modes and underlying information
#define CUSTOM						0x08		// settings for custom modes
#define SET_PRESENCE_THRESHOLD		0x08		// set presence threshold command
#define GET_PRESENCE_THRESHOLD		0x88		// get presence threshold command
#define SET_PRESENCE_RANGE			0x0A		// set range for presence detection
#define GET_PRESENCE_RANGE			0x8A		// get range for presence detection
#define SET_MOTION_THRESHOLD		0x09		// set motion threshold command
#define GET_MOTION_THRESHOLD		0x89		// get motion threshold command
#define SET_MOTION_RANGE			0x0B		// set range for motion detection
#define GET_MOTION_RANGE			0x8B		// get range for motion detection
#define SET_MOTION_VALID_TIME		0x0C		// set how long to wait before changing motions status
#define GET_MOTION_VALID_TIME		0X8C		// get how long to wait before changing motions status
#define SET_STATIONARY_VALID_TIME	0x0D		// set how long to wait before changing presense status
#define GET_STATIONARY_VALID_TIME	0x8D		// set how long to wait before changing presense status

// stuff for the human status reports
#define HUMAN_STATUS    			0x80        // human status packet
#define PRESENCE         			0x01        // human present
#define MOTION          			0x02        // somoene moving
#define AMPLITUDE_DATA				0x03		// amplitude data
#define POSITION_EVENT				0x0B 		// report on position
#define SET_TIME_OF_ABSENCE			0x0A		//	set time of absence command
#define GET_TIME_OF_ABSENCE			0x8A		//	get time of absence command
#define GET_PRESENCE_EVENT			0x81		// get presence / absence event
#define GET_MOTION_AMP_EVENT		0x82		// get motion amplitude event
#define GET_MOTION_AMP_DATA			0x83		// get motion amplitude quantified data
#define GET_POSITIONB_EVENT			0x8B		// get 0osition event.

// system frames
#define SYSTEM          			0x01        // heartbeat packet or reset
#define HEARTBEAT					0x01		// heartbeat frame
#define RESET						0x02		// reset command

#define UNDERLYING					0x08		// control byte for underlying data
#define SET_UNDERLYING				0x00		// command to set underlying data on/off
#define GET_UNDERLYING				0x80		// command to get current undelying data byte

#define TIME_TO_WAIT				5000		// time to wait on a return frame match

/*!
 * @struct		Frame
 * @param		msg		buffer to hold the frame
 * @param		l		length of the frame
 */

struct Frame {
	unsigned char msg[20];
	unsigned int l;
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
		unsigned int char_to_int(unsigned char* data);
		void int_to_char(unsigned char* data, unsigned int i);
		bool buildFrame(Frame* frame, byte control, byte command, unsigned int data_length, unsigned char* data);
		bool validateFrame(Frame* frame, byte control, byte command, unsigned char* data, bool check_data);
		bool setParam(byte control, byte command, unsigned char* data);
		bool getParam(byte control, byte command, unsigned char* data);
		unsigned int getDataLength(byte control, byte command);
	public:
		Radar(Stream *s);
		void streamFrames(unsigned long t);
		bool resetRadar();

		bool setScenario(byte scenario);
		byte getScenario();
		bool setSensitivity(byte sensitivity);
		byte getSensitivity();
		bool setTimeOfAbsence(byte threshold);
		byte getTimeOfAbsence();

		bool openCustomMode(byte mode);
		bool exitCustomMode();
		bool setPresenceThreshold(byte threshold);
		byte getPresenceThreshold();
		bool setPresenceRange(byte range);
		byte getPresenceRange();
		bool setStationaryValidTime(unsigned int t);
		unsigned int getStationaryValidTime();
		bool setMotionThreshold(byte threshold);
		byte getMotionThreshold();
		bool setMotionRange(byte range);
		byte getMotionRange();
		bool setMotionValidTime(unsigned int t);
		unsigned int getMotionValidTime();

		bool setUnderlying(byte onoff);
		byte getUnderlying();
		
		bool updateStatus();
		bool isPresent();
		bool isMoving();
};

#endif