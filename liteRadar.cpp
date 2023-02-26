/*
 * liteRadar is a simple library for using the Seeed 24GHz Human Static Presence Lite module
 * in straight forward motion and presence detection sensors.
 * 
 * The module may be used with preset scenarios and sensitivity settings or it may bs used in 
 * "custom" modes where presence and motion range and thresholds may be set. 
 * 
 */


#include "liteRadar.h"

Radar::Radar(Stream *s)
	: stream(s) {
	
}

/*!
 * @fn getFrame
 * @brief reads a frame from the radar module
 * @param frame frame structure to hold returned data and length read
 * @returns true is successful read, false if nothing to read
 */
bool Radar::getFrame(Frame* frame) {
	int l = 0;
	byte c;
	while (stream->available()) {
		c = stream->read();
		if (c == HEAD1) {
			frame->msg[l] = c;
			l++;
			c = stream->read();
			if (c == HEAD2) {
				frame->msg[l] = c;
				l++;
				c = stream->read();
				while (c != END2) {
					frame->msg[l] = c;
					l++; 
					c = stream->read();
				}
				frame->l = l+1;
				frame->msg[l] = c;
				return true;
			}
		}
	}
	frame->l = 0;
	return false;
}

/*!
 * @fn printFrame
 * @brief prints a frame to Serial. Primarily for debugging
 * @param frame frame structure to be printed
 */

void Radar::printFrame(Frame* frame) {
	char output[4];
	for (int n = 0; n < frame->l; n++) {
		sprintf(output, "%02X", frame->msg[n]);
		Serial.print(output);
		Serial.print(' ');
	}
	Serial.println();
	
}

/*!
 * @fn putFrame
 * @brief sends a frame to the radar module
 * @param frame frame structure to be sent
 */
void Radar::putFrame(Frame* frame) {
	stream->write(frame->msg, frame->l);
	stream->flush();
}

/*!
 * @fn streamFrames
 * @brief gets a frame and prints it
 */
void Radar::streamFrames(unsigned long t) {
	Frame frame;
	unsigned long start = millis();
	unsigned long elapsed = 0;
	while (elapsed <= t) {
		if (getFrame(&frame)) {
			printFrame(&frame);
		}
		elapsed = millis() - start;

	}
	Serial.printf("times up... %lu\n", elapsed);
}


/*! 
 * @fn buildFrame()
 * @brief constructs a frame to be sent to the radar
 * @param control	unsigned char for the control byte
 * @param command	unsigned chr for the command byte
 * @param data_length number of data bytes
 * @param the value to be sent
 * @returns true if successful false if failed
 * 
 */

bool Radar::buildFrame(Frame* frame, unsigned char control, unsigned char command, int data_length, int data) {
	frame->l = 9 + data_length;
	frame->msg[0] = HEAD1;
	frame->msg[1] = HEAD2;
	frame->msg[CONTROL] = control;
	frame->msg[COMMAND] = command;
	frame->msg[4] = (char)(data_length >> 8) & 0xFF;
	frame->msg[5] = (char)data_length & 0xFF;;
	if (data_length == 1) {
		frame->msg[DATA] = (char)data & 0xFF;
	}
	else if (data_length == 2) {
		frame->msg[DATA] = (char)(data >> 8) & 0xFF;
		frame->msg[DATA+1] = data & 0xFF;
	} else if (data_length == 4) {
		frame->msg[DATA] = (char)(data >> 24) & 0xFF;
		frame->msg[DATA+1] = (char)(data >> 16) & 0xFF;
		frame->msg[DATA+2] = (char)(data >> 8) & 0xFF;
		frame->msg[DATA+3] = data & 0xFF;
	} else return false;
	unsigned char checksum = 0;
	int cs_byte = frame->l - 3;
	for (int i = 0; i < cs_byte; i++) {
		checksum = checksum + frame->msg[i];
	}
	frame->msg[cs_byte] = checksum;
	frame->msg[frame->l - 2] = END1;
	frame->msg[frame->l - 1] = END2;
	frame->msg[frame->l] = 0X00;

	return true;
}

/*!
 * @fn validateFrame
 * @brief validates a frame by checking expected control, command and checksums and if successful
 * 		returns the valuie from the frame. 
 * @param control	the expected control int he frame
 * @param command the expected command in the frame
 * @returns if successful returns value from the frame. If failed returns -1 for bad control match
 * 		-2 for bad command match and -3 for checksum failure, -4 is for bad data length
 * 
 */

int Radar::validateFrame(Frame* frame, byte control, byte command) {
	byte checksum = 0;
	int cs_byte = frame->l - 3;
	int v = -4;

	if (frame->msg[CONTROL] != control) return -1;	// command did not match
	if (frame->msg[COMMAND] != command) return -2;	// control did not match
	for (int i = 0; i < cs_byte; i++) {
		checksum = checksum + frame->msg[i];
	}
	if (checksum != frame->msg[cs_byte]) return -3;	// checksum failed
	int data_length = frame->l - 9;
	if (data_length == 1) {
		v = (int)frame->msg[DATA];
	} else if (data_length == 2) {
		v = (int)(frame->msg[DATA] << 8);
		v = v + (int)(frame->msg[DATA+1]);
		return v;
	} else if (data_length == 4) {
		v = (int)(frame->msg[DATA] << 24);
		v = v + (int)(frame->msg[DATA+1] << 16);
		v = v + (int)(frame->msg[DATA+2] << 8);
		v = v + (int)(frame->msg[DATA+3]);
		return v;
	}
	return v; // unexpected data length
}



/*!
 * @fn setParam
 * @brief constructs a frame and sends it to the module. Then reads up to 10 following
 * 		frames to see if the correct response is received
 * @param control byte to hold control value
 * @param command byte to hold command specifiying parameter
 * @param value	byte to hold new value for parameter
 */
bool Radar::setParam(byte control, byte command, int value) {									 // currently only works with single byte data
	Frame req;
	Frame ret;

	// figure out the number of data bytes from the control and command
	int data_length = 1;								// just 1 byte for now

	// printFrame(&req);
	if (buildFrame(&req, control, command, data_length, value)) {
			unsigned int start = millis();
			unsigned int elapsed = 0;
			putFrame(&req);
			while (true && elapsed < TIME_TO_WAIT) {
				if (getFrame(&ret)) {
					// printFrame(&ret);
					int v = validateFrame(&ret, control, command);
					if (v == value) return v;
				}
			elapsed = millis() - start;
			}
			Serial.printf("elapsed time = %lu\n", elapsed);
			return false;
	}
	return false;
}

/*!
 * @fn getParam
 * @brief contstructs a frame to requet a parameter value and returns it
 * @param control byte to hold control value
 * @param command byte to hold command specifiying parameter
 * @returns returns byte value of the parameter and or -1 if request failed
 */
int Radar::getParam(byte control, byte command) {				// currently only works with single byte data
	Frame req;
	Frame ret;

	// figure out the number of data bytes from the control and command
	int data_length = 1;								// just 1 byte for now

	if (buildFrame(&req, control, command, data_length, 0xFF)) {
		unsigned int start = millis();
		unsigned int elapsed = 0;
		putFrame(&req);
		while (true && elapsed < TIME_TO_WAIT) {
			if (getFrame(&ret)) {
				return validateFrame(&ret, control, command);
			}
		elapsed = millis() - start;
		}
	return false;
	}
}

/*!
 * @fn resetRadar
 * @brief resets the radar module 
 * 		note that this does not factory reset all of the settings.
 * @returns
 */
bool Radar::resetRadar() {
	return setParam(SYSTEM, RESET, 0xFF);
}

/*!
 * @fn setSenario
 * @brief sets the scenario used by the module
 * 		note that this does not factory reset all of the settings.
 * @param scenario  the scenario to be used 
 * 					can be LIVING_ROOM, AREA_DETECTION, BEDROOM, or BATHROOM
 * @returns true on success, false if failed
 */
bool Radar::setScenario(byte scenario) {
	return setParam(WORKING_STATUS, SET_SCENARIO, scenario);
}

/*!
 * @fn getScanario
 * @brief gets the current scenario value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getScenario() {
	return getParam(WORKING_STATUS, GET_SCENARIO);
}

/*!
 * @fn setSensitvity
 * @brief sets the sensitivity used by the module
 * 		can be 1-3
 * @returns true on success, false if failed
 */
bool Radar::setSensitivity(byte scenario) {
	return setParam(WORKING_STATUS, SET_SENSITIVITY, scenario);
}

/*!
 * @fn getSensitivity
 * @brief gets the current sensitivity value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getSensitivity() {
	return getParam(WORKING_STATUS, GET_SENSITIVITY);
}

/*!
 * @fn openCustomMode
 * @brief opens custom mode to allow more control of module settings
 * @param mode byte value 1-4
 * @returns true if success, false if failed
 */
bool Radar::openCustomMode(byte mode) {
	return setParam(WORKING_STATUS, OPEN_CUSTOM, mode);
}

/*!
 * @fn closeCustomMode
 * @brief closes custom mode and saves values to module
 * @returns true if success, false if failed
 */
bool Radar::exitCustomMode() {
	return setParam(WORKING_STATUS, EXIT_CUSTOM, 0xFF);
}

/*!
 * @fn setPresenceThreshold
 * @brief set presence threshold for the current custom mode
 * @param byte value between 0 and 250
 * @returns true for success, false for failed
 */
bool Radar::setPresenceThreshold(byte threshold) {
	return setParam(CUSTOM, SET_PRESENCE_THRESHOLD, threshold);
}

/*!
 * @fn getPresenceThreshold
 * @brief gets the current presence threshold value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getPresenceThreshold() {
	return getParam(CUSTOM, GET_PRESENCE_THRESHOLD);
}

/*!
 * @fn setPresenceRange
 * @brief set presence range for the current custom mode
 * @param byte values from 0 (0m) to 0A (5m) are valid
 * @returns true for success, false for failed
 */
bool Radar::setPresenceRange(byte range) {
	return setParam(CUSTOM, SET_PRESENCE_RANGE, range);
}

/*!
 * @fn getPresenceRange
 * @brief gets the current presence range value
 * @returns values from 0 (0m) to 0A (5m) are valid, -1 on failure values from 0 (0m) to 0A (5m) are valid
 */
byte Radar::getPresenceRange() {
	return getParam(CUSTOM, GET_PRESENCE_RANGE);
}

/*!
 * @fn timeOfAbsence
 * @brief set the time to wait before absence is reported
 * @param byte value between 0 and 08
 * @returns true for success, false for failed
 */
bool Radar::setTimeOfAbsence(byte t) {
	return setParam(HUMAN_STATUS, SET_TIME_OF_ABSENCE, t);
}

/*!
 * @fn getTimeOfAbsence
 * @brief gets the current time before absence is reported value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getTimeOfAbsence() {
	return getParam(HUMAN_STATUS, GET_TIME_OF_ABSENCE);
}


/*!
 * @fn setMotionThreshold
 * @brief set motion threshold for the current custom mode
 * @param byte value between 0 and 250
 * @returns true for success, false for failed
 */
bool Radar::setMotionThreshold(byte threshold){
	return setParam(CUSTOM, SET_MOTION_THRESHOLD, threshold);
}

/*!
 * @fn getMotionThreshold
 * @brief gets the current motion threshold value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getMotionThreshold() {
	return getParam(CUSTOM, GET_MOTION_THRESHOLD);
}

/*!
 * @fn setMotionRange
 * @brief set motion range for the current custom mode
 * @param byte values from 0 (0m) to 0A (5m) are valid
 * @returns true for success, false for failed
 */
bool Radar::setMotionRange(byte range) {
	return setParam(CUSTOM, SET_MOTION_RANGE, range);
}

/*!
 * @fn getMotionRange
 * @brief gets the current motion range value
 * @returns values from 0 (0m) to 0A (5m) are valid, -1 on failure values from 0 (0m) to 0A (5m) are valid
 */
byte Radar::getMotionRange() {
	return getParam(CUSTOM, GET_MOTION_RANGE);
}
/*!
 * @fn isPresent
 * @brief returns current state of presence
 * @returns true for presnce, false for absence
 */
bool Radar::isPresent() {
	return presence;
}

/*!
 * @fn isPresent
 * @brief returns current state of presence
 * @returns true for presnce, false for absence
 */
bool Radar::isMoving() {
	if (motion == 0x02) return true;
	return false;
}

/*!
 * @fn updateStatus
 * @brief goes in loop to  get frames abd update presnec snd motion status. It is non blocking and
 * passes through if no frames are available.
 * @returns true for new data, false for no change
 */

bool Radar::updateStatus() {
	Frame f;
	bool changed = false;
	if (getFrame(&f)) {
		switch (f.msg[CONTROL]) {
			case HUMAN_STATUS:
				switch (f.msg[COMMAND]) {
					case PRESENCE:
						if (f.msg[DATA] != presence) {
							presence = f.msg[DATA];
							changed = true;
						}
						break;
					case MOTION:
						if (f.msg[DATA] != motion) {
							motion = f.msg[DATA];
							changed = true;
						}
						break;
					default:
						changed = false;
						break;

				}
		}
	return changed;
	}
}