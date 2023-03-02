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
 * @fn getDataLength(byte control, byte command) 
 * @brief command to get the data length of a particular control command combination
 * @param control  	control byte
 * @param command	command byte
 * @returns number of data bytes for the control command combination
 */

unsigned int Radar::getDataLength(byte control, byte command) {
	switch (control) {
		case WORKING_STATUS_RANGE:
			switch(command) {
				case GET_MAX_ACTIVE_RANGE:
					return 1;
					break;
				case GET_MAX_STATIONARY_RANGE:
					return 1;
					break;
				default:
					return 2;
					break;
			}
		case CUSTOM:
			switch(command) {
				case SET_MOTION_VALID_TIME:
					return 4;
					break;
				case SET_STATIONARY_VALID_TIME:
					return 4;
					break;
				case SET_ABSENCE_VALID_TIME:
					return 4;
					break;
				default:
					return 1;
					break;
			}
		default:
			return 1;
			break;
	}
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
	Serial.print("msg = ");
	for (int n = 0; n < frame->l; n++) {
		sprintf(output, "%02X", frame->msg[n]);
		Serial.print(output);
		Serial.print(' ');
	}
	Serial.print("  l = ");
	Serial.print(frame->l);
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
}

/*!
 * @fn char_to_int
 * @brief convert a 4 byte character string and  to unsigned int
 * @param data   4 byte char string ordered big endian
 * @returns unsigned int
 */
unsigned int Radar::char_to_int(unsigned char* data) {
	unsigned int v;

	v = (unsigned int)(data[0] << 24);
	v = v + (unsigned int)(data[1] << 16);
	v = v + (unsigned int)(data[2] << 8);
	v = v + (unsigned int)(data[3]);
	return v;
}

/*!
 * @fn int_to_char
 * @brief  converts unsigned int to 4 byte char
 * @param data 4 byte chr string
 * @param i unsigned int 
 * @returns 4 byte char*
 */
void Radar::int_to_char(unsigned char* data, unsigned int i) {
	data[0] = (char)(i >> 24) & 0xFF;
	data[1] = (char)(i >> 16) & 0xFF;
	data[2] = (char)(i >> 8) & 0xFF;
	data[3] = (char)i & 0xFF;
}

bool Radar::buildFrame(Frame* frame, unsigned char control, unsigned char command, unsigned int data_length, unsigned char* data) {
	frame->l = 9 + data_length;
	frame->msg[0] = HEAD1;
	frame->msg[1] = HEAD2;
	frame->msg[CONTROL] = control;
	frame->msg[COMMAND] = command;
	frame->msg[4] = (char)(data_length >> 8) & 0xFF;
	frame->msg[5] = (char)data_length & 0xFF;;
	if (data_length == 1) {
		frame->msg[DATA] = data[3];
	}
	else if (data_length == 2) {
		frame->msg[DATA] = data[2];
		frame->msg[DATA+1] = data[3];
	} else if (data_length == 4) {
		frame->msg[DATA] = data[0];
		frame->msg[DATA+1] = data[1];
		frame->msg[DATA+2] = data[2];
		frame->msg[DATA+3] = data[3];
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
 * @param data expected data
 * @param check_data if true check to see if the data matches what is expected
 * @returns if successful returns value from the frame. If failed returns -1 for bad control match
 * 		-2 for bad command match and -3 for checksum failure, -4 is for bad data length
 * 
 */

bool Radar::validateFrame(Frame* frame, byte control, byte command, unsigned char* data, bool check_data) {
	byte checksum = 0;
	unsigned int cs_byte = frame->l - 3;
	int data_length = frame->l - 9;
	if (frame->msg[CONTROL] != control) return false;	// control did not match
	if (frame->msg[COMMAND] != command) return false;	// command did not match
	for (int i = 0; i < cs_byte; i++) {
		checksum = checksum + frame->msg[i];
	}
	if (checksum != frame->msg[cs_byte]) return false;	// checksum failed
	if(!check_data) return true;
	if (data_length == 1) {
		if (frame->msg[DATA] != data [3]) return false;
	} else if (data_length == 2) {
		if (frame->msg[DATA] != data[2]) return false;
		if (frame->msg[DATA+1] != data[3]) return false;
	} else if (data_length == 4) {
		if (frame->msg[DATA] != data[0]) return false;
		if (frame->msg[DATA+1] != data[1]) return false;
		if (frame->msg[DATA+2] != data[2]) return false;
		if (frame->msg[DATA+3] != data[3]) return false;
	} else return false; // unexpected data length
	return true; 
}



/*!
 * @fn setParam
 * @brief constructs a frame and sends it to the module. Then reads up to 10 following
 * 		frames to see if the correct response is received
 * @param control byte to hold control value
 * @param command byte to hold command specifiying parameter
 * @param data	 data to be sent
 */
bool Radar::setParam(byte control, byte command, unsigned char* data) {									 // currently only works with single byte data
	Frame req;
	Frame ret;
	unsigned int data_length = getDataLength(control, command);
	if (buildFrame(&req, control, command, data_length, data)) {
			unsigned long start = millis();
			unsigned long elapsed = 0;
			putFrame(&req);
			while (true && elapsed < TIME_TO_WAIT) {
				if (getFrame(&ret)) {
					if (validateFrame(&ret, control, command, data, true)) return true;
				}
			elapsed = millis() - start;
			}
			return false;
	}
	return false;
}

/*!
 * @fn getParam
 * @brief contstructs a frame to requet a parameter value and returns it
 * @param control byte to hold control value
 * @param command byte to hold command specifiying parameter
 * @param data char array to receive the data
 * @returns returns byte value of the parameter and or -1 if request failed
 */
bool Radar::getParam(byte control, byte command, unsigned char* data) {				// currently only works with single byte data
	Frame req;
	Frame ret;

	int data_length = getDataLength(control, command);
	if (buildFrame(&req, control, command, data_length, data)) {
		unsigned long start = millis();
		unsigned long elapsed = 0;
		putFrame(&req);
		while (true && elapsed < TIME_TO_WAIT) {
			if (getFrame(&ret)) {
				if (validateFrame(&ret, control, command, data, false)) {
					data_length = ret.l - 9;
					Serial.printf(" data_length = %d\n, data_length");
					if (data_length == 1 ) {
						data[0] = 0x00;
						data[1] = 0x00;
						data[2] = 0x00;
						data[3] = ret.msg[DATA];
						return true;
					} else if (data_length == 2) {
						data[0] = 0x00;
						data[1] = 0x00;
						data[2] = ret.msg[DATA];
						data[3] = ret.msg[DATA+1];
						return true;
					} else if (data_length == 4) {
						data[0] = ret.msg[DATA];
						data[1] = ret.msg[DATA+1];
						data[2] = ret.msg[DATA+2];
						data[3] = ret.msg[DATA+3];
						return true;
					}
				} 
				return false;

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
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	return setParam(SYSTEM, RESET, data);
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
	unsigned char data[] = {0x00, 0x00, 0x00, scenario};
	return setParam(WORKING_STATUS, SET_SCENARIO, data);
}

/*!
 * @fn getScanario
 * @brief gets the current scenario value
 * @returns byte value on success, -1 on failure
 */
byte Radar::getScenario() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(WORKING_STATUS, GET_SCENARIO, data)) {
		return data[3];
	}
}

/*!
 * @fn setSensitvity
 * @brief sets the sensitivity used by the module
 * 		can be 1-3
 * @returns true on success, false if failed
 */
bool Radar::setSensitivity(byte s) {
	unsigned char data[] = {0x00, 0x00, 0x00, s};
	return setParam(WORKING_STATUS, SET_SENSITIVITY, data);
}

/*!
 * @fn getSensitivity
 * @brief gets the current sensitivity value
 * @returns unsigned int value on success, -1 on failure
 */
byte Radar::getSensitivity() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(WORKING_STATUS, GET_SENSITIVITY, data)) {
		return data[3];
	}
}

/*!
 * @fn timeOfAbsence
 * @brief set the time to wait before absence is reported
 * @param unsigned int value between 0 and 08
 * @returns true for success, false for failed
 */
bool Radar::setTimeOfAbsence(byte t) {
	unsigned char data[] = {0x00, 0x00, 0x00, t};
	return setParam(HUMAN_STATUS, SET_TIME_OF_ABSENCE, data);
}

/*!
 * @fn getTimeOfAbsence
 * @brief gets the current time before absence is reported value
 * @returns unsigned int value on success, -1 on failure
 */
byte Radar::getTimeOfAbsence() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(HUMAN_STATUS, GET_TIME_OF_ABSENCE, data)) {
		return data[3];
	}
}

/*!
 * @fn openCustomMode
 * @brief opens custom mode to allow more control of module settings
 * @param mode unsigned int value 1-4
 * @returns true if success, false if failed
 */
bool Radar::openCustomMode(byte mode) {
	unsigned char data[] = {0x00, 0x00, 0x00, mode};
	return setParam(WORKING_STATUS, OPEN_CUSTOM, data);
}

/*!
 * @fn closeCustomMode
 * @brief closes custom mode and saves values to module
 * @returns true if success, false if failed
 */
bool Radar::exitCustomMode() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	return setParam(WORKING_STATUS, EXIT_CUSTOM, data);
}

/*!
 * @fn setPresenceThreshold
 * @brief set presence threshold for the current custom mode
 * @param unsigned int value between 0 and 250
 * @returns true for success, false for failed
 */
bool Radar::setPresenceThreshold(byte threshold) {
	unsigned char data[] = {0x00, 0x00, 0x00, threshold};
	return setParam(CUSTOM, SET_PRESENCE_THRESHOLD, data);
}

/*!
 * @fn getPresenceThreshold
 * @brief gets the current presence threshold value
 * @returns unsigned int value on success, -1 on failure
 */
byte Radar::getPresenceThreshold() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_PRESENCE_THRESHOLD, data)) {
		return data[3];
	}
}

/*!
 * @fn setPresenceRange
 * @brief set presence range for the current custom mode
 * @param unsigned int values from 0 (0m) to 0A (5m) are valid
 * @returns true for success, false for failed
 */
bool Radar::setPresenceRange(byte range) {
	unsigned char data[] = {0x00, 0x00, 0x00, range};
	return setParam(CUSTOM, SET_PRESENCE_RANGE, data);
}

/*!
 * @fn getPresenceRange
 * @brief gets the current presence range value
 * @returns values from 0 (0m) to 0A (5m) are valid, -1 on failure values from 0 (0m) to 0A (5m) are valid
 */
byte Radar::getPresenceRange() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_PRESENCE_RANGE, data)) {
		return data[3];
	}
}

/*!
 * @fn setMotionThreshold
 * @brief set motion threshold for the current custom mode
 * @param unsigned int value between 0 and 250
 * @returns true for success, false for failed
 */
bool Radar::setMotionThreshold(byte threshold){
	unsigned char data[] = {0x00, 0x00, 0x00, threshold};
	return setParam(CUSTOM, SET_MOTION_THRESHOLD, data);
}

/*!
 * @fn getMotionThreshold
 * @brief gets the current motion threshold value
 * @returns unsigned int value on success, -1 on failure
 */
byte Radar::getMotionThreshold() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_MOTION_THRESHOLD, data)) {
		return data[3];
	}
}

/*!
 * @fn setMotionRange
 * @brief set motion range for the current custom mode
 * @param unsigned int values from 0 (0m) to 0A (5m) are valid
 * @returns true for success, false for failed
 */
bool Radar::setMotionRange(byte range) {
	unsigned char data[] = {0x00, 0x00, 0x00, range};
	return setParam(CUSTOM, SET_MOTION_RANGE, data);
}

/*!
 * @fn getMotionRange
 * @brief gets the current motion range value
 * @returns values from 0 (0m) to 0A (5m) are valid, -1 on failure values from 0 (0m) to 0A (5m) are valid
 */
byte Radar::getMotionRange() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_MOTION_RANGE, data)) {
		return data[3];
	}
}

/*!
 * @fn setStationaryValidTime
 * @brief set presence range for the current custom mode
 * @param unsigned int values in ms
 * @returns true for success, false for failed
 */
bool Radar::setStationaryValidTime(unsigned int t) {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	int_to_char(data, t);
	return setParam(CUSTOM, SET_STATIONARY_VALID_TIME, data);
}

/*!
 * @fn getStationaryValidTime
 * @brief gets the current stationary valid time value
 * @returns values in ms -1 on failure 
 */
unsigned int Radar::getStationaryValidTime() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_STATIONARY_VALID_TIME, data)) {
		int i = char_to_int(data);
		return i;
	}
}

/*!
 * @fn setMotionValidTime
 * @brief set presence range for the current custom mode
 * @param unsigned int values in ms
 * @returns true for success, false for failed
 */
bool Radar::setMotionValidTime(unsigned int t) {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	int_to_char(data, t);
	return (setParam(CUSTOM, SET_MOTION_VALID_TIME, data));
}

/*!
 * @fn getMotionValidTime
 * @brief gets the current stationary valid time value
 * @returns values in ms -1 on failure 
 */
unsigned int Radar::getMotionValidTime() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_MOTION_VALID_TIME, data)) {
		int i = char_to_int(data);
		return i;
	}
}

/*!
 * @fn setAbsenceValidTime
 * @brief set absence valid time for the current custom mode
 * @param unsigned int values in ms
 * @returns true for success, false for failed
 */
bool Radar::setAbsenceValidTime(unsigned int t) {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	int_to_char(data, t);
	return (setParam(CUSTOM, SET_ABSENCE_VALID_TIME, data));
}

/*!
 * @fn getAbsenceValidTime
 * @brief gets the current absence valid time value
 * @returns values in ms -1 on failure 
 */
unsigned int Radar::getAbsenceValidTime() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(CUSTOM, GET_ABSENCE_VALID_TIME, data)) {
		int i = char_to_int(data);
		return i;
	}
}

/*!
 * @fn setUnderlying
 * @brief turns on an off the automatic reporting of underlying data
 * @param onoff byte value of 0 or 1 turning off or on the underlying data
 * @returns bool true if successful false if failed
 */
bool Radar::setUnderlying(byte onoff) {
	unsigned char data[] = {0x00, 0x00, 0x00, onoff};
	return setParam(UNDERLYING, SET_UNDERLYING, data);
}

/*!
 * @fn getUnderlying
 * @brief  returns byte indicating current  status of the underlying data switch
 * @returns byte value 0 or 1
*/
byte Radar::getUnderlying() {
	unsigned char data[] = {0x00, 0x00, 0x00, 0x0F};
	if (getParam(UNDERLYING, GET_UNDERLYING, data)) {
		return data[3];
	}
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
