# liteRadar

This is a driver library for the 24GHz Seeed Human Static Presence Lite mmWave sensor module. It is intended to be simple to use and oriented toward writing sensors for home automation. It is <em>not</em> intended to provide sophisticated access to underlying data from the module, just allow you to set some parameters and get both presence and motion detection done using the UART interface.

### Functions


| **Function** | **Description** |
| ------------------------ | -------------|
| bool resetRadar(); | resets the radar module. Note that this does not appear to eliminate any parameters saveed, but reloads the settings. This appears to be necessary occassionally. returns true on success. |
| bool setScenario(byte scenario); | sets the built-in scenario model to use. presets are: living room, area detection, bedroom and bathroom. returns true if successful. |
| byte getScenario(); | returns the current scenario in use. |
| bool setSensitivity(byte sensitivity); | sets the sensitivity to use with the built in model. range 0 - 3 |
| byte getSensitivity(); | returns the current sensitvity setting. |
| byte getTimeOfAbsence(); | returns the current wait time before reporting absence. See Users Manual for values. |
| bool setTimeOfAbsence(byte threshold); | sets the time to wait wihtout presence before reporting absence. See Users Manual for values. returns true if successful |
| bool openCustomMode(byte mode); | opens custom mode. mode specifies the mode to use range 1-4. returns true if success |
| bool exitCustomMode(); | Saves parameters into the open custom mode and exits. It appears that these prameters remain in use until on of the built-in scenarios is set. |
| bool setPresenceThreshold(byte threshold); | sets the threshold for presence detection. range 0-250 returns true if successful |
| byte getPresenceThreshold(); | returns the current presence threshold value range 0-250  |
| bool setPresenceRange(byte range); | sets the range for presence detection. see Users Manual for values. returns true if successful. |
| byte getPresenceRange(); | returns the current range for presence detection . see Users Manual for values. |
| bool setMotionThreshold(byte threshold); | sets the threshold for motion detection. see Users Manual for values. returns true if successful. |
| byte getMotionThreshold(); | returns the current threshold for motion detection . see Users Manual for values. |
| bool setMotionRange(byte range); | sets the range for motion detection. see Users Manual for values. returns true if successful. |
| byte getMotionRange(); | returns the range for motion detection. see Users Manual for values. |
|bool setStationaryValidTime(unsigned int t); | sets time to wait before changing presence status |
|int getStationaryValidTime(); | gets time to wait before changing presence status |
|bool setMotionValidTime(unsigned int t); | sets time to wait before changing motion status |
|int getMotionValidTime(); | gets time to wait before changing motion status |
| bool updateStatus(); | this is the function to be placed in a loop to check for messages and update values |
| bool isPresent(); | returns true for present, false for absent after time of absence delay |
| bool isMoving(); | returns true for motion, false for no motion |

