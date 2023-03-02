#include <Arduino.h>
#include "liteRadar.h"

#include <HardwareSerial.h>
HardwareSerial UART(0);
Radar radar = Radar(&UART);

#define LED_PIN 14

void setup() {
    char buff[40];

    Serial.begin(115200);
    UART.begin(115200, SERIAL_8N1, RX, TX);

    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(5000);
    Serial.println("ready");

    byte ret;

    Serial.println("\nstarting setup\n");

    if (!radar.resetRadar()) {
        Serial.println("reset failed, trying again");
        if(!radar.resetRadar()) {
            Serial.println("failed again");
        }
        else Serial.println("reset succeeds");
    }
    else Serial.println("reset succeeds");
    delay(1000);

    // always need to turn off underlying data to be sure.

    if (!radar.setUnderlying(0x00)) Serial.println("set underling failed");
    else Serial.println("set underlying succeeds");
    sprintf(buff, "underlying status = %02X\n", radar.getUnderlying());
    Serial.print(buff);

// test setting parameters for built-in models //

    if (!radar.setScenario(BATHROOM)) Serial.println("set scenario failed");
    else Serial.println("set senario succeeds");
    sprintf(buff, "current scenario = %02X\n", radar.getScenario());
    Serial.print(buff);

    if (!radar.setSensitivity(0x02)) Serial.println("set sensitivity failed");
    else Serial.println("set sensitivity succeeds");
    sprintf(buff, "current sensitivity = %02X\n", radar.getSensitivity());
    Serial.print(buff);

    if (!radar.setTimeOfAbsence(0x01)) Serial.println("set time of absence failed");
    else Serial.println("set time of absence succeeds");
    sprintf(buff, "current time of absence = %02X\n", radar.getTimeOfAbsence());
    Serial.print(buff);

// test setting parameters for custom modes //

    if(!radar.openCustomMode(MODE_1)) Serial.println("open custom mode failed");
    else Serial.println("open custom mode succeeds");

    if (!radar.setPresenceThreshold((unsigned int)0x10)) Serial.println("set presence threshold failed");
    else Serial.println("set presence threshold succeeds");
    sprintf(buff, "current presence threshold = %02X\n", radar.getPresenceThreshold());
    Serial.print(buff);

    if (!radar.setPresenceRange((unsigned int)0x0A)) Serial.println("set presence range failed");
    else Serial.println("set presence range succeeds");
    sprintf(buff, "current presence range = %02X\n", radar.getPresenceRange());
    Serial.print(buff);
    
    if (!radar.setMotionThreshold(0x08)) Serial.println("set presence threshold failed");
    else Serial.println("set motion threshold succeeds");
    sprintf(buff, "current motion threshold = %02X\n", radar.getMotionThreshold());
    Serial.print(buff);

    if (!radar.setMotionRange(0x0A)) Serial.println("set motion range failed");
    else Serial.println("set motion range succeeds");
    sprintf(buff, "current motion range = %02X\n", radar.getMotionRange());
    Serial.print(buff);

    if (!radar.setMotionValidTime(3000)) Serial.println("set motion valid time failed");
    else Serial.println("set motion valid time succeeds");
    sprintf(buff, "current motion valid time = %d\n", radar.getMotionValidTime());
    Serial.print(buff);

    if (!radar.setStationaryValidTime(3000)) Serial.println("set stationary valid time failed");
    else Serial.println("set stationary valid time succeeds");
    sprintf(buff, "current stationary valid time = %d\n", radar.getStationaryValidTime());
    Serial.print(buff);

    if (!radar.setAbsenceValidTime(10000)) Serial.println("set absence valid time failed");
    else Serial.println("set absence valid time succeeds");
    sprintf(buff, "current absence valid time = %d\n", radar.getAbsenceValidTime());
    Serial.print(buff);


    if(!radar.exitCustomMode()) Serial.println("exit custom mode failed");
    else Serial.println("exit custom mode succeeds");

    if (!radar.resetRadar()) Serial.println("reset failed");
    else Serial.println("reset succeeds");

    Serial.println("\nfinished setup\n");

}


void loop() {
    char buff[40];
    bool changed = radar.updateStatus();
    if (changed) {
        sprintf(buff, "presence is: %s\n", radar.isPresent() ? "true" : "false");
        Serial.print(buff);
        sprintf(buff, "motion value is: %s\n", radar.isMoving() ? "true" : "false");
        Serial.print(buff);
        digitalWrite(LED_PIN, radar.isPresent());
    }
    delay(20);
}