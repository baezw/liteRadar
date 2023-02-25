#include <Arduino.h>
#include "liteRadar.h"

#include <HardwareSerial.h>
HardwareSerial UART(0);

#define LED_PIN 10

Radar radar = Radar(&UART);

void setup() {
    Serial.begin(115200);
    UART.begin(115200, SERIAL_8N1, D7, D6);

    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(3000);
    Serial.println("ready");

    byte ret;

    Serial.println("\nstarting setup\n");

    if (!radar.resetRadar()) Serial.println("reset failed");
    else Serial.println("reset succeeds");
    delay(2000);

// test setting parameters for built-in models //

    if (!radar.setScenario(BATHROOM)) Serial.println("set scenario failed");
    else Serial.println("set scenario succeeds");
    Serial.printf("current scenario = %02X\n", radar.getScenario());

    if (!radar.setSensitivity(0x02)) Serial.println("set sensitivity failed");
    else Serial.println("set sensitivity succeeds");
    Serial.printf("current sensitivity = %02X\n", radar.getSensitivity());

    if (!radar.setTimeOfAbsence(0x01)) Serial.println("set time of absence failed");
    else Serial.println("set time of absence succeeds");
    Serial.printf("current time of absence = %02X\n", radar.getTimeOfAbsence());

// test setting parameters for custom modes //

    if(!radar.openCustomMode(MODE_1)) Serial.println("open custom mode failed");
    else Serial.println("open custom mode succeeds");

    if (!radar.setPresenceThreshold(0x1E)) Serial.println("set presence threshold failed");
    else Serial.println("set presence threshold succeeds");
    Serial.printf("current presence threshold = %02X\n", radar.getPresenceThreshold());

    if (!radar.setPresenceRange(0x09)) Serial.println("set presence range failed");
    else Serial.println("set presence range succeeds");
    Serial.printf("current presence range = %02X\n", radar.getPresenceRange());

    if (!radar.setMotionThreshold(0x0E)) Serial.println("set presence threshold failed");
    else Serial.println("set motion threshold succeeds");
    Serial.printf("current motion threshold = %02X\n", radar.getMotionThreshold());

    if (!radar.setMotionRange(0x09)) Serial.println("set motion range failed");
    else Serial.println("set motion range succeeds");
    Serial.printf("current motion range = %02X\n", radar.getMotionRange());

    if(!radar.exitCustomMode()) Serial.println("exit custom mode failed");
    else Serial.println("exit custom mode succeeds");

    if (!radar.resetRadar()) Serial.println("reset failed");
    else Serial.println("reset succeeds");

    Serial.println("\nfinished setup\n");

}


void loop() {
    bool changed = radar.updateStatus();
    if (changed) {
        Serial.printf("presence is: %s\n", radar.isPresent() ? "true" : "false");
        Serial.printf("motion value is: %s\n", radar.isMoving() ? "true" : "false");
        digitalWrite(LED_PIN, radar.isPresent());
    }
    delay(20);
}