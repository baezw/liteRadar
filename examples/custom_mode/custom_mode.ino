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

    Serial.println("\nstarting setup\n\n");

    if (!radar.resetRadar()) Serial.println("reset failed");
    delay(1000);
    if(!radar.openCustomMode(MODE_1)) Serial.println("open custom mode failed");
    if (!radar.setPresenceThreshold(0x1E)) Serial.println("set presence threshold failed");
    if (!radar.setMotionThreshold(0x0E)) Serial.println("set presence threshold failed");
    if(!radar.exitCustomMode()) Serial.println("exit custom mode failed");

    if (!radar.resetRadar()) Serial.println("reset failed");

    Serial.println("\nsetup done\n\n\n");
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