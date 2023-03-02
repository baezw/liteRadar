#include <Arduino.h>
#include "liteRadar.h"

#include <HardwareSerial.h>
HardwareSerial UART(0);
Radar radar = Radar(&UART);

#define LED_PIN 10

void setup() {
    char buff[40];

    Serial.begin(115200);
    UART.begin(115200, SERIAL_8N1, D7, D6);

    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(10000);
    Serial.println("ready");

    byte ret;

    Serial.println("\nstarting setup\n");

    if (!radar.resetRadar()) Serial.println("reset failed");
    else Serial.println("reset succeeds");
    delay(1000);


    // always need to turn off underlying data to be sure.

    if (!radar.setUnderlying(0x01)) Serial.println("set underlying failed");
    else Serial.println("set underlying succeeds");
    sprintf(buff, "underlying status = %02X\n", radar.getUnderlying());
    Serial.print(buff);

    radar.streamFrames(20000);
    Serial.println("\nfinished setup\n");

    if (!radar.setUnderlying(0x00)) Serial.println("set underlying failed");
    else Serial.println("set underlying succeeds");
    sprintf(buff, "underlying status = %02X\n", radar.getUnderlying());
    Serial.print(buff);
}


void loop() {
}