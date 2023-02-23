

/*!
 * This is example of using the modules built in scenarios
 *
 * One will need to appropriately modify the second Serial refernce - here it is Serial 2 -
 * in order to make it work on the board you are working with.
 * 
 */

#include <Arduino.h>
#include "liteRadar.h"

// #include <HardwareSerial.h>
// HardwareSerial UART(0);

Radar radar = Radar(&Serial2);

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);

    delay(2000);
    Serial.println("ready");

    byte ret;

    if (!radar.resetRadar()) Serial.println("reset failed");
    delay(1000);

    if (!radar.setScenario(AREA_DETECTION)) Serial.println("set scenario failed");
    delay(1000);
    if (!radar.setSensitivity(0x02)) Serial.println("set sensitivity failed");
    delay(1000);

    if (!radar.resetRadar()) Serial.println("reset failed");
    delay(1000);

    Serial.println("\nsetup done\n\n\n");
}


void loop() {
    bool changed = radar.updateStatus();
    if (changed) {
        Serial.printf("presence is: %s\n", radar.isPresent() ? "true" : "false");
        Serial.printf("motion value is: %s\n", radar.isMoving() ? "true" : "false");
    }
    delay(20);
}