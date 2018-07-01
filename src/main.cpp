#ifdef ARDUINO

#include <Arduino.h>
#include "LacrosseReceiver.h"

LacrosseReceiver receiver(5); // RF receiver connected to pin 2
uint32_t msec;

void setup() {
    Serial.begin(115200);
    receiver.enableReceive();
    msec = millis();
}

void loop() {
    if (millis() - msec > 1000) {
        msec = millis();
        measure m = receiver.getNextMeasure();
        while (m.type != UNKNOWN) {
            Serial.print("Sensor #");
            Serial.print(m.sensorAddr);
            Serial.print(": ");
            Serial.print(m.units);
            Serial.print(".");
            Serial.print(m.decimals);
            Serial.println((m.type == TEMPERATURE)? " °C" : " %rh");

            m = receiver.getNextMeasure();
        }
    }
}

#else
// This is needed to enable build in native environment
#include <iostream>

int main(int argc, char *argv[]) {
    std::cout << "This environment is for testing purposes only!";
    return 0;
}
#endif // #ifdef ARDUINO
