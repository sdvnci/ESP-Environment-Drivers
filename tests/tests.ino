#include <AUnit.h>

void setup() {
    #if !defined(EPOXY_DUINO)
        delay(1000);
    #endif
    Serial.begin(115200);
    while (!Serial);
}

void loop() {
    aunit::TestRunner::run();
}
