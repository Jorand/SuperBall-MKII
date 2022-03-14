#include <ELClient.h>
#include <ELClientWebServer.h>

String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete

ELClient esp(&Serial);
ELClientWebServer webServer(&esp);

// Callback made form esp-link to notify that it has just come out of a reset. This means we
// need to initialize it!
void resetCb(void) {
  Serial.println("EL-Client (re-)starting!");
  bool ok = false;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println("EL-Client sync failed!");
  } while(!ok);
  Serial.println("EL-Client synced!");

  webServer.setup();
}

void setup() {
	Serial.begin(115200);

  // Esp-link
  pidInit();

  esp.resetCb = resetCb;
  resetCb();
}

void loop() {
  esp.Process();
}
