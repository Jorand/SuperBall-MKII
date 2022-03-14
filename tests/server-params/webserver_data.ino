#include <EEPROM.h>
#include <ELClientWebServer.h>

#define LED_PIN 13

#define CONFIG_VERSION "v01"
#define CONFIG_ADDR 0

struct MyObject {
  float motorP;
  float motorI;
  float motorD;
  int motorSpeed;
  float battVotage;
  char version_of_program[4];
} userData = {
  0.06,
  0.05,
  0.01,
  20,
  0.0,
  CONFIG_VERSION
};

void LoadCb(char * url)
{
  webServer.setArgFloat(F("motor_p"), userData.motorP);
  webServer.setArgFloat(F("motor_i"), userData.motorI);
  webServer.setArgFloat(F("motor_d"), userData.motorD);

  LoadAndRefreshCb(url);
}

void RefreshCb(char * url)
{
  LoadAndRefreshCb(url);
}

void LoadAndRefreshCb(char * url)
{
  if( digitalRead(LED_PIN) )
    webServer.setArgString(F("text"), F("LED is on"));
  else
    webServer.setArgString(F("text"), F("LED is off"));
}

void ButtonPressCb(char * btnId)
{
  String id = btnId;
  if( id == F("btn_on") )
    digitalWrite(LED_PIN, true);
  else if( id == F("btn_off") )
    digitalWrite(LED_PIN, false);
}

void FieldCb(char * fieldId)
{
  String fld = fieldId;

  if(fld == F("motor_p")) {
    userData.motorP = webServer.getArgFloat();
    updateEEPROM();
  }
  else if(fld == F("motor_i")) {
    userData.motorI = webServer.getArgFloat();
    updateEEPROM();
  }
  else if(fld == F("motor_d")) {
    userData.motorD = webServer.getArgFloat();
    updateEEPROM();
  }
}


// initialization
void pidInit()
{
  loadEEPROM();

  URLHandler *pageHandler = webServer.createURLHandler(F("/PID.html.json"));
  pageHandler->loadCb.attach(&LoadCb);
  pageHandler->refreshCb.attach(&RefreshCb);
  pageHandler->buttonCb.attach(&ButtonPressCb);
  pageHandler->setFieldCb.attach(&FieldCb);
}

void pidLoop() {

}


void clearEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void setDefaultEEPROM() {
  clearEEPROM();
  updateEEPROM();
}

void loadEEPROM() {
  // Load settings from EEPROM to custom struct
  if (EEPROM.read(CONFIG_ADDR + sizeof(userData) - 2) == userData.version_of_program[2] &&
      EEPROM.read(CONFIG_ADDR + sizeof(userData) - 3) == userData.version_of_program[1] &&
      EEPROM.read(CONFIG_ADDR + sizeof(userData) - 4) == userData.version_of_program[0])
  {
    EEPROM.get(CONFIG_ADDR, userData);
  } else {
    setDefaultEEPROM();
  }

}

void updateEEPROM() {
  EEPROM.put(CONFIG_ADDR, userData);
}
