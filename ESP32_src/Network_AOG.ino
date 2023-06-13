#include <stdarg.h>
#include <Update.h>
#include "WiFi.h"
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>


AsyncWebServer server(80);
 
void Send_DEBUG_Data()
{
   
}

void WiFi_Start_AP() 
{
  uint64_t chipid;
  unsigned long timeout;
  
  SPIFFS.begin();

  chipid = ESP.getEfuseMac();
  
  //uint16_t chipid16 = (uint16_t)(chipid >> 32);
  //Serial.printf("chip_serial: %04X-%08X\n", chipid16, (uint32_t)chipid);

  snprintf(ssid, 23, "%s", ssid_ap);
  
  WiFi.mode(WIFI_AP);
  WiFi.setHostname("manual");
  Serial.printf("start wifi AP - %s\n",ssid);
  
  WiFi.softAP(ssid, password_ap, 3,0);//3,true,4); true 4 to siec ukryta
  while (!SYSTEM_EVENT_AP_START) // wait until AP has started
   {
    delay(100);
    //Serial.print(".");
   }
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(gwip, gwip, mask);  // set fix IP for AP
  delay(2000); // VERY IMPORTANT
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("myip: ");
  Serial.println(myIP);

  WiFi.setTxPower(WIFI_POWER_19_5dBm); 
  delay(2000);
  int a = WiFi.getTxPower();
  Serial.print("TX power:");
  Serial.println(a);

/*
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  */

  //tutaj odbywa sie obsługa zapytań
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ //na otrzymane od klienta zapytania pod adresem "/" typu GET, 
    request->send(SPIFFS, "/index.html", "text/html");         //odpowiedz plikiem index.html z SPIFFS (można to zmienić na kartę SD) 
                                                               //zawierającym naszą stronę będącą plikem tekstowym HTML
  });
  
// server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  server.begin();
  
  my_WiFi_Mode = WIFI_AP;
}


//---------------------------------------------------------------------
void UDP_Start()
{

}




//---------------------------------------------------------------------
void Send_UDP()
{
  
}





//---------------------------------------------------------------------


void DebugUDP(char *fmt, ...)
{

}
