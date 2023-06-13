#include <stdarg.h>
#include <Update.h>
#include "WiFiGeneric.h"

 

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>agroOSA login</b></font></center>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='agroOSA' && form.pwd.value=='ja tu prowadze!')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";


void Send_DEBUG_Data()
{
   
}

void WiFi_Start_AP() 
{
uint64_t chipid;
unsigned long timeout;

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

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });

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
