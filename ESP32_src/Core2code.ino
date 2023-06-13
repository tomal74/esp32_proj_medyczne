//Core2: this task only serves the Webpage
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>


WebServer server(80);

void TakeEncoData(void)
{
  
  // Paweł Nowak 2023 BEGIN

    int32_t newPositionRight = encoder_right.getCount();
    int32_t newPositionLeft  = encoder_left.getCount();
    //Serial.printf("L:%d  R:%d\n",newPositionLeft, newPositionRight);
    currentPositionRight = newPositionRight * 360.0 / 2500.0; // Przeliczenie pozycji na stopnie dla prawego silnika
    currentPositionLeft = newPositionLeft * 360.0 / 2500.0; // Przeliczenie pozycji na stopnie dla lewego silnika
  
    velocityRight = (currentPositionRight - oldPositionRight) / (dT/1000.0); // Obliczanie prędkości (pierwsza pochodna) dla prawego silnika
    velocityLeft = (currentPositionLeft - oldPositionLeft) / (dT/1000.0); // Obliczanie prędkości (pierwsza pochodna) dla lewego silnika

    accelerationRight = (velocityRight - oldVelocityRight) / dT; // Obliczanie przyspieszenia (druga pochodna) dla prawego silnika
    accelerationLeft = (velocityLeft - oldVelocityLeft) / dT; // Obliczanie przyspieszenia (druga pochodna) dla lewego silnika

    oldPositionRight = currentPositionRight;
    oldPositionLeft = currentPositionLeft;
    oldVelocityRight = velocityRight;
    oldVelocityLeft = velocityLeft;
/*    Serial.println("Prawy::");
    Serial.println("pozycja:");
    Serial.println(currentPositionRight);
   */
    //Serial.printf("predkosc: %5.2f\n", velocityRight);
    // Serial.println("przyspieszenie:");
    //  Serial.println(accelerationRight);
    //Serial.printf("Prawy:: pozycja: %f, predkosc: %f, przyspieszenie: %f \n", currentPositionRight, velocityRight, accelerationRight );
    //Serial.printf("Lewy:: pozycja: %f, predkosc: %f, przyspieszenie: %f \n", currentPositionLeft, velocityLeft, accelerationLeft );

    // Paweł Nowak 2023 END
}

void Core2code( void * pvParameters )
{
  static unsigned long timeoutMelodia=0;
  
  Serial.println();
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  OTA_Activ = false;
  
// Start WiFi Client
 while (!EE_done){  // wait for eeprom data
  delay(10);
 }

  //staruje AP
  WiFi_Start_AP();
  UDP_Start();
 
  pinMode(ENCODER_LEFT_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN2, INPUT_PULLUP);

  encoder_right.attachSingleEdge(ENCODER_RIGHT_PIN1,ENCODER_RIGHT_PIN2);
  encoder_left.attachSingleEdge(ENCODER_LEFT_PIN1,ENCODER_LEFT_PIN2);

  for(;;) 
  {
    esp_task_wdt_reset();
    vTaskDelay(1);
    
    //Serial2_Traffic();
    vTaskDelay(1);
    Serial_Traffic(); //data from easyAGRO
    //Serial_Driver(); //data to easyAGRO
    vTaskDelay(1);
    
    
    if(OTA_Activ==true) 
    {
      noInterrupts();
      server.handleClient();
    }

    Serial2_Traffic();
    vTaskDelay(1);
    
    //Serial_Driver();
    vTaskDelay(1);
    
  }//od for(;;)
}


/////////////////////////////////
//////////////serial ////
////////////////////////////////
char s2_c;
int s2_len,s2_idx;
int s2_state=0;
byte s2_type;
byte s2_buf[1024];
byte s2_checksum;

void Serial_Traffic()
{
int i;
byte check_xor;
static unsigned long last_t = 0;
unsigned long tt;
static int Serial1Tryier=0;
uint8_t dataRAWfromFIFO=0xff;

if(++Serial1Tryier>500)
{
  //Serial.println("Serial1 - nic nie przyszlo dluzszy czas\n");
  Serial1Tryier-=300;
  
  //dataRAWfromFIFO = (uint8_t)((*(volatile uint32_t*)(0x3FF50000 + 0x1C))&0xFF);
  //Serial.printf("RX_FIFO CNT REG = %d\n",dataRAWfromFIFO);
}

//dataRAWfromFIFO = (uint8_t)((*(volatile uint32_t*)(0x3FF50000 + 0x1C))&0xFF);
//dataRAWfromFIFO==0? : Serial.printf("RX_FIFO CNT REG = %d, s2_state = %d\n",dataRAWfromFIFO, s2_state);

// 0xAA 0x55 0x00 0x02 0x1F 0x00 0x01 0x01 0xXX
// aa    55  lenH lenL type    DATA------...XOR 
while (Serial1.available())
{
   Serial1Tryier=0;
   s2_c = Serial1.read();
    //Serial.print(s2_c,HEX); 

   switch(s2_state)
   {
    case 0:
          if(s2_c==0xAA) s2_state = 1;
          //else { Serial.print(s2_c,HEX);}
      break;
    case 1:
          if(s2_c==0x55) s2_state = 2;
          else
          {
            if(s2_c!=0xAA)
            {
              s2_state=0;
              Serial.println("parser reset");
              rs232errors++;
            }
          }
      break;
    case 2:
          s2_len = (s2_c<<8);
          s2_state = 3;
      break;
    case 3:
          s2_len |= s2_c;
          if(s2_len > 512) 
          {
            s2_len = 0;
            s2_state = 0;
          }
          else s2_state = 4;
      break;
    case 4:
          s2_type = s2_c;
          s2_state = 5;
          s2_idx=0;
      break;
    case 5: //czytamy cala zawartosc data
          s2_buf[s2_idx] = s2_c;
          if(++s2_idx == s2_len) s2_state = 6;
      break;
    case 6: //ostatni bajt to checksuma
          s2_checksum = s2_c;
           //liczmy checksum xor
          check_xor = 0;
          for(i=0;i<s2_len;i++) check_xor ^= s2_buf[i];
          if(check_xor == s2_checksum)
          {
            switch(s2_type)
            {
              /*
               *   
                            easySteerPingFrame = 0,
                            easySteerDataFrame = 1,
                            easySteerSettingsFrame = 2,
                            ffree = 3,
                            correctionFrame = 4,
                            GPSFrame = 5,
                            IMUFrame = 6,
                            InfoFrame = 7,
                            GPS2Frame = 8,
                            GPSHeadingFrame = 9,
                            GPSFrameCPX = 10,
                            free
                            free
                            free
                            free
                            free
                            free
                            easySteerSettingExtendedFrame = 22;
               */
              case 0:
              case 3:
              case 5:
              case 6:
              case 7:
                  // sterowanie silnikami
                  Steer.watchdogTimer = 0;
                  MOTOR_L.STOP = 0;
                  MOTOR_R.STOP = 0;
                  /*
                  Serial.print("--> ");
                  for(int i=0; i<s2_len; i++)
                  {
                    Serial.print( s2_buf[i] );
                    Serial.print( " " );
                  }
                  Serial.println(" <--");
                  */

                  memcpy( (char*)&motors_data_from_matlab, s2_buf, sizeof(motors_data_from_matlab) );   //memcpy() function


                  
                  /*
                  Serial.print("frame_cnt      =");   Serial.println(motors_data_from_matlab.frame_cnt);
                  Serial.print("l_motor_cmd    =");   Serial.println(motors_data_from_matlab.l_motor_cmd);
                  Serial.print("l_motor_current=");   Serial.println(motors_data_from_matlab.l_motor_current);

                  Serial.print("r_motor_cmd    =");   Serial.println(motors_data_from_matlab.r_motor_cmd);
                  Serial.print("r_motor_current=");   Serial.println(motors_data_from_matlab.r_motor_current);
                  */
                  break;
              case 8:
              case 9:
              case 10:
                Serial.printf("RS232 other frame len=%d\n\r",s2_len+6);
                break;
              case 1:                
                  tt = millis();
                  if(tt-last_t > 500) Serial.printf("data frame time > 500ms = %ums\n\r", tt-last_t);
                  last_t = tt;
                  DataPacket(s2_buf, s2_len);
                break;
              case 2:
                  //Serial.println("sett type");
                  //SettPacket(s2_buf, s2_len);
                break;
              case 4:
                  //Serial.printf("corr type len=%d\n\r",s2_len+6);

                  for(i=0;i<s2_len;i++) Serial2.write(s2_buf[i]); //ntrip packet to GPSo on RS232
                  if(udp3!=NULL)
                  {
                    udp3num6633_2++;
                    if (udp6num6650>0)   udp3.writeTo(s2_buf, s2_len, ipDestGPSo,  portNTRIP );    // routujemy to easyGPSo :-)
                    if (udp6num6650_2>0) udp3.writeTo(s2_buf, s2_len, ipDestGPSo2, portNTRIP );    // routujemy to easyGPSo :-)
                  }
                  else Serial.print("some problem with udp3\n\r");
                break;


             case 22: //easySteerSettingExtendedFrame
                  esp_task_wdt_reset();
                  Serial.println("sett ext type");
                  SettExtPacket(s2_buf, s2_len); 
                break;
             case 30: //start OTA
                  OTAStartPacket(s2_buf, s2_len);
                break;
             case 33: //ramka easyFrame do sterownika easyFrame (easyAGRO) dla ramki hydrauicznej
                  if(udp1!=NULL)
                  {              
                    if (udp6num6650_2>0) udp1.writeTo(s2_buf, s2_len, ipDestGPSo2, portEasyFrame );    // routujemy to do sterownika nr2
                  }
                break;
            }
          }
          else
          {
            Serial.printf("Serial1 i've frame len %d - but incorrect\n\r",s2_len);
            //for(int j=0;j<s2_len;j++) Serial.printf("%02X ",s2_buf[j]);
            //Serial.println();
            
            
            rs232errors++;
          }
          s2_state = 0;
          break;
       default:
          s2_state = 0;
          break;
   }
}//od while
}  




char ss_c;
int ss_len,ss_idx;
int ss_state=0;
byte ss_type;
byte ss_buf[1024];
byte ss_checksum;

void Serial2_Traffic()
{
int i;
byte check_xor;
static unsigned long last_t = 0;
unsigned long tt;
static int Serial1Tryier2=0;

if(++Serial1Tryier2>500)
{
  //Serial.println("Serial2 - nic nie przyszlo dluzszy czas\n");
  Serial1Tryier2-=300;
  
  //dataRAWfromFIFO = (uint8_t)((*(volatile uint32_t*)(0x3FF50000 + 0x1C))&0xFF);
  //Serial.printf("RX_FIFO CNT REG = %d\n",dataRAWfromFIFO);
}


while (Serial2.available())
{
   ss_c = Serial2.read();
   //Serial.print(ss_c,HEX); 
   Serial1Tryier2 = 0;
   switch(ss_state)
   {
    case 0:
          if(ss_c==0xAA) ss_state = 1;
          //else { Serial.print(s2_c,HEX);}
      break;
    case 1:
          if(ss_c==0x55) ss_state = 2;
          else
          {
            if(ss_c!=0xAA)
            {
              ss_state=0;
              Serial.println("parser reset");
              rs232errors++;
            }
          }
      break;
    case 2:
          ss_len = (ss_c<<8);
          ss_state = 3;
      break;
    case 3:
          ss_len |= ss_c;
          if(ss_len > 1024) 
          {
            ss_len = 0;
            ss_state = 0;
          }
          else ss_state = 4;
      break;
    case 4:
          ss_type = ss_c;
          ss_state = 5;
          ss_idx=0;
      break;
    case 5: //czytamy cala zawartosc data
          ss_buf[ss_idx] = ss_c;
          if(++ss_idx == ss_len) ss_state = 6;
      break;
    case 6: //ostatni bajt to checksuma
          ss_checksum = ss_c;
           //liczmy checksum xor
          check_xor = 0;
          for(i=0;i<ss_len;i++) check_xor ^= ss_buf[i];
          if(check_xor == ss_checksum)
          {
            switch(ss_type)
            {
              /*
               *   
                            easySteerPingFrame = 0,
                            easySteerDataFrame = 1,
                            easySteerSettingsFrame = 2,
                            ffree = 3,
                            correctionFrame = 4,
                            GPSFrame = 5,
                            IMUFrame = 6,
                            InfoFrame = 7,
                            GPS2Frame = 8,
                            GPSHeadingFrame = 9,
                            GPSFrameCPX = 10,
                            free
                            free
                            free
                            free
                            free
                            free
                            easySteerSettingExtendedFrame = 22;
               */
              default:
              
                Serial.printf("RS232 other frame len=%d\n\r",s2_len+6);
                break;

              
              case 10:       
                  
                      //debug
                      //Serial.printf("CPX: %d\n", ss_len);   
                      //for(i=0;i<10;i++) Serial.printf("%02X ",*(ss_buf+i));
                      //Serial.printf("-\n");      
                  //odebralismy na rsie z anteny paczke CPX wysylamy do tabletu
                 // RS232Send(ss_buf, ss_len, 10, 1);  
                break;
            }
          }
          else
          {
            Serial.print("Serial2 i've frame - but incorrect\n\r");
            rs232errors++;
          }
          ss_state = 0;
          break;
       default:
          ss_state = 0;
          break;
   }
}//od while
}  


////////////////////////////
///////////////////////////
///////////////////////////////
void OTAStartPacket(byte* buf, int len)
{
  if ( *(buf+0) == 0x7F && *(buf+1) == 0xE0) //startOTA Packet
  {
    OTA_Activ = true;
    Steer.steerEnable     = false;
    Steer.DiagnosticMode  = false;
    Steer.Permition       = false;
  
    Serial.print("start OTA packet recv\n\r");
  }
}

void DataPacket(byte* buf, int len)
{
unsigned char toGPSo[7];

  if ( *(buf+0) == 0x7F && *(buf+1) == 0xFE) //Data Packet
  {
    Steer.watchdogTimer = 0;
  
    Steer.oldPermition        = Steer.Permition;          
    Steer.oldDiagnosticMode   = Steer.DiagnosticMode;
    Steer.oldDistanceFromLine = Steer.DistanceFromLine;
   
   
   Steer.relay = *(buf+2);           // read relay control from easyAGRO   

   /*if(Steer.relay != *(buf+2))
   {
    Steer.relay = *(buf+2);           // read relay control from easyAGRO   
    Serial.printf("Relay change to: x%02X\n", Steer.relay);
   }*/
   
   Steer.speedf = 0.25 * (float)*(buf+3);     //actual speed times 4, single byte

   //distance from the guidance line in mm
   Steer.idistanceFromLine = (*(buf+4) << 8 | *(buf+5));   //high,low bytes     
   Steer.DistanceFromLine = (float)Steer.idistanceFromLine;

   //set point steer angle * 10 is sent
   Steer.isteerAngleSetPoint = ((*(buf+6) << 8 | *(buf+7))); //high low bytes 
   Steer.AngleSetPoint = (float)Steer.isteerAngleSetPoint * 0.01;  

   //8 - youturn byte
    
   //9 - debug byte
   if(*(buf+9)&0x01)
   {
    if(Steer.oldDiagnosticMode == false)
    {
      Steer.steerEnable     = false;
      Steer.DiagnosticMode  = true;
      Steer.Permition       = true;
      if(*(buf+9)&0x02)
           Steer.DiagnosticModeType    = 2; //olways set pwmmin   
      else Steer.DiagnosticModeType    = 1; //normal diagnostic
    }
   }
   else
   {
    Steer.DiagnosticModeType    = 0;
    if(Steer.oldDiagnosticMode == true)
    {
      Steer.steerEnable     = false;
      Steer.DiagnosticMode  = false;
      Steer.Permition       = false;        
    }
   }
   
  //auto Steer is off if 32020,Speed is too slow, Wheelencoder above Max
  
  if (Steer.DistanceFromLine == 32020)
  { 
      Steer.steerEnable     = false;
      Steer.Permition  = false;
  }
  else          //valid conditions to turn on autosteer
  {
      //nie wlaczam automatycznie, czekam na przycisk kierowcy
      Steer.Permition = true;
  }
  if (Steer.Permition != Steer.oldPermition) 
  {
    if(Steer.Permition==true)  
    {
      if(Steer.DiagnosticMode == true)
      {
        lastInfo = INFO_DIAGNOSTIC_PERMITION;
        Serial.println("Steer Permition ON - diagnostic mode");
        DebugUDP("Steer Permition ON - diagnostic mode");
      }
      else  
      {
        Serial.println("Steer Permition ON");
        DebugUDP("Steer Permition ON");
      }
    }
    else                            
    {
      lastInfo = INFO_AUTO_OFF_PERMITION_OFF;
      Serial.println("Steer-Break:  Steer Permition OFF");
      DebugUDP("Steer-Break:  Steer Permition OFF");
    }
  }


  
  
UDP_data_time = millis();
 }
  
}

///////////////////////
///////////////////////
///////////////////////


void SettExtPacket(byte* buf, int len)
{
int x=0;
uint8_t funkcje=0;
 
  if ( *(buf+0) == 0x7F && *(buf+1) == 0xF2)
     {
      steerSettings.Kp = (float)(*(buf+2)) * 0.1;   // read Kp from easyAGRO
      steerSettings.Ki = (float)(*(buf+3)) * 0.1;   // read Ki from easyAGRO
      steerSettings.Kd = (float)(*(buf+4)) * 0.1;   // read Kd from easyAGRO
      steerSettings.Ko = (float)(*(buf+5)) * 0.1;   // read Ko from easyAGRO
      steerSettings.maxI = (float)(*(buf+6)) * 0.1; // 0.0 - 2.55

      steerSettings.PWMProgramNumber = (int)(*(buf+7));  // 0 - 100
      

      steerSettings.cutOffPressure = *(buf+8); //0- wylaczony, od 10-200bar
      steerSettings.clutchDuty = *(buf+9); // procentach
      
      x = (*(buf+10))*10; //in hertz
      if(x != steerSettings.freqPWM)
      {
        steerSettings.freqPWM = x;

        //reconfigure pwm's freq
        ledcSetup(0,steerSettings.freqPWM,8);  // PWM Output with channel 0, 1kHz, 8-bit resolution (0-255)
        ledcSetup(1,steerSettings.freqPWM,8);  // PWM Output with channel 1, 1kHz, 8-bit resolution (0-255)
        ledcSetup(2,steerSettings.freqPWM,8);  // PWM Output with channel 1, 1kHz, 8-bit resolution (0-255)
        ledcSetup(3,steerSettings.freqPWM,8);

        //re attach

        
        
        /*ledcAttachPin(PWM_Pin,0);  // attach PWM PIN to Channel 0
        ledcAttachPin(DIR_Pin,1);  // attach PWM PIN to Channel 1
        ledcAttachPin(P8_BUZZ_Pin,2);  // attach PWM PIN to Channel 2
        ledcAttachPin(P7_CLUTCH_Pin,3); //attach PWM PIN to Channel 3
     */
     
     }
       
      steerSettings.minPWML = *(buf+11); //read the minimum amount of PWM for Left
      steerSettings.minPWMR = *(buf+12); //read the minimum amount of PWM for Right
      steerSettings.lowmaxPWML = *(buf+13);
      steerSettings.lowmaxPWMR = *(buf+14);
      steerSettings.himaxPWML = *(buf+15);
      steerSettings.himaxPWMR = *(buf+16);
      steerSettings.currentCutOff = (*(buf+17)+150)*100;

      steerSettings.wasMaxLeft    = (*(buf+18)<<8)|(*(buf+19));
      steerSettings.was20degLeft  = (*(buf+20)<<8)|(*(buf+21));
      steerSettings.was5degLeft   = (*(buf+22)<<8)|(*(buf+23));
      steerSettings.wasZero       = (*(buf+24)<<8)|(*(buf+25));
      steerSettings.was5degRight  = (*(buf+26)<<8)|(*(buf+27));
      steerSettings.was20degRight = (*(buf+28)<<8)|(*(buf+29));
      steerSettings.wasMaxRight   = (*(buf+30)<<8)|(*(buf+31));
      steerSettings.maxDegLeft    = *(buf+32);
      steerSettings.maxDegRight   = *(buf+33);
      steerSettings.modDITHER     = *(buf+34);
      if(steerSettings.modDITHER > 1) steerSettings.modDITHER = 1;  //
      steerSettings.nonZero       = *(buf+35);
      if(steerSettings.nonZero > 60) steerSettings.nonZero = 0;

      steerSettings.KDeadBand = (float)(*(buf+36)) * 0.01;   // read KdeadBand from easyAGRO

      funkcje = *(buf+37);
      
      if(funkcje&0x01) steerSettings.WASDIFF = true;
      else steerSettings.WASDIFF = false;
      
      //wolne 38,39

      //while(SPILOCK) {};

      
      esp_task_wdt_reset();
 
      //Access your resource here.
      EEprom_write_all();
      

      //dither test
      if(steerSettings.modDITHER>0)    
      {
        DEBUG_PRINT("\n------------------------- DITHER is ON -----------------------------\n");
        ledcWrite(0,127);
      }
      else   
      {
        DEBUG_PRINT("\n------------------------- DITHER is OFF -----------------------------\n");
        ledcWrite(0,0);
      }

        

      // for PWM High to Low interpolator
      Steer.LowMinPerDegL = (steerSettings.lowmaxPWML - steerSettings.minPWML) / 5.0;
      Steer.LowMinPerDegR = (steerSettings.lowmaxPWMR - steerSettings.minPWMR) / 5.0;
      Steer.highLowPerDegL = (steerSettings.himaxPWML - steerSettings.lowmaxPWML) / 5.0;
      Steer.highLowPerDegR = (steerSettings.himaxPWMR - steerSettings.lowmaxPWMR) / 5.0;
      

      Print_Settings();
    }
   /* else 
    {
      Serial.printf("err packet\n");
      for(int i=0;i<len;i++) Serial.printf("%02X ",*(buf+i));
      Serial.printf("\n");
    }*/

}


void Print_Settings()
{
      esp_task_wdt_reset();
      Serial.printf("Kp=%3.1f  Ki=%3.2f  Kd=%3.1f  Ko=%3.1f  mxI=%3.1f\n", steerSettings.Kp,steerSettings.Ki, \
                                                                         steerSettings.Kd,steerSettings.Ko, steerSettings.maxI);
      Serial.printf("himaxPWML=%d  lowmaxPWML=%d  minPWML=%d  | minPWMR=%d  lowmaxPWMR=%d  himaxPWMR=%d\n",steerSettings.himaxPWML, steerSettings.lowmaxPWML, steerSettings.minPWML, steerSettings.minPWMR, steerSettings.lowmaxPWMR, steerSettings.himaxPWMR);
      Serial.printf("CutOFF=%d  ClutchDuty=%d  PWMfreq=%d cutOffPressure=%d\n",steerSettings.currentCutOff, steerSettings.clutchDuty, steerSettings.freqPWM, steerSettings.cutOffPressure); 
      Serial.printf("modDITHER=%d nonZero=%d\n",steerSettings.modDITHER, steerSettings.nonZero);
      //Serial.printf("SPZ=%d  WASReverse=%s\n",steerSettings.steerPositionZero, (steerSettings.Invert_WAS==true)?"true":"false");
      Serial.printf("WAS Diff? : %s", (steerSettings.WASDIFF==true)?"true":"false");
      Serial.printf("       DeadBand : %3.2f\n\n", steerSettings.KDeadBand);
      esp_task_wdt_reset();
}
