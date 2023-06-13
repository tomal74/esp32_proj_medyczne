//Core1:  Autosteer Code
#include <ESP32Encoder.h>

#define INFO_AUTO_OFF_SPEED 1
#define INFO_AUTO_OFF_DRIVER_WHEEL 2
#define INFO_AUTO_OFF_DRIVER_BUTTON 3
#define INFO_AUTO_OFF_LOST_COMM 4
#define INFO_AUTO_OFF_PERMITION_OFF 5

#define INFO_AUTO_ON 10

#define INFO_CANT_AUTO_SPEED 11
#define INFO_CANT_AUTO_LINE 12
#define INFO_CANT_AUTO_LOST_COMM 13
#define INFO_CANT_AUTO_NO_PERMITION 14

#define INFO_DIAGNOSTIC_PERMITION 15
#define INFO_DIAGNOSTIC_AUTO_ON 16
#define INFO_AUTO_OFF_MOTOR_CURRENT 17
#define INFO_DIAGNOSTIC_AUTO_OFF 18

//encodery Pawel robil

#define ENCODER_RIGHT_PIN1 39 // Podłącz enkoder pin A do GPIO 2
#define ENCODER_RIGHT_PIN2 36 // Podłącz enkoder pin B do GPIO 3

#define ENCODER_LEFT_PIN1 34 // Podłącz enkoder pin A do GPIO 2
#define ENCODER_LEFT_PIN2 35 // Podłącz enkoder pin B do GPIO 3

float oldPositionRight = 0.0;
float oldPositionLeft = 0.0;
float currentPositionRight = 0.0;
float currentPositionLeft = 0.0;
float velocityRight = 0.0;
float velocityLeft = 0.0;
float oldVelocityRight = 0.0;
float oldVelocityLeft = 0.0;
float accelerationRight = 0.0;
float accelerationLeft = 0.0;

ESP32Encoder encoder_right;
ESP32Encoder encoder_left;

//tu pawel skonczyl robic 

void Core1Setup() {
  Serial.println();
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  // Waiting for WiFi Access
  while ((my_WiFi_Mode == 0)) {
    esp_task_wdt_reset();
    //Serial.print(my_WiFi_Mode);
    Serial.println(" Waiting for WiFi Access\n");
    delay(4000);
  }
}

void resetDiagnostic() {
  starttime = millis();
}


#define SWAP16(value) ((0x00ff & value) << 8) | ((0xff00 & value) >> 8)

#define SWAP32(value) ((0x000000ff & value) << 24) | ((0x0000ff00 & value) << 8) | ((0x00ff0000 & value) >> 8) | ((0xff000000 & value) >> 24)

void sendDiagnostic() {
  DebugFrameType2.cnt++;
  DebugFrameType2.timestamp = SWAP32((millis() - starttime));

  DebugFrameType2.setpoint = SWAP16((int16_t)(Steer.AngleSetPoint * 100.0));
  DebugFrameType2.was = SWAP16((int16_t)(Steer.AngleActual * 100.0));
  DebugFrameType2.was_cnt = was_cnt;
  DebugFrameType2.current = SWAP16(Steer.pwmDrive * 10);  //ze znakiem

  if (Steer.DiagnosticMode == true) DebugFrameType2.speeding = (uint8_t)(Steer.speedf * 4) - 8;
  else DebugFrameType2.speeding = (uint8_t)(Steer.speedf * 4);
  DebugFrameType2.delimeter = 0xAA;
}


void Core1code() {
  static unsigned long lastT = 0, lastT2 = 0;
  static int numer = 0;
  int16_t tip;
  uint16_t tip1[16];
  uint16_t i;
  int16_t ax, ay, az;
  uint8_t CalibState;
  uint8_t showcal = 90;

  uint16_t looper1 = 0;


  esp_task_wdt_reset();
  vTaskDelay(2);  //nizej jeszcze 3, razem 5ms


  if (do_reset_BD62220 == true) {
    do_reset_BD62220 = false;
    Serial.println("FAIL OCCUR on BD62220 - reset 100ms");
    DebugUDP("FAIL OCCUR on BD62220 - reset 100ms");
    delay(100);
    digitalWrite(PS_RESET_Pin, HIGH);
    delay(10);
    attachInterrupt(digitalPinToInterrupt(FAIL_Pin), FAIL_ISR, FALLING);
  }



  int am, bm, cm, dm, em, fm, gm, hm;

  //chcilbym aby PWM process lecial conajmniej 50Hz czyli co 20ms, pomiar WAS damy na 64SPS czyli 15ms, a pomiar reszty walniemy szybkim 475SPS (2ms *2)


  enum {
    STOP = 0u,
    REVERSE = 1u,
    FORWARD = 2u,
    BRAKE = 3u
  };

  if (++core0run > 1000) {
    core0run = 0;
    DEBUG_PRINT(String("core1code @") + String(xPortGetCoreID()) + String(" is running"));
    DEBUG_PRINT(String("Free heap: ") + String(ESP.getFreeHeap()) + String("\n"));
    //lastT = t;

    //Serial.printf("%3.1f packets/s recv to me\n",udp1num/15.0);
    //Serial.printf("%3.1f packets/s routed from port 6620\n",udp2num6620/15.0);
    //Serial.printf("%3.1f packets/s routed from port 6630\n",udp2num6630/15.0);
    //Serial.printf("%3.1f packets/s routed from port 6640\n",udp2num6640/15.0);
    //Serial.printf("%3.1f packets/s routed from port 6633\n",udp3num6633/15.0);
    //Serial.printf("%3.1f packets/s routed from port 6650\n",udp6num6650/15.0);
    //Serial.printf("%d errors in RS232 incomming packets\n\n",rs232errors);

    DebugUDP("6620: %d  6630: %d  6640: %d  6633: %d  6633_2: %d  6650_1: %d  6650_2: %d", udp2num6620, udp2num6630, udp2num6640, udp3num6633, udp3num6633_2, udp6num6650, udp6num6650_2);

    //udp1num = udp2num6620 = udp2num6630 = udp2num6640 = udp3num6633 =  udp3num6633_2 = 0;
    //udp6num6650 = udp6num6650_2 = 0;
  }


  if (OTA_Activ == false) {
    currentTime = millis();
    if (currentTime - pidTime >= 19) {
      //guard na pomiary z przetwornika
      if (++pomiar_timer >= 10) {
        pomiar_timer = 0;
        esp_task_wdt_reset();
        SPI_Init();
        delay(10);
        Serial.printf("ADC STA=%02X\n", ADC_Status());
        ADC_Config();
        delay(10);
        ADC_Start();

        EXP_Config();
        delay(10);
        EXP_Config();
        delay(10);
        EXP_Config();
        delay(10);
        EXP_Outputs(0x00);
        esp_task_wdt_reset();
        DebugUDP("SPI reinit");
      }


      pidTime = currentTime;
      //WAS_PRESS_Process();  //steering position and steer angle
      //Steer.AngleError = Steer.AngleActual - Steer.AngleSetPoint;



      MOTOR_L.u_current = abs( motors_data_from_matlab.l_motor_current );
      MOTOR_R.u_current = abs( motors_data_from_matlab.r_motor_current );

      if(motors_data_from_matlab.l_motor_current < 0) MOTOR_L.dir = FORWARD;    else MOTOR_L.dir = REVERSE;
      if(motors_data_from_matlab.r_motor_current < 0) MOTOR_R.dir = FORWARD;    else MOTOR_R.dir = REVERSE;
      
      if(motors_data_from_matlab.l_motor_cmd == 0) MOTOR_L.dir = STOP; else if(motors_data_from_matlab.l_motor_cmd == 1) MOTOR_L.dir = BRAKE; else if(motors_data_from_matlab.l_motor_cmd > 2) MOTOR_L.dir = STOP;
      if(motors_data_from_matlab.r_motor_cmd == 0) MOTOR_R.dir = STOP; else if(motors_data_from_matlab.r_motor_cmd == 1) MOTOR_R.dir = BRAKE; else if(motors_data_from_matlab.r_motor_cmd > 2) MOTOR_R.dir = STOP;

      if(MOTOR_L.STOP) { MOTOR_L.dir = STOP; MOTOR_L.u_current = 0.0; }
      if(MOTOR_R.STOP) { MOTOR_R.dir = STOP; MOTOR_R.u_current = 0.0; }

      MotorControlUnit(&MOTOR_L);
      MotorControlUnit(&MOTOR_R);

      if (Steer.DiagnosticMode == true) {
        sendDiagnostic();
        Send_DEBUG_Data();
      }
    }
  } else {
    //DEBUG_PRINT("ota active\n");
  }



  //SAFETY valve check position
  if ((Steer.steerEnable == false) && (SafetyValveDuty > 0)) {
    SafetyValveDuty = 0;
    ledcWrite(3, 0);
    DebugUDP("Safety valve OFF");

    /*
    if (SafetyCheck==false)
    {
      SafetyCheck = true;
      SafetyCheckTime = millis();
    }
    else
    {
      if( (millis() - SafetyCheckTime) > 1500)
      {
        SafetyValveDuty = 0;
        ledcWrite(3, 0);
        DebugUDP("Safety valve OFF");
      }
    }*/
  }





  //* Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = millis();
  unsigned int time = currentTime;
  if (currentTime - lastTime >= LOOP_TIME) {
    //DEBUG_PRINT("core1 looptime");

    dT = currentTime - lastTime;
    lastTime = currentTime;

    TakeEncoData();
    SendSteerInfoData();

    //If connection lost to easyAGRO, the watchdog will count up and turn off steering
    if (Steer.watchdogTimer++ > 100) Steer.watchdogTimer = 100;


    if (steerSettings.cutOffPressure != 0) {
      if (Steer.valvePressure - (Steer.valvePressure_mean / 100) > steerSettings.cutOffPressure) Steer.workSwitch = 0;
      else Steer.workSwitch = 1;


      if (Steer.workSwitch != Steer.workSwitchOld) {
        if (Steer.workSwitch > 0) {
          Serial.println("workswitch: ON1");
          //DebugUDP("workswitch: ON");
        } else {
          Serial.println("workSwitch: OFF1");
          //DebugUDP("workswitch: OFF");
        }

        Steer.workSwitchOld = Steer.workSwitch;
      }
    } else {
      // z workswitcha robimy guzik wlaczonej lub nie kierownicy
      //jesli wlaczone cisnieniowe to cisnienie pokazuje wlaczenie lub nie

      Steer.workSwitch = !digitalRead(WORKSW_Pin);  // read digital work switch

      if (Steer.workSwitch != Steer.workSwitchOld) {
        if (Steer.workSwitch > 0) {
          Serial.println("workswitch: ON2");
          //DebugUDP("workswitch: ON");
        } else {
          Serial.println("workSwitch: OFF2");
          //DebugUDP("workswitch: OFF");
        }

        Steer.workSwitchOld = Steer.workSwitch;
      }
    }


    //-------------------------------------------------------------------




    Steer.switchByte = (Steer.toggleMAPing << 5) | (Steer.DiagnosticMode << 4) | (Steer.workSwitch << 3) | (Steer.Permition << 2) | (Steer.steerEnable << 1);


   // SendSteerInfoData();
    EXP_Outputs(Steer.relay);

    if (Steer.speedf == 0) {
      if (Steer.speed_timer < 100) Steer.speed_timer++;
    } else Steer.speed_timer = 0;

    //DEBUG_PRINT("core1 looptime end");
  }  //end of loop_time ...


  if (Steer.watchdogTimer >= 30 && !MOTOR_L.STOP && !MOTOR_R.STOP) {
     
    //we've lost the comm to easyAGRO
    MOTOR_L.STOP = 1;
    MOTOR_R.STOP = 1;
    Serial.println("Communication with PC Timeout!");

    Steer.pulseACount = Steer.pulseBCount = 0;  //Reset counters if Autosteer is offline
  }


  //////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////

  //section control

  /*if(Steer.relay&0x01)  digitalWrite(P1_Pin, HIGH);
  else                  digitalWrite(P1_Pin, LOW);
  if(Steer.relay&0x02)  digitalWrite(P2_Pin, HIGH);
  else                  digitalWrite(P2_Pin, LOW);
  if(Steer.relay&0x04)  digitalWrite(P3_Pin, HIGH);
  else                  digitalWrite(P3_Pin, LOW);
  if(Steer.relay&0x08)  digitalWrite(P4_Pin, HIGH);
  else                  digitalWrite(P4_Pin, LOW);
  if(Steer.relay&0x10)  digitalWrite(P5_Pin, HIGH);
  else                  digitalWrite(P5_Pin, LOW);
  if(Steer.relay&0x20)  digitalWrite(P6_Pin, HIGH);
  else                  digitalWrite(P6_Pin, LOW);
  */


  vTaskDelay(3);

  if (Steer.steerEnable == true) {
    if (steerSettings.cutOffPressure != 0) {
      //if(Steer.valvePressure > steerSettings.cutOffPressure)
      if (Steer.valvePressure - (Steer.valvePressure_mean / 100) > steerSettings.cutOffPressure) {
        Serial.println("Steer-Break: valve pressure > valvepressure cutoff settings");  // Debug only
        DebugUDP("Steer-Break: valve pressure > valvepressure cutoff settings");
        Steer.steerEnable = false;
        lastInfo = INFO_AUTO_OFF_MOTOR_CURRENT;
      }
    } else if (Steer.motorCurrent > steerSettings.currentCutOff) {
      Serial.println("Steer-Break: motor Current >cutoff settings");  // Debug only
      DebugUDP("Steer-Break: motor Current >cutoff settings");
      Steer.steerEnable = false;
      lastInfo = INFO_AUTO_OFF_MOTOR_CURRENT;
    }

    if (Steer.speed_timer == 100) {
      Serial.println("Steer-Break: speed=0 more then 10s");  // Debug only
      DebugUDP("Steer-Break: speed=0 more then 10s");
      Steer.steerEnable = false;
      lastInfo = INFO_AUTO_OFF_SPEED;
    }
  }

  if (Steer.toggleSteerEnable == 1) {
    Serial.println("trigger off");
    Steer.toggleSteerEnable = 0;
    if (Steer.steerEnable == true) {
      if (Steer.DiagnosticMode == true) {
        Serial.println("Steer-Break: Driver Button pressed - diag");  // Debug only
        DebugUDP("Steer-Break: Driver Button pressed - diag");
        Steer.steerEnable = false;
        lastInfo = INFO_DIAGNOSTIC_AUTO_OFF;

      } else {
        Serial.println("Steer-Break: Driver Button pressed");  // Debug only
        DebugUDP("Steer-Break: Driver Button pressed");
        Steer.steerEnable = false;
        lastInfo = INFO_AUTO_OFF_DRIVER_BUTTON;
      }
    } else {  //byl steerEnable false

      if ((Steer.watchdogTimer < 30) && (Steer.Permition == true)) {
        //test na odleglosc od lini
        //test na predkosc > 1km/h
        if (Steer.DiagnosticMode == true) {
          resetDiagnostic();
          Steer.steerEnable = true;
          lastInfo = INFO_DIAGNOSTIC_AUTO_ON;
          Serial.println("Steer-ON: zalaczono tryb diagnostyczny");  // Debug only
          DebugUDP("Steer-ON: zalaczono tryb diagnostyczny");
        } else {                                                                                                                                                                         //normalna praca nie diagnostyka
          if ((((Steer.DistanceFromLine >= 0) && (Steer.DistanceFromLine <= 4000)) || ((Steer.DistanceFromLine < 0) && (Steer.DistanceFromLine >= -4000))) && (Steer.speedf >= 0.75)) {  //jest ok zalaczam
            resetDiagnostic();
            Steer.steerEnable = true;
            lastInfo = INFO_AUTO_ON;
            Serial.println("Steer-ON: Driver Button pressed");  // Debug only
            DebugUDP("Steer-ON: Driver Button pressed");
          } else {
            if (((Steer.DistanceFromLine > 0) && (Steer.DistanceFromLine > 4000)) || ((Steer.DistanceFromLine < 0) && (Steer.DistanceFromLine < -4000))) {
              //za daleko ok linii
              Serial.println("Steer not ON: distance>4m");  // Debug only
              DebugUDP("Steer not ON: distance>4m");
              lastInfo = INFO_CANT_AUTO_LINE;
              Steer.melodia = 3;  //buuu
            }

            if (Steer.speedf < 0.75) {
              //za mala predkosc
              Serial.println("Steer not ON: speed<1km/h");  // Debug only
              DebugUDP("Steer not ON: speed<1km/h");
              lastInfo = INFO_CANT_AUTO_SPEED;
              Steer.melodia = 3;  //buu
            }
            Serial.println("no activate1");
          }
        }
      } else {
        if (Steer.watchdogTimer >= 18) {
          Serial.println("Steer not ON: no comm");  // Debug only
          DebugUDP("Steer not ON: no comm");
          lastInfo = INFO_CANT_AUTO_LOST_COMM;
          Steer.melodia = 3;  //buu
        }
        if (Steer.Permition == false) {
          Serial.println("Steer not ON: no permition");  // Debug only
          DebugUDP("Steer not ON: no permition");
          lastInfo = INFO_CANT_AUTO_NO_PERMITION;
          Steer.melodia = 3;  //buu
        }
        Serial.println("no activate2");
      }
    }
  }

  hm = millis();
  //Serial.printf("%i ms, %i ms, %i ms, %i ms, %i ms\n", bm-am, dm-cm, fm-em, gm-fm, hm-gm);

#ifdef UDP_DEBUG
  Send_UDP();
#endif


}  // End of core1code



void SendSteerInfoData() 
{
  //encoders_data_to_matlab
  encoders_data_to_matlab.r_motor_degPos = currentPositionRight;  encoders_data_to_matlab.r_motor_vel = velocityRight;  encoders_data_to_matlab.r_motor_acc = accelerationRight;
  encoders_data_to_matlab.l_motor_degPos = currentPositionLeft;   encoders_data_to_matlab.l_motor_vel = velocityLeft;   encoders_data_to_matlab.l_motor_acc = accelerationLeft;

  //wysylamy przez RS232
  SendDataOverRS232((byte*)&encoders_data_to_matlab, 33, 0x08);  // 0x08 = infotype frame  EXT from easySteer
  encoders_data_to_matlab.frame_cnt++;
}


typedef struct s_buf {
  unsigned short len;
  byte bdata[256];
} tBuf;

struct s_serial {
  tBuf buf[10];
  unsigned char bdelay;
  unsigned char nin;
  unsigned char nout;
} MySerial;


void Serial_Driver_Init() {
  for (int i = 0; i < 10; i++) {
    MySerial.buf[i].len = 0;
  }
  MySerial.nin = 0;
  MySerial.nout = 0;
}

// encoders_data_to_matlab
void Serial_Driver() {
  //bierzemy nout bufer i wysylamy jesli len!=0
  int nx = MySerial.buf[MySerial.nout].len;
  if (nx != 0) {
    for (int i = 0; i < nx; i++) Serial1.write(MySerial.buf[MySerial.nout].bdata[i]);
    MySerial.buf[MySerial.nout].len = 0;
    MySerial.nout++;
    if (MySerial.nout > 9) MySerial.nout = 0;
  }
}

void SendDataOverRS232(byte* dane, int len, byte type)
{
  int i;
  byte bxor;
  byte buff[256];

     //ramka nie moze byc dluzsza niz 250
  if (len > 250) {
    //DEBUG_PRINT("too long frame\n");
    return;
  }

  buff[0] = 0xAA;
  buff[1] = 0x55;
  buff[2] = (byte)((len >> 8) & 0x00FF);
  buff[3] = (byte)(len & 0x00FF);
  buff[4] = type;

  bxor = 0;
  for (i = 0; i < len; i++) {
    buff[5 + i] = (*(dane + i));
    bxor ^= *(dane + i);
  }
  buff[5 + len] = bxor;

  int frame_len_wit_header = 5 + len + 1;
  for (int i = 0; i < frame_len_wit_header; i++) Serial1.write( buff[i] );
}

void RS232Send(byte* dane, int len, byte type, byte prior) {
  int i;
  byte bxor;

  //ramka nie moze byc dluzsza niz 250
  if (len > 250) {
    //DEBUG_PRINT("too long frame\n");
    return;
  }



  //bierzemy nin bufer i ladujemy do niego ramke razem z xor jesli jest pusty (len==0)
  int nx = MySerial.buf[MySerial.nin].len;
  if (nx == 0) {
    MySerial.buf[MySerial.nin].bdata[0] = 0xAA;
    MySerial.buf[MySerial.nin].bdata[1] = 0x55;
    MySerial.buf[MySerial.nin].bdata[2] = (byte)((len >> 8) & 0x00FF);
    MySerial.buf[MySerial.nin].bdata[3] = (byte)(len & 0x00FF);
    MySerial.buf[MySerial.nin].bdata[4] = type;
    bxor = 0;
    for (i = 0; i < len; i++) {
      MySerial.buf[MySerial.nin].bdata[5 + i] = (*(dane + i));
      bxor ^= *(dane + i);
    }
    MySerial.buf[MySerial.nin].bdata[5 + len] = bxor;

    //wysylka natychmiast
    if (prior == 1) {
      nx = 5 + len + 1;
      for (int i = 0; i < nx; i++) Serial1.write(MySerial.buf[MySerial.nin].bdata[i]);

      //debug
      //for(int i=0;i<10;i++) Serial.printf("%02X ",MySerial.buf[MySerial.nin].bdata[i]);
      //Serial.printf("-\n");

      return;
    }


    MySerial.buf[MySerial.nin].len = 5 + len + 1;
    MySerial.nin++;
    if (MySerial.nin > 9) MySerial.nin = 0;
  } else {
    //DEBUG_PRINT("no free slots\n");
  }
}
