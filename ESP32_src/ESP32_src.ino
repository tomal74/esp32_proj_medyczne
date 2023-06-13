//board: esp32 dev module
//partition scheme: default 4MB 1.2/1.5
const byte soft_version = 124;

// andrzej jurkowski 09-04-2020
// Pawel Nowak       12-06-2023
// Tomasz Konieczka  12-06-2023

//#define UDP_DEBUG



TaskHandle_t Core1;
TaskHandle_t Core2;

struct Storage {

  byte pulseCountMax = 30;  // Switch off Autosteer after x Pulses from Steering wheel encoder


  //ustawienie zwiazanie z PID i kierownica
  float Ko = 0.05f;   //overall gain
  float Kp = 5.0f;    //proportional gain
  float Ki = 0.001f;  //integral gain
  float Kd = 1.0f;    //derivative gain
  float maxI = 2.0f;  //max value for integral PID component

  byte minPWML = 10;
  byte minPWMR = 10;
  byte lowmaxPWML = 100;
  byte lowmaxPWMR = 100;
  byte himaxPWML = 200;
  byte himaxPWMR = 200;
  int currentCutOff = 10000;  //cutoff dla pradu silnika, przekroczenie przerywa sterowanie kierownica/zaworem
  int cutOffPressure = 0;     //cisnienie wylaczenia zaworu autosteer, 0 -wylaczona funkcja / normalnie od 10-200bar

  //kalibracja WAS
  int wasMaxLeft = 2000;
  int was20degLeft = 4000;
  int was5degLeft = 5500;
  int wasZero = 6000;
  int was5degRight = 6600;
  int was20degRight = 8000;
  int wasMaxRight = 10000;
  int maxDegLeft = 38;
  int maxDegRight = 38;
  int freqPWM = 50;     //it's 1000Hz (zakres 50Hz - 2000Hz)
  int clutchDuty = 66;  //w procentach poziom pwm'a po 5sekundach na wyjsciu clutch,
  //do oszczędzania cewki zaworu lub cewki elektromagnesu kierownicy, mniej grzeje
  //5sekund idzie 100% pozniej trzyma ten poziom w %
  //minimalnie 33%
  byte modDITHER = 1;
  byte nonZero = 0;
  float KDeadBand = 0.05f;
  bool WASDIFF = false;
  int PWMProgramNumber = 0;
};
Storage steerSettings;



//Accesspoint name and password:
const char* ssid_ap = "Konieczka & Nowak VSA";
const char* password_ap = "";

//static IP
IPAddress gwip(192, 168, 4, 1);  // Gateway & Accesspoint IP
IPAddress mask(255, 255, 255, 0);
IPAddress myDNS(8, 8, 8, 8);  //optional

//unsigned int portAGRO     = 6666;      // port to listen for easyAGRO
//unsigned int portAGROUBX  = 6667;      // port dla GPS ciagnika
//unsigned int portAGRO2UBX = 6668;      // port dla GPS maszyny ciagnietej
//unsigned int portAGROUBXHEAD = 6669;   // port dla GPS heading only

unsigned int portAGROCPX = 6670;   //tylko ramki cpx
unsigned int portAGRO2CPX = 6672;  // port dla GPS maszyny ciagnietej

unsigned int portMy = 6610;  // this is port of this module
//unsigned int portMy     = 6611; // port to listen from easyAGRO

//IP address to send UDP data to:
IPAddress ipDestAll(192, 168, 12, 255);
IPAddress ipDestPC(192, 168, 12, 2);
IPAddress ipDestGPSo(192, 168, 12, 81);
IPAddress ipDestGPSo2(192, 168, 12, 82);

//IPAddress newIP(0,0,0,80);
//IPAddress newGWIP(0,0,0,1);

unsigned int portDestination = 9999;  // Port of AOG that listens
unsigned int portDebug = 9996;        // debug na terminal
unsigned int portDebugAOG = 9997;     // to samo co do AOG ale do mojego programu
unsigned int portNTRIP = 6633;        // ntrip port w gpso
unsigned int portGPSoIn = 6610;       // dane do GPSo
unsigned int portEasyFrame = 6644;    // dane do ramki przesuwnej hydraulicznej easyFrame (agroFrame)

char toDebug[128];
unsigned char toDebugLen = 0;
unsigned char debug7F = 0;
int framelen;
unsigned long APtimeout;
//unsigned long SafetyCheckTime;
//bool SafetyCheck=false;
bool LOCK = false;

// IO pins -------------------------------
//wejscia optoizolowane
#define encA_Pin 39
#define encB_Pin 36
#define STEERSW_Pin 34
#define WORKSW_Pin 35

//wyjscia posrednie prze expander I/O na SPI
//#define P1_Pin        0
//#define P2_Pin        1
//#define P3_Pin        2
//#define P4_Pin        3
//#define P5_Pin        4
//#define P6_Pin        5

//wyjscia sterujace kierunkiem pracy zaworu - uwaga mozliwe jednoczesne w lewo i prawo - co nie jest prawidlowe
#define IN1B_Pin 32
#define IN2B_Pin 33
//#define DITHER_Pin      25
//#define P7_CLUTCH_Pin   22

//wyjscie buzzera
//#define P8_BUZZ_Pin     15

// interface SPI
#define SPI_SDI_Pin 23
#define SPI_SDO_Pin 19
#define SPI_SCK_Pin 18

#define CS_DAC_Pin 5
#define CS_DAC2_Pin 15
#define CS_ADC_Pin 4
#define CS_EXP_Pin 27

#define IN1A 25
#define IN1B 32

#define IN2A 22
#define IN2B 33

//wejscia
#define ADC_IRQ 26

#define RESV4 1
#define RESV3 3


#define TXD0 17
#define RXD0 16

//dla v5
#define TXD3 12
#define RXD3 13


#define FAIL_Pin 21
#define PS_RESET_Pin 0

//dla v2
//#define TXD3           18
//#define RXD3           5







//libraries -------------------------------
//#include "Wire.h"
#include "Network_AOG.h"
#include "EEPROM.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>
#include "driver/uart.h"

static QueueHandle_t uart2_queue;

TaskHandle_t loopTaskHandle = NULL;
//SemaphoreHandle_t  xFIFOSemaphore ;

TaskHandle_t core0TaskHandle = NULL;
TaskHandle_t core1TaskHandle = NULL;

//TaskHandle_t TaskHandle_FIFOInfo = NULL;
//TaskHandle_t TaskHandle1 = NULL;

void IRAM_ATTR onTime();


#if CONFIG_AUTOSTART_ARDUINO

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

bool loopTaskWDTEnabled;


void Core1Task(void* pvParameters) {
  esp_task_wdt_reset();
  MainSetup();
  esp_task_wdt_reset();
  Core1Setup();
  esp_task_wdt_reset();

  for (;;) {
    //if(loopTaskWDTEnabled)
    esp_task_wdt_reset();

    Core1code();
    //vTaskDelay(2);///portTICK_PERIOD_MS);
  }
}

extern "C" void app_main() {
  loopTaskWDTEnabled = false;
  initArduino();
  xTaskCreatePinnedToCore(Core1Task, "Core1Task", 32768 * 2, NULL, configMAX_PRIORITIES - 2, &loopTaskHandle, ARDUINO_RUNNING_CORE);


  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  /*if ( xFIFOSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
    {
    xFIFOSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xFIFOSemaphore ) != NULL )
      xSemaphoreGive( ( xFIFOSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
    }*/


  xTaskCreatePinnedToCore(
    Core2code,         // Function that should be called
    "Core2code",       // Name of the task (for debugging)
    32768 * 2,         // Stack size (bytes)
    NULL,              // Parameter to pass
    0,                 // Task priority
    &core1TaskHandle,  // Task handle
    0);


  /*xTaskCreatePinnedToCore(
      Task_FIFOInfo,
      "FIFOInfo",
      8192,  // Stack size
      NULL,
      0,  // Priority
      &TaskHandle_FIFOInfo,
      1
              );*/
}

#endif



#define DEBUG_PRINT(...) \
  { \
    Serial.print(millis()); \
    Serial.print("ms - "); \
    Serial.println(__VA_ARGS__); \
  }
//Serial.flush(); }


// Variables ------------------------------

unsigned long UDP_data_time = 0;

//loop time variables in microseconds
const unsigned int LOOP_TIME = 100;     //10hz
const unsigned int DEBUG_TIME = 4000;  //1hz
unsigned int lastTime = LOOP_TIME;
unsigned int debuglastTime = DEBUG_TIME;
unsigned int currentTime = LOOP_TIME;
unsigned int pidTime = 0;
unsigned int dT = 50000;
byte count = 0;
byte dzielnik_pomiaru = 0;

//Kalman variables
float rollK = 0, Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
//float XeRoll = 0;
const float varRoll = 0.1;       // variance,
const float varProcess = 0.006;  //0.0055; //0,00025 smaller is more filtering

//program flow
bool isDataFound = false, isSettingFound = false, AP_running = 0, EE_done = 0;
int header = 0, tempHeader = 0, temp;
int AnalogValue = 0;

struct s_Steer {
  byte watchdogTimer = 0;
  bool steerEnable = false;
  bool oldEnable = false;
  bool toggleSteerEnable = false;
  bool Permition = false;
  bool oldPermition = false;
  byte relay = 0;
  byte uTurn = 0;
  float speedf = 0;
  int speed_timer = 0;
  byte workSwitch = 0;
  byte workSwitchOld = 0;
  //byte steerSwitch = 1;
  byte toggleMAPing = 0;
  byte switchByte = 0;
  float DistanceFromLine = 0;
  float oldDistanceFromLine = 0;
  int16_t idistanceFromLine = 0;
  float corr = 0;  // not used

  //steering variables
  float AngleActual = 0;
  int steerPrevSign = 0;
  int steerCurrentSign = 0;         // the steering wheels angle currently and previous one
  int16_t isteerAngleSetPoint = 0;  //the desired angle from easyAGRO
  float AngleSetPoint = 0;
  float highLowPerDegL = 0;
  float highLowPerDegR = 0;
  float LowMinPerDegL = 0;
  float LowMinPerDegR = 0;
  long steerPositionRAW = 0;  //from WAS sensor
  //long steerPosition = 0;
  //long steerPosition_corr = 0;


  float AngleError = 0;     //setpoint - actual
  float distanceError = 0;  //
  int pulseACount = 0;
  int pulseBCount = 0;  // Steering Wheel Encoder
  int pulseAB_sec = 0;
  int pulseAB_last = 0;
  float engineLoad[4];
  float engineLoadFiltered;
  //IMU, inclinometer variables
  bool imu_initialized = 0;
  bool imu_locked = 0;
  int16_t roll = 0;
  uint16_t x_;
  uint16_t y_;
  uint16_t z_;

  //pwm variables
  int pwmDrive = 0;
  int drive = 0;
  int pwmDisplay = 0;
  int pwmOut = 0;
  int pwmCopy = 0;
  int lastpwmsign = 0;
  float pValue = 0;
  float iValue = 0;
  float dValue = 0;

  //integral values - **** change as required *****
  //int maxIntErr = 200; //anti windup max
  //float maxIntegralValue = 2.0; //max PWM value for integral PID component
  uint16_t ppp = 0;
  //byte state_previous=0;
  byte stateMel = 0;
  byte stateMelNext;
  byte melodia = 0;
  bool oldDiagnosticMode;
  bool DiagnosticMode;
  int DiagnosticModeType;
  long motorCurrent;
  long valvePressure;
  long valvePressure_mean;

  uint32_t RAW_WAS;
  uint32_t RAW_PRESS;
  float Temperature;
  float AVDD;
  float VCM;
  int setCurrent;
};

s_Steer Steer;

/*struct s_DebugFR
{
  uint8_t PID;
  uint8_t SID;
  uint8_t cnt;
  int16_t timestamp;      //time from angleSet change
  int16_t angleSet;       //kat zadany
  int16_t angleActual;    //kat aktualny
  int16_t WASRAW;         //
  int16_t pError;         //pError
  int16_t pValue;         //pValue
  int16_t iError;         //iError
  int16_t iValue;         //iValue
  int16_t dError;         //dError
  int16_t dValue;         //dValue
  int16_t sumPWM;         //sumPWM
  uint8_t modul;          //modul
  int16_t newMax;         //newMax
  int16_t pwmDrive;       //pwmDrive
};
s_DebugFR DebugFrame;
*/

struct s_Debug9997 {
  uint8_t cnt;
  int32_t timestamp;  //time from angleSet change
  uint8_t was_cnt;
  int16_t was;
  int16_t current;
  int16_t setpoint;
  uint8_t speeding;
  uint8_t delimeter;  //0xAA
} __attribute__((packed));


s_Debug9997 DebugFrameType2;

byte lastInfo = 0;
long debugADC;


struct TMOTORS_CONTROL {
  uint8_t frame_cnt;

  uint8_t r_motor_cmd;
  uint8_t l_motor_cmd;

  float r_motor_current;
  float l_motor_current;

  uint8_t void_pad[9];
} __attribute__((packed));  // size 20 bytes

TMOTORS_CONTROL motors_data_from_matlab;


struct TENCODERS_CONTROL {
  uint8_t frame_cnt;

  float r_motor_degPos;
  float l_motor_degPos;

  float r_motor_vel;
  float l_motor_vel;

  float r_motor_acc;
  float l_motor_acc;

  uint16_t arm_raw;
  uint16_t arm_tension;

  uint8_t void_pad[4];
} __attribute__((packed));  // size 33 bytes

TENCODERS_CONTROL encoders_data_to_matlab;

unsigned long startPilot, stopPilot;
unsigned int pilotTime;
uint8_t was_cnt = 0;
static unsigned int starttime;
boolean OTA_Activ = false;


//Array to send data back to easyAGRO
byte toSend[1024];
byte toSendDEBUG[64];

//data that will be received from server
uint8_t data[12];


// Debug ----------------------------------
byte state_after = 0, breakreason = 0;
unsigned short core0run, core1run;
unsigned char new_prog, last_prog;
bool do_reset_BD62220 = false;
int SafetyValveDuty = 0;
hw_timer_t* timer = NULL;
int pomiar_timer = 0;

// Instances ------------------------------

//WiFiServer server(80);
//WiFiClient client;

//AsyncUDP udp,udp2,udp3,udp4,udp5;

#ifdef UDP_DEBUG
AsyncUDP udp2;  //- udp dla debug stringa
#endif

AsyncUDP udp1, udp3, udp4, udp6;
//udp6 - gpso wifi na traktorze
//udp4 - gpso wifi na maszynie
//udp3 - router ntrip do gpso obu

unsigned int udp1num, udp2num6620, udp2num6630, udp2num6640, udp3num6633, udp3num6633_2, udp6num6650, udp6num6650_2;
unsigned int rs232errors;
//#include <WiFiUdp.h>
//WiFiUDP udp4;
char incomingPacket[1024];
char ssid[23];


void ICACHE_RAM_ATTR Steersw_ISR();
void ICACHE_RAM_ATTR EncoderA_ISR();
void ICACHE_RAM_ATTR EncoderB_ISR();
void ICACHE_RAM_ATTR ADC_IRQ_ISR();
void ICACHE_RAM_ATTR FAIL_ISR();

struct TMotor {
  uint8_t dir;
  float u_current;
  uint8_t INxPin[2];
  uint8_t CSxPin;
  uint8_t STOP;
};

TMotor MOTOR_L;
TMotor MOTOR_R;


void MainLoop() {
  esp_task_wdt_reset();
}


// Setup procedure ------------------------
void MainSetup() {

  esp_task_wdt_init(10, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);       //add current thread to WDT watch
  esp_task_wdt_reset();

  uint32_t jooo = getCpuFrequencyMhz();
  setCpuFrequencyMhz(240);

  pinMode(PS_RESET_Pin, OUTPUT);

  digitalWrite(PS_RESET_Pin, LOW);
  delay(50);
  digitalWrite(PS_RESET_Pin, HIGH);


  pinMode(FAIL_Pin, INPUT);

  pinMode(IN1B_Pin, OUTPUT);
  pinMode(IN2B_Pin, OUTPUT);
  //pinMode(DITHER_Pin, OUTPUT);

  digitalWrite(IN1B_Pin, LOW);
  digitalWrite(IN2B_Pin, LOW);
  //digitalWrite(DITHER_Pin, LOW);

  //pinMode(P7_CLUTCH_Pin, OUTPUT);
  //  pinMode(P8_BUZZ_Pin, OUTPUT);

  //digitalWrite(P7_CLUTCH_Pin, LOW);
  //digitalWrite(P8_BUZZ_Pin, LOW);

  ledcSetup(0, 50, 8);   // PWM Output with channel 0, 100Hz, 8-bit resolution (0-255)  ---dither
  ledcSetup(2, 100, 8);  // PWM Output with channel 2, 100Hz, 8-bit resolution (0-255)
  ledcSetup(3, 100, 8);  // PWM Output with channel 3, 100Hz, 8-bit resolution (0-255)

  // ledcAttachPin(DITHER_Pin, 0); // attach PWM PIN to Channel 0
  //ledcAttachPin(P8_BUZZ_Pin, 2); // attach PWM PIN to Channel 2
  //ledcAttachPin(P7_CLUTCH_Pin, 3); //attach PWM PIN to Channel 3

  ledcWrite(0, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);

  MotorInit(&MOTOR_L, IN1A, IN1B, CS_DAC_Pin);
  MotorInit(&MOTOR_R, IN2A, IN2B, CS_DAC2_Pin);

  /*pinMode(CS_DAC_Pin, OUTPUT);
    pinMode(CS_ADC_Pin, OUTPUT);
    pinMode(CS_EXP_Pin, OUTPUT);
  */

  esp_task_wdt_reset();
  delay(5000);
  esp_task_wdt_reset();
  restoreEEprom();

  //uwaga wszystkie 3 uarty maja max 1024B ram na FIFO
  //zarowno FIFO TX jak i RX
  //

  Serial_Driver_Init();
  Serial.begin(115200);
  Serial.setRxBufferSize(32);
  //Serial.setTxBufferSize(32);

  pinMode(TXD0, OUTPUT);
  pinMode(RXD0, INPUT);
  Serial1.begin(115200, SERIAL_8N1, RXD0, TXD0);
  Serial1.setRxBufferSize(256);
  Serial.println("\nSerial1 started\n");

  pinMode(TXD3, OUTPUT);
  pinMode(RXD3, INPUT);

  Serial2.begin(115200, SERIAL_8N1, RXD3, TXD3);
  Serial2.setRxBufferSize(256);
  Serial.println("\nSerial2 started\n");

  Serial.print("CPU:");
  Serial.println(jooo);
  /*
  //Configuro la porta Serial2 (tutti i parametri hanno anche un get per effettuare controlli)
    uart_config_t Configurazione_UART2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &Configurazione_UART2);
 
 
 
    //Firma: void esp_log_level_set(const char *tag, esp_log_level_tlevel)
    //esp_log_level_set(TAG, ESP_LOG_INFO);
 
 
   
    //Firma: esp_err_tuart_set_pin(uart_port_tuart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
    uart_set_pin(UART_NUM_2, TXD3, RXD3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 
 
    //Firma: uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    //       uart_driver_install(Numero_porta, RXD_BUFFER, TXD_Buffer, event queue handle and size, flags to allocate an interrupt)
    uart_driver_install(UART_NUM_2, 192, 192, 20, &uart2_queue, 0);
 
 
    //Create a task to handler UART event from ISR
    xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);

*/


  SPI_Init();
  Serial.println("\nSPI started\n");

  /*pinMode(P1_Pin, OUTPUT);
    pinMode(P2_Pin, OUTPUT);
    pinMode(P3_Pin, OUTPUT);
    pinMode(P4_Pin, OUTPUT);
    pinMode(P5_Pin, OUTPUT);
    pinMode(P6_Pin, OUTPUT);
  */


  /*digitalWrite(P1_Pin, LOW);
    digitalWrite(P2_Pin, LOW);
    digitalWrite(P3_Pin, LOW);
    digitalWrite(P4_Pin, LOW);
    digitalWrite(P5_Pin, LOW);
    digitalWrite(P6_Pin, LOW);
  */

  if (steerSettings.clutchDuty < 33) steerSettings.clutchDuty = 33;  //minimalnie 33%
  if (steerSettings.freqPWM < 50) steerSettings.freqPWM = 50;
  if (steerSettings.freqPWM > 2000) steerSettings.freqPWM = 2000;



  if (steerSettings.modDITHER > 1) steerSettings.modDITHER = 1;
  if (steerSettings.modDITHER > 0) {
    DEBUG_PRINT("\n------------------------- DITHER is ON -----------------------------\n");
    ledcWrite(0, 127);
  } else {
    DEBUG_PRINT("\n------------------------- DITHER is OFF -----------------------------\n");
    ledcWrite(0, 0);
  }

  // for PWM High to Low interpolator
  Steer.highLowPerDegL = (steerSettings.himaxPWML - steerSettings.lowmaxPWML) / 5.0;
  Steer.highLowPerDegR = (steerSettings.himaxPWMR - steerSettings.lowmaxPWMR) / 5.0;

  Steer.engineLoad[3] = 0;
  Steer.engineLoad[2] = 0;
  Steer.engineLoad[1] = 0;
  Steer.engineLoad[0] = 0;

  Steer.oldDiagnosticMode = false;
  Steer.DiagnosticMode = false;
  Steer.oldPermition = false;
  Steer.Permition = false;
  Steer.melodia = 4;  //music na start
  restoreEEprom();
  rs232errors = 0;

  udp2num6630 = 0;  //jesli ta wartosc wieksza od zera, poleca na adres gpso_machine pakiety ntrip
  udp1num = udp2num6620 = udp2num6630 = udp2num6640 = udp3num6633 = udp3num6633_2 = udp6num6650 = udp6num6650_2 = 0;



  //------------------------------------------------------------------------------------------------------------
  //create a task that will be executed in the Core1code() function, with priority 1 and executed on core 0
  //  xTaskCreatePinnedToCore(Core1code, "Core1", 10000, NULL, 1, &Core1, 0);
  //  delay(500);
  //create a task that will be executed in the Core2code() function, with priority 1 and executed on core 1
  //  xTaskCreatePinnedToCore(Core2code, "Core2", 10000, NULL, 1, &Core2, 1);
  //  delay(500);
  //------------------------------------------------------------------------------------------------------------

  esp_task_wdt_reset();
  Print_Settings();
  Serial.println("\nSystem restarted\n");
  DebugUDP("System restarted!!!");

  while ((my_WiFi_Mode == 0)) {  // Waiting for WiFi Access
    delay(1000);
    esp_task_wdt_reset();
  }
  esp_task_wdt_reset();

  //Setup Interrupt -Steering Wheel encoder + SteerSwitchbutton
  delay(2000);
  esp_task_wdt_reset();



  pinMode(WORKSW_Pin, INPUT);
  pinMode(STEERSW_Pin, INPUT);
  pinMode(encA_Pin, INPUT);
  pinMode(encB_Pin, INPUT);
  pinMode(ADC_IRQ, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEERSW_Pin), Steersw_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ADC_IRQ), ADC_IRQ_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encB_Pin), EncoderB_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAIL_Pin), FAIL_ISR, FALLING);

  // Configure the Prescaler at 80 the quarter of the ESP32 is cadence at 80Mhz
  // 80000000 / 80 = 1000000 tics / seconde
  //timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(timer, &onTime, true);
  // Sets an alarm to sound every second
  //timerAlarmWrite(timer, 25000, true); //every 25ms
  //timerAlarmEnable(timer);


  Serial.println("Setup() finished");

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
}


/*void loop() {

  esp_task_wdt_reset();
  }*/

//UWAGA w przerwaniach nie uzywac niczego co moze sie dluzej wykonywac bo system sie wywali
//np.: Serial.print
//
//
//ISR SteerSwitch Interrupt
void IRAM_ATTR Steersw_ISR()  // handle pin change interrupt for Steersw Pin
{
  esp_task_wdt_reset();
  LOCK = true;

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 300ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 300) {
    //steerEnable = !steerEnable;
    //Serial.println("trigger");
    Steer.toggleSteerEnable = 1;
  }
  last_interrupt_time = interrupt_time;

  if (OTA_Activ == true) detachInterrupt(digitalPinToInterrupt(STEERSW_Pin));

  LOCK = false;
}


//ISR Steering Wheel Encoder
void IRAM_ATTR EncoderA_ISR() {
  LOCK = true;

#if (SWEncoder >= 0)
  Steer.pulseACount++;
  //digitalWrite(led1, !digitalRead(led1));

#endif

  LOCK = false;
}



//ISR Steering Wheel Encoder
void IRAM_ATTR EncoderB_ISR() {
  LOCK = true;

  static unsigned long last_interrupt_timeB = 0;
  unsigned long interrupt_timeB = millis();
  // If interrupts come faster than 300ms, assume it's a bounce and ignore
  if (interrupt_timeB - last_interrupt_timeB > 300) {
    //Serial.println("trigger MAP");
    Steer.toggleMAPing = 1 - Steer.toggleMAPing;
  }
  last_interrupt_timeB = interrupt_timeB;

  if (OTA_Activ == true) detachInterrupt(digitalPinToInterrupt(encB_Pin));

  LOCK = false;
}

void IRAM_ATTR ADC_IRQ_ISR() {
  LOCK = true;
  if (ADC_Get() == true) pomiar_timer = 0;

  if (OTA_Activ == true) detachInterrupt(digitalPinToInterrupt(ADC_IRQ));

  LOCK = false;
}



//ISR FAIL on BD62220 current regulator
void IRAM_ATTR FAIL_ISR() {
  LOCK = true;

  digitalWrite(PS_RESET_Pin, LOW);
  detachInterrupt(digitalPinToInterrupt(FAIL_Pin));
  do_reset_BD62220 = true;

  LOCK = false;
}


/*void IRAM_ATTR onTime() 
{
static int dziel = 0;

  if ((Steer.steerEnable == false) && (SafetyValveDuty > 0))
  {
    if(SafetyValveDuty > 5)
    {
      SafetyValveDuty -=5;
    }
    else SafetyValveDuty = 0;

    dziel = 1-dziel;
    if(dziel)
      ledcWrite(3, SafetyValveDuty);//wylacz sprzeglo gdy 0
    else
      ledcWrite(3, 0);
  }
  if (OTA_Activ == true)  
  {
    timerDetachInterrupt(timer);
    Steer.steerEnable = false;
    ledcWrite(3,0);
  }
}
*/
/*static void UART_ISR_ROUTINE(void *pvParameters)
{
uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(1024);
    bool exit_condition = false;
   
    //Infinite loop to run main bulk of task
    while (1) {
        //Loop will continually block (i.e. wait) on event messages from the event queue
        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            //Handle received event
            if (event.type == UART_DATA) {
                //Handle your uart reading here

                uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
                Serial.printf("DATA len = %d\n",event.size);
                
            }
            else if (event.type == UART_FRAME_ERR) {
                //Handle frame error event
            }
            //... //Keep adding else if statements for each UART event you want to support
            else {
                //Final else statement to act as a default case
            }      
        }
       
        //If you want to break out of the loop due to certain conditions, set exit condition to true
        if (exit_condition) {
            break;
        }
    }
   
    //Out side of loop now. Task needs to clean up and self terminate before returning
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);

    
}*/
