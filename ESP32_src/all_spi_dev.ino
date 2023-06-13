#include <SPI.h>


const int CS_ADC = CS_ADC_Pin;
const int CS_DAC = CS_DAC_Pin;
const int CS_DAC2 = CS_DAC2_Pin;
const int CS_EXP = CS_EXP_Pin;

bool SPILOCK = false;


void SPI_Init()
{
  pinMode(SPI_SDI_Pin, INPUT_PULLUP);
  pinMode(SPI_SDO_Pin, OUTPUT);
  pinMode(SPI_SCK_Pin, OUTPUT);
  
  
  pinMode(CS_ADC,  OUTPUT);
  pinMode(CS_DAC,  OUTPUT);
  pinMode(CS_DAC2, OUTPUT);
  pinMode(CS_EXP,  OUTPUT);
  
  SPI.end();
  SPI.setDataMode(0);
  SPI.setFrequency(2000000); //2MHz
  SPI.begin(SPI_SCK_Pin, SPI_SDO_Pin, SPI_SDI_Pin, -1); //sck,miso,mosi,ss
  
  Serial.print("SPI clock div: ");
  Serial.println(SPI.getClockDivider());

  SPILOCK = false;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////
/////////// EXP - MCP23S08
///////////
///////////////////////
#define EXP_IODIR 0x00
#define EXP_GPIO  0x09
#define EXP_OLAT  0x0A

void EXP_Config()
{
  uint16_t toSend = 0x0000; //iodir 0x00 + value 0x00 - all outputs
  
  if(SPILOCK==true) return;
  SPILOCK = true;

  delayMicroseconds(5);
  digitalWrite(CS_EXP, LOW);
  delayMicroseconds(5); 
  SPI.write(0x40);       //DEV addres opcode
  SPI.write(0x00);
  SPI.write(0x00);
  delayMicroseconds(5); 
  digitalWrite(CS_EXP, HIGH);
  delayMicroseconds(5);
  SPILOCK = false;
}



void EXP_Outputs(uint8_t outs)
{
  if(SPILOCK==true) return;
  SPILOCK = true;

/*  digitalWrite(CS_EXP, LOW);
  
  delayMicroseconds(5); 
  SPI.write(0x40);       //DEV addres opcode
  SPI.write(0x00);
  SPI.write(0x00);
  delayMicroseconds(5); 
  
  digitalWrite(CS_EXP, HIGH);
  delayMicroseconds(50);
*/
  
  delayMicroseconds(5);
  digitalWrite(CS_EXP, LOW);
  delayMicroseconds(5); 
  SPI.write(0x40);       //DEV addres opcode
  SPI.write(0x09);
  SPI.write(outs);
  delayMicroseconds(5); 
  digitalWrite(CS_EXP, HIGH);
  delayMicroseconds(5);
  

  /*digitalWrite(CS_EXP, LOW);
  delayMicroseconds(5); 
  SPI.write(0x40);       //DEV addres opcode
  SPI.write(0x0A);
  SPI.write(outs);
  delayMicroseconds(5); 
  digitalWrite(CS_EXP, HIGH);
  delayMicroseconds(50);
*/

  SPILOCK = false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////
/////////// DAC - MCP4821
///////////
///////////////////////


//write to DAC - MCP4821
//dla Gain = 1, 12bit,  2.048V/4096 = 0.5mV na LSB
//na razie przeliczam 1.2:1 1200mV = 1000mA
//value w mA
//poczatkowe bity (najstarsze) 0 0 1 1 (gain=1x oraz shdn=1 )
void Set_Current(uint16_t value, uint8_t CS_pin)
{
  uint16_t toSend = 0x3000;
  
  if(SPILOCK==true) return;
  SPILOCK = true;
  
  int voltage = value * 1.2;
  
  if(voltage > 2000) voltage = 2000;
  if(voltage < 0) voltage = 0;
  
  voltage = voltage * 2; //0.5mV/lsb
  toSend = toSend | voltage;
  
  digitalWrite(CS_pin, LOW);
  delayMicroseconds(5); 
  
  SPI.write16(toSend);
  delayMicroseconds(5); 
  
  digitalWrite(CS_pin, HIGH);
  SPILOCK = false;
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////
/////////// ADC - MCP3462R - with internal REF
///////////
///////////////////////


//mux mode
/*
#define CFG0  0xE2  
#define CFG0_2 0xE3
#define CFG1  0x10
#define CFG2  0x8B
#define CFG3  0xB0
#define IRQ0  0x03
#define MUX0  0x00

#define SCAN0 0x00
#define SCAN1 0x00
#define SCAN2 0x00

#define TMR3 0x00
#define TMR2 0x00
#define TMR1 0x00
#define TMR0 0x00
*/


//scan mode
#define CFG0  0xE2  
#define CFG0_2 0xE3
#define CFG1  0x20



#define CFG2  0x8B
#define CFG3  0xF0 
#define IRQ0  0x06
#define MUX0  0x00

#define SCAN0_0     0x01
#define SCAN0_TEMP  0x00
#define SCAN0_PRES  0x03

#define SCAN1_0     0x00
#define SCAN1_TEMP  0x10
#define SCAN1_PRES  0x00

#define SCAN2       0x00

#define TMR3 0x00
#define TMR2 0x00
#define TMR1 0x00
#define TMR0 0x00


uint8_t ADC_Config()
{
  uint8_t toSend = 0x40;
  uint8_t out,out0,out1,out2,out3 = 0;

  SPILOCK = true;
  // ADC full reset entire register map to default
  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(5); 

  out = SPI.transfer(0x78); //full reset cmd

  delayMicroseconds(5); 
  digitalWrite(CS_ADC, HIGH);
  delayMicroseconds(5); 

  
  
  
  //config addr od 1 - 4
  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(5); 

  //incrementa write at address
  toSend = 0x42 | (0x01<<2);

  //  send in the address and value via SPI:
  out = SPI.transfer(toSend);
  out = SPI.transfer(CFG0);
  out = SPI.transfer(CFG1);
  out = SPI.transfer(CFG2);
  out = SPI.transfer(CFG3);
  out = SPI.transfer(IRQ0);
  out = SPI.transfer(MUX0);
  out = SPI.transfer(SCAN2);
  out = SPI.transfer(SCAN1_0);
  out = SPI.transfer(SCAN0_0);
  out = SPI.transfer(TMR3);
  out = SPI.transfer(TMR2);
  out = SPI.transfer(TMR1);
  out = SPI.transfer(TMR0);
  
  

  delayMicroseconds(5); 
  digitalWrite(CS_ADC, HIGH);
  delayMicroseconds(5); 

  
  
  //dla pewnosci odczyt
 

    // take the SS pin low to select the chip:
  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(5); 
  //incrementa read at address
  toSend = 0x43 | (0x01<<2);
  out = SPI.transfer(toSend);
  out0 = SPI.transfer(0xFF);
  out1 = SPI.transfer(0xFF);
  out2 = SPI.transfer(0xFF);
  out3 = SPI.transfer(0xFF);

  delayMicroseconds(5); 

  // take the SS pin high to de-select the chip:

  digitalWrite(CS_ADC, HIGH);
  
  Serial.printf("CFG0=%02X  CFG1=%02X  CFG2=%02X  CFG3=%02X\n",out0,out1,out2,out3);

  SPILOCK = false;
  return out;
}


void ADC_Config_Scan_0()
{
  uint8_t toSend = 0;

  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(1); 

  toSend = 0x42 | (0x07<<2);

  //  send in the address and value via SPI:
  SPI.transfer(toSend);
  SPI.transfer(SCAN2);
  SPI.transfer(SCAN1_0);
  SPI.transfer(SCAN0_0);
 

  delayMicroseconds(1); 
  digitalWrite(CS_ADC, HIGH);
  
  return;
}

void ADC_Config_Scan_Temp()
{
  uint8_t toSend = 0;

  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(1); 

  toSend = 0x42 | (0x07<<2);

  //  send in the address and value via SPI:
  SPI.transfer(toSend);
  SPI.transfer(SCAN2);
  SPI.transfer(SCAN1_TEMP);
  SPI.transfer(SCAN0_TEMP);
 

  delayMicroseconds(1); 
  digitalWrite(CS_ADC, HIGH);
  
  return;
}

void ADC_Config_Scan_Pres()
{
  uint8_t toSend = 0;

  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(1); 

  toSend = 0x42 | (0x07<<2);

  //  send in the address and value via SPI:
  SPI.transfer(toSend);
  SPI.transfer(SCAN2);
  SPI.transfer(SCAN1_PRES);
  SPI.transfer(SCAN0_PRES);
 

  delayMicroseconds(1); 
  digitalWrite(CS_ADC, HIGH);

  return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t ADC_Status()
{
  uint8_t out = 0;
  
  if(SPILOCK==true) return 0;

  SPILOCK = true;
  delayMicroseconds(5);
  // take the SS pin low to select the chip:
  digitalWrite(CS_ADC, LOW);

  delayMicroseconds(5); 

  //  send in the address and value via SPI:
  
  out = SPI.transfer(0x40);

  delayMicroseconds(5); 

  // take the SS pin high to de-select the chip:

  digitalWrite(CS_ADC, HIGH);
  delayMicroseconds(5);
  
  //Serial.printf("ST=%02X\n",out);

  SPILOCK = false;
  return out;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*uint8_t ADC_ReadByte(uint8_t addr)
{
  uint8_t toSend;
  uint8_t out = 0;

  if(SPILOCK==true) return 0;

  SPILOCK = true;
  
  toSend = 0x43 | ((addr)<<2);
  
  // take the SS pin low to select the chip:
  digitalWrite(CS_ADC, LOW);

  delayMicroseconds(50); 

  //  send in the address and value via SPI:
  
  out = SPI.transfer(toSend);
  out = SPI.transfer(0xFF);
  delayMicroseconds(50); 

  // take the SS pin high to de-select the chip:

  digitalWrite(CS_ADC, HIGH);
  
  SPILOCK = false;
  return out;
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t ADC_Start()
{
   //start conv
  delayMicroseconds(5); 
  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(5); 

  SPI.transfer(0x68);

  delayMicroseconds(5); 
  digitalWrite(CS_ADC, HIGH);
  delayMicroseconds(5);  
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ADC_Get()
{
uint32_t out32;
uint8_t out;
static uint32_t last_t = 0;
uint32_t t=0;
uint8_t channel = 0;
uint32_t value=0;
static int ve=0, vd=0, vc=0, v0=0, v1 = 0;

//static int licznik=0;
static int counter=0;
bool pomiar_ok = false;

  if(SPILOCK==true) return false;
  SPILOCK = true;

  
  digitalWrite(CS_ADC, LOW);
  delayMicroseconds(1); 

  out = SPI.transfer(0x43);
  out32 = SPI.transfer32(0xFFFFFFFF);
  
  delayMicroseconds(1); 
  digitalWrite(CS_ADC, HIGH);


  channel = out32>>28;
  value   = out32&0x0000FFFF;
  switch(channel)
  {
    case 0x00:  //WAS
        pomiar_ok = true;
        v0++;
        encoders_data_to_matlab.arm_raw = value;
       // Serial.println(value);

        //was_cnt++;
        
        //if(++counter%8)
        //{
          //co 5pomiarow wlaczam pomiar PRESSURE
          ADC_Config_Scan_Pres();
        //}
        
        
      break;
    case 0x01:  //PRESSURE
        pomiar_ok = true;
        v1++;
        encoders_data_to_matlab.arm_tension = value;

        //wylaczam scan Pressure
        ADC_Config_Scan_0();
      break;
    
      break;  
  }


  SPILOCK = false;
  return pomiar_ok;
}
