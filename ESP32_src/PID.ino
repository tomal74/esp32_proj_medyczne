
double globalError = 0;

enum
{
  STOP    =0u,
  REVERSE =1u,
  FORWARD =2u,
  BRAKE   =3u
};


void calcSteeringPID(void) 
 {
 }

//---------------------------------------------------------------------
// select output Driver
//---------------------------------------------------------------------
void motorDrive(void) 
{
}

//---------------------------------------------------------------------
// Used with BD62220 driver
//---------------------------------------------------------------------
void spoolDrive(void) 
{

    //Serial.print(Steer.steerPositionRAW); Serial.print("  ");Serial.print(Steer.AngleActual); Serial.print(" "); Serial.print(Steer.AngleSetPoint); Serial.print("  "); Serial.print(Steer.AngleError); Serial.print("  -  ");Serial.println(Steer.pwmDrive);
}

// ***********************************************************************

void MotorInit(struct TMotor * m, uint8_t INxAPin, uint8_t INxBPin, uint8_t CSxPin)
{
  m->INxPin[0] = INxBPin;
  m->INxPin[1] = INxAPin;
  m->CSxPin    = CSxPin;

  pinMode(INxAPin,  OUTPUT);
  pinMode(INxBPin,  OUTPUT);
  pinMode(CSxPin,   OUTPUT);

  m->dir = BRAKE;
  m->u_current = 0.0;

  m->STOP = 1;
}

void Set_Dir(struct TMotor * m)
{
  uint8_t cmd;
  
  for(uint8_t i=0; i<2; i++)
  {
    cmd = (m->dir >> i) & 0x01;
    digitalWrite(m->INxPin[i], cmd ? HIGH : LOW);
  }
}

void MotorControlUnit(struct TMotor * m)
{
  uint16_t mA_current = 1000.0 * m->u_current;

  Set_Dir(m);
  Set_Current(mA_current, m->CSxPin);
}




 
