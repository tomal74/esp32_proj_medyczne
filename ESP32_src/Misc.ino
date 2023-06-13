
void SetRelays(void)
 {
    //if (bitRead(Steer.relay,0)) digitalWrite(led1, HIGH);
    //else digitalWrite(led1, LOW);
    //if (bitRead(Steer.relay,1)) digitalWrite(led2, HIGH);
    //else digitalWrite(led2, LOW); 
    //if (bitRead(relay,2)) digitalWrite(led3, HIGH);
    //else digitalWrite(led3, LOW); 
    //if (bitRead(relay,3)) digitalWrite(led4, HIGH);
    //else digitalWrite(led4, LOW); 
    
    //if (bitRead(relay,4)) bitSet(PORTB, 1); //Digital Pin 9
    //else bitClear(PORTB, 1); 
    //if (bitRead(relay,5)) bitSet(PORTB, 4); //Digital Pin 12
    //else bitClear(PORTB, 4); 
    //if (bitRead(relay,6)) bitSet(PORTC, 4); //analog Pin A4
    //else bitClear(PORTC, 4); 
    //if (bitRead(relay,7)) bitSet(PORTC, 5); //Analog Pin A5
    //else bitClear(PORTC, 5); 
  }

//--------------------------------------------------------------
//  EEPROM Data Handling
//--------------------------------------------------------------
#define EEPROM_SIZE 160
#define EE_ident1 0xDE  // Marker Byte 0 + 1
#define EE_ident2 0xE3

//--------------------------------------------------------------
//  Restore EEprom Data
//--------------------------------------------------------------
/*void restoreEEprom(){
  //byte get_state  = digitalRead(restoreDefault_PIN);
  byte get_state = false;
  
  if (EEprom_empty_check()==1 || get_state) { //first start?
    
    Serial.println(" default settings set !!!!!!!");
    EEprom_write_all();     //write default data
   }
  if (EEprom_empty_check()==2) { //data available
    EEprom_read_all();
   }
  EEprom_show_memory();  //
  EE_done =1;   
}*/

void restoreEEprom(){
 int i=0;
  byte test = 0;
  for(i=0;i<30;i++)
  {
    test = EEprom_empty_check(); 
    esp_task_wdt_reset();
    if(test<2)
    {
      delay(1000);
      esp_task_wdt_reset();
      if(i>=28)
      { 
        //first start?
        EEprom_write_all();     //write default data
        Serial.println(" default settings set !!!!!!!");
      }
    }
    else
    {
      if(test==2)
      {
        //data available
        EEprom_read_all();  
        //EEprom_show_memory();  //
        EE_done =1;  
        break; ///end for loop
      }
    }
  }
}
  
//--------------------------------------------------------------
byte EEprom_empty_check(){
    
  if (!EEPROM.begin(EEPROM_SIZE))  
    {
     Serial.println("failed to initialise EEPROM"); delay(1000);
     return false;
    }
  if (EEPROM.read(0)!= EE_ident1 || EEPROM.read(1)!= EE_ident2)
     {
      Serial.println("EEprom is empty");
      return true;  // is empty
     }
  
  if (EEPROM.read(0)== EE_ident1 && EEPROM.read(1)== EE_ident2)
     {
      Serial.println("EEprom is OK");
      return 2;     // data available
     }
     Serial.println("EEprom ?");
 }
//--------------------------------------------------------------
void EEprom_write_all()
{
  while(LOCK == true)
  {
    noInterrupts();
    delay(5);
  };
  
  noInterrupts();
  delay(25);
  //EEPROM code
  Serial.println("eeprom write start");
  EEPROM.write(0, EE_ident1);
  EEPROM.write(1, EE_ident2);
  EEPROM.put(4, steerSettings);
  //EEPROM.commit();
  EEPROM.end(); //end = commit + ram free
  esp_task_wdt_reset();
  Serial.println("eeprom write end");

  delay(25);
  interrupts();
  delay(1);
}
//--------------------------------------------------------------
void EEprom_read_all(){
    EEPROM.get(4, steerSettings); 
}
//--------------------------------------------------------------
void EEprom_show_memory(){
byte c2=0, data_;
int len = sizeof(steerSettings)+4;  //plus4 na iden1 iden2 oraz 2 wolne
  Serial.print("Reading ");
  Serial.print(len);
  Serial.println(" bytes from Flash . Values are:");
  for (int i = 0; i < len; i++)
  { 
    data_=byte(EEPROM.read(i));
    if (data_ < 0x10) Serial.print("0");
    Serial.print(data_,HEX); 
    if (c2>=15) {
       Serial.println();
       c2=-1;
      }
    else Serial.print(" ");
    c2++;
  }
}




   
