
void WAS_Calib(uint16_t tip)
{
long lastA, lastB, lastC;
int i;
static int looper;
long VAL;

//bool wasdebug=true;
bool wasdebug=false;
bool wasdebug2=false;

  if((tip>32700)||(tip<2000))
    {
      //Serial.printf("blad pomiaru wasRAW = %d\n",tip);
      return;
    }

    Steer.steerPositionRAW = tip;// >> 1; //by 2
    
    //teraz przepuszaczamy przez kalibracje aby uzyskac kat skretu kol

    //szukamy przedzialu, mamy 8 przedzialow 0-maxleft-20degL-5degL- 0 - 5degR-20degR-maxR-MAXADC
    //w przedziale juz dzialamy liniowo/proporcjonalnie
    int valnadeg = 0;
    int delta = 0;
    float ang = 0.0;
    float rrr;
    VAL = Steer.steerPositionRAW;

  //jesli kalibracja odwrotna
    if(steerSettings.wasMaxLeft > steerSettings.wasMaxRight) 
    {
      if( VAL > steerSettings.wasMaxLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -1\n");
          //tutaj jest problem bo nie mamy rozdzeilczosci bo nie znamy wartosci kat dla val=0, zakladamy taki skok jak dla Maxdeg-20deg
          valnadeg = (steerSettings.was20degLeft - steerSettings.wasMaxLeft)/(steerSettings.maxDegLeft - 20.0);
          delta    = steerSettings.wasMaxLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -((float)(steerSettings.maxDegLeft) + ang);
        }
        else if( VAL > steerSettings.was20degLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -2\n");
          valnadeg = (steerSettings.was20degLeft - steerSettings.wasMaxLeft)/(steerSettings.maxDegLeft - 20.0);
          delta    = steerSettings.was20degLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(20.0 + ang); 
        }
        else if( VAL > steerSettings.was5degLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -3\n");
          valnadeg = (steerSettings.was5degLeft - steerSettings.was20degLeft)/(20 - 5);
          delta    = steerSettings.was5degLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(5.0 + ang);
        }
        else if( VAL > steerSettings.wasZero)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -4\n");
          valnadeg = (steerSettings.wasZero - steerSettings.was5degLeft)/( 5 - 0);
          delta    = steerSettings.wasZero - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(0.0 + ang);
        }
        else if( VAL > steerSettings.was5degRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -5\n");
          valnadeg = (steerSettings.was5degRight - steerSettings.wasZero)/(5 - 0);
          delta    = steerSettings.was5degRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = 5.0 - ang;
        }
        else if( VAL > steerSettings.was20degRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -6\n");
          valnadeg = (steerSettings.was20degRight - steerSettings.was5degRight)/(20 - 5);
          delta    = steerSettings.was20degRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = 20.0 - ang;
        }
        else if( VAL > steerSettings.wasMaxRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial -7\n");
          valnadeg = (steerSettings.wasMaxRight - steerSettings.was20degRight)/(steerSettings.maxDegRight - 20);
          delta    = steerSettings.wasMaxRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = ((float)(steerSettings.maxDegRight)) - ang;
        }
        else
        { 
          if (wasdebug2==true) Serial.printf("Przedzial -8\n");
          //wieksze niz maxR
          //przedzial rozdzielczosc jak dla poprzedniego przedzialu
          valnadeg = (steerSettings.wasMaxRight - steerSettings.was20degRight)/(steerSettings.maxDegRight - 20);
          delta    = steerSettings.wasMaxRight - VAL; //uwaga bedzie ujemne lub zero
          ang      = (float)(delta) / valnadeg;
          rrr = ((float)(steerSettings.maxDegRight)) - ang; //bo ujemna delta !
        }
    }
    else
    {
        if( VAL < steerSettings.wasMaxLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 1\n");
          //tutaj jest problem bo nie mamy rozdzeilczosci bo nie znamy wartosci kat dla val=0, zakladamy taki skok jak dla Maxdeg-20deg
          valnadeg = (steerSettings.was20degLeft - steerSettings.wasMaxLeft)/(steerSettings.maxDegLeft - 20.0);
          delta    = steerSettings.wasMaxLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -((float)(steerSettings.maxDegLeft) + ang);
        }
        else if( VAL < steerSettings.was20degLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 2\n");
          valnadeg = (steerSettings.was20degLeft - steerSettings.wasMaxLeft)/(steerSettings.maxDegLeft - 20.0);
          delta    = steerSettings.was20degLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(20.0 + ang); 
        }
        else if( VAL < steerSettings.was5degLeft)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 3\n");
          valnadeg = (steerSettings.was5degLeft - steerSettings.was20degLeft)/(20 - 5);
          delta    = steerSettings.was5degLeft - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(5.0 + ang);
        }
        else if( VAL < steerSettings.wasZero)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 4\n");
          valnadeg = (steerSettings.wasZero - steerSettings.was5degLeft)/( 5 - 0);
          delta    = steerSettings.wasZero - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = -(0.0 + ang);
        }
        else if( VAL < steerSettings.was5degRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 5\n");
          valnadeg = (steerSettings.was5degRight - steerSettings.wasZero)/(5 - 0);
          delta    = steerSettings.was5degRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = 5.0 - ang;
        }
        else if( VAL < steerSettings.was20degRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 6\n");
          valnadeg = (steerSettings.was20degRight - steerSettings.was5degRight)/(20 - 5);
          delta    = steerSettings.was20degRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = 20.0 - ang;
        }
        else if( VAL < steerSettings.wasMaxRight)
        {
          if (wasdebug2==true) Serial.printf("Przedzial 7\n");
          valnadeg = (steerSettings.wasMaxRight - steerSettings.was20degRight)/(steerSettings.maxDegRight - 20);
          delta    = steerSettings.wasMaxRight - VAL;
          ang      = (float)(delta) / valnadeg;
          rrr = ((float)(steerSettings.maxDegRight)) - ang;
        }
        else
        { 
          if (wasdebug2==true) Serial.printf("Przedzial 8\n");
          //wieksze niz maxR
          //przedzial rozdzielczosc jak dla poprzedniego przedzialu
          valnadeg = (steerSettings.wasMaxRight - steerSettings.was20degRight)/(steerSettings.maxDegRight - 20);
          delta    = steerSettings.wasMaxRight - VAL; //uwaga bedzie ujemne lub zero
          ang      = (float)(delta) / valnadeg;
          rrr = ((float)(steerSettings.maxDegRight)) - ang; //bo ujemna delta !
        }
    }
    
    Steer.AngleActual = rrr;
    

      if(++looper>=30)
      {
        looper=0;
        if (wasdebug==true) 
        {

          if( (lastA >  Steer.steerPositionRAW+3) || (lastA <  Steer.steerPositionRAW-3)|| (lastB != Steer.valvePressure) )
              Serial.printf("WAS=%d  ActualAngle=%3.2f   valvePressure=%d   temperature=%5.2fC\n\r", Steer.steerPositionRAW, Steer.AngleActual, Steer.valvePressure, Steer.Temperature);
          
          lastA = Steer.steerPositionRAW;
          lastB = Steer.valvePressure;
          lastC = Steer.motorCurrent;
        }
      }
//Serial.print(millis()); Serial.print(" - "); Serial.printf("WAS=%3.1f\n",Steer.AngleActual);
}

void PRESS_Calib(uint16_t tip)
{
         
        if((tip>38000)||(tip<2000))
        {
          //Serial.printf("blad pomiaru pressureRAW = %d\n",tip);
          DebugUDP("blad pomiaru pressureRAW = %d\n",tip);
        }
        else 
        {
           Steer.valvePressure = (((tip-5461)*206)/21845); //rozdzielczosc 1bar/100bit
           Steer.valvePressure_mean = (long)(0.9*Steer.valvePressure_mean + 10*Steer.valvePressure);

           //debug
           /*if( (Steer.valvePressure*100) > Steer.valvePressure_mean + 1000)
           {
              Serial.printf("PSENS=%d  ActualPressure=%d bar, mean=%d\n\r", tip, Steer.valvePressure, Steer.valvePressure_mean);
           }*/
        }
        
        //rezystor na 100R, prad 4mA = 0bar, 20mA=206bar
        // 4mA *100R = 0.4V, 0.4/2.4 *32768 = 5461
        //20mA *100R = 2.0V, 2.0/2.4 *32768 = 27306
        // kontrolnie 1.0V /2.4 * 32768 =~13700  (77bar)

        
        //rezystor ma 150R, prad 4mA = 0bar, 20mA=206bar
        //4mA  *150R = 0.6V,  0.6/6.144V *32768 =  3200b    (4267 dla 200R)
        //20mA *150R = 3.0V,  3.0/6.144V *32768 = 16000b    (21333 dla 200R)
        //rozdzielczosc 0.016bar/bit
        /////////////////////////////////        
}


void WAS_PRESS_Process()
{
    WAS_Calib(Steer.RAW_WAS);
    PRESS_Calib(Steer.RAW_PRESS);
    ///DebugUDP("%d   %d \n",Steer.RAW_WAS, Steer.RAW_PRESS);  
}     
