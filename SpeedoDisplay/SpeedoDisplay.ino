/*
  TinyDuino 16 LED Demo
  Language: Wiring/Arduino
 
  This program lights the LEDs one at a time using Charlieplexing 
  on the TinyCircuits 16 Edge LED TinyShield Board.
  
 Created 15 Jan 2013
 by Ken Burns

 This example code is in the public domain.

 http://www.tiny-circuits.com

 */
int byCount = 0; // byCount holds the LED to turn on (1-16)
int byDir = 0; // byDir holds the direction the scan is going.
int speed_mph = 0;
int LED_Blink = 0;
boolean speed_GPSlost = true;
unsigned long time_increment_speed = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(5, INPUT);  
  pinMode(6, INPUT);  
  pinMode(7, INPUT);  
  pinMode(8, INPUT);  
  pinMode(9, INPUT);  
}

void loop()
{
  // Emulate speed changes
  if (millis() >= time_increment_speed)
  {
    speed_mph +=1 ;
    time_increment_speed += 111;
    if (speed_mph >= 90) {speed_mph = 0;}
  }
  
  if ( !speed_GPSlost )
  {
    // Plan on this display mode when there is not a valid speed/GPS
    ScanLED();
  }
  else
  {
    // Each SOLID LED is to be worth 5 MPH
    //LED_On = speed_mph / 5;
    LedOn(speed_mph / 5);
    
    // A blinking LED for each 10 MPH bracket (0.625 mph per LED 0 - 15)
    LED_Blink = speed_mph / 10;
  }
  // 20 ms per LED ~= to 50 fps display
  
}

void ScanLED()
{
  Serial.print(byDir);
  Serial.print("\t");
  Serial.println(byCount);
  
  if( !byDir )
  {
    byCount++;
    if( byCount >= 16 )
    {
      byDir = 1;
    }
  } else {
    byCount--;
    if( byCount == 1 )
    {
      byDir = 0;
    }
  }
  LedOn( byCount );
  delay( 100 );
}

void LedOn( int iLedNum )
{
   if( iLedNum == 1 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 2 )   
   {
      digitalWrite( 5, LOW );
      digitalWrite( 6, HIGH );
     
      pinMode(5, OUTPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 3 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 4 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 5, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   }   
   else if( iLedNum == 5 )   
   {
      digitalWrite( 6, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 6 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   }  
   else if( iLedNum == 7 )   
   {
      digitalWrite( 6, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }    
   else if( iLedNum == 8 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }   
   else if( iLedNum == 9 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 10 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 5, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }   
    else if( iLedNum == 11 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 12 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }  
   else if( iLedNum == 13 )   
   {
      digitalWrite( 9, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, OUTPUT);        
   }   
   else if( iLedNum == 14 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 9, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, OUTPUT);        
   } 
   else if( iLedNum == 15 )   
   {
      digitalWrite( 9, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, OUTPUT);        
   } 
   else if( iLedNum == 16 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 9, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, OUTPUT);        
   }   
}

