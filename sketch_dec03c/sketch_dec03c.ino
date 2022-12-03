    


#include <AFMotor.h>                                              
#include "Wire.h"                                                 
//#include "I2Cdev.h"                                            
#include "HMC5883L.h"                                            
#include <Servo.h>                                               
#include <SoftwareSerial.h>                                       
#include <TinyGPS++.h>                                           
                                                                  
                                                                 
                                                                 
// GPS Variables & Setup

int GPS_Course;                                                    
int Number_of_SATS;                                                
TinyGPSPlus gps;                                                    
                                                                 
                                                                   

AF_DCMotor motor1(1, MOTOR12_64KHZ);                               
AF_DCMotor motor2(2, MOTOR12_64KHZ);                               
AF_DCMotor motor3(3, MOTOR12_64KHZ);                               
AF_DCMotor motor4(4, MOTOR12_64KHZ);                               

int turn_Speed = 175;                                          
int mtr_Spd = 250;                                                

// Compass Variables & Setup

HMC5883L compass;                                                 
int16_t mx, my, mz;                                                
int desired_heading;                                               
int compass_heading;                                               
int compass_dev = 5;                                               
                                                                   
                                                                   

int Heading_A;                                                     
int Heading_B;                                                     
int pass = 0;                                                     


// Servo Control

Servo myservo;                                                      
int pos = 0;                                                      

// Ping Sensor for Collision Avoidance

boolean pingOn = false;                                           

int trigPin = 43;                                                  
int echoPin = 42;                                                 
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;                                  
const long interval = 200;                                         
 

// Bluetooth Variables & Setup

String str;                                                        
int blueToothVal;                                                  
int bt_Pin = 34;                                                   


// GPS Locations

unsigned long Distance_To_Home;                                   

int ac =0;                                                        
int wpCount = 0;                                                   
double Home_LATarray[50];                                          
double Home_LONarray[50];                                          


int increment = 0;

void setup() 
{  
  Serial.begin(115200);                                            
  Serial1.begin(9600);                                             
  Serial2.begin(9600);                                             
  myservo.attach(9);                                              
  
  pinMode(36, OUTPUT);                                             
  pinMode(bt_Pin, INPUT);                                          

  // Ping Sensor
  pinMode(trigPin, OUTPUT);                                       
  pinMode(echoPin, INPUT);                                         

  // Compass
  Wire.begin();                                                    
  compass.begin();                                                 
  compass.setRange(HMC5883L_RANGE_1_3GA);                          
  compass.setMeasurementMode(HMC5883L_CONTINOUS);                   
  compass.setDataRate(HMC5883L_DATARATE_30HZ);                     
  compass.setSamples(HMC5883L_SAMPLES_8);                            
  compass.setOffset(0,0);                                           

  Startup();                                                       
}




void loop()
{ 
  
  bluetooth();                                                                                                   
  getGPS();                                                        
  getCompass();                                                    
  Ping();                                                         

}
void Startup()
{
     myservo.detach(); 
     Serial.println("Pause for Startup... ");
             
     for (int i=5; i >= 1; i--)                       
      {         
        Serial1.print("Pause for Startup... "); 
        Serial1.print(i);
        delay(1000);                                   
      }    
    
  Serial1.println("Searching for Satellites "); 
  Serial.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 4)                         
  {                                  
    getGPS();                                         
    Number_of_SATS = (int)(gps.satellites.value());         
    bluetooth();                                           
  }    
  setWaypoint();                                      
  wpCount = 0;                                       
  ac = 0;                                             
  
  Serial1.print(Number_of_SATS);
  Serial1.print(" Satellites Acquired");    
}   

void Forward()
{
  Ping();
  Serial.println("Forward");
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);                           
  
  motor1.run(FORWARD);                                                       
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}



void Forward_Meter()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);     
  
  motor1.run(FORWARD);                                                       
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
}



void Reverse()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}



void LeftTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                       
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(100);    
}



void RightTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);   

  motor1.run(FORWARD);                                              
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(100);                                                                     
}




void SlowLeftTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);        
  
  motor1.run(BACKWARD);                         
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}





void SlowRightTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                  
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);         

  motor1.run(FORWARD);                                                           
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}


void StopCar()
{
   
  
  motor1.run(RELEASE);                                                         
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
   
}

void CompassTurnRight()                                                         
{
  StopCar();    
  getCompass();                                                                       

  desired_heading = (desired_heading + 90);                                       
  if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}      

  while ( abs(desired_heading - compass_heading) >= compass_dev)                 
      {                                                                          
    getCompass();                                                                
    bluetooth();                                                                        
    if (blueToothVal == 5){break;}                                               
        
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}                                               
                                                                
    int x = (desired_heading - 359);                                              
    int y = (compass_heading - (x));                                             
    int z = (y - 360);                                                           
            
        if ((z <= 180) && (z >= 0))                                             
            {                                                                    
              SlowLeftTurn();                            
            } 
            else
            {
              SlowRightTurn();        
            }  
        }    
    {
      StopCar();                                                                 
    }
 }    




void CompassTurnLeft()                                                          
{
  StopCar();    
  getCompass();                                                                                                                                                  
  //desired_heading = (compass_heading - 90);                                    
  desired_heading = (desired_heading - 90);                                     
  if (desired_heading <= 0) {desired_heading = (desired_heading + 360);}         
  while ( abs(desired_heading - compass_heading) >= compass_dev)                
      {                                                                          
    getCompass();                                                               
    bluetooth();                                                                               
    if (blueToothVal == 5){break;}                                              
    
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}                                               
                                                                
    int x = (desired_heading - 359);                                              
    int y = (compass_heading - (x));                                            
    int z = (y - 360);                                                           
        if (z <= 180)                                                                  
       // if ((z <= 180) && (z >= 0))                                             
            {                                                                   
              SlowLeftTurn();                             
            } 
            else
            {
              SlowRightTurn();              
            }  
        }    
    {
      StopCar();                                                                 
    }
 }   



void Compass_Forward()                                               
{            
  while (blueToothVal == 9)                                           

  //while (true)                                                        
   {  
    getCompass();                                                     
    bluetooth();                                                      
    if (blueToothVal == 5) {break;}                                   
    
    if ( abs(desired_heading - compass_heading) <= compass_dev )      
                                                                     
       {
         Forward(); 
         Ping();       
       } else 
         {    
            int x = (desired_heading - 359);                          
            int y = (compass_heading - (x));                         
            int z = (y - 360);                                        
                     
            if ((z <= 180) && (z >= 0))                              
            {                                                         
              SlowLeftTurn();
              Ping();           
            }
            else
            {
              SlowRightTurn();
              Ping(); 
            }              
        } 
 }                                                                   
}                                                                   



void turnAround()                                                  
 {                                                                 
                                                                     
    if (pass == 0) { CompassTurnRight(); }                          
    
    else { CompassTurnLeft(); }                                    
      
    //Forward_Meter();                                              
    StopCar();                                                     
    
       
    if (pass == 0)                                                 
      {       
        CompassTurnRight();                                        
        pass = 1 ;                                                  
      }
      
    else 
      {     
         
    if (desired_heading == Heading_A)                               
     {                                                             
      desired_heading = Heading_B;
     }
    else if (desired_heading == Heading_B)
     {
      desired_heading = Heading_A;
     }        
          
        CompassTurnLeft();                                          
        pass = 0;                                                   

      }
      
  Compass_Forward();                                              
}
   
 

void sweep()                          
{ 
  myservo.attach(9); 
  StopCar();
  Forward_Meter();
  StopCar();
    
  for(pos = 60; pos <= 120; pos += 1) 
  {                                   
    myservo.write(pos);               
    delay(15);                        
  } 
  for(pos = 120; pos>=60; pos-=1)     
  {                                
    myservo.write(pos);               
    delay(15);                        
  } 

    myservo.write(90);               
    delay(15);                         

   myservo.detach();                  
}     
void bluetooth()
{
 while (Serial1.available())                                   
 {  
  {  
      str = Serial1.readStringUntil('\n');                      
      Serial.print(str);                                     
  } 
    
    blueToothVal = (str.toInt());                              
    Serial.print("BlueTooth Value ");
    Serial.println(blueToothVal);    

//switch of bluetooth

 switch (blueToothVal) 
 {
      case 1:                                
        Serial1.println("Forward");
        Forward();
        break;

      case 2:                 
        Serial1.println("Reverse");
        Reverse();
        break;

      case 3:         
        Serial1.println("Left");
        LeftTurn();
        StopCar();
        break;
        
      case 4:                     
        Serial1.println("Right");
        RightTurn();
        StopCar();
        break;
        
      case 5:                                            
        Serial1.println("Stop Car ");
        StopCar();
        break; 

      case 6:                 
        setWaypoint();
        break;
      
      case 7:        
        goWaypoint();
        break;  
      
      case 8:        
        Serial1.println("Turn Around");
        turnAround();
        break;
      
      case 9:        
         Serial1.println("Compass Forward");
        setHeading();
        Compass_Forward();
        break;
      
      case 10:
        setHeading();
        break; 

      case 11:
         gpsInfo();
        break;
      
      case 12:  
        Serial1.println("Compass Turn Right");
        CompassTurnRight();
        break;
      
      case 13:  
        Serial1.println("Compass Turn Left");
        CompassTurnLeft();
        break;
        
      case 14:  
        Serial1.println("Calibrate Compass");
        calibrateCompass();
        break;

      case 15:  
        pingToggle();
        break;  

      case 16:
        clearWaypoints();
        break;  

      case 17:                    
        ac = 0;
        Serial1.print("Waypoints Complete");
        break;
      

 } 

 
// Slider Value for Speed

if (blueToothVal)                                    
  {    
   //Serial.println(blueToothVal);
  if (blueToothVal >= 1000)
{
    Serial1.print("Speed set To:  ");
    Serial1.println(blueToothVal - 1000);
    turn_Speed = (blueToothVal - 1000); 
    Serial.println();
    Serial.print("Turn Speed ");
    Serial.println(turn_Speed);
 } 
  }  

// Detect for Mines - Sweep Not Used

 else if (blueToothVal== 15)                                   
  {    
    Serial1.println("Detecting");
    sweep();
  }



 }                                                              
                                                              
   if (Serial1.available() < 0)                                 
    {
     Serial1.println("No Bluetooth Data ");          
    }
  
} // end of bluetooth procedure

void Ping()
{ 
  currentMillis = millis();
  
 if ((currentMillis - previousMillis >= interval) && (pingOn == true)) 
 {
  previousMillis = currentMillis;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  
  int inches = (duration / 2) / 74;                       
  Ping_distance == inches;  
 
    if ((inches < 12) && (blueToothVal != 5))
      {
        Serial1.print("Crash! ");
        Serial1.println(inches);
        Reverse();                                        
        delay(100);
        StopCar();   
        blueToothVal = 5;                                 
      } 
  
  }       
 
}         

void pingToggle()                                         
 {
  if (pingOn == true) {
    pingOn = false;
    Serial1.print("Collision Avoidance OFF");
  }
    else if (pingOn == false) {
    pingOn = true;
    Serial1.print("Collision Avoidance ON");
  }
  
 }

void goWaypoint()
{   
 Serial1.println("Go to Waypoint");
Serial.print("Home_Latarray ");
Serial.print(Home_LATarray[ac],6);
Serial.print(" ");
Serial.println(Home_LONarray[ac],6);   

Serial1.print("Distance to Home");   
Serial1.print(Distance_To_Home);

Serial1.print("ac= ");
Serial1.print(ac);

 while (true)  
  {                                                               
  bluetooth();                                                     
  if (blueToothVal == 5){break;}                                   
  getCompass();                                                                                            
  getGPS();                                                        
  
  if (millis() > 5000 && gps.charsProcessed() < 10)                
    Serial1.println(F("No GPS data: check wiring"));     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[ac],Home_LONarray[ac]);                                 
   

    if (Home_LATarray[ac] == 0) {
      Serial1.print("End of Waypoints");
      StopCar();      
      break;
      }      
    
    if (Distance_To_Home == 0)                                   
        {
        StopCar();                                               
        Serial1.println("You have arrived!");                          
        ac++;                                                    
        break;                                                    
                                                                 
        }   
   
   
   if ( abs(GPS_Course - compass_heading) <= 15)                                                                                  
                                                                  
       {
         Forward();                                               
       } else 
         {                                                       
            int x = (GPS_Course - 360);                          
            int y = (compass_heading - (x));                      
            int z = (y - 360);                                    
            
            if ((z <= 180) && (z >= 0))                          
                  { SlowLeftTurn();  }
             else { SlowRightTurn(); }               
        } 
    

  }                                                              

  
}                                                                
void calibrateCompass()                                           
{
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0; 

  for (int i=1000; i >= 1; i--) 
  {
  Vector mag = compass.readRaw();                                
  
  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  
  offX = (maxX + minX)/2;                                         
  offY = (maxY + minY)/2;
  
  delay(10);
  //Serial.print("Compass X & Y offset: ");
  //Serial.print(offX);
  //Serial.print(" ");
  //Serial.print(offY);
  //Serial.print("\n");
  
  }                                                               
  
  StopCar();

  Serial1.print("Compass X & Y offset: ");
  Serial1.print(offX);
  Serial1.print(" ");
  Serial1.print(offY);
  Serial.print("\n");
  compass.setOffset(offX,offY);                                  
}
 
 

void getGPS()                                                
{
    while (Serial2.available() > 0)
    gps.encode(Serial2.read());
} 


void setWaypoint()                                            
{

//if ((wpCount >= 0) && (wpCount < 50))
if (wpCount >= 0)
  {
    Serial1.print("GPS Waypoint ");
    Serial1.print(wpCount + 1);
    Serial1.print(" Set ");
    getGPS();                                                 
    getCompass();                                              
                                               
    Home_LATarray[ac] = gps.location.lat();                   
    Home_LONarray[ac] = gps.location.lng();                      
                                                              
    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0],6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1],6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2],6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3],6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4],6);

    wpCount++;                                                  
    ac++;                                                       
        
  }         
  else {Serial1.print("Waypoints Full");}
}


void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             
   wpCount = 0;                                                 
   ac = 0;
   
   Serial1.print("GPS Waypoints Cleared");                      
  
}

 
 
void getCompass()                                              
 {  

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 
  if(heading < 0)
     heading += 2 * M_PI;      
  compass_heading = (int)(heading * 180/M_PI);                                                                            

 }



void setHeading()
                                                                 
                                                                 
{
   for (int i=0; i <= 5; i++)                                    
      { 
        getCompass();                                          
      }                                               
    
    desired_heading = compass_heading;                        
    Heading_A = compass_heading;                                 
    Heading_B = compass_heading + 180;                           

      if (Heading_B >= 360)                                      
         {
          Heading_B = Heading_B - 360;
         }
     
    Serial1.print("Compass Heading Set: "); 
    Serial1.print(compass_heading);   
    Serial1.print(" Degrees");

    Serial.print("desired heading");
    Serial.println(desired_heading);
    Serial.print("compass heading");
    Serial.println(compass_heading);

}


 
void gpsInfo()                                                  
  {
        Number_of_SATS = (int)(gps.satellites.value());         
        Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);    
        Serial1.print("Lat:");
        Serial1.print(gps.location.lat(),6);
        Serial1.print(" Lon:");
        Serial1.print(gps.location.lng(),6);
        Serial1.print(" ");
        Serial1.print(Number_of_SATS); 
        Serial1.print(" SATs ");
        Serial1.print(Distance_To_Home);
        Serial1.print("m"); 
Serial.print("Distance to Home ");
Serial.println(Distance_To_Home);
  
  }
