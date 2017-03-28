#define NUMREADINGS 5
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>



const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;

// time vars
int currTime = 0; 
int prevTime = 0; 



float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
  float Lp = 0.5;
  float Li = 0.05;
  float Ld = 0.4;
  float offsetLoc = 0;
  float pT,iT,dT = 0;
  float errorS = 0;
  float prevE = 0;

float imuValues[3];
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

//FILter
unsigned long loopTime = 0;
const float COMPLEMENTARY_GAIN = 0.995;
float lastPitch = 0;
unsigned long lastStartTime = 0;
float pitch = 0;
int dt = 0;
float gyro[3];
float angle = 0;
//**********************Setup Variables for the IR Sensor*********************
//****************************************************************************
int IRpin = 5;
unsigned long IRDT = 0;
int ledpin = 13;
float distance = 0.00;
//****************************************************************************
//****************************************************************************


//**********************Setup Variables for Gyro and Accel********************
//****************************************************************************
int readings[NUMREADINGS];                // the readings from the analog input
int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
double average = 0.00;                          // the average
int rVal = 0;
int refPin = 1;
int rPin = 0;
int yPin = 2;
//*****************************************************************************
//*****************************************************************************


//********************Setup other Balancing Variables**************************
//*****************************************************************************
double vTorque = 0.0;
double newTorque = 0.0;
double newTor = 0.0;
int z = 0;
float a1 = 0.0;
float a2 = 0.0;
float a3 = 0.0;
int m1Dir1 = 6;
int m1Dir2 = 7;
int m1PWM = 2;
int m2Dir1 = 11;
int m2Dir2 = 10;
int m2PWM = 3;


double vAngle = 0.0;
double vHighPass = 0.0;
double vLowPass = 0.0;
double vSetpoint = 4.00;
double vRate = 0.0;
double last_angle = 0.0;
double Pterm = 0.0;
double Iterm = 0.0;
double Istate = 0.0;
double Dterm = 0.0;
double Pgain = 1.9;
double Igain = 0.12;
double Dgain = 2.7;

int mydt = 5;
unsigned long last_PID = 0;
unsigned long lastread = 0;
double vYaxis = 0.00;

//*****************************************************************************
//*****************************************************************************

//***********************Variables for driving*********************************
//*****************************************************************************
#define encoder0PinA  4
#define encoder0PinB  5
int vrDist = 0;
int vlDist = 0;
int last_vrDist = 0;
int last_vlDist = 0;
int vrencodeb = 0;
double vrSpeed = 0;
double last_vrSpeed = 0;
int vlSpeed = 0;
int vTurn = 0;
int vTurnR = 0;
int vTurnL = 0;
int vDrive = 0;
int vAdjust = 0;
int vBalAdjust = 0;
int vrDirection = 0;
int last_vrDirection = 0;
int myBaldt = 50;
unsigned long lastBalupdate = 0;
double sPterm = 0.0;
double sIterm = 0.0;
double sIstate = 0.0;
double sDterm = 0.0;
double sPgain = 3.5;
double sIgain = 1.0;
double sDgain = 4.5;
double vrTarget_Speed = 0;


void setup()
{
  
  Serial.begin(9600);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  Serial.begin(9600);                     // initialize serial communication with computer
  
  //Setup Motor pins*******************  
  pinMode(m1Dir1,OUTPUT);
  pinMode(m1Dir2,OUTPUT);
  pinMode(m2Dir1,OUTPUT);
  pinMode(m2Dir2,OUTPUT);
  pinMode(m1PWM,OUTPUT);
  pinMode(m2PWM,OUTPUT);
  //***********************************
  
  //*Calibrate balance point************
  delay(100);
  /*
  for (int a = 0; a < 20; a ++)
  {
  readings[a] = analogRead(yPin); // read from the sensor
  if (a != 5)
  total += readings[a];               // add the reading to the total
  delay(40);
  Serial.println(readings[a]);
  }
  */
  
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  digitalWrite(encoder0PinB, HIGH);
  attachInterrupt(1, rencoder, FALLING);

 //***********************************
 
}

void loop()
{
  long starttime = millis();
  //sixDOF.getEuler(angles);
  void readAccel(int* x, int* y, int* z);
  sixDOF.getValues(imuValues);  
  //updateAngle();

 
  //Serial.print(angles[0]);
  //Serial.print(" | ");  
  //Serial.print(angles[1]);
  //Serial.print(" | ");
  //Serial.println(angles[2]);


  //**********************Main Code loop*******************************************
  //*******************************************************************************
  //*******************************************************************************

  Complimentaryfilter();            //run complimentary filter function

  //VTOR();
  
  Calculate_Torque();        //run torque calculations through PID
  
  
  //Calculate_Speed();
  
  debugImu();
  //Read_IR();

  dt = millis()-starttime;

  //*****************End Main Code Loop********************************************
  //*******************************************************************************
  //*******************************************************************************
}


//******************************************************************************* 
//******************************************************************************* 
//********************Setup PID loop to determine torque*************************

void Calculate_Torque()
{
  if((millis()-last_PID) >= mydt)          // sample every dt ms -> 1000/dt hz.
  { 
    last_PID = millis();


    Pterm = vAngle * Pgain;
    Istate = (Istate + vAngle);
    Iterm = Istate * Igain;
    Dterm = (vAngle - last_angle) * Dgain;

    a2 = angles[1];
    a3 = angles[3];

    Serial.print(" | ");
    float aa = 3;
    float ba = 2;
    float ca = 4;
if (angle > 0)
      {
       angle += 0.0;
        }
    if (angle < 0)
      {
       angle -= 0.0;
        }   
    
    //newTorque = angle*19;
    
  if (angle > 30)
  {
    angle = 0.0;
    vTorque = 0.0;
    newTorque = 0.0;
  }
  if (angle < -30)
  {
    angle = 0.0;
    vTorque = 0.0;
    newTor = 0.0;
  }
 
    if (angle > 0)
      {
        vTorque = angle * 15 + 0.7 * newTorque;
        //newTorque = vTorque;
        }
    if (angle < 0)
      {
        vTorque = angle * 15 + 0.7 * newTorque;
        //newTor = vTorque;
        }

    if (angle < -1)
      {
        vTorque = angle * 6 + 0.7 * newTorque;
        //newTor = vTorque;
      }
    if (angle > 1)
      {
        vTorque = angle * 6 + 0.7 * newTorque;
        //newTorque = vTorque;
      }

    if (angle < -3)
      {
        vTorque = angle * 11 + 0.8 * newTorque;
        //newTor = vTorque;
      }
    if (angle > 3)
      {
        vTorque = angle *11 + 0.8 * newTorque;
        //newTorque = vTorque;
      }

      if (angle < -5)
      {
        vTorque = angle * 12 + 0.9 * newTorque;
      //  newTor = vTorque;
      }
    if (angle > 5)
      {
        vTorque = angle * 12 + 0.9 * newTorque;
      //  newTorque = vTorque;
      }
    if (angle < -10)
      {
        vTorque = angle * 14 + 0.9 * newTorque;
    //    newTor = vTorque;
      }
    if (angle > 10)
      {
        vTorque = angle * 14 + 0.9 * newTorque;
      //  newTorque = vTorque;
      }
        
    if (angle = 0)
     {
      vTorque = 0;  
     }

  vTorque = vTorque;
  
    //vTorque = newTorque;

       // vTorque = angle*20;//pow(fabs(angle), 1.0/3.0)*10 ;
        
  /*  if (angle<0){
      vTorque = -vTorque;
    }
    */
    last_angle = vAngle;

    vTorque = constrain(vTorque,-255,255);
    
    Drive_Motor(vTorque);     //Send torque to motor drive code 
  }
  //*************End Setup PID Loop************************************************
  //*******************************************************************************
  //*******************************************************************************
}

//*********************Drive Motors**********************************************
//*******************************************************************************
//*******************************************************************************

int Drive_Motor(int vTorque)
{
  if (vTorque >= 0)                                  //indicates drive motors forward
  {
   if (vTurn >= 0)                                 //indicates turn to the right
     {
      vTurnR = vTorque + vTurn;
      vTurnL = vTorque - vTurn;
      digitalWrite(m1Dir1, LOW);                       //Signals H-Bridge to move power forward
      digitalWrite(m1Dir2, HIGH);
      
      if (vTurnL < 0)
        {
    digitalWrite(m2Dir1, HIGH);                       //Signals H-Bridge to move power forward
        digitalWrite(m2Dir2, LOW);
        }
      else
        {
        digitalWrite(m2Dir1, LOW);                       //Signals H-Bridge to move power forward
        digitalWrite(m2Dir2, HIGH);
        }
      vTurnL = abs(vTurnL);
      vTurnR = abs(vTurnR);
      analogWrite(m1PWM,vTurnR);
      analogWrite(m2PWM,vTurnL);
     }
    
   if (vTurn < 0)                                 //indicates turn to the left
     {
      vTurnR = vTorque + vTurn;
      vTurnL = vTorque - vTurn;
      digitalWrite(m1Dir1, LOW);                       //Signals H-Bridge to move power forward
      digitalWrite(m1Dir2, HIGH);
      
      if (vTurnL < 0)
        {
    digitalWrite(m2Dir1, HIGH);                       //Signals H-Bridge to move power forward
        digitalWrite(m2Dir2, LOW);
        }
      else
        {
        digitalWrite(m2Dir1, LOW);                       //Signals H-Bridge to move power forward
        digitalWrite(m2Dir2, HIGH);
        }
      vTurnR = abs(vTurnR);
      vTurnL = abs(vTurnL);
      analogWrite(m1PWM,vTurnR);
      analogWrite(m2PWM,vTurnL);
     }
  }

else if (vTorque < 0)
  {
   vTorque = abs(vTorque);
   if (vTurn >= 0)                                 //indicates turn to the right
     {
      vTurnR = vTorque - vTurn;
      vTurnL = vTorque + vTurn;
      digitalWrite(m2Dir1, HIGH);                       //Signals H-Bridge to move power forward
      digitalWrite(m2Dir2, LOW);
      
      if (vTurnR < 0)
        {
    digitalWrite(m1Dir1, LOW);                       //Signals H-Bridge to move power forward
        digitalWrite(m1Dir2, HIGH);
        }
      else
        {
        digitalWrite(m1Dir1, HIGH);                       //Signals H-Bridge to move power forward
        digitalWrite(m1Dir2, LOW);
        }
      vTurnL = abs(vTurnL);
      vTurnR = abs(vTurnR);
      analogWrite(m1PWM,vTurnR);
      analogWrite(m2PWM,vTurnL);
     }
    
   if (vTurn < 0)                                 //indicates turn to the left
     {
      vTurnR = vTorque - vTurn;
      vTurnL = vTorque + vTurn;
      digitalWrite(m2Dir1, HIGH);                       //Signals H-Bridge to move power forward
      digitalWrite(m2Dir2, LOW);
      
      if (vTurnR < 0)
        {
    digitalWrite(m1Dir1, LOW);                       //Signals H-Bridge to move power forward
        digitalWrite(m1Dir2, HIGH);
        }
      else
        {
        digitalWrite(m1Dir1, HIGH);                       //Signals H-Bridge to move power forward
        digitalWrite(m1Dir2, LOW);
        }
      vTurnR = abs(vTurnR);
      vTurnL = abs(vTurnL);
      analogWrite(m1PWM,vTurnR);
      analogWrite(m2PWM,vTurnL);
     }  

  }
  
  


  //******** End of Motor Drive Function*******************************************
  //*******************************************************************************
  //*******************************************************************************
}




//**************************Complimsentary Filtering*************************************
//*******************************************************************************


void Complimentaryfilter()
{
  updateAngle();
  
  if((millis()) )          // sample every dt ms -> 1000/dt hz.
  { 
    lastread = millis();
    

    sixDOF.getEuler(gyro);
  // = 0.98 * (imuValues[0] + imuValues[4] * 10) + (0.02) * (imuValues[0]);  
  angle = 0.03 * (angle + gyro[1] * dt/1000) + 0.97  * (angles[1]);
  //angle = 0.99 * (angle + gyro[1] * dt) + 0.01 * (angles[1]);

  //angle = 0.98 * gyro[1] + 0.02* angles[1];

      Serial.print(newTorque);
      Serial.print(" | ");
      Serial.print(angle);
      
 
  }
    
         
      if (angle > 0)
    {
      newTorque = newTorque + 25;
    }
    if (angle < 0)
    {
      newTorque = newTorque - 25;
    }
  //***********************************************************************
  //*******************End Filtering********************************
}

//********************Calculate encoder************************************
//*************************************************************************

void rencoder()
{
  if (digitalRead(encoder0PinB) == HIGH) vrDist ++;
  if (digitalRead(encoder0PinB) == LOW) vrDist --;
}

//********************Calculate speed**************************************
//*************************************************************************

void Calculate_Speed()
{
   if((millis()-lastBalupdate) >= myBaldt)  
  { 
    lastBalupdate = millis();
    
    vrSpeed = vrDist - last_vrDist;
    vrSpeed = (vrSpeed / myBaldt);
    vrSpeed = vrSpeed * 15.5;
    
    last_vrDist = 0;
    vrDist = 0;
    
    sPterm = vrSpeed * sPgain;
    sIstate = (sIstate + vrSpeed);
    sIterm = sIstate * sIgain;
    sDterm = (vrSpeed - last_vrSpeed) * sDgain;
    
    last_vrSpeed = vrSpeed;
    
    vrSpeed = 0.0;
    
    vrSpeed = sPterm + sDterm; 
    
    vrSpeed = map(vrSpeed,-400,400,-30,30);
   
    if (distance < 50.0){
      vrTarget_Speed = 5;
      digitalWrite(ledpin, HIGH);
    }
    else{
      vrTarget_Speed = 0;
      digitalWrite(ledpin, LOW);
    }
    
    vrSpeed = vrSpeed + vrTarget_Speed;
    //Serial.println(vrSpeed);
   
    vAngle = vAngle - vrSpeed;
  }
}

//********************Read IR**********************************************
//*************************************************************************

void Read_IR()
{
  if((millis()-IRDT) >= 100)  
  { 
    IRDT = millis();
    float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
    distance = 65*pow(volts, -1.10);
    //Serial.println(distance);
  }
}

 
 void debugImu() {
    sixDOF.getValues(imuValues);  

 Serial.print("aX: ");
 printInt(imuValues[0], 4);
 Serial.print("\taY: ");
 printInt(imuValues[1], 4);
 Serial.print("\taZ: ");
 printInt(imuValues[2], 4);
 Serial.print("\tgX: ");
 printInt(imuValues[3], 4);
 Serial.print("\tgY: ");
 printInt(imuValues[4], 4);
 Serial.print("\tgZ: ");
 printInt(imuValues[5], 4);
  Serial.print("\n");
 }
 void printInt(int number, byte width) {
 int currentMax = 10;
 if (number < 0) 
 currentMax = 1;
 for (byte i=1; i<width; i++){
 if (fabs(number) < currentMax) {
 Serial.print(" ");
 }
 currentMax *= 10;
 }
 Serial.print(number);
 }
 void printFloat(float number, byte width) {
 int currentMax = 10;
 if (number < 0) 
 currentMax = 1;
 for (byte i=1; i<width; i++){
 if (fabs(number) < currentMax) {
 Serial.print(" ");
 }
 currentMax *= 10;
 }
 Serial.print(number);
 }
   
void updateAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}
void keyPressed() 
{
  vTorque = 0;
  newTorque = 0;
  newTor = 0;
  angle = 0;
}
void VTOR()
{
  
    if (angle > 0)
    {
      newTorque += 5.0;
    }
    if (angle < 0)
    {
      newTorque -= 5.0;
    }
    if (angle = 0)
    {
      newTorque = 0.0;
    }
}

