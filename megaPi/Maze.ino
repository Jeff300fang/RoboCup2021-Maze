#include "MeMegaPi.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Tpa81.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"


//Sensor Objects
Tpa81 thermo = Tpa81(1);
VL53L0X sensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo myservo;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

/////////////////////////////////////////
//variables

int twoPow[8] = {1,2,4,8,16,32,64,128};
int TOFOrder[6] = {6,1,7,4,0,3};
int sensorsTOF[7] = {0};
char buff[255] = {0};

int currentlyFacing = 0;
int IMURotation;
int IMUPitch;

int CURRENTC = 0;
int CURRENTR = 0;

int distanceForward = 0;

bool rotFlag = 0;
bool rslt = 0;

int *queue;
int qIndex = 0;

int convertL, convertR, convertSFL, convertSBL,convertSFR, convertSBR;
boolean blackFlag = false;
typedef void (*FUNC)();
FUNC movements[4] = {&goForward,&goRight,&goU,&goLeft};

enum wall{NORTH,EAST,SOUTH,WEST};

struct Tile{
  unsigned char wallNorth : 1;
  unsigned char wallEast : 1;
  unsigned char wallSouth : 1;
  unsigned char wallWest : 1;
  unsigned char seen : 1;
  unsigned char ball : 1;
};

struct Tile* theMap;

int rows = 20;
int columns = 20;


////////////////////////////////////
//Motor Class

class MegaPiPort: public MeMegaPiDCMotor
{
  public:
    //variable stuff
    bool backwards;
    volatile uint8_t port;
    volatile uint8_t interruptPin;
    volatile uint8_t encoderPin;
    volatile long encoderCount;
    volatile int16_t motorSpeed;
    MegaPiPort(uint8_t port_n, uint8_t int_pin, uint8_t en_pin)
      : MeMegaPiDCMotor(port_n), port(port_n), interruptPin(int_pin), encoderPin(en_pin) { };

    inline void runMotors(int16_t spd) {
      int theSpeed = (backwards) ? -spd : spd;
      MeMegaPiDCMotor::run(theSpeed);
      motorSpeed = theSpeed;
    }
};

volatile MegaPiPort Motor[] =
{ {PORT1B, 18, 31}, {PORT2B, 19, 38},
  {PORT3B, 3,  49}, {PORT4B, 2,  A1}
};

#define Right 0
#define Left 2
#define INIT_INTERRUPT(index) attachInterrupt(digitalPinToInterrupt(Motor[index].interruptPin), motor_interrupt<index>, RISING)

template<int PN>
void motor_interrupt() {
  if (Motor[PN].backwards)
    (digitalRead(Motor[PN].encoderPin)) ? Motor[PN].encoderCount++ : Motor[PN].encoderCount--;
  else
    (digitalRead(Motor[PN].encoderPin)) ? Motor[PN].encoderCount-- : Motor[PN].encoderCount++;
}


void encoder_forward(int distance = 10, int speed_value = 255) {
  float enc_target = 0;
  Motor[Right].encoderCount = 0;
  Motor[Left].encoderCount = 0;

  Motor[Left].runMotors(144);
  Motor[Right].runMotors(130);
  enc_target = 28.5 * float(distance);
  while ( abs(Motor[Right].encoderCount) < enc_target || abs(Motor[Left].encoderCount) < enc_target );
  Motor[Right].runMotors(0);
  Motor[Left].runMotors(0);
}

/////////////////////////////////////////////

//button press wait
void startSetup(){
  Serial.println("Waiting for button press...");
  while(digitalRead(4)!= 0);
  delay(2000);
}

//round function
int rnd(int num1, int num2){
  int cmp = num1/num2;
  float act = (float)num1/num2;
  if((float)act - cmp >= 0.75){
    return(cmp+1);
  }
  return(cmp);
}



int counter = 0;
int flag = 0;

void dispenseLeft(){
  myservo.write(87);
  delay(1500);
  myservo.write(10);
  delay(1500);
  myservo.write(87);
}
void dispenseRight(){
  myservo.write(87);
  delay(1500);
  myservo.write(164);
  delay(1500);
  myservo.write(87);
}

int checkRotation(){
  getIMU();
  int presentRotation = IMURotation;
  if(presentRotation < 0){
    presentRotation += 360;
  }
  int goal = currentlyFacing * 90;
  if((goal - 2) < presentRotation && presentRotation < (goal+2)){
    return(0);
  }
  Motor[Left].runMotors(0);
  Motor[Right].runMotors(0);
  if(goal == 270){
    goal = -90;
  }

  while(IMURotation != goal){
    getIMU();
    if(goal == 0){
      if(IMURotation < 0){
         Motor[Left].runMotors(74);
         Motor[Right].runMotors(-67);
      }
      else{
        Motor[Left].runMotors(-67);
        Motor[Right].runMotors(74);
      }
    }
    if(goal == 90){
      if(IMURotation < 90){
        Motor[Left].runMotors(74);
        Motor[Right].runMotors(-67);
      }
      else{
        Motor[Left].runMotors(-67);
        Motor[Right].runMotors(74);
      }
    }
    if(goal == 180){
      if(IMURotation < 180 && IMURotation > 0){
        Motor[Left].runMotors(74);
        Motor[Right].runMotors(-67);
      }
      else{
        Motor[Left].runMotors(-67);
        Motor[Right].runMotors(74);
      }
    }
    if(goal == -90){
      if(IMURotation > -90){
        Motor[Left].runMotors(-67);
        Motor[Right].runMotors(74);
      }
      else{
        Motor[Left].runMotors(74);
        Motor[Right].runMotors(-67);
      }
    }  
    //Serial.println(IMURotation);
  }
  Motor[Right].runMotors(0);
  Motor[Left].runMotors(0);
  return(1);
}
int checkShift(){
  int avgL, avgR;
  int preL, preR;
  getTOF();
  preL = ((convertSFL + convertSBL) >> 1);
  preR = ((convertSFR + convertSBR) >> 1);
  avgL = preL % 300;
  avgR = preR % 300;
  int difference = avgL - avgR;
  difference = (difference > 0) ? difference : difference * -1;
  /*if(difference < 23 || (abs(adjustedFSL-adjustedBSL) > 150 && abs(adjustedFSR-adjustedBSR) > 150)){
    return(0);
  }*/
  if(difference < 13){
    return(0);
  }
  if(preL < 200){
    if(abs(preL - 80) < 13){
      return(0);
    }
    if(preL - 80 > 0){
      Motor[Left].runMotors(95);
      Motor[Right].runMotors(120);
    }
    else{
      Motor[Left].runMotors(120);
      Motor[Right].runMotors(95);
    }
  }
  else if(preR < 200){
    if(abs(preR - 80) < 13){
      return(0);
    }
    if(preR - 80 > 0){
      Motor[Left].runMotors(120);
      Motor[Right].runMotors(95);
    }
    else{
      Motor[Left].runMotors(95);
      Motor[Right].runMotors(120);
    }
  }
  else if(avgL < avgR){
    Motor[Left].runMotors(120);
    Motor[Right].runMotors(95);
  }
  else{
    Motor[Left].runMotors(95);
    Motor[Right].runMotors(120);
  }
  return(1);
  
}
int checkPi(){
  if(theMap[CURRENTR * columns + CURRENTC]. ball == 1){
    return(0);
  }
  theMap[CURRENTR * columns + CURRENTC]. ball = 1;
  getTOF();
  if(( ((convertSFR + convertSBR) >> 1) ) < 100){
    Serial2.write('3');
    while(!Serial2.available());
    int val = Serial2.read();
    if(val == 'B'){
      
      Motor[Left].runMotors(0);
      Motor[Right].runMotors(0);
      while(!Serial2.available());
      val = Serial2.read();
      if(val == 'H'){
        dispenseRight();
        dispenseRight();
        dispenseRight();
      }
      if(val == 'S'){
        dispenseRight();
        dispenseRight();
      }
      return(1);
    }  
  }
  if(( ((convertSFL + convertSBL) >> 1) ) < 100){
    Serial2.write('1');
    while(!Serial2.available());
    int val;
    val = Serial2.read();
    if(val == 'B'){
       Motor[Left].runMotors(0);
       Motor[Right].runMotors(0);
       while(!Serial2.available());
       val = Serial2.read();
       if(val == 'H'){
        dispenseLeft();
        dispenseLeft();
        dispenseLeft();
       }
       if(val == 'S'){
        dispenseLeft();
        dispenseLeft();
       }
    }
    return(1);
  }
  return(0);
}

//turn based on degrees

void turn(int degreesz, int side = 0){
  getIMU();
  int startRot = IMURotation;
  if(side == Right){
    Motor[Left].runMotors(150);
    Motor[Right].runMotors(-135);
  }
  else{
    Motor[Left].runMotors(-135);
    Motor[Right].runMotors(150);
  }
  if(140 < ((startRot+360)%360) && ((startRot+360)%360) < 220){
    startRot = (startRot + 360) % 360;
    IMURotation = startRot;
    while(abs(IMURotation - startRot) < degreesz){
       getIMU();
       if(IMURotation < 0){
        IMURotation += 360;  
       }
    }
  }
  else if(degreesz == 180 && -110  < startRot && startRot < -70){
     startRot = abs(startRot);
     IMURotation = startRot;
     while(abs(IMURotation - startRot) < degreesz){
       getIMU();
       if(IMURotation < 0){
        IMURotation = abs(IMURotation);  
       }
       else{
        IMURotation = 360 - IMURotation;
       }
    }
  }
  else{
    while(abs(IMURotation - startRot) < degreesz){
      getIMU();  
      Serial.print("Start:");
      Serial.println(startRot);
      Serial.println(IMURotation);
    }
  }
  Motor[Right].runMotors(0);
  Motor[Left].runMotors(0);
}
void linear(int lens, int side = 0){
  int mult = (side == 0) ? 1 : -1;
  Motor[Left].runMotors(144 * mult);
  Motor[Right].runMotors(130 * mult);
  getTOF();
  rotFlag = 0;
  if(side == 0){
    while ( ((convertL + convertR) >> 1) > lens && !checkBlack()){
      getTOF();
      checkPi();
      if(abs(convertL - convertR) > 100){
        if(convertL > convertR){
          convertR = convertL;
        }
        else{
          convertL = convertR;
        }
      }
      if(!rotFlag){
        rslt = checkRotation();
        rotFlag = checkShift();
        if(rslt || rotFlag){
          continue;
        }
      }
      else{
        rotFlag = checkShift();
        continue;
      }
      Motor[Left].runMotors(144);
      Motor[Right].runMotors(130);
    }
  }
  else{
    while ( ((convertL + convertR) >> 1) < lens && !checkBlack()){
      if(checkRotation()){
        continue;
      }
      Motor[Left].runMotors(-144);
      Motor[Right].runMotors(-130);
      getTOF();
      checkPi();
      if(abs(convertL - convertR) > 100){
        if(convertL > convertR){
          convertR = convertL;
        }
        else{
          convertL = convertR;
        }
      }
    }
  }
  Motor[Right].runMotors(0);
  Motor[Left].runMotors(0);
}

//////////////////////////////////////////
//Movements

//go forward to next box
void goForward(){
  getTOF();
  if(convertL > 1000 || convertR > 1000 || convertR < 0){
    encoder_forward(30,130);
    return;
  }
  if(abs(convertL - convertR) > 100){
    if(convertL > convertR){
      convertR = convertL;
    }
    else{
      convertL = convertR;
    }
  }
  int goal = (rnd(((convertL + convertR) >> 1),300)-1)*300 + 60;
  linear(goal,0);
}
void goRight(){
  getTOF();
  int goal = (rnd(((convertL + convertR) >> 1),300))*300 + 56;
  linear(goal,1);
  delay(100);
  checkRotation();
  delay(100);
  turn(90,Right);
  currentlyFacing = (currentlyFacing + 1) % 4;
  delay(100);
  goForward();
}
void goLeft(){
  getTOF();
  int goal = (rnd(((convertL + convertR) >> 1),300))*300 + 56;
  linear(goal,1);
  delay(100);
  checkRotation();
  delay(100);
  turn(90,Left);
  currentlyFacing = (currentlyFacing + 3) % 4;
  delay(100);
  goForward();
}
void goU(){
  getTOF();
  int goal = (rnd(((convertL + convertR) >> 1),300))*300 + 56;
  linear(goal,1);
  delay(100);
  turn(180,Left);
  currentlyFacing = (currentlyFacing + 2) % 4;
  delay(100);
  goForward();
}

/////////////////////////
//sensors

//selects the channel on the mux for each sensor
void select(int val){
  Wire.beginTransmission(0x70);
  Wire.write(twoPow[val]);
  Wire.endTransmission();
}

//IMU
//Gets IMU values, IMURotation, IMUPitch
void getIMU(){
  sensors_event_t event;
  bno.getEvent(&event);

  IMURotation = (int)event.orientation.x;
  IMUPitch = (int)event.orientation.z;

  if(IMURotation > 180){
    IMURotation -= 360;
  }
}

//TOF
//gets each TOF sensor value and puts it into the array sensorsTOF
void getTOF(){
  for(int i = 0; i < 6; i++){
    select(TOFOrder[i]);
    sensorsTOF[i] = sensor.readRangeContinuousMillimeters();
  }
  if(sensorsTOF[0] < 800){
    convertL = 4.17 + 0.908 * sensorsTOF[0] + 0.0000888 * sensorsTOF[0]*sensorsTOF[0];
  }
  else{
    convertL = 711 - 0.941* sensorsTOF[0] + 0.00134 * sensorsTOF[0] * sensorsTOF[0];
  }
  if(sensorsTOF[1] < 600){
    convertR = 6.62 + 0.915 * sensorsTOF[1] + 0.0000773 * sensorsTOF[1] * sensorsTOF[1];
  }
  else{
    convertR = -307 + 1.66* sensorsTOF[1] - 0.000343 * sensorsTOF[1] * sensorsTOF[1];
  }
  //convertR = 3 + 0.862 * sensorsTOF[1] + 0.00016 * sensorsTOF[1] * sensorsTOF[1];
  if(sensorsTOF[2] < 400){
    convertSFL = -11.7 + 1.04 * sensorsTOF[2]  -0.000242 * sensorsTOF[2] * sensorsTOF[2];
  }
  else{
    convertSFL = 167 + 0.433 * sensorsTOF[2] + 0.000375 * sensorsTOF[2] * sensorsTOF[2];
  }
  if(sensorsTOF[3] < 500){
    convertSBL = -10.7 + 1.05 * sensorsTOF[3] - 0.000298 * sensorsTOF[3]* sensorsTOF[3];
  }
  else{
    convertSBL = 1140 - 2.79 * sensorsTOF[3] + 0.00306 * sensorsTOF[3] * sensorsTOF[3];
  }
  if(sensorsTOF[4] < 600){
    convertSFR = -6.22 + 0.927 *sensorsTOF[4] + 0.0000411 * sensorsTOF[4] * sensorsTOF[4];
  }
  else{
    convertSFR = 2.05 + 0.832 *sensorsTOF[4] + 0.000157 * sensorsTOF[4] * sensorsTOF[4];
  }
  if(sensorsTOF[5] < 600){
    convertSBR = -0.102 + 0.842 *sensorsTOF[5]  + 0.000188 * sensorsTOF[5] * sensorsTOF[5];
  }
  else{
    convertSBR = 251 + 0.149 * sensorsTOF[5] + 0.000624 * sensorsTOF[5]* sensorsTOF[5];
  }
}

/////////////////////////////////////////
//main code

int checkThermal(){
  //TPA
  int pointArr[8];
  thermo.getData(pointArr);
  for(int i = 0; i < 7; i++){
    pointArr[i] = (pointArr[i] + pointArr[i+1]) >> 1;
    if(pointArr[i] > 32){
      dispenseLeft();
      return(1);
    }
  }
  //sprintf(buff, "%d %d %d %d %d %d %d", pointArr[0],pointArr[1],pointArr[2],pointArr[3],pointArr[4],pointArr[5],pointArr[6]);
  //Serial.println(buff);
}
int checkBlack(){
  uint16_t r, g, b, c, colorTemp, lux;
  select(5);
  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  if(lux > 60){
    return(0);
  }
  Motor[Left].runMotors(-144);
  Motor[Right].runMotors(-130);
  while( lux < 60){
    tcs.getRawData(&r, &g, &b, &c);
    // colorTemp = tcs.calculateColorTemperature(r, g, b);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);
  }
  Motor[Left].runMotors(0);
  Motor[Right].runMotors(0);
  delay(400);
  getTOF();
  int goal = (rnd(((convertL + convertR) >> 1),300))*300 + 46;
  linear(goal,1);
  qIndex --;
  blackFlag = true;
  return(1);
}

void setup() {

  ////////
  //Motors
  Serial.begin(9600);
  Serial.println("Motor class start up");
  INIT_INTERRUPT(Right);
  INIT_INTERRUPT(Left);
  Motor[Right].encoderCount = 0;
  Motor[Left].encoderCount = 0;
  Motor[Right].backwards = false;
  Motor[Left].backwards = true;
  //servo.setServoControl(SERVO_PIN);
  //servo.setKp(1.0);
  //servo.rotate(242, 1);
  
  ////////
  //button
  pinMode(INPUT,4);
  
  startSetup();
  
  /////
  //start communication with pi
  Serial2.begin(9600);
  while(!Serial2);
  Serial2.write('A');
  Serial.println("delivered");

  //servo
  myservo.attach(A6);
  
  ////////
  //IMU
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);


  ////////
  //close all mux channels  
  Wire.begin();
  Wire.beginTransmission(0x70);
  Wire.write(0x00);
  if(!Wire.endTransmission())
  {
    Serial.println("Success Disable");
  }
  else{
    Serial.println("Failed");
  }
  
  ////////
  //TOF
  //Initiate each channel for the 6 sensors
  
  for(int i = 0; i < 6; i++){
    Wire.beginTransmission(0x70);
    Wire.write(twoPow[TOFOrder[i]]);
    if(!Wire.endTransmission())
    {
    Serial.print("Success Enable   ");
    }
    else{
     Serial.println("Failed");
    }
    sensor.setTimeout(500);
    if (!sensor.init())
    {
    Serial.print("Failed to detect and initialize sensor!");
    Serial.println(i);
    while (1) {}
    }
    sensor.startContinuous();
  }

  //color
   Wire.beginTransmission(0x70);
   Wire.write(twoPow[5]);
   if(!Wire.endTransmission()){
   Serial.print("Success Enable   ");
   }
  else{
    Serial.println("Failed");
   }
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } 
  else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  delay(100);
  Serial.println("");
  setMap();
  Serial.println("Start");
}

////////////////////////////////////////
//map functions

void setMap(){
  //initiate every tile and set everything to 0 by default
  queue = (int*)malloc(sizeof(int) * (columns +1) * (rows+1));
  theMap = (Tile*)malloc(sizeof(Tile) * 400);
  for (int i = 0; i < rows*columns; i+= columns) {
    for (int j = 0; j < columns; j++) {
      struct Tile* temp = new Tile;
      temp->wallNorth = 0;
      temp->wallEast = 0;
      temp->wallSouth = 0;
      temp->wallWest = 0;
      temp->seen = 0;
      temp->ball = 0;
      theMap[i+j] = *temp;
    }
  }
}
void printMap(int cor, int coc, int dir){
  char buff[100] = {0};
  char sym[2] = {' ', '-'};
  char other[2] = {' ','|'};
  char mov[4] = {'^','>','v','<'};
  //print walls
  for(int i = 0; i < rows; i++){
    for(int j = 0; j < columns; j++){
      sprintf(buff, " %c%c%c", sym[theMap[i*columns + j].wallNorth],sym[theMap[i*columns + j].wallNorth],sym[theMap[i*columns + j].wallNorth]);
      Serial.print(buff);
    }
    Serial.println();
    for(int j = 0; j < columns; j++){
      if(i == cor && j == coc){
        sprintf(buff, "%c %c ", other[theMap[i*columns + j].wallWest], mov[dir]);
      }else{
        sprintf(buff, "%c   ", other[theMap[i*columns + j].wallWest]);
      }
      Serial.print(buff);
    }
    Serial.println(other[theMap[i*columns + columns-1].wallEast]);
  }
  for(int j = 0; j < columns; j++){
    sprintf(buff, " %c%c%c", sym[theMap[(columns * (rows-1)) + j].wallSouth],sym[theMap[(columns * (rows-1)) + j].wallSouth],sym[theMap[(columns * (rows-1))+ j].wallSouth]);
    Serial.print(buff);
  }
}
void getWalls(int ro, int co, int facingDirection){
  getTOF();
  int index = ro * columns + co;
  //0 --> True North
  //1 --> True East
  //2 --> True South
  //3 --> True West
  int wallShift[4] = {0};
  unsigned long sensorAvg[6] = {0};
  for(int i = 0; i < 32; i++){
    getTOF();
    for(int j = 0; j < 6; j++){
      sensorAvg[j] += sensorsTOF[j];
      if(i == 31){
        sensorAvg[j] = sensorAvg[j] >> 5;
      }
    }
  }
  Serial.print(sensorAvg[4]);
  Serial.print(" ");
  Serial.println(sensorAvg[5]);
  //start here
  if(sensorAvg[0] < 150 && sensorAvg[1] < 150){
    wallShift[0 + facingDirection] = 1;
  }
  //check right
  if(sensorAvg[4] < 250 && sensorAvg[5] < 250){
     wallShift[(1 + facingDirection) % 4] = 1;
  }
  //check wall left
  if(sensorAvg[2] < 250 && sensorAvg[3] < 250){
    wallShift[(3 + facingDirection) % 4] = 1;
  }
  if(wallShift[0]){
    theMap[index].wallNorth = 1;
    if(ro != 0){
      theMap[index-rows].wallSouth = 1;  
    }
  }
  if(wallShift[1]){
    theMap[index].wallEast = 1;
    if(co != columns - 1){
      theMap[index+1].wallWest = 1;
    }
  }
  if(wallShift[2]){
    theMap[index].wallSouth = 1;
    if(ro != rows - 1){
      theMap[index+rows].wallNorth = 1;
    }
  }
  if(wallShift[3]){
    theMap[index].wallWest = 1;
    if(co != 0){
      theMap[index-1].wallEast = 1;
    }
  }
}
int getDirection(int ro, int co, int facingDirection){
  int currentPosition = ro*columns + co;
  char wallList[4] = {theMap[currentPosition].wallNorth,theMap[currentPosition].wallEast,theMap[currentPosition].wallSouth,theMap[currentPosition].wallWest };
  int makeMove[4] = {-columns, 1,columns,-1};
  int newD;
  queue[qIndex] = currentPosition;
  qIndex ++;
  for(int change = 0; change < 5; change ++){
    if(change == 4){
      return(-1);
    }
    if(wallList[(facingDirection + change) % 4] == 0 && !theMap[currentPosition + makeMove[(facingDirection + change) % 4]].seen){
       newD = (facingDirection + change) % 4;
       return(newD);
    }
  }
  return(-1);
}
int determineDirection(int currentPos, int newPos){
  int makeMove[4] = {-columns,1,columns,-1};
  for(int check = 0; check < 4; check++){
    if(newPos - currentPos == makeMove[check]){
      Serial.println(check);
      return(check);
    }
  }
  Serial.println("Get direction failed");
  while(1);
  return(-1);
}

void printQueue(){
  for(int i = 0; i < qIndex; i ++){
    Serial.print(queue[i]);
    Serial.print(" ");
  }
  Serial.println();
}

int backTrack(int *currentR, int *currentC, int daDirection){
  printQueue();
  int currentPosition = *currentR * columns + *currentC;
  bool found = false;
  //find next tile that hasnt been visited
  //if all visied then done

  int wallList[4] = {0};
  int makeMove[4] = {-columns,1,columns,-1};
  int future, check;
  Serial.println(queue[qIndex-1]);
  for(future = qIndex - 1; future > -2 && !found; future --){
    if(future == 0){
      return(-1);
    }
    wallList[0] = theMap[queue[future]].wallNorth;
    wallList[1] = theMap[queue[future]].wallEast;
    wallList[2] = theMap[queue[future]].wallSouth;
    wallList[3] = theMap[queue[future]].wallWest;
    for(check = 0; check < 4; check ++){
      if(!wallList[check] && !theMap[queue[future] + makeMove[check]].seen){
        found = true;
        break;
      }
    }
  }
  future++;
  Serial.println(currentPosition);
  Serial.println(queue[future]);
  Serial.println(queue[qIndex-1]);
  int dir = 0;
  int moveRC[4] = {-1,0,1,0};
  while((qIndex-1) != future){
    //determine true direction
    currentPosition = *currentR * columns + *currentC;
    dir = determineDirection(currentPosition, queue[qIndex-2]);
    movements[(dir - daDirection  + 4) % 4]();
    *currentR = *currentR + moveRC[dir];
    *currentC = *currentC + moveRC[(dir+1)%4];
    daDirection = dir;
    qIndex --;
    delay(200);
  }
  qIndex--;
  Serial.print("Queue index:");
  Serial.println(queue[qIndex]);
  return(daDirection);
}
void search(){
  //start in the middle
  int currentR = rows/2, currentC = columns/2;
  //starting direction is north
  int theDirection = NORTH;
  //variable to be used to tell when done
  bool done = false;
  //heading direction
  int newDir;
  //used for changes in currentR and currentC
  int moveRC[4] = {-1,0,1,0};
  theMap[currentR * columns + currentC].wallSouth = 1;
  theMap[currentR * columns + currentC + columns].wallNorth = 1;
  //start moving and finding tiles
  while(!done){
    //set the current tile to be seen]
    delay(200);
    theMap[currentR * columns + currentC].seen = 1;
    //get the surrounding walls using the TOF sensors
    getWalls(currentR,currentC,theDirection);
    //printMap(currentR,currentC,theDirection);
    //get direction of next movement
    newDir = getDirection(currentR, currentC, theDirection);
    Serial.println(newDir);
    //startSetup();
    if(newDir < 0){
      Serial.println("Back track");
      newDir = backTrack(&currentR, &currentC, theDirection);
      if(newDir == -1){
        done = true;
      }
    }
    else{
      //Serial.println((newDir - theDirection + 4) % 4); 
      
      movements[(newDir - theDirection + 4) % 4]();
      if(!blackFlag){ 
        currentR += moveRC[newDir];
        currentC += moveRC[(newDir+1)%4];
        CURRENTC = currentC;
        CURRENTR = currentR;
      }
      else{
        int index = (currentR + moveRC[newDir]) * columns +  (currentC + moveRC[(newDir+1)%4]);
        theMap[index].wallNorth = 1;
        if(currentR != 0){
          theMap[index-rows].wallSouth = 1;  
        }
        theMap[index].wallEast = 1;
        if(currentC != columns - 1){
          theMap[index+1].wallWest = 1;
        }
        theMap[index].wallSouth = 1;
        if(currentR != rows - 1){
          theMap[index+rows].wallNorth = 1;
        }
        theMap[index].wallWest = 1;
        if(currentC != 0){
          theMap[index-1].wallEast = 1;
        }
        blackFlag = false;
      }
      
    }
    theDirection = newDir;
  }
  return; 
}

int comVal;

void loop(){
  Serial.println("--------------");
  Serial.println("Maze");
  Serial.println();

  //H - 3
  //S - 2
  search();

  getTOF();
  int goal = (rnd(((convertL + convertR) >> 1),300))*300 + 56;
  linear(goal,1);
  delay(100);
  checkRotation();
  delay(100);
  turn(90,Right);
  delay(100);
  checkPi();
  
  /*Serial.println("pinged");
  Serial2.write('1');
  while(!Serial2.available());
  comVal = Serial2.read();
  Serial.println(comVal);
  if(comVal == 7){
    Serial.println("continue");
  }
  if(comVal == 8){
    Serial.print("Stop  ");
    while(!Serial2.available());
    comVal = Serial2.read();
    Serial.println(comVal);
  }*/

  
  Serial.println("--------------");
  Serial.println("Done with maze");
  
  /*while(1){
    getIMU();
    Serial.println(IMURotation);
  }*/
  /*while(1){
    startSetup();
    checkRotation();
  }*/
  while(1);
}