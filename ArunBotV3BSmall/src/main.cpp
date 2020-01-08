#include "vex.h"
//#include "vex_motor.h"
#include <cmath> 
#include <string>
#include <list> 
#include <iterator> 
using namespace vex;



// /*----------------------------------------------------------------------------*/
// /*                                                                            */
// /*    Module:       main.cpp                                                  */
// /*    Author:       Robohawks                                                 */
// /*    Created:      Sat Sep 21 2019                                           */
// /*    Description:  V5 project                                                */
// /*                                                                            */
// /*----------------------------------------------------------------------------*/
// #include "vex.h"
// 
// // ---- START VEXCODE CONFIGURED DEVICES ----
// // Robot Configuration:
// // [Name]               [Type]        [Port(s)]
// // ---- END VEXCODE CONFIGURED DEVICES ----
// 
using namespace vex;
vex::competition Competition;
vex::controller  Controller1;
 
//MOTOR DECLARATIONS:
//Chassis
vex::motor ChassisLF(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor ChassisLB(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor ChassisRF(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor ChassisRB(vex::PORT4, vex::gearSetting::ratio18_1, true);
//Grabbers
vex::motor LeftGrabber(vex::PORT7, vex::gearSetting::ratio18_1, true); 
vex::motor RightGrabber(vex::PORT8, vex::gearSetting::ratio18_1, false); 
//Misc.
vex::motor GrabberLift(vex::PORT5, vex::gearSetting::ratio36_1, true); 
vex::motor StackMotor(vex::PORT6, vex::gearSetting::ratio36_1, true); //is actually 36_1, programmed as 6_1 for slower speeds
 
//---------------------------------------------------------------------------------------------------------------

//MISCELLANEOUS SETTERS/DECLARATIONS:
std::list<std::string> autonRecord; 
double outtakeOn = false;
double chassisMultiplier = .7;
bool slowModeEnabled = false;
bool FwdGrabberStop = true;
bool RevGrabberStop = true;
const double pi = 3.1415927;
int rollerTiltCheck = 1;
const int encoderTicksPerRev = 900;
const int chassisWheelDia = 4; //4in chassis  wheel
const float ticksPerTurn = 3000; //tbd value through testing
const int maxMotorRPM = 200;

const int wheelRadius = 2; //in inches
const int wheelCircumference = 2*wheelRadius*pi; //in inches
const double turningRadius = 5.5;// in inches, distance from center of robot to center of wheel



//Stack Positioning Values (degrees)
double stackStartPos = 0.00;
double stackUnloadPos = 1000.00; //was 1025
double moreThanSevenPos = 3000.0; //2428.8;
double stackCurrentPos;
bool stackInStart = true;
bool stackInUnload = false;

//Grabber Positioning Values (degrees)
double bottomPos = 5.00;
double midTowerPos = 560.20;
double lowTowerPos = 411.20;
double grabberCurrentPos = 0.00;
bool grabberAtBottom = true;
bool grabberAtLow = false;
bool grabberAtMid = false;

double Limit(double val, double max, double min);

//---------------------------------------------------------------------------------------------------------------


//CHASSIS SETTERS:
void chassisPIDMove (double inches);
void chassisPIDMoveMax(double inches, double maxSpeed);
void chassis_move_coast(double rotation, int velocity);
void chassis_move(double distance,double velocity);
bool chassis_move_non_blocking(double distance,double velocity);

void chassis_move_for(double rotation, int velocity);
void chassis_move_auton(double rotation, int velocity);
void turn(double degrees,int velocity);
void leftDrive(vex::directionType type, int percentage);
void rightDrive(vex::directionType type, int percentage);
void leftSpin(double velocity);
void rightSpin(double velocity);


// 
// //---------------------------------------------------------------------------------------------------------------
// 
// 
// //CALLBACKS:
void l1Pressed();
void l1PressedAuton();
void l2Pressed();
void r1Pressed();
void r1PressedAuton(double extra);
void r2Pressed();
void r2PressedAuton();
void l1PressedTest();
void l2PressedTest();
void r1PressedTest();
void r2PressedTest();
void upPressed();
void downPressed();
void leftPressed();
void rightPressed();
void xPressed();
void yPressed();
void a1Pressed();
void b2Pressed();

void chassisControl(){
  if(outtakeOn) {
    leftDrive(vex::directionType::fwd, Controller1.Axis2.value()*chassisMultiplier);
    rightDrive(vex::directionType::fwd, Controller1.Axis2.value()*chassisMultiplier);
  }
  else {
    leftDrive(vex::directionType::fwd, Controller1.Axis3.value()*chassisMultiplier);
    rightDrive(vex::directionType::fwd, Controller1.Axis2.value()*chassisMultiplier);

  }
  /*
    ChassisLF.spin(vex::directionType::fwd, Controller1.Axis3.value()*chassisMultiplier, vex::velocityUnits::pct);
    ChassisLB.spin(vex::directionType::fwd, Controller1.Axis3.value()*chassisMultiplier, vex::velocityUnits::pct);
    ChassisRF.spin(vex::directionType::fwd, Controller1.Axis2.value()*chassisMultiplier, vex::velocityUnits::pct);
    ChassisRB.spin(vex::directionType::fwd, Controller1.Axis2.value()*chassisMultiplier, vex::velocityUnits::pct);   
*/}
//--------------------------------------------------------------------------------------------------------------- 
//METHODS:

//PID Methods:
//unit conversion from inches to encoder ticks
int inch2Tick (float inch) {
  int ticks;
  ticks = inch * encoderTicksPerRev /(chassisWheelDia * pi);
  return ticks;
} 

//function getting the sign of number
int signNum (double num){
  if (num < 0) return -1;
  else if (num > 0) return 1;
  else return 0;
}  

//unit conversion from degree angle to encoder ticks
//this function requires measurement of # ticks per robot turn
int degree2Tick (float degree) {//need to measure based on robot
  int ticks = degree * ticksPerTurn/encoderTicksPerRev;
  return ticks;
}

//unit conversion from degree angle to encoder ticks
//this function is based on wheel travel trajectory during a turn
//need fine tune of turningRadius to get correct angle conversion
int degree2Tick_2 (float degree) {
  float turningCirc = 2 * pi * turningRadius;
  double ticksPerTurn = inch2Tick(turningCirc);
  int ticks = degree * ticksPerTurn/360;
  return ticks; 
}

//time delay at the end of a chassis move. Minumum is 250msec regardless what the setting is
int waitTime_msec (double rawSeconds)   {
  int miliSeconds;
  miliSeconds = rawSeconds * 1000;
  if (miliSeconds < 150) //was 250
  {
   miliSeconds = 150; //was 250
  }
  return miliSeconds;
}


double Limit(double val, double min, double max){
    if(val > max) {
      return max;
    }
    else if(val < min) {
      return min;
    }
    else {
      return val;
    }
}

//-----------------------------------------------------------
//Auton Methods
void backupAuton(void) {
    ChassisLB.setVelocity(75, velocityUnits::pct);
    ChassisRB.setVelocity(75, velocityUnits::pct);
    ChassisLF.setVelocity(75, velocityUnits::pct);
    ChassisRF.setVelocity(75, velocityUnits::pct);
    ChassisLB.rotateFor(directionType::rev, 2, rotationUnits::rev);
    ChassisRB.rotateFor(directionType::rev, 2, rotationUnits::rev);
    ChassisLF.rotateFor(directionType::rev, 2, rotationUnits::rev);
    ChassisRF.rotateFor(directionType::rev, 2, rotationUnits::rev);
    vex::task::sleep(1000);
    ChassisLB.setVelocity(75, velocityUnits::pct);
    ChassisRB.setVelocity(75, velocityUnits::pct);
    ChassisLF.setVelocity(75, velocityUnits::pct);
    ChassisRF.setVelocity(75, velocityUnits::pct);
    ChassisLB.rotateFor(directionType::fwd, 2, rotationUnits::rev);
    ChassisRB.rotateFor(directionType::fwd, 2, rotationUnits::rev);
    ChassisLF.rotateFor(directionType::fwd, 2, rotationUnits::rev);
    ChassisRF.rotateFor(directionType::fwd, 2, rotationUnits::rev);

}
void backAuton(void){
  ChassisLF.setVelocity(200,velocityUnits::rpm);
    ChassisLB.setVelocity(200,velocityUnits::rpm);
    ChassisRB.setVelocity(200,velocityUnits::rpm);
    ChassisRF.setVelocity(200,velocityUnits::rpm);
    
    ChassisLF.rotateFor(-10,rotationUnits::rev,false);
    ChassisLB.rotateFor(-10,rotationUnits::rev,false);
    ChassisRF.rotateFor(-10,rotationUnits::rev,false);
    ChassisRB.rotateFor(-10,rotationUnits::rev, true);

  //chassis_move(-100, 200);
  //chassis_move(15, 200);
}

void flipOut(void) {
  ChassisLF.setStopping(hold);
  ChassisRF.setStopping(hold);
  ChassisLB.setStopping(hold);
  ChassisRB.setStopping(hold);
  l2Pressed();
  StackMotor.rotateFor(-4, rotationUnits::deg);
  StackMotor.resetRotation();
  r1PressedAuton(10);
  
  vex::task::sleep(250);
  r2PressedAuton();
  l2PressedTest();
  chassis_move_non_blocking(-3, 25);
  l1PressedAuton();
  //r2Pressed();
  //chassisPIDMove(24);
  return;
}

void fiveCubes(void) {
  /*chassis_move(41, 45); //distance was 39 // speed was 40
  //chassisPIDMove(41);
  vex::task::sleep(150);
  l1Pressed();
  //chassis_move(-10, 20);
  turn(-145, 65); //was -145
  //chassis_move(36, 50); //distance was 37 // speed was 40
  double totalDistance = 0;
  double currentSpeed = 80;
  while(totalDistance <= 36) {
      chassis_move(3.6, currentSpeed);
      currentSpeed -= 5;
      totalDistance += 3.6;
  }
  upPressed();
  if(stackInUnload) {
    vex::task::sleep(250);
    xPressed();
  }*/

  //chassis_move(41, 40); //distance was 39
  //chassis_move(5,40);
  chassisPIDMoveMax(40, 120);
  //vex::task::sleep(150);
  l1PressedAuton(); //was regular l1pressed
  //chassis_move(-18, 40);
  //vex:task::sleep(2000);
  turn(-150, 65); //was -135 //was 155
  //vex::task::sleep(5000);
  chassisPIDMove(45); //was 37 //was regular move 40
  upPressed();
  if(stackInUnload) {
    vex::task::sleep(250);
    xPressed();
  }

}
void fiveCubesEXPERIMENTAL(void) {
  chassisPIDMoveMax(40, 125); //was 120
  l1PressedAuton();
  vex::task::sleep(500);
  chassisPIDMoveMax(-20, 100); //was -25
  vex::task::sleep(500);
  turn(-130, 40); //was 65
  vex::task::sleep(150);
  chassisPIDMoveMax(20, 130);
  upPressed();
  
  //if(stackInUnload) {
    //StackMotor.rotateFor(-4, rotationUnits::deg);
    vex::task::sleep(250);
    xPressed();
  //}
}
void autonomous( void ) {
    flipOut(); //always flip out before auton
    //chassisPIDMove(8*pi);
    //fiveCubes();
    fiveCubesEXPERIMENTAL();
    //backAuton();
    //smallSideAuton();
    //backupAuton();
}
//-----------------------------------------------------------
//Chassis Methods:
void leftDrive(vex::directionType type, int percentage) {
  std::string str = "leftDrive(,";
  str += percentage;
  str += ");";
  autonRecord.push_back(str);

  if(percentage >= 0) {
    percentage = .9*pow(1.043,percentage) - 1 + .4*percentage;
  }
  else {
    percentage = -percentage; // unnecessary?
    percentage = .9*pow(1.043,percentage) - 1 + .4*percentage;
    percentage = -percentage;
  }

  ChassisLF.spin(type, percentage, velocityUnits::pct);
  ChassisLB.spin(type, percentage, velocityUnits::pct);
}
void rightDrive(vex::directionType type, int percentage) {
  std::string str = "rightDrive(,";
  str += percentage;
  str += ");";
  autonRecord.push_back(str);
  if(percentage >= 0) {
    percentage = .9*pow(1.043,percentage) - 1 + .4*percentage;
  }
  else {
    percentage = -percentage; // unnecessary?
    percentage = .9*pow(1.043,percentage) - 1 + .4*percentage;
    percentage = -percentage;
  }

  ChassisRF.spin(type, percentage, velocityUnits::pct);
  ChassisRB.spin(type, percentage, velocityUnits::pct);
}


void chassis_move_for(double rotation, int velocity){
    ChassisLF.setVelocity(velocity,velocityUnits::rpm);
    ChassisLB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRF.setVelocity(velocity,velocityUnits::rpm);
    ChassisLF.setStopping(brakeType::coast);
    ChassisLB.setStopping(brakeType::coast);
    ChassisRF.setStopping(brakeType::coast);
    ChassisRB.setStopping(brakeType::coast);
    ChassisLF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisLB.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRB.rotateFor(rotation,rotationUnits::rev);
}

void chassis_move_coast(double rotation, int velocity){
    ChassisLF.setVelocity(velocity,velocityUnits::rpm);
    ChassisLB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRF.setVelocity(velocity,velocityUnits::rpm);
    ChassisLF.setStopping(brakeType::coast);
    ChassisLB.setStopping(brakeType::coast);
    ChassisRF.setStopping(brakeType::coast);
    ChassisRB.setStopping(brakeType::coast);
    ChassisLF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisLB.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRB.rotateFor(rotation,rotationUnits::rev);
}
void chassis_move(double distance, double velocity){
  double rotation = distance / wheelCircumference; //takes distance in inches and converts to wheel rotations
  
  ChassisLF.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisRB.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisRF.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisLB.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct, true);
}

bool chassis_move_non_blocking(double distance, double velocity){
  bool completed = false;

  double rotation = distance / wheelCircumference; //takes distance in inches and converts to wheel rotations
  
  ChassisLF.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisRB.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisRF.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct,false);
  ChassisLB.rotateFor(rotation,rotationUnits::rev,velocity, velocityUnits::pct, false);

  completed = true;
  return completed;
}

void chassis_move_auton(double rotation, int velocity){
    ChassisLF.setVelocity(velocity,velocityUnits::rpm);
    ChassisLB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRB.setVelocity(velocity,velocityUnits::rpm);
    ChassisRF.setVelocity(velocity,velocityUnits::rpm);

    ChassisLF.setStopping(vex::brakeType::brake);
    ChassisLB.setStopping(vex::brakeType::brake);
    ChassisRF.setStopping(vex::brakeType::brake);
    ChassisRB.setStopping(vex::brakeType::brake);
    
    ChassisLF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisLB.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRF.rotateFor(rotation,rotationUnits::rev,false);
    ChassisRB.rotateFor(rotation,rotationUnits::rev);
}

void leftSpin(double velocity) {
  ChassisLF.setStopping(vex::brakeType::hold);
  ChassisLB.setStopping(vex::brakeType::hold);
  ChassisLF.setVelocity(velocity,velocityUnits::rpm);
  ChassisLB.setVelocity(velocity,velocityUnits::rpm);
  
  ChassisLF.spin(vex::directionType::fwd);
  ChassisLB.spin(vex::directionType::fwd);
}

void rightSpin(double velocity) {
  ChassisRF.setStopping(vex::brakeType::hold);
  ChassisRB.setStopping(vex::brakeType::hold);
  ChassisRF.setVelocity(velocity,velocityUnits::rpm);
  ChassisRB.setVelocity(velocity,velocityUnits::rpm);
  
  ChassisRF.spin(vex::directionType::fwd);
  ChassisRB.spin(vex::directionType::fwd);
}

void turn(double degrees, int velocity) {
  double inches = (2*pi*turningRadius*degrees)/360; // simplifies to pi*turningRadius*degrees/180
  //inches = inches/2; //because for point turn, left and right side move, so you don't want to run it twice
  double rotation = inches/wheelCircumference;


  ChassisLF.setVelocity(velocity,velocityUnits::rpm);
  ChassisLB.setVelocity(velocity,velocityUnits::rpm);
  ChassisRF.setVelocity(velocity,velocityUnits::rpm);
  ChassisRB.setVelocity(velocity,velocityUnits::rpm);
  
  ChassisLF.rotateFor(rotation,rotationUnits::rev,false);
  ChassisLB.rotateFor(rotation,rotationUnits::rev,false);
  ChassisRF.rotateFor(-rotation,rotationUnits::rev,false);
  ChassisRB.rotateFor(-rotation,rotationUnits::rev,true);
}

void chassisStop () {
  ChassisLF.stop(brakeType::brake);
  ChassisLB.stop(brakeType::brake);
  ChassisRF.stop(brakeType::brake);
  ChassisRB.stop(brakeType::brake); 
}


//Chassis encoder reset
void chassisEncoderReset() {
  ChassisLF.resetRotation();
  ChassisLB.resetRotation();
  ChassisRF.resetRotation();
  ChassisRB.resetRotation();  
}


void slowMode() {
  slowModeEnabled = !slowModeEnabled;
  if(slowModeEnabled == false) {
    chassisMultiplier = .7;
  }
  else if(slowModeEnabled == true) {
    chassisMultiplier = .17;
  }
}

/*------------------------------------PID Control Move and Turn----------------------------------------*/

void setChassisLSmooth(int speed){
  double inertia = 0.97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  leftSpin(currentSpeed);
}

void setChassisRSmooth(int speed){
  double inertia = 0.97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  rightSpin(currentSpeed);
}
 
double kP = 0.8; //was 1
double kD = 1; //was 1 , 0.5

void chassisPIDMoveMax(double inches, double maxSpeed){
   double revolutions = inches / (4*pi);//wheel circumference. Assumes radius of 2 in.
   double degrees = revolutions * 360;//How many degrees the wheels need to turn
 
   //double mtrDegrees = (degrees * 6) / 5;//How many degrees the motors need to spin
 
   ChassisLF.resetRotation();
   ChassisRF.resetRotation();
 
   double distanceL = ChassisLF.rotation(rotationUnits::deg);
   double distanceR = ChassisRF.rotation(rotationUnits::deg);
 
   double errorL = degrees - distanceL;
   double errorR = degrees - distanceL;
 
   double lastErrorL = errorL;
   double lastErrorR = errorR;
 
   double proportionalL;
   double proportionalR;
 
   double speedL;
   double speedR;
 
   double derivativeL;
   double derivativeR;
 
   double powerL;
   double powerR;
 
   while(std::abs(errorL) > 20 || std::abs(errorR) > 20) {
     distanceL = ChassisLF.rotation(rotationUnits::deg);
     distanceR = ChassisRF.rotation(rotationUnits::deg);
 
     errorL = degrees - distanceL;
     errorR = degrees - distanceL;
 
     proportionalL = errorL * kP;
     proportionalR = errorR * kP;
 
     speedL = lastErrorL - errorL;
     speedR = lastErrorR - errorR;
 
     derivativeL = -speedL * kD;
     derivativeR = -speedR * kD;
 
     lastErrorL = errorL;
     lastErrorR = errorR;

      
     powerL = Limit((proportionalL + derivativeL) * 0.6, maxSpeed * -1 , maxSpeed);
     powerR = Limit((proportionalR + derivativeR) * 0.6, maxSpeed * -1 , maxSpeed);
 
     setChassisLSmooth(powerL);
     setChassisRSmooth(powerR);
     
     vex::task::sleep(10);
   }

   leftSpin(0);
   rightSpin(0);
 }

 void chassisPIDMove(double inches){
   double revolutions = inches / (4*pi);//wheel circumference. Assumes radius of 2 in.
   double degrees = revolutions * 360;//How many degrees the wheels need to turn
 
   //double mtrDegrees = (degrees * 6) / 5;//How many degrees the motors need to spin
 
   ChassisLF.resetRotation();
   ChassisRF.resetRotation();
 
   double distanceL = ChassisLF.rotation(rotationUnits::deg);
   double distanceR = ChassisRF.rotation(rotationUnits::deg);
 
   double errorL = degrees - distanceL;
   double errorR = degrees - distanceL;
 
   double lastErrorL = errorL;
   double lastErrorR = errorR;
 
   double proportionalL;
   double proportionalR;
 
   double speedL;
   double speedR;
 
   double derivativeL;
   double derivativeR;
 
   double powerL;
   double powerR;
 
   while(std::abs(errorL) > 20 || std::abs(errorR) > 20) {
     distanceL = ChassisLF.rotation(rotationUnits::deg);
     distanceR = ChassisRF.rotation(rotationUnits::deg);
 
     errorL = degrees - distanceL;
     errorR = degrees - distanceL;
 
     proportionalL = errorL * kP;
     proportionalR = errorR * kP;
 
     speedL = lastErrorL - errorL;
     speedR = lastErrorR - errorR;
 
     derivativeL = -speedL * kD;
     derivativeR = -speedR * kD;
 
     lastErrorL = errorL;
     lastErrorR = errorR;

      
     powerL = Limit((proportionalL + derivativeL) * 0.6,-160,160);
     powerR = Limit((proportionalR + derivativeR) * 0.6,-160,160);
 
     setChassisLSmooth(powerL);
     setChassisRSmooth(powerR);
     
     vex::task::sleep(10);
   }

   leftSpin(0);
   rightSpin(0);
 }

//-----------------------------------------------------------
//Button Methods
void upPressed() { //make stack vertical
  autonRecord.push_back( "upPressed();");
  stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
  if(stackInStart == true) {
    StackMotor.setVelocity(40, velocityUnits::pct); //was 20 pct
    LeftGrabber.spin(directionType::fwd, 3, velocityUnits::pct);
    RightGrabber.spin(directionType::fwd, 3, velocityUnits::pct);
    StackMotor.rotateTo(stackUnloadPos, rotationUnits::deg);
    StackMotor.stop(hold);
    LeftGrabber.stop(hold);
    RightGrabber.stop(hold);
    stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
    stackInStart = false;
    stackInUnload = true;
    /*LeftGrabber.spin(directionType::fwd, 5, velocityUnits::pct);
    RightGrabber.spin(directionType::fwd, 5, velocityUnits::pct);
    vex::task::sleep(1500);
    LeftGrabber.stop(hold);
    RightGrabber.stop(hold);*/

    return;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}

void downPressed() { //bring stack down to loading position
  autonRecord.push_back( "downPressed();");
  stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
  if(stackInUnload == true) {
    StackMotor.setVelocity(-200, velocityUnits::rpm);
    StackMotor.rotateTo(stackStartPos, rotationUnits::deg);
    StackMotor.stop(hold);
    stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
    stackInUnload = false;
    stackInStart = true;
    return;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
  
}

void leftPressed() { //for more than seven cubes. not used
  autonRecord.push_back( "leftPressed();");
  stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
  if(stackInStart == true) {
    StackMotor.setVelocity(200, velocityUnits::rpm);
    StackMotor.rotateTo(moreThanSevenPos, rotationUnits::deg);
    StackMotor.stop(hold);
    stackCurrentPos = StackMotor.rotation(rotationUnits::deg);
    stackInStart = false;
    stackInUnload = true;
    return;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}

void rightPressed() { //slow outtake
  autonRecord.push_back( "rightPressed();");
  if(RevGrabberStop) {
    LeftGrabber.spin(directionType::fwd, 25, velocityUnits::pct); //was 15 pct
    RightGrabber.spin(directionType::fwd, 25, velocityUnits::pct);  //was 15 pct
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("SLOW OUTTAKE ON"); 
    RevGrabberStop = false;
    FwdGrabberStop = true;
    outtakeOn = true;
  } 
  else {
    LeftGrabber.stop(brakeType::hold);
    RightGrabber.stop(brakeType::hold);
    RevGrabberStop = true;
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("SLOW OUTTAKE OFF");  
    outtakeOn = false;
  }
}

void xPressed() { //outtake macro
  /*autonRecord.push_back( "xPressed();");
  downPressed();
  rightPressed();
  //chassisPIDMove(-12, 6, .7);;
  chassis_move(-6, 15);*/
  rightPressed();
  autonRecord.push_back( "xPressed();");
  chassis_move_non_blocking(-12, 10);
  downPressed();
}
void yPressed(){
  autonRecord.push_back( "yPressed();");
}

void aPressed() {
    autonRecord.push_back( "aPressed();");
    slowMode();
}

void bPressed() {
  autonomous();
}

void l1Pressed(){ //spin rollers
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("SLOW OUTTAKE OFF");  
  outtakeOn = false;
  autonRecord.push_back( "l1Pressed();");
  if(FwdGrabberStop) {
    LeftGrabber.spin(directionType::rev, 75, velocityUnits::pct);
    RightGrabber.spin(directionType::rev, 75, velocityUnits::pct);
    FwdGrabberStop = false;
    RevGrabberStop = true;
  }
  else {
    LeftGrabber.stop(brakeType::hold);
    RightGrabber.stop(brakeType::hold);
    FwdGrabberStop = true;
  }
}

void l1PressedAuton(){ //spin rollers
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("SLOW OUTTAKE OFF");  
  outtakeOn = false;
  autonRecord.push_back( "l1Pressed();");
  if(FwdGrabberStop) {
    LeftGrabber.spin(directionType::rev, 100, velocityUnits::pct); //was 90
    RightGrabber.spin(directionType::rev, 100, velocityUnits::pct);
    FwdGrabberStop = false;
    RevGrabberStop = true;
  }
  else {
    LeftGrabber.stop(brakeType::hold);
    RightGrabber.stop(brakeType::hold);
    FwdGrabberStop = true;
  }
  
}

void l2Pressed(){ //outtake rollers
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("SLOW OUTTAKE OFF");  
  outtakeOn = false;
  autonRecord.push_back( "l2Pressed();");
  if(RevGrabberStop) {
    LeftGrabber.spin(directionType::fwd, 75, velocityUnits::pct);
    RightGrabber.spin(directionType::fwd, 75, velocityUnits::pct);      
    RevGrabberStop = false;
    FwdGrabberStop = true;
  } 
  else {
    LeftGrabber.stop(brakeType::hold);
    RightGrabber.stop(brakeType::hold);
    RevGrabberStop = true;
  }
  
}

void r1Pressed(){ //bring grabber to tower positions
  autonRecord.push_back( "r1Pressed();");
  grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
  if(grabberAtBottom == true) {
    GrabberLift.setVelocity(120, velocityUnits::pct);
    GrabberLift.rotateTo(lowTowerPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtBottom = false;
    grabberAtLow = true;
    return;
  }
  else if(grabberAtLow == true) {
    GrabberLift.setVelocity(120, velocityUnits::pct);
    GrabberLift.rotateTo(midTowerPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtLow = false;
    grabberAtMid = true;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}

void r1PressedAuton(double extra){ //brings grabbers up extra for flipout
  autonRecord.push_back( "r1PressedAuton(extra);");
  grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
  if(grabberAtBottom == true) {
    GrabberLift.setVelocity(120, velocityUnits::pct);
    GrabberLift.rotateTo(midTowerPos + extra, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtBottom = false;
    grabberAtMid = true;
    return;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}

void r2Pressed(){ //brings grabbers down
  /*autonRecord.push_back( "r2Pressed();");
  grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
  if(grabberAtMid == true) {
    GrabberLift.setVelocity(-75, velocityUnits::pct);
    GrabberLift.rotateTo(lowTowerPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtMid = false;
    grabberAtLow = true;
    return;
  }
  else if(grabberAtLow == true) {
    GrabberLift.setVelocity(-75, velocityUnits::pct);
    GrabberLift.rotateTo(bottomPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtLow = false;
    grabberAtBottom = true;
  }*/
  if(grabberAtLow == true || grabberAtMid == true) {
    GrabberLift.setVelocity(-100, velocityUnits::pct);
    GrabberLift.rotateTo(bottomPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtLow = false;
    grabberAtMid = false;
    grabberAtBottom = true;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}

void r2PressedAuton(){ //brings grabbers up extra for flipout
  autonRecord.push_back( "r1PressedAuton(extra);");
  grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
  if(grabberAtMid == true) {
    GrabberLift.setVelocity(75, velocityUnits::pct);
    GrabberLift.rotateTo(bottomPos, rotationUnits::deg);
    GrabberLift.stop(hold);
    grabberCurrentPos = GrabberLift.rotation(rotationUnits::deg);
    grabberAtMid = false;
    grabberAtBottom = true;
    return;
  }
  else { //FILL IN ELSE CASE NOW BEFORE YOU DO ANYTHING ELSE
    return;
  }
}
//-----------------------------------------------------------
//Button Test Methods

void l1PressedTest(){
  
}
void l2PressedTest(){
  
}

void r1PressedTest(){
  StackMotor.spin(vex::directionType::fwd, 50, velocityUnits::rpm);
  vex::task::sleep(500);
  StackMotor.stop(hold);
}
void r2PressedTest(){
  StackMotor.spin(vex::directionType::rev, 50, velocityUnits::rpm);
  vex::task::sleep(500);
  StackMotor.stop(hold);
}

//Program-Specific Methods
void pre_auton( void ) {

}



void usercontrol( void ) {
  // User control code here, inside the loop
 //Callbacks
    Controller1.ButtonL1.pressed(*l1Pressed);
    Controller1.ButtonL2.pressed(*l2Pressed);
    Controller1.ButtonR1.pressed(*r1Pressed);
    Controller1.ButtonR2.pressed(*r2Pressed);
    Controller1.ButtonUp.pressed(*upPressed);
    Controller1.ButtonDown.pressed(*downPressed);
    Controller1.ButtonLeft.pressed(*leftPressed);
    Controller1.ButtonRight.pressed(*rightPressed);
    Controller1.ButtonX.pressed(*xPressed);
    Controller1.ButtonY.pressed(*yPressed);
    Controller1.ButtonA.pressed(*aPressed);
    Controller1.ButtonB.pressed(*bPressed);
    Controller1.Axis2.changed(*chassisControl);
    Controller1.Axis3.changed(*chassisControl);

  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(StackMotor.rotation(rotationUnits::deg));
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//--------------------------------------------------------------------------------------------------------------


int main() {
    vexcodeInit();
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();

    //Prevent main from exiting with an infinite loop.                        
    //while(1) {
      //chassisControl();
      
      //vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    //}
}