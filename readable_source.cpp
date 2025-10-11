#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;
 

// device configuration

// brain Brain;

controller Controller1;

motor frontleft = motor(PORT2, ratio6_1, true);
motor frontright = motor(PORT9, ratio6_1, false);
motor backleft = motor(PORT12, ratio6_1, false);
motor backright = motor(PORT20, ratio6_1, true);
 

motor intakebottom(PORT1, ratio6_1, true);
motor intakemiddle(PORT10, ratio6_1, false);
motor intaketop(PORT11, ratio6_1, true);

motor storage(PORT3, ratio6_1, false);

digital_out scraper(Brain.ThreeWirePort.A);


// Intake/storage mode enums

enum Mode { NONE, A_MODE, B_MODE, R1_MODE, R2_MODE, DOWN_MODE, RIGHT_MODE};

Mode activeMode = NONE;



// previous button states (used later)

bool preva = false, prevb = false;

bool prevy = false, prevr2 = false;

bool prevdown = false, prevr1 = false;

bool prevright = false;

void DriveForward(double dist) {
  frontleft.spinFor(forward, dist * 360, degrees, false);
  frontright.spinFor(forward, dist * 360, degrees, false);
  backleft.spinFor(forward, dist * 360, degrees, false);
  backright.spinFor(forward, dist * 360, degrees, false);
  wait(2, seconds);
}
void Turn(double degree) {
  frontleft.spinFor(forward, degree * 3, degrees, false);
  frontright.spinFor(reverse, degree * 3, degrees, false);
  backleft.spinFor(forward, degree * 3, degrees, false);
  backright.spinFor(reverse, degree * 3, degrees, false);
  wait(2, seconds);
}
int main() {
  intakebottom.setStopping(brake);
  intakemiddle.setStopping(brake);
  intaketop.setStopping(brake);

  intakebottom.setVelocity(100, percent);
  intakemiddle.setVelocity(100, percent);
  intaketop.setVelocity(100, percent);

  frontleft.setVelocity(100, percent);
  frontright.setVelocity(100, percent);
  backleft.setVelocity(100, percent);
  backright.setVelocity(100, percent);
  //autonomous code
  while (!Controller1.ButtonUp.pressing()) {
    // if (Controller1.ButtonUp.pressing()) {
    Brain.Timer.clear();
      // break;
    
  }
  // go forward 4-ish feet and intake blocks
  // int autonForward = 720;
  DriveForward(4);
  intakebottom.spin(forward);
  intakemiddle.spin(forward);
  intaketop.spin(forward);
  //turn about 30 degrees

  Turn(30.0);
  scraper.set(false);
  DriveForward(2.236);

  wait(5, seconds);
  intakebottom.stop();
  intakemiddle.stop();
  intaketop.stop();
  DriveForward(-0.4);
  intakebottom.spin(forward);
  intakemiddle.spin(forward);
  intaketop.spin(reverse);
  Turn(130.0);
  DriveForward(1.7);
  storage.spin(forward);
  while (true) {
    if (Brain.Timer.time(seconds) > 14.5) {
      intakebottom.stop();
      intakemiddle.stop();
      intaketop.stop();
      storage.stop();
      break;
    }
  }

  //autonomous over
  frontleft.spin(forward);
  frontright.spin(forward);
  backleft.spin(forward);
  backright.spin(forward);
  //driver control starts
  while (true) {



    frontleft.setVelocity(Controller1.Axis3.position()+Controller1.Axis1.position(), percent);
    frontright.setVelocity(Controller1.Axis3.position()-Controller1.Axis1.position(), percent);
    backleft.setVelocity(Controller1.Axis3.position()+Controller1.Axis1.position(), percent);
    backright.setVelocity(Controller1.Axis3.position()-Controller1.Axis1.position(), percent);

    // current button states, if they are being pressed.
    bool curra = Controller1.ButtonA.pressing();
    bool currb = Controller1.ButtonB.pressing();
    bool currr1 = Controller1.ButtonR1.pressing();
    bool currr2 = Controller1.ButtonR2.pressing();
    // bool currl1 = Controller1.ButtonL1.pressing();
    // bool currx = Controller1.ButtonX.pressing();
    bool curry = Controller1.ButtonY.pressing();
    bool currdown = Controller1.ButtonLeft.pressing();
    bool currright = Controller1.ButtonRight.pressing();
 

    // intake toggle logic
    ////long goal scoring
    if ((curra == true) && (preva == false)) {
      activeMode = (activeMode == A_MODE) ? NONE : A_MODE;
      if (activeMode == A_MODE) {
        intakebottom.spin(forward, 100, percent);
        intaketop.spin(reverse, 100, percent);
        intakemiddle.spin(forward, 100, percent);
        storage.spin(forward, 100, percent);
      }
      else {
        intakebottom.stop();
        intakemiddle.stop();
        intaketop.stop();
        storage.stop();
      }
    }
    ////upper center goal scoring 
    if ((currb == true) && (prevb == false)) {
      activeMode = (activeMode == B_MODE) ? NONE : B_MODE;
        if (activeMode == B_MODE) {
        intakebottom.spin(forward, 100, percent);
        // intaketop.spin(reverse, 100, percent);
        intakemiddle.spin(reverse, 100, percent);
        storage.spin(forward, 100, percent);
        intaketop.stop();
      }
      else {
        intakebottom.stop();
        intakemiddle.stop();
        // intaketop.stop();
        storage.stop();
      }
    }
    ////into storage intake
    if ((currr1 == true) && (prevr1 == false)) {
      activeMode = (activeMode == R1_MODE) ? NONE : R1_MODE;
      if (activeMode == R1_MODE) {
        intaketop.spin(forward, 100, percent);
        intakebottom.spin(forward, 100, percent);
        intakemiddle.spin(forward, 100, percent);
        storage.stop();
      }
      else {
        intakebottom.stop();
        intakemiddle.stop();
        intaketop.stop();
      }
    } 
    ////lower center goal scoring
    if ((currr2 == true) && (prevr2 == false)) {
      activeMode = (activeMode == R2_MODE) ? NONE : R2_MODE;
      if (activeMode == R2_MODE) {
        storage.spin(forward, 100, percent);
        intakebottom.spin(reverse, 100, percent);
        intakemiddle.spin(reverse, 100, percent);
        intaketop.spin(reverse, 100, percent);
      }
      else {
        intakebottom.stop();
        storage.stop();
        intakemiddle.stop();
        intaketop.stop();
      }
    } 
    //storage forward
    if ((currdown == true) && (prevdown == false)) {
      activeMode = (activeMode == DOWN_MODE) ? NONE : DOWN_MODE;
      if (activeMode == DOWN_MODE) {
        storage.spin(forward, 100, percent);
      }
      else {
        storage.stop();
      }
    } 
    //storage backwards
    if ((currright == true) && (prevright == false)) {
      activeMode = (activeMode == RIGHT_MODE) ? NONE : RIGHT_MODE;
      if (activeMode == RIGHT_MODE) {
        storage.spin(reverse, 100, percent);
      }
      else {
        storage.stop();
      }
    } 

 

 

 

    // update previous button states
    preva = curra;
    prevb = currb;
    prevr1 = currr1;
    prevr2 = currr2;
    // prevl1 = currl1;
    // prevx = currx;
    prevy = curry;
    prevdown = currdown;
    prevright = currright;
 

    // pneumatic control
    if (Controller1.ButtonL1.pressing()) {
      scraper.set(false);
    } else if (Controller1.ButtonL2.pressing()) {
      scraper.set(true);
    }

    wait(20, msec);
  }

}

 