/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\rober                                            */
/*    Created:      Sat Nov 30 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

vex::competition Competition;
vex::motor LF = vex::motor(vex :: PORT1);
vex::motor RF = vex::motor(vex :: PORT10, true);
vex::motor LB = vex::motor(vex :: PORT11);
vex::motor RB = vex::motor(vex :: PORT20, true);
vex::motor liftleft = vex::motor(vex :: PORT2);
vex::motor liftright = vex::motor(vex :: PORT9, true);
vex::motor claw = vex::motor(vex :: PORT19);
vex::controller Controller1 = vex::controller();

const double DEADZONE = 0.02;
static double smooth_power = 0.55;
const double JOY_SCALE = 127.0;
static double cur_lp = 0.;
static double cur_rp = 0.;
motor leftmotors[] {LF, LB};
motor rightmotors[] {RF, RB};

void spin_motors(double lp, double rp) {
    for (int i = 0; i < 2; i++) {
        leftmotors[i].spin(vex::directionType::fwd, lp, percentUnits::pct);
        rightmotors[i].spin(vex::directionType::fwd, rp, percentUnits::pct);
    }
}

double scalejoystick(double input)
{
  double result;
	if (input > DEADZONE) {
		result = fmin((input - DEADZONE) / (1.0 - DEADZONE), 1.0);
		result *= pow(result, smooth_power);  // fine control, optional
	}
  else {
    return 0.;
  }
	return result;
}

void lifter(void)
{
  liftleft.spin(vex::directionType::fwd, Controller1.Axis3.position(), vex::velocityUnits::pct);
  liftright.spin(vex::directionType::fwd, Controller1.Axis3.position(), vex::velocityUnits::pct);
}

void openclaw(void)
{
  claw.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
}

void closeclaw(void)
{
  claw.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
}

void stopclaw(void)
{
  claw.stop();
}

void stopallmotors(void)
{
  LF.stop();
  LB.stop();
  RF.stop();
  RB.stop();
  liftleft.stop();
  liftright.stop();
  claw.stop();
}

void drive(void)
{
  double px = Controller1.Axis1.value();  //Gets the value of the joystick axis on a scale from -127 to 127.
  double py = Controller1.Axis2.value();
    
  double d = sqrt(px*px + py*py) / JOY_SCALE; // distance from the origin, 0 to ~ 1
  double scale = scalejoystick(d);  // rescale that distance
  px *= scale / JOY_SCALE;  // rescale to fraction 0 to ~1
  py *= scale / JOY_SCALE;
  double lp = py + px;  // from 0 to ~2
  double rp = py - px;

  // if |motor power| > 1, rescale them both 
  double mapow = fmax(fabs(lp), fabs(rp));
  if (mapow > 1.0) {
      lp /= mapow;
      rp /= mapow;
  }
  
  lp *= 100; // turn into percent, for motor input
  rp *= 100;
  
  if (lp != cur_lp || rp != cur_rp) {  //   
      spin_motors(lp, rp);
      cur_lp = lp;
      cur_rp = rp;
  }
}

void autonomous(void)
{
  /*for(int i = 0; i < 2; i++)
  {
    leftmotors[i].spin(fwd, 100, velocityUnits::pct);
    rightmotors[i].spin(fwd, 100, velocityUnits::pct);
  }*/
}

void usercontrol (void)
{
    while(1)
    {
      //Controller1.Axis2.changed(forwardbackward); //drive forward and backward
      //Controller1.Axis1.changed(leftright); //turn left and right
      drive();
      Controller1.Axis3.changed(lifter);
      if(Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing())
      {
        closeclaw();
      }
      else if(Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing())
      {
        openclaw();
      }
      Controller1.ButtonL1.released(stopclaw);
      Controller1.ButtonL2.released(stopclaw);
      Controller1.ButtonR2.released(stopclaw);
      Controller1.ButtonR1.released(stopclaw);
      //Turn left and right
      //LF.spin(vex::directionType::rev, Controller1.ButtonLeft.pressing(), vex::velocityUnits::pct);
      //RF.spin(vex::directionType::fwd, Controller1.ButtonLeft.pressing(), vex::velocityUnits::pct);
      //LB.spin(vex::directionType::rev, Controller1.ButtonLeft.pressing(), vex::velocityUnits::pct);
      //RB.spin(vex::directionType::fwd, Controller1.ButtonLeft.pressing(), vex::velocityUnits::pct);
      vex::task::sleep(20);
    }
    
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  while(1)
  {
    vex::task::sleep(100);
  }
}
