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

vex::motor LF = vex::motor(vex :: PORT11);
vex::motor RF = vex::motor(vex :: PORT1, true);
vex::motor LB = vex::motor(vex :: PORT20);
vex::motor RB = vex::motor(vex :: PORT10, true);
vex::controller Controller1 = vex::controller();
vex::inertial InertialSensor = vex::inertial(vex:: PORT19);
inertial::quaternion  Inertial_quaternion;
vex::competition Competition;


const double DEADZONE = 0.02;
static double smooth_power = 0.2;
const double JOY_SCALE = 127.0;
const int numMotors = 2;
static double cur_lp = 0.;
static double cur_rp = 0.;
motor leftmotors[] {LF, LB};
motor rightmotors[] {RF, RB};

void spin_motors(double lp, double rp) {
    for (int i = 0; i < numMotors; i++) {
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

/**
Drives the robot parallel to a wall for a certain amount of distance. Uses PID controls to initially find a wall to drive parallel to.
*/
void autonomous (void)
{
  double error =  10 - RangeFinderA.distance(inches);
  double kp = 1.0;
  double ki = 1.0;
  double kd = 1.0;
  double integral = 0;
  double derivative = 0;
  double integralmax = 10.0;
  double previous_error = 0;
  double dt = 0.5;
  double distanceTravelled = 0;
  while(distanceTravelled < 50)
  {
    error =  10 - RangeFinderA.distance(inches);
    integral += error; 
    if(error==0)
    {
      integral = 0;
    }
    if(fabs(error) >= integralmax)
    {
      integral = 0;
    }
    derivative = error - previous_error;
    previous_error = error;
    double speed = kp * error + ki * integral;
    spin_motors(speed, speed);
    distanceTravelled += speed*dt;
  }

  while(fabs(error) < 0.05)
  {
    spin_motors(70, 70);
  }

  
  /*while(RangeFinderA.distance(inches) > 10)
  {
    spin_motors(70, 100);
  }*/

  /*while(RangeFinderA.distance(inches) < 10)
  {
    spin_motors(100, 70);
  }*/ 
}

void lifter(void)
{
  //lift.spin(vex::directionType::fwd, Controller1.Axis3.position(), vex::velocityUnits::pct);
}

void openclaw(void)
{
  //claw.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
}

void closeclaw(void)
{
  //claw.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
}



void stopallmotors(void)
{
  LF.stop();
  LB.stop();
  RF.stop();
  RB.stop();
  //lift.stop();
  //claw.stop();
}

void calibrateInertial()
{
  InertialSensor.calibrate();
}

void printInertialInfo()
{
  
  Inertial_quaternion = InertialSensor.orientation();

  Brain.Screen.clearScreen();        

  Brain.Screen.setFont( mono15 );
  Brain.Screen.setPenColor( white );
  Brain.Screen.setFillColor( black );
  
  Brain.Screen.printAt( 20,  30, "GX  %8.3f", InertialSensor.gyroRate( xaxis, dps ) );
  Brain.Screen.printAt( 20,  45, "GY  %8.3f", InertialSensor.gyroRate( yaxis, dps ) );
  Brain.Screen.printAt( 20,  60, "GZ  %8.3f", InertialSensor.gyroRate( zaxis, dps ) );

  Brain.Screen.printAt( 20,  90, "AX  %8.3f", InertialSensor.acceleration( xaxis ) );
  Brain.Screen.printAt( 20, 105, "AY  %8.3f", InertialSensor.acceleration( yaxis ) );
  Brain.Screen.printAt( 20, 120, "AZ  %8.3f", InertialSensor.acceleration( zaxis ) );

  Brain.Screen.printAt( 20, 150, "A   %8.3f", Inertial_quaternion.a );
  Brain.Screen.printAt( 20, 165, "B   %8.3f", Inertial_quaternion.b );
  Brain.Screen.printAt( 20, 180, "C   %8.3f", Inertial_quaternion.c );
  Brain.Screen.printAt( 20, 195, "D   %8.3f", Inertial_quaternion.d );

  Brain.Screen.printAt( 150, 30, "Roll     %7.2f", InertialSensor.roll() );
  Brain.Screen.printAt( 150, 45, "Pitch    %7.2f", InertialSensor.pitch() );
  Brain.Screen.printAt( 150, 60, "Yaw      %7.2f", InertialSensor.yaw() );

  Brain.Screen.printAt( 150, 90, "Heading  %7.2f", InertialSensor.heading() );
  Brain.Screen.printAt( 150,105, "Rotation %7.2f", InertialSensor.rotation() );

  if( InertialSensor.isCalibrating() )
    Brain.Screen.printAt( 20,225, "Calibration  In Progress" );
  else
    Brain.Screen.printAt( 20,225, "Calibration  Done" );

  Brain.Screen.render();

  // Allow other tasks to run
  this_thread::sleep_for(10);
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
  printInertialInfo();
}

void usercontrol (void)
{
    calibrateInertial();
    while(1)
    {
      //Controller1.Axis2.changed(forwardbackward); //drive forward and backward
      //Controller1.Axis1.changed(leftright); //turn left and right
      Controller1.Screen.clearScreen();
      drive();
      //Brain.Screen.print(ultrasonic.distance(mm));
      Controller1.Axis3.changed(lifter);
      if(Controller1.ButtonRight.pressing())
      {
        closeclaw();
      }
      else if(Controller1.ButtonLeft.pressing())
      {
        openclaw();
      }
      Controller1.ButtonLeft.released(stopallmotors);
      Controller1.ButtonRight.released(stopallmotors);
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
  vexcodeInit();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while(1)
  {
    vex::task::sleep(100);
  }
  /*while (true) 
  {
    Controller1.Screen.print("%.2f", RangeFinderA.distance(inches));
    wait(200, msec);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearScreen();
    wait(5, msec);
  }*/
}