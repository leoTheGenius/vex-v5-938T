#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
// #include "pros/rotation.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
#include <cmath>
#include <algorithm>

pros::MotorGroup leftdrive({3, 2, 1}, pros::v5::MotorGears::blue);
pros::MotorGroup rightdrive({-9, -8, -10}, pros::v5::MotorGears::blue);
pros::Motor intake(5, pros::v5::MotorGears::blue);
pros::Motor outtake(-7, pros::v5::MotorGears::blue);

// pros::Rotation pidright(4);
// pros::Rotation pidleft(6);
enum Mode { no, a, b, r1, r2, down, right, x, y, l1, l2, up, left};

Mode activeMode = no;
// some constants, im not sure about wheelgap though as we haven't put it on the bot yet
const double wheeldiameter = 3.25;
const double ticksperspin = 360.0;
const double tickpin = ticksperspin / (M_PI * wheeldiameter);
const double wheelgap = 12.5;  // dist between pid wheel,im just using drivetrain wheels as pid wheels

struct pidstuff {
	double leftpos = 0;
	double rightpos = 0;
	
	void reset() {
		leftpos = 0;
		rightpos = 0;
	}
	
	void update() {
		leftpos = leftdrive.get_position();
		rightpos = rightdrive.get_position();
	}
	void updateturn() { // positive is turning right
		leftpos = leftdrive.get_position();
		rightpos = rightdrive.get_position();
	}

	double turndegrees(double width) {
		return avgturn() * wheeldiameter / width;
	}
	
	double avgpos() {
		return (leftpos + rightpos) / 2.0;
	}
	double avgturn() {
		return (leftpos - rightpos) / 2.0;
	}
	double diff() {
		return rightpos - leftpos;
	}
};
double drivewheelgap = 12.5;
int autonchoice = 0;


void initialize() {
	//by the way, what does this even do?
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	//this is the amazing auton selector. sadly i dont have an auton to use it on yet. also, how do i test it without the field control system?
	master.print(0, 0, "select auton:");
    master.print(1, 0, "a=left b=right y=skils otherwise none by default");


    while (pros::competition::is_disabled()) {
        if (master.get_digital_new_press(DIGITAL_A)) {
            autonchoice = 1;
            master.print(2, 0, "auton left   ");
        }
        if (master.get_digital_new_press(DIGITAL_B)) {
            autonchoice = 2;
            master.print(2, 0, "auton right  ");
        }
        if (master.get_digital_new_press(DIGITAL_Y)) {
            autonchoice = 3;
            master.print(2, 0, "auton skills ");
        }
        pros::delay(20);
    }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 //should i remove speed?
 //actually no, for small spaces speed slow  might be goooooooooooooooooo0o00o0o00o00o0o000o000o0000o0o00oo00o0o0o0o0o0o00o0o0o0o0o0o0o0o0d
void driveforward(double dist, double speed) {//!inches
	leftdrive.tare_position();
	rightdrive.tare_position();
	
	pidstuff pid;

	// idk the tuning stuff isnt tuned
	const double kppos = 0.37;
	const double kipos = 0.04;
	const double kdpos = 0.08;

	const double kpdrift = 0.3;
	const double kidrift = 0.02;
	const double kddrift = 0.03;
 
	const double posrange = 0.2; //!in tolerance for endpoint
	const int cyclereq = 15;


	const double toldeg = posrange * tickpin; 
	const double veloestimthresh = 1.0 * tickpin; 

	// target distance in degrees
	double targetdeg = dist * tickpin;

	double posintegral = 0.0;
	double pospreverr = 0.0;
	double driftintegral = 0.0;
	double driftpreverror = 0.0;

	int stablecycles = 10;
	std::int32_t prevtime = pros::millis();

	while (true) {
		pid.update();
		std::int32_t currtime = pros::millis();
		double dt = (currtime - prevtime) / 1000.0;
		if (dt <= 0.0) dt = 0.01;
		prevtime = currtime;

		double pos = pid.avgpos(); // wheel degrees
		double poserror = targetdeg - pos; // degrees error
		if (poserror < 0) {
			if (posintegral > 0) {
				posintegral = 0.0;
			}
		}
		posintegral += poserror * dt;
		double posderiv = (poserror - pospreverr) / dt;
		pospreverr = poserror;

		double posoutput = kppos * poserror + kipos * posintegral + kdpos * posderiv;

		double speedlimit = std::clamp(std::abs(speed), 1.0, 127.0);
		posoutput = std::clamp(posoutput, -speedlimit, speedlimit);

		double drifterror = pid.diff();
		driftintegral += drifterror * dt;
		double driftderiv = (drifterror - driftpreverror) / dt;
		driftpreverror = drifterror;

		double driftoutput = kpdrift * drifterror + kidrift * driftintegral + kddrift * driftderiv;
		driftoutput = std::clamp(driftoutput, -40.0, 40.0);

		int leftvoltage = (int)std::round(posoutput + driftoutput);
		int rightvoltage = (int)std::round(posoutput - driftoutput);
		leftvoltage = std::clamp(leftvoltage, -127, 127);
		rightvoltage = std::clamp(rightvoltage, -127, 127);

		leftdrive.move(leftvoltage*0.8);
		rightdrive.move(rightvoltage*0.8);

		double velestim = posderiv; //! degrees/sec estimate
		if (std::abs(poserror) < toldeg && std::abs(velestim) < veloestimthresh) {
			stablecycles++;
		} else {
			stablecycles = 0;
		}
		if (stablecycles >= cyclereq) break;

		pros::delay(10);
	}

	leftdrive.move(0);
	rightdrive.move(0);
}
void turn(double degrees, double speed){
	// reset drivetrain motor encoder positions before starting the turn
	leftdrive.tare_position();
	rightdrive.tare_position();

	// pidleft.reset_position();
	// pidright.reset_position();

	pidstuff pid;
	pid.reset();

	const double kpturn = 0.7;
	const double kiturn = 0.0;
	const double kdturn = 0.1;

	const double angletol = 5.0; //! degrees range to be considerd finished wit hturn
	const int stablereq = 15;

	double integral = 0.0;
	double preverror = 0.0;
	int stable = 0;
	std::int32_t prevt = pros::millis();
	std::int32_t startt = prevt;
	const std::int32_t timeout_ms = 8000; // safety timeout to avoid infinite spin

	while (true) {
		pid.updateturn();

		std::int32_t now = pros::millis();
		double dt = (now - prevt) / 1000.0;
		if (dt <= 0.0) dt = 0.01;
		prevt = now;

		double heading = pid.turndegrees(drivewheelgap);
		double err = degrees - heading;

		// Diagnostic output to controller for debugging
		master.print(2, 0, "L: %.2f R: %.2f  ", pid.leftpos, pid.rightpos);
		master.print(3, 0, "hdg: %.2f err: %.2f dt: %.3f", heading, err, dt);

		// safety timeout
		if ((now - startt) > timeout_ms) {
			master.print(4, 0, "Turn timeout (ms=%d)", timeout_ms);
			break;
		}

		integral += err * dt;
		double deriv = (err - preverror) / dt;
		preverror = err;

		double out = kpturn * err + kiturn * integral + kdturn * deriv;

		double speedlim = std::clamp(std::abs(speed), 1.0, 127.0);
		out = std::clamp(out, -speedlim, speedlim);

		int leftvolt = (int)std::round(out);
		int rightvolt = (int)std::round(-out);
		leftvolt = std::clamp(leftvolt, -127, 127);
		rightvolt = std::clamp(rightvolt, -127, 127);

		leftdrive.move(leftvolt);
		rightdrive.move(rightvolt);

		double anglevelocity = deriv; // deg sec
		if (std::abs(err) < angletol && std::abs(anglevelocity) < 5.0) {
			stable++;
		} else {
			stable = 0;
		}
		if (stable >= stablereq) {
			break;
		};

		pros::delay(10);
	}

	leftdrive.move(0);
	rightdrive.move(0);
}
void autonomous() {
	leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	outtake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	if (autonchoice == 0) {
		driveforward(24, 127);
//default auton/no auton selected
	}
	if (autonchoice == 1) {
//left auton
    } else if (autonchoice == 2) {
//right auton
    } else if (autonchoice == 3) {
//skills auton
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	outtake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	turn(90, 127);
	leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	outtake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	bool preva = false, prevb = false, prevy = false, prevx = false, prevleft = false, prevright = false, prevup = false, prevdown = false, prevl1 = false, prevl2 = false, prevr1 = false, prevr2 = false;
	bool curra = false, currb = false, curry = false, currx = false, currleft = false, currright = false, currup = false, currdown = false, currl1 = false, currl2 = false, currr1 = false, currr2 = false;
	while (true) {
		int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		leftdrive.move(forward + turn);
		rightdrive.move(forward - turn);
		curra = master.get_digital(DIGITAL_A);
		currb = master.get_digital(DIGITAL_B);
		curry = master.get_digital(DIGITAL_Y);
		currx = master.get_digital(DIGITAL_X);
		currleft = master.get_digital(DIGITAL_LEFT);
		currright = master.get_digital(DIGITAL_RIGHT);
		currup = master.get_digital(DIGITAL_UP);
		currdown = master.get_digital(DIGITAL_DOWN);
		currl1 = master.get_digital(DIGITAL_L1);
		currl2 = master.get_digital(DIGITAL_L2);
		currr1 = master.get_digital(DIGITAL_R1);
		currr2 = master.get_digital(DIGITAL_R2);
		if ((curra == true) && (preva == false)) {
      		activeMode = (activeMode == a) ? no : a;
      		if (activeMode == a) {
        		intake.move_velocity(600);
				outtake.move_velocity(0);
    		} else {
				intake.move_velocity(0);
				outtake.move_velocity(0);
	  		}
		}
		else if ((currleft == true) && (prevleft == false)) {
			activeMode = (activeMode == left) ? no : left;
			if (activeMode == left) {
				outtake.move_velocity(600);
				intake.move_velocity(600);
			} else {
				outtake.move_velocity(0);
				intake.move_velocity(0);
			}
		}
		else if ((currb == true) && (prevb == false)) {
			activeMode = (activeMode == b) ? no : b;
			if (activeMode == b) {
				intake.move_velocity(-600);
				outtake.move_velocity(-600);
			} else {
				intake.move_velocity(0);
				outtake.move_velocity(0);
			}
		}
		else if ((currdown == true) && (prevdown == false)) {
			activeMode = (activeMode == down) ? no : down;
			if (activeMode == down) {
				outtake.move_velocity(-600);
				intake.move_velocity(0);
			} else {
				outtake.move_velocity(0);
				intake.move_velocity(0);
			}
		}
		
        
		// update previous button states
		preva = curra;
		prevb = currb;
		prevy = curry;
		prevx = currx;
		prevleft = currleft;
		prevright = currright;
		prevup = currup;
		prevdown = currdown;
		prevl1 = currl1;
		prevl2 = currl2;
		prevr1 = currr1;
		prevr2 = currr2;

		pros::delay(20);
	}
}




