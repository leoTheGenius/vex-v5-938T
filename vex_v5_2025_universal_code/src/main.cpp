#include "main.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftdrive({3, 2, 1}, pros::v5::MotorGears::blue);
pros::MotorGroup rightdrive({-9, -8, -10}, pros::v5::MotorGears::blue);
pros::Motor intake(5, pros::v5::MotorGears::blue);
pros::Motor outtake(7, pros::v5::MotorGears::blue);

pros::Rotation odometryright(4);
pros::Rotation odometryleft(6);

// some constants, im not sure about wheelgap though as we havent put it on the bot yet
const double wheeldiameter = 2;
const double ticksperspin = 360.0;
const double tickperinch = ticksperspin / (M_PI * wheeldiameter);
const double wheelgap = 1;  // dist between odom wheel

struct odomstuff {
	double leftpos = 0;
	double rightpos = 0;
	
	void reset() {
		leftpos = 0;
		rightpos = 0;
	}
	
	void update() {
		leftpos = odometryleft.get_position() / 100.0 / tickperinch;
		rightpos = odometryright.get_position() / 100.0 / tickperinch;
	}
	
	double avgpos() {
		return (leftpos + rightpos) / 2.0;
	}
	
	double dif() { //this made me think of biff and eho from math is cool problems
		return rightpos - leftpos;
	}
};

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
void drive_forward(double dist, double speed) {
	//so this resets the odometry positions to 0
	odometryleft.reset_position();
	odometryright.reset_position();
	
	odomstuff odom;
	int target_ticks = (int)(dist * tickperinch);
	//constants for drift might not work idk idk i dont know what im doing blah blah blah im the goat
	const double drivemultiply = 1.2;
	const double driftmultiply = 0.8;
	const double minpow = 0.1;
	const double maxpow = 1.0;

	while (odom.avgpos() < dist) {
		odom.update();
		
		double current_position = odom.avgpos();
		double position_error = dist - current_position;
		double drift = odom.dif();
		
		// drive power
		double base_power = (speed / 127.0) * (position_error / dist);
		base_power = fmax(minpow, fmin(maxpow, base_power));
		
		// drift correction
		double drift_correction = -drift * driftmultiply / 127.0;
		drift_correction = fmax(-0.3, fmin(0.3, drift_correction));
		
		int left_voltage = (int)(127.0 * (base_power + drift_correction));
		int right_voltage = (int)(127.0 * (base_power - drift_correction));
		
		leftdrive.move(left_voltage);
		rightdrive.move(right_voltage);
		
		pros::delay(10);
	}
	
	leftdrive.move(0);
	rightdrive.move(0);
}
void autonomous() {
	if (autonchoice == 0) {
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
	leftdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightdrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
		if (curra && !preva) {
			intake.move_velocity(600);
		} else if (!curra && preva) {
			intake.move_velocity(0);
		}
		if (currb && !prevb) {
			intake.move_velocity(-600);
		} else if (!currb && prevb) {
			intake.move_velocity(0);
		}
		if (curry && !prevy) {
			outtake.move_velocity(600);
		} else if (!curry && prevy) {
			outtake.move_velocity(0);
		}
		if (currx && !prevx) {
			outtake.move_velocity(-600);
		} else if (!currx && prevx) {
			outtake.move_velocity(0);
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
