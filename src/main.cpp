#include "main.h"
#include "lemlib/api.hpp"

//
// ===============================
// MOTORS
// ===============================
//

// Left drive
						//port, motor type (speed), reversed
pros::Motor left_motor_a(1, pros::E_MOTOR_GEARSET_BLUE, false);
pros::Motor left_motor_b(2, pros::E_MOTOR_GEARSET_BLUE, false);
pros::Motor left_motor_c(3, pros::E_MOTOR_GEARSET_BLUE, false);

// Right drive
pros::Motor right_motor_a(8, pros::E_MOTOR_GEARSET_BLUE, true);
pros::Motor right_motor_b(9, pros::E_MOTOR_GEARSET_BLUE, true);
pros::Motor right_motor_c(10, pros::E_MOTOR_GEARSET_BLUE, true);

// Intake / outtake
pros::Motor intake_motor(7, pros::E_MOTOR_GEARSET_BLUE, false);
pros::Motor outtake_motor(6, pros::E_MOTOR_GEARSET_BLUE, true);


//
// ===============================
// MOTOR GROUPS
// ===============================
//

pros::MotorGroup left_drive({left_motor_a, left_motor_b, left_motor_c});
pros::MotorGroup right_drive({right_motor_a, right_motor_b, right_motor_c});


//
// ===============================
// CONTROLLER
// ===============================
//

pros::Controller controller(pros::E_CONTROLLER_MASTER);


//
// ===============================
// PNEUMATICS
// ===============================
//
							//port
pros::ADIDigitalOut wing_piston('A');
pros::ADIDigitalOut matchloader_piston('B');
pros::ADIDigitalOut midscore_piston('C');


//
// ===============================
// LEMLIB DRIVETRAIN SETUP
// ===============================
//

// Track width and wheel diameter MUST be tuned
lemlib::Drivetrain drivetrain(
    &left_drive,
    &right_drive,
    10,//distance between left/right wheels
    lemlib::Omniwheel::NEW_4, // wheel type, change based on what we have
    600, //max chassis rpm - Ask about this to like sanjo
    2 //Drift (CHANGE!!!)
);

/*=================
     ODOM STUFF
====================*/

pros::Imu imu(10); // change port later

// No sensors yet (add later)
lemlib::OdomSensors sensors(
    nullptr, // Left tracking wheel sensor (not used yet)
    nullptr, // Right tracking wheel sensor (not used yet)
    nullptr, // Back/lateral tracking wheel sensor (not used yet)
    &imu, // Optional IMU for heading (not used yet)
    nullptr  // Extra/unused sensor slot (not used yet)
);



lemlib::ControllerSettings lateral_controller(
    10,   // kP (proportional gain)
    0,    // kI (integral gain)

    3,    // kD (derivative gain)

    0,    // anti-windup

    1,    // small error range (inches)

    100,  // small error timeout (ms)

    3,    // large error range (inches)

    500,  // large error timeout (ms)

    20    // max acceleration (slew rate)
);

//Same for this controller
lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3, 1, 100, 3, 500, 20
);

lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);


//
// ===============================
// INITIALIZE
// ===============================
//

void initialize() {

    left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    wing_piston.set_value(true);
    matchloader_piston.set_value(false);
    midscore_piston.set_value(true);
}


//
// ===============================
// AUTONOMOUS
// ===============================
//

void autonomous() {
    pros::lcd::print(0, "AUTONOMOUS");

}


//
// ===============================
// DRIVER CONTROL
// ===============================
//

bool matchloader_pressed = false;
bool wing_pressed = false;
bool midscore_pressed = false;

void opcontrol() {

    while (true) {

        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int turning = controller.get_analog(ANALOG_RIGHT_X);

        double scale = 0.8;

        left_drive.move((forward + turning) * scale);
        right_drive.move((forward - turning) * scale);

        // ---------------------------
        // INTAKE
        // ---------------------------
        if (controller.get_digital(DIGITAL_R1))
            intake_motor.move(-127);
        else if (controller.get_digital(DIGITAL_L1))
            intake_motor.move(127);
        else
            intake_motor.brake();

        // ---------------------------
        // OUTTAKE
        // ---------------------------
        if (controller.get_digital(DIGITAL_R2))
            outtake_motor.move(-127);
        else if (controller.get_digital(DIGITAL_L2))
            outtake_motor.move(127);
        else
            outtake_motor.brake();

        // ---------------------------
        // PNEUMATICS TOGGLES
        // ---------------------------

        if (controller.get_digital(DIGITAL_X) && !matchloader_pressed) {
            matchloader_piston.set_value(!matchloader_piston.get_value());
            matchloader_pressed = true;
        }
        if (!controller.get_digital(DIGITAL_X))
            matchloader_pressed = false;

        if (controller.get_digital(DIGITAL_A) && !wing_pressed) {
            wing_piston.set_value(!wing_piston.get_value());
            wing_pressed = true;
        }
        if (!controller.get_digital(DIGITAL_A))
            wing_pressed = false;

        if (controller.get_digital(DIGITAL_Y) && !midscore_pressed) {
            midscore_piston.set_value(!midscore_piston.get_value());
            midscore_pressed = true;
        }
        if (!controller.get_digital(DIGITAL_Y))
            midscore_pressed = false;

        pros::delay(20);
    }
}
