#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11,-12,-13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14,15,16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(1);
pros::Rotation rotation(2);
// motor configs
pros::Motor intake(9);
pros::Controller controller1();
pros::Motor hangLeft(7);
pros::Motor hangRight(8);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              460, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void auton1(){
	//right Side red, working?
		static pros::adi::DigitalOut clamp('A');
		clamp.set_value(false);
		chassis.setPose(-63, -24, 90);
	chassis.moveToPoint(-63, -24, 3000, {.forwards = false, .maxSpeed = 60});
	chassis.moveToPoint(-45, -24, 3000, {.forwards = false, .maxSpeed = 60});
	chassis.moveToPoint(-36, -24, 3000, {.forwards = false, .maxSpeed = 30});
		pros::delay(1000);
		clamp.set_value(true);
		pros::delay(10000);
		intake.move(127);
	chassis.moveToPoint(-23.465, -47, 3000,{.maxSpeed = 60});
		pros::delay(1000);
		intake.move(127);
	chassis.moveToPoint(-60, -47, 3000,{.maxSpeed = 60});
		intake.move(127);
		pros::delay(10000);
		intake.brake();
}

 void auton2(){
	//Left Side Blue, working?
		static pros::adi::DigitalOut clamp('A');
		clamp.set_value(false);
		chassis.setPose(63, -24, 90);
	chassis.moveToPoint(63, -24, 3000, {.forwards = false, .maxSpeed = 60});
	chassis.moveToPoint(45, -24, 3000, {.forwards = false, .maxSpeed = 60});
	chassis.moveToPoint(36, -24, 3000, {.forwards = false, .maxSpeed = 30});
		pros::delay(1000);
		clamp.set_value(true);
		pros::delay(1000);
		intake.move(127);
	chassis.moveToPoint(23.465, -47, 3000,{.maxSpeed = 60});
		pros::delay(1000);
		intake.move(127);
	chassis.moveToPoint(60, -47, 3000,{.maxSpeed = 60});
		intake.move(127);
		pros::delay(10000);
		intake.brake();
 }
void auton3(){
	//Right Side Blue
		static pros::adi::DigitalOut clamp('A');
		clamp.set_value(false);
		chassis.setPose(62, 22, 90);

	chassis.moveToPose(62, 22, 90, 3000);
	chassis.moveToPose(31.945, 24.021, 90, 3000, {.forwards = false, .maxSpeed= 50});
	chassis.moveToPose(23.788, 46.956, 0, 3000);
	clamp.set_value(true);
	intake.move(127);
	pros::delay(1000);
	chassis.moveToPose(8.04, 50.51, 270, 3000);
	intake.move(-127);
	pros::delay(500);
	intake.move(127);
	chassis.moveToPose(17.893, 45.503, 270, 3000 , {.forwards = false});
	intake.move(127);
	chassis.moveToPose(8.04, 42.272, 270, 3000);
	intake.move(127);
	pros::delay(1000);
	chassis.moveToPose(22.577, 4.962, 180, 3000);
}
void auton4(){
	//left Side Red
  chassis.setPose(-62, 22, 270);

		static pros::adi::DigitalOut clamp('A');
    clamp.set_value(false);
chassis.moveToPose(-62, 22, 270, 3000);
chassis.moveToPose(-32, 24, 270, 3000, {.forwards = false, .maxSpeed= 50});
	pros::delay(1000);
	intake.move(127);	
  clamp.set_value(true);
chassis.moveToPose(-35, 47, 0, 3000);
	intake.move(127);
	pros::delay(1000);
chassis.moveToPose(-18, 53, 90, 3000);
	pros::delay(500);
	intake.move(127);
  pros::delay(1000);
chassis.moveToPose(-30, 53, 90, 3000, {.forwards = false});
	intake.move(127);
chassis.moveToPose(-18, 42, 90, 3000);
	intake.move(127);
	pros::delay(1000);
//chassis.moveToPose(-22, 5, 180, 3000);



}
//ao9wdeiuhfauwehfg
//aiuwehgijshwrg

int arm_state = 0;
bool enable_pid = false;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    //lemlib::bufferedStdout().setRate(10);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			rotation.set_position(0);
    		pros::lcd::print(3, "HighStake: %f", (rotation.get_angle()/100));
			                
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    pros::Task arm_task([&]() { 
        double angle = 0;
        int time_settled = 0;
        double kp = 3; 
        double kd = 0.5;
        int maxvolt = 127;
        double previous_error = 0;
        while (true) {
            if (arm_state == 0) {
                angle = 75;
            }
            else if (arm_state == 1) {
                angle = 101;
            }
            else if (arm_state == 2) {
                angle = 208;
            }

            double error = angle - rotation.get_angle() / 100.0;

            if (time_settled < 60 && enable_pid) {
                int speed = kp * error + kd * (error - previous_error);
                if (speed < -maxvolt) {
                    speed = -maxvolt;
                }
                if (speed > maxvolt) {
                    speed = maxvolt;
                }
                hangLeft.move(speed);
                hangRight.move(-speed);
                if (fabs(error) <= 0.2) {
                    time_settled += 20;
                }
                else {
                    time_settled = 0;
                }

            } 
            else {
                time_settled = 0;
                hangLeft.brake();
                hangRight.brake();
                enable_pid = false;
            }

            previous_error = error;

            pros::delay(10);
        }
});
}


/*int auton = 1;
int noa = 4;
void autonselector() {
 if (auton>noa){
   auton=1;
 }
if (auton<1){
   auton=noa;
}
if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) > 5){
   auton++;
   pros::delay(500);
}
if(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < -5){
   auton--;
   pros::delay(500);
}
if(auton == 1){
  pros::lcd::print(5, "right Red");
}
if(auton == 2){
  pros::lcd::print(5, "left blue");

  }
if(auton == 3){
  pros::lcd::print(5, "right blue");

  }  
if(auton == 4){
  pros::lcd::print(5, "left red");

  }  
}*/

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
ASSET(leftred_txt);

void autonomous() {
//chassis.follow(leftred_txt, 15, 1500);
 /* if (auton == 1){
  auton1();
  }

  if (auton == 2){
  auton2();
  }

  if (auton == 3){
  auton3();
  }

   if (auton == 4){
  auton4();
  }*/
  auton3();

}
void takein() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // in
    intake.move(127); // Spins the intake motor forward at full speed
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // out
    intake.move(-127); // Spins the intake motor reverse at full speed
  } else {
    intake.brake(); // Stops the intake motor
  }
}

bool button_pressed = false;
void arm_control() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (!button_pressed) {
            arm_state = (arm_state + 1) % 3;
            enable_pid = true;
            button_pressed = true;
        }
    }
    else {
        button_pressed = false;
    }
}

void setclamp() {
  static bool toggle = false;
  static bool latch = false;

  // Check if the DOWN button is pressed
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    if (!latch) {
      toggle = !toggle; // Toggle the state
      latch = true;     // Latch to prevent multiple toggles
    }
  } else {
    latch = false;       // Reset latch when button is released
  }

  // Use the new DigitalOut API to control the clamp
  static pros::adi::DigitalOut clamp('A'); // Replace 'A' with your actual digital port
  clamp.set_value(toggle);                 // Set the clamp based on toggle state
}
/**
 * Runs in driver control
 */

void opcontrol() {
//auton4();
//pros::delay(10000);
	//rotation.reset_position();
	//rotation.reset();
  
    while (true) {
      //autonselector();
		arm_control();
		setclamp();
		takein();

 		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);

        pros::delay(10);
    }
}