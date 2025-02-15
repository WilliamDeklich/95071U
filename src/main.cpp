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
pros::MotorGroup leftMotors({-17,-12,-13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14,15,11}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(3);
pros::Rotation rotation_sensor(20);
// motor configs
pros::Motor intake(9);
pros::Controller controller1();
pros::Motor hangLeft(7);
pros::Motor intake2(-16);
lemlib::TrackingWheel horizontal_tracking_wheel(&rotation_sensor, lemlib::Omniwheel::NEW_2, 0);

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
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void auton1(){
	//right Side red (Goal Rush)
		static pros::adi::DigitalOut clamp('A');
		clamp.set_value(false);
    chassis.setPose(-55,-15, 90);
		chassis.moveToPose(-55, -15, 90, 2000);
chassis.moveToPose(-28.22, -43.009, 135, 2000);
chassis.moveToPose(-23.052, -53,90, 2000);
chassis.moveToPose(-15.784, -53,90, 2000);
chassis.moveToPose(-39.365, -53.185, 90, 2000);
chassis.moveToPose(-32.42, -49.308, 270, 2000);
chassis.moveToPose(-51.479, -47.37, 277, 2000);

}

 void auton2(){
	//left Side red (4 Stack Rush)
  static pros::adi::DigitalOut clamp('A');
  chassis.setPose(-56,15,270);
//chassis.moveToPose(-56, 15, 250, 2000, {.forwards = false});
chassis.moveToPose(-33, 23, 270, 2000, {.forwards = false});
chassis.moveToPose(-23, 23, 270, 2000, {.forwards = false});
  clamp.set_value(true);
chassis.moveToPose(-12, 40, 45,  2000);
  clamp.set_value(true);
  intake.move(127);
    intake2.move(127);

chassis.moveToPose(-19, 44, 315, 2000);
  clamp.set_value(true);
  intake.move(127);
    intake2.move(127);

chassis.moveToPose(-28, 48, 90, 2000);
  clamp.set_value(true);
  intake.move(127);
    intake2.move(127);

chassis.moveToPose(-12, 50, 90, 2000);
  clamp.set_value(true);
  intake.move(127);
    intake2.move(127);

  pros::delay(2000);
chassis.moveToPose(-23.698, 3.993, 180, 2000);
 }
void auton3(){
	//Right Side Blue (4 stack Rush alliance stake)
	static pros::adi::DigitalOut clamp('A');
  clamp.set_value(false);
  chassis.setPose(55,15,90);
chassis.moveToPose(55, 15, 90, 1500);
chassis.moveToPose(60, 9, 135, 1500);
  pros::delay(1000);
  hangLeft.move(-80);
  
  pros::delay(1000);
  hangLeft.move(80);

  pros::delay(1000);

  hangLeft.brake();
chassis.moveToPose(47, 23, 135,  1500, {.forwards = false, .minSpeed = 70});
chassis.moveToPose(27, 23,90,  1500,{.forwards = false});
	pros::delay(2000);
  clamp.set_value(true);
  pros::delay(1000);
  intake.move(127);
  intake2.move(127);

chassis.moveToPose(23, 35,0,  1500,{.minSpeed = 80});
chassis.moveToPose(23, 49,0,  1500);
chassis.moveToPose(16, 51,270,  1500);
chassis.moveToPose(10, 51,270,  1500);
  pros::delay(2000);
chassis.moveToPose(14, 46,270,  1500,{.forwards = false});
chassis.moveToPose(10, 41,270,  1500);
  pros::delay(1500);
chassis.moveToPose(24, 8,180,  1500);
  pros::delay(1500);
  intake.brake();
  intake.brake();

}
void auton4(){
//left Side blue (goalrush)
	static pros::adi::DigitalOut clamp('A');
  static pros::adi::DigitalOut doinker('B');
  static pros::adi::DigitalOut doinker2('C');
  chassis.setPose(59.205, -36.283, 270);
chassis.moveToPose(17.804, -39.126, 265, 1500, {.minSpeed = 80});
doinker.set_value(true);
chassis.moveToPose(49.4, -61.9, 90, 1500);
doinker2.set_value(true);

pros::delay(100);
chassis.moveToPose(23.668, -38.949, 180,  1500, {.forwards = false});
doinker2.set_value(false);
pros::delay(1500);
doinker.set_value(false);
intake.move(127);
intake2.move(127);
chassis.moveToPose(30.775, -35.573, 180, 1500, {.forwards = false});
pros::delay(1500);
}
/*void auton5(){
   chassis.setPose(-53, 0, 270);
  static pros::adi::DigitalOut clamp('A');
  clamp.set_value(false);
  hangLeft.move(-127);
 
  hangRight.move(127);
  pros::delay(700);
  chassis.moveToPoint(-47, 0, 3000, {.forwards = false});
   
  pros::delay(200);
  hangLeft.move(127);
  hangRight.move(-127);
  pros::delay(500);

*/


/*

 chassis.turnToPoint(-47, 23.5, 3000, {.maxSpeed = 60});
  chassis.moveToPoint(-49, -26.5, 3000, {.forwards = false, .maxSpeed = 60});
  pros::delay(700);
  clamp.set_value(true);
  pros::delay(600);
  intake.move(127);
  //pros::delay(100);
  chassis.turnToPoint(-23.5, -23.5, 3000);
  chassis.moveToPoint(-23.5, -26.5, 3000);
  //pros::delay(600);
  chassis.turnToPoint(0, -59, 3000);
  chassis.moveToPoint(-4, -59, 3000);
  //pros::delay(700);
  //chassis.turnToPoint(-23.5, -52, 3000);
  chassis.moveToPoint(-23.5, -52, 3000);
 // pros::delay(600);
  chassis.moveToPoint(-47, -50, 3000, {.maxSpeed = 50});
  pros::delay(1000);
  chassis.moveToPoint(-58.5, -48.5, 3000, {.maxSpeed = 50});


  pros::delay(1000);
  chassis.turnToPoint(-47, -60, 3000);
  chassis.moveToPoint(-40, -67, 3000, {.maxSpeed = 80});
  pros::delay(1000);
  chassis.turnToPoint(-23.5, -47, 3000);
   //clamp.set_value(false);
   //pros::delay(200);
  chassis.moveToPoint(-70, -70, 3000, {.forwards = false, .maxSpeed = 60});


 
  pros::delay(200);
  clamp.set_value(false);
  chassis.moveToPoint(-47, -47, 3000);
  chassis.turnToPoint(-47, 23.5, 3000);
  pros::delay(100);
  chassis.moveToPoint(-47, 0, 3000);
  chassis.turnToPoint(-47, -23.5, 3000);
 // pros::delay(300);
  chassis.moveToPoint(-47, 27.5, 3000, {.forwards= false, .maxSpeed = 80});
  pros::delay(800);
  clamp.set_value(true);
  pros::delay(200);
  intake.move(127);
  pros::delay(100);
  chassis.turnToPoint(-23.5, 23.5, 3000);
  chassis.moveToPoint(-21, 30, 3000);
  //pros::delay(500);
  chassis.turnToPoint(3, 59, 3000);
  chassis.moveToPoint(0, 64, 3000);
  //pros::delay(700);
  //chassis.turnToPoint(-23.5, 47, 3000);
  chassis.moveToPoint(-23.5, 55, 3000);
  //pros::delay(200);
  chassis.moveToPoint(-47.5, 53.5, 3000, {.maxSpeed = 50});
  pros::delay(1000);
  chassis.moveToPoint(-59.5, 49.5, 3000, {.maxSpeed = 50});




  pros::delay(1000);
  chassis.turnToPoint(-49, 65, 3000);
  chassis.moveToPoint(-49, 60, 3000, {.maxSpeed = 70});
  pros::delay(1000);
  chassis.turnToPoint(-23.5, 47, 3000);
  chassis.moveToPoint(-68, 70, 3000, {.forwards = false});
  clamp.set_value(false);
  chassis.moveToPoint(-23.5, 47, 3000, {.maxSpeed = 80});
  pros::delay(200);
  chassis.moveToPoint(23.5, 55, 3000, {.maxSpeed = 80});
  pros::delay(500);
  chassis.moveToPoint(47, -3, 3000, {.forwards= false, .maxSpeed = 100});
  pros::delay(300);
  clamp.set_value(true);
  /* *
  chassis.moveToPoint(23.5, 23.5, 3000, {.forwards = false});
  chassis.moveToPoint(47, 0, 3000);
  chassis.moveToPoint(23.5, -23.5, 3000);
  chassis.moveToPoint(23.5, -47, 3000);
  pros::delay(500);
  chassis.turnToPoint(47, -47, 3000, {.maxSpeed = 60});
  chassis.moveToPoint(47, -47, 3000, {.maxSpeed = 60});
  pros::delay(700);
  chassis.moveToPoint(47, -60, 3000, {.maxSpeed = 60});
  pros::delay(1000);
  chassis.turnToPoint(23.5, -47, 3000);
  chassis.moveToPoint(70, -70, 3000);
  clamp.set_value(false);
  chassis.moveToPoint(23.5, -47, 3000);
  chassis.turnToPoint(0, -70, 3000);
  chassis.moveToPoint(60, 23.5, 3000, {.forwards = false, .maxSpeed = 100});
  chassis.moveToPoint(70, 70, 3000, {.forwards = false});
  clamp.set_value(false);
  pros::delay(10000);
  










 


  pros::delay(1000000);
}*/
//int arm_state = 0;
//bool enable_pid = false;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading    
			      pros::lcd::print(3, "Rotation Sensor: %i", rotation_sensor.get_position());
            pros::lcd::print(4, "Hang Sensor: %i", hangLeft.get_position());
        pros::delay(10);          
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    //pros::Task arm_task([&]() { 
        /*double angle = 0;
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

            pros::delay(10);*/
        //}
}//);
//}


int auton = 1;
int noa = 5;
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
if(auton ==5){
  pros::lcd::print(5, "skills");

}
}


void disabled() {}

void competition_initialize() {

}


void autonomous() {
  
  auton3();

}
void takein() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // in
    intake.move(127); // Spins the intake motor forward at full speed
      intake2.move(127);

  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // out
    intake.move(-127); // Spins the intake motor reverse at full speed
      intake2.move(-127);

  } else {
    intake.brake(); // Stops the intake motor
      intake2.brake();

  }
}
void skib(){
  static bool toggle = false;
  static bool latch = false;
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      toggle = true;
      //latch = true;
    }
    

  
  if (toggle){
    hangLeft.move(-127);

    pros::delay(170);
    hangLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    hangLeft.brake();


    toggle = false;
  }

}

//bool button_pressed = false;
void backpack(){


  // Check if the DOWN button is pressed
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    hangLeft.move(120);

  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

    hangLeft.move(-120);
  } else {
    hangLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);



    hangLeft.brake();

  }

}

void doinker2(){
  static bool toggle = false;
  static bool latch = false;
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
    if (!latch) {
      toggle = !toggle; // Toggle the state
      latch = true;     // Latch to prevent multiple toggles
    }
  } else {
    latch = false;       // Reset latch when button is released
  }


  // Use the new DigitalOut API to control the clamp
  static pros::adi::DigitalOut doinker2('C'); // Replace 'A' with your actual digital port
  doinker2.set_value(toggle);
}

void doinker(){
  static bool toggle = false;
  static bool latch = false;
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    if (!latch) {
      toggle = !toggle; // Toggle the state
      latch = true;     // Latch to prevent multiple toggles
    }
  } else {
    latch = false;       // Reset latch when button is released
  }


  // Use the new DigitalOut API to control the clamp
  static pros::adi::DigitalOut doinker('B'); // Replace 'A' with your actual digital port
  doinker.set_value(toggle);

}
bool button_pressed = false;
/*void arm_control() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (!button_pressed) {
            arm_state = (arm_state + 1) % 3;
            enable_pid = true;
            button_pressed = true;
        }
    }
    else {
        button_pressed = false;
    }
}*/

void setclamp() {
  static bool toggle = true;
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

  //auton2();
    while (true) {
		//arm_control();
		setclamp();
		takein();
    doinker();
    doinker2();
    skib();
    backpack();
 		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);

        pros::delay(10);
    }
}