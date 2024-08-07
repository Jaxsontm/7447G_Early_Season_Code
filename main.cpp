#include "main.h"
#include "auton.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "../include/robodash/api.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
using pros::Motor;

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Green Ziptie

pros::MotorGroup DTLeft({-1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup DTRight({12, -13, 14}, pros::MotorGearset::blue);

pros::Imu inertial_sensor(4); 
//

//Blue Ziptie
Motor Intake(11, pros::MotorGearset::blue);

pros::Distance DistanceIntake(5);

pros::adi::Pneumatics intakePiston('B', false);
//

//Yellow Ziptie
pros::MotorGroup Descore({20, -19}, pros::MotorGearset::green);

pros::Distance DistanceDescore(10);
//

//Red Ziptie
pros::Distance DistanceMogo(18);

pros::adi::Pneumatics Mogo('A', false);
//






// drivetrain settings
lemlib::Drivetrain drivetrain(&DTLeft, // left motor group
                              &DTRight, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(12, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            35, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            1000, // small error range timeout, in milliseconds
                                            30, // large error range, in inches
                                            3000, // large error range timeout, in milliseconds
                                            2 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4.05, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             22, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             500, // small error range timeout, in milliseconds
                                             750, // large error range, in degrees
                                             750, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

pros::Rotation horizontal_sensor(6);
pros::Rotation vertical_sensor(7);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_275, 2);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -1.5);
// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial_sensor // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void waitUntilTankDist(double inches){ // creates a new function with the parameter inches
lemlib::Pose lastPose = chassis.getPose(); // creates a new value under the lemlib:pose class that holds the initial position of the robot
while(inches > 0) { // while inches is greater than 0, the inside bit runs
inches -= chassis.getPose().distance(lastPose); // redeclares param inches as inches minus distance away from the last position
lastPose = chassis.getPose(); // updates current position of robot
pros::delay(10); // adds delay to not overload sensors
} // slowly whittles down at inches until it is equal to 0, in which it will run the next line of code in queue

}




void screen() {
    while (true) {
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
        pros::delay(50);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Intake State Machine
// enum representing the possible states of the mechanism
// states with higher priorities have lower numbers
// additional states can be added as needed
enum State {
  LOAD = 0,
  IDLE = 1,
  SCORE = 2,
  UNLOAD = 3,
  BRAKE = 4
};

// the current state of the mechanism
State current_state = BRAKE;

// function used to request a new state
void request_new_state(State requested_state) {
  if (requested_state < current_state) {
    current_state = requested_state;
  }
  if (requested_state > current_state) {
    current_state = requested_state;
  }
}

// function which constantly updates the state of the mechanism
void state_machine() {
  // run forever
  while (true) {
    // switch statement to select what to do based on the current state
    switch (current_state) {
      // the Intake should be spinning
      case State::LOAD: {
        // if the Sensor does detect something, stop the intake
        if (DistanceIntake.get() < 1) current_state = State::IDLE;
        // if the Sensors doesn't detect anything, keep spinning the intake
        else Intake.move(-127);
        // break out of the switch statement
        break;
      }
      // the Intake should stop
      case State::IDLE: {
        if (DistanceMogo.get() < 49 or DistanceDescore.get() < 125) current_state = State::SCORE;
        //Stop the Intake from spinning
        else if (DistanceIntake.get() > 52 or DistanceDescore.get() > 127) current_state = State::BRAKE;
        // make the Intake hold its position
        else Intake.brake();
        // break out of the switch statement
        break;
      }
      case State::SCORE: {
        Intake.move(-127);
        //break out of the switch statement
        break;
      }
      case State::UNLOAD:{
        //reverse Intake for Driver Control
        Intake.move(127);
        //break out of the switch statement
        break;
      }
      case State::BRAKE:{
        //keep the Intake from spinning
        Intake.brake();
        //break out of the switch statement
        break;
      }
    }
    // delay to save resources
    pros::delay(10);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum StateMogo {
  LOCATE = 0,
  GRAB = 1,
  RELEASE = 2
};

// the current state of the mechanism
StateMogo current_state2 = RELEASE;

// function used to request a new state
void request_new_state_mogo(StateMogo requested_state_mogo) {
  if (requested_state_mogo < current_state2) {
    current_state2 = requested_state_mogo;
  }
  if (requested_state_mogo > current_state2) {
    current_state2 = requested_state_mogo;
  }
}

// function which constantly updates the state of the mechanism
void state_machine_mogo() {
  // run forever
  while (true) {
    // switch statement to select what to do based on the current state
    switch (current_state2) {
      // the Intake should be spinning
      case StateMogo::LOCATE: {
        // if the Sensor does detect something, stop the intake
        if (DistanceMogo.get() < 39) current_state2 = StateMogo::GRAB;
        // if the Sensors doesn't detect anything, keep spinning the intake
        else Mogo.set_value(false);
        // break out of the switch statement
        break;
      }
      case StateMogo::GRAB:{
        Mogo.set_value(true);
        break;
      }
      case StateMogo::RELEASE:{
        Mogo.set_value(false);
        break;
      }
    }
    // delay to save resources
    pros::delay(10);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RightAWP() { //finished

    chassis.setPose(0, 0, 0);

            request_new_state_mogo(StateMogo::LOCATE);

    chassis.moveToPoint(0, -30, 1200, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 1});

    chassis.turnToHeading(-90, 500);

    chassis.moveToPose(-24, -25, -90, 1000, {.lead = 0});
    
      request_new_state(State::IDLE);
pros::delay(2250);

    chassis.moveToPose(16, -6, 90, 2500, {.lead = 0.2, .maxSpeed = 80});

            request_new_state_mogo(StateMogo::RELEASE);
      request_new_state(State::BRAKE);

              intakePiston.set_value(true);
              
      request_new_state(State::LOAD);

    chassis.moveToPoint(18, -6, 1000);
chassis.waitUntilDone();

              intakePiston.set_value(false);
      request_new_state(BRAKE);


    chassis.moveToPoint(15, -6, 1000, {.minSpeed = 100});

    chassis.turnToHeading(-160, 800);

    chassis.moveToPose(24, 18, 180, 1000, {.forwards = false, .lead = 0}, false);
pros::delay(250);

      request_new_state(State::SCORE);
pros::delay(1000);

    chassis.moveToPose(20, -44, 135, 2000);
}

void LeftAWP() { //finished
    chassis.setPose(0, 0, 0);

            request_new_state_mogo(StateMogo::LOCATE);

    chassis.moveToPoint(0, -27, 1200, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 1});

    chassis.turnToHeading(90, 500);

    chassis.moveToPose(19, -25, -90, 1000, {.lead = 0});
    
      request_new_state(State::IDLE);
pros::delay(3000);

    chassis.moveToPose(-16, -8, -50, 2500, {.lead = 0.2, .maxSpeed = 80});

            request_new_state_mogo(StateMogo::RELEASE);

        request_new_state(State::SCORE);
pros::delay(1000);

    chassis.moveToPoint(-23, -3, 1000);

        request_new_state(State::LOAD);

    chassis.turnToHeading(180, 500);

    chassis.moveToPose(-22.5, 14, 180, 1100, {.forwards = false, .lead = 0, .maxSpeed = 80});

      request_new_state(State::IDLE);
pros::delay(1000);

    chassis.moveToPose(-20, -44, -135, 2000);
}

void Skills() {
    chassis.setPose(0,0,0);
    
}

void Score() {
    chassis.setPose(0,0,0);

}

void Forwards() { //finished
    chassis.setPose(0,0,0);

    chassis.moveToPose(0, 4, 0, 450, {.minSpeed = 127});
}

void BlueRight() {
    chassis.setPose(0, 0, 0);

        request_new_state_mogo(StateMogo::LOCATE);

    chassis.moveToPoint(0, -27, 1200, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 1});

    chassis.turnToHeading(-90, 500);

    chassis.moveToPose(-19, -25, -90, 1000, {.lead = 0});
    
      request_new_state(State::IDLE);
      
    chassis.turnToHeading(180, 800);

    chassis.moveToPoint(-22, -40, 1000 );

      request_new_state(State::SCORE);
pros::delay(1250);

    chassis.moveToPose(16, -6, 90, 10000);

          intakePiston.set_value(true);

chassis.waitUntilDone();

      request_new_state(State::LOAD);

    chassis.moveToPose(10, -6, 90, 1000, {.forwards = false});

          intakePiston.set_value(false);
}


/** top ring of the double stack
    intakePiston.set_value(true);

    request_new_state(State::LOAD);

    chassis.moveToPoint(0, 3, 1000);
    
    chassis.waitUntilDone();

    intakePiston.set_value(false);

    chassis.moveToPoint(0, -5, 1000, {.forwards = false});
 */

rd::Selector selector({
   {"Right AWP", &RightAWP},
   {"Left AWP", &LeftAWP},
   {"Blue SUPER SCORE", &BlueRight},
   {"Red SUPER SCORE", &Score},
   {"Forwards", &Forwards}, 
   {"Skills", &Skills},
   
   
}); 

rd::Console console;



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    console.println("Initializing robot...");
    pros::Task state_machine_task(state_machine);
    pros::Task state_machine_task_mogo(state_machine_mogo);
    Intake.set_brake_mode(pros::MotorBrake::brake);
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);//heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50); 
        }
    });
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
void competition_initialize() {}

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
void autonomous() {
        pros::delay(500);
    console.println("Running auton...");
	selector.run_auton();
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

 bool static yPressed = false;
 bool static yState = false;

 bool static rightPressed = false;
 bool static rightState = false;


void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {

    /** get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        //move the robot
        chassis.tank(leftY, rightY);
    */

    /////////////////////////////////////////////////////////////////

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            request_new_state(State::SCORE);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            request_new_state(State::UNLOAD);
        } else {
            request_new_state(State::BRAKE);
        }

    //////////////////////////////////////////////////////

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !yPressed && !yState) {
            request_new_state_mogo(StateMogo::GRAB);
            yPressed = true;
            yState = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !yPressed && yState) {
            request_new_state_mogo(StateMogo::RELEASE);
            yPressed = true;
            yState = false;
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            yPressed = false;
        }

    ////////////////////////////////////////////////////////////    
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !rightPressed && !rightState) {
            intakePiston.set_value(true);
            rightPressed = true;
            rightState = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !rightPressed && rightState) {
            intakePiston.set_value(false);
            rightPressed = true;
            rightState = false;
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            rightPressed = false;
        }

    //////////////////////////////////////////////////////
    

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);

    

     }
	}
