#include "auton.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "../include/robodash/api.h"

extern lemlib::Chassis chassis;



//Blue Ziptie
pros::Motor Intake(11, pros::MotorGearset::blue);

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
