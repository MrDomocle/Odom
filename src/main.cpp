#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/misc/lv_color.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// motor groups
pros::MotorGroup leftMotors({-10, -20, -19},pros::MotorGearset::green); // left motor group ports -10, -20, -19 (reversed)
pros::MotorGroup rightMotors({1, 11, 12}, pros::MotorGearset::green); // right motor group  ports 1, 11, 12


// Inertial Sensor on port 9
pros::Imu imu(9);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port -5, reversed
pros::Rotation horizontalEnc(-5);
// vertical tracking wheel encoder. Rotation sensor, port 6, not reversed
pros::Rotation verticalEnc(6);
// horizontal tracking wheel. 2.75" diameter, 3" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.0);
// vertical tracking wheel. 2.75" diameter, 0.75" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.75);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                            &rightMotors, // right motor group
                            12.0, // 12 inch track width
                            lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                            360, // drivetrain rpm is 360
                            2 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                          nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                          &horizontal, // horizontal tracking wheel
                          nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                          &imu // inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                   10, // minimum output where drivetrain will move out of 127
                                   1.019 // expo curve gain
);




// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                10, // minimum output where drivetrain will move out of 127
                                1.019 // expo curve gain
);


// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


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
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of 10ms


  // thread to for brain screen and position logging
  pros::Task screenTask([&]() {
      while (true) {
          // print robot location to the brain screen
          pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
          pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
          pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
          // log position telemetry
          lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
          // delay to save resources
          pros::delay(50);
      }
  });
}






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
ASSET(path_jerryio9_txt); // '.' replaced with "_" to make c++ happy




/**
* Runs during auto
*
* This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
*/
void autonomous() {


  chassis.setPose(0,0,0);


 // //This command tells the robot to move to a specific point on the field.
  chassis.moveToPoint(-0.5, 22.5, 2000, {.forwards = true, .maxSpeed = 127 },true);
  chassis.waitUntil(10);


  //This command tells the robot to swing (turn) to a specific heading or direction.
  chassis.turnToHeading(-90, 1000);
  chassis.waitUntil(10);


  chassis.moveToPoint(-36.0, 20.0, 1000, {.forwards = true, .maxSpeed = 127 },true);
  chassis.waitUntil(10);


  chassis.turnToHeading(0, 1000);
 }




/**
* Runs in driver control
*/
void opcontrol() {
   // controller
   // loop to continuously update motors
   while (true) {
       // get joystick positions
       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
       int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
       // move the chassis with curvature drive
       chassis.arcade(leftY, rightX);


       // delay to save resources
       pros::delay(10);
 }
}




