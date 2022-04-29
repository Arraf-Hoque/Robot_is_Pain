#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-11, 1 ,-12, -13}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{20,-10, 19, 18}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.66666666667

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

pros::Motor& chainB_l = chassis.left_motors[1];
pros::Motor& chainB_r = chassis.right_motors[1];
pros::ADIDigitalOut pto_chainL_piston('A');
pros::ADIDigitalOut pto_chainR_piston('H');
bool pto_enabled = false;
pros::ADIDigitalOut clampPiston('C');
bool clamp_enabled = false;
pros::ADIDigitalOut twobarL('B');
pros::ADIDigitalOut twobarR('G');
pros::ADIDigitalOut highGoal('F');
bool highScorer_State = false;
int pto_button_lock = 0;
bool twobar_enabled = false;
int twobar_buttonlock = 0;
int clamp_button_lock = 0;
bool shift_key_pressed = false;
int last_press = 0;


void pto_active(bool toggle) {
  pto_enabled = toggle;
  chassis.pto_toggle({chainB_l, chainB_r}, toggle);
  pto_chainL_piston.set_value(toggle);
  pto_chainR_piston.set_value(toggle);
  if (toggle) {
    chainB_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chainB_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
}
void twobar_active(bool toggle) {
  twobar_enabled = toggle;
  twobarL.set_value(toggle);
  twobarR.set_value(toggle);
}
void clamp_active(bool toggle) {
  clamp_enabled = toggle;
  clampPiston.set_value(toggle);
}

void set_chainBpos(int input, int speed) {
  if (!pto_enabled) return;
    chainB_l.move_absolute(input, speed);
    chainB_r.move_absolute(input, speed);
}
void set_chainb(int input) {
  if (!pto_enabled) return;
  chainB_l = input;
  chainB_r = input;
}

void chainBcontrol(){
  if (master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2) && pto_enabled == false) {
    pto_enabled = !pto_enabled;
    pto_active(pto_enabled);
    master.rumble("..."); 
    last_press = 1;
  }
  else if (master.get_digital(DIGITAL_L2) && master.get_digital(DIGITAL_L1) && pto_enabled == true){
    pto_enabled = !pto_enabled;
    pto_active(pto_enabled);
    last_press = 0;
  }
  if (master.get_digital(DIGITAL_L1)  && pto_enabled == true){
    set_chainb(-127);
  }
  else if (master.get_digital(DIGITAL_L2) && pto_enabled == true){
    set_chainb(127);
  }
  else{
    set_chainb(0);
  }
  pros::delay(20);
}
void clampControl(){
  if (master.get_digital_new_press(DIGITAL_R2)){
    clamp_enabled = !clamp_enabled;
    clampPiston.set_value(clamp_enabled);
  }
}
void backControl(){
  if (master.get_digital_new_press(DIGITAL_R1)) {
    twobar_enabled = !twobar_enabled;
    twobarL.set_value(twobar_enabled);
    twobarR.set_value(twobar_enabled);
  }
}
void setBack_State(bool state){
  twobarL.set_value(state);
  twobarR.set_value(state);
}
void highControl(){
  if (master.get_digital_new_press(DIGITAL_R2) && master.get_digital_new_press(DIGITAL_R1)) {
    highGoal.set_value(!highScorer_State);
  }
}
void setHigh_State(bool state){
    highGoal.set_value(state);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Use This Auton", right_side_auton()),
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



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
  // . . .
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
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
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
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  while (true) {

    // chassis.tank(); // Tank control
     chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade
  
    // . . .
    // Put more user control code here!
    // . . .
    chainBcontrol();
    clampControl();
    highControl();
    backControl();
    double joy = master.get_analog(ANALOG_LEFT_Y);
    printf("%d", pto_enabled);

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
