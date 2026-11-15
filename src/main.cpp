#include "main.h"

// Chassis constructor
// ports carried over from the old bot, VERIFY all of these against the current build
ez::Drive chassis(
    {19, 17, 6},     // left, negative port reverses it
    {-20, -18, -3},  // right

    5,      // imu port
    2.75,   // wheel diameter
    450);   // wheel rpm, verify with a tape measure over 24"

// ---- other devices ----

pros::MotorGroup cascade({-9, 10}, pros::MotorGearset::green); // 9 faces the other way
pros::Motor toggle(8, pros::MotorGears::green);
pros::Optical optical(16);
pros::Motor claw(2, pros::MotorGears::green);

// ---- toggle manual override ----
// failsafe latch for a dead or lying optical sensor: A cuts the sensor out, stays latched.
// driver-control state, so it lives here with the other button bindings
static bool toggleManual = false;
bool toggleManualMode() { return toggleManual; }

void initialize() {
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // no curve-edit buttons, they would steal A and the arrows, which the toggle needs
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  // middle motor of each side is green, the outer two are blue
  chassis.left_motors[0].set_gearing(pros::MotorGears::blue);
  chassis.left_motors[1].set_gearing(pros::MotorGears::green);
  chassis.left_motors[2].set_gearing(pros::MotorGears::blue);
  chassis.right_motors[0].set_gearing(pros::MotorGears::blue);
  chassis.right_motors[1].set_gearing(pros::MotorGears::green);
  chassis.right_motors[2].set_gearing(pros::MotorGears::blue);

  cascade.set_brake_mode(pros::MotorBrake::coast); // so it can be pushed down by hand pre match
  cascade.tare_position();                         // here is zero, so it MUST start fully down

  optical.set_led_pwm(100);         // sensor light full blast, keeps hue off the room lighting
  optical.set_integration_time(20); // default 100ms is too slow for a spinning toggle
  toggle.set_brake_mode(pros::MotorBrake::hold); // stop ON the color instead of coasting past

  claw.set_brake_mode(pros::MotorBrake::hold);
  claw.set_current_limit(CLAW_CURRENT_LIMIT); // mA, so a held grip can never cook the motor
  claw.tare_position();                       // here is zero = open, so the claw MUST start fully open

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"PID Tuning", tuning},
      {"Skills Route", skills},
      {"Red Right", red_right},
      {"Red Left", red_left},
      {"Blue Right", blue_right},
      {"Blue Left", blue_left},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  cascade.set_brake_mode(pros::MotorBrake::hold); // the route needs it to stay where it stops

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // EZ's live PID tuner is disabled on purpose: it steals X (claw half), A (toggle
    // failsafe) and the arrows (toggle spin). tune on the field by editing default_constants
    // if (master.get_digital_new_press(DIGITAL_X))
    //   chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // PID tuner disabled, see above
    // chassis.pid_tuner_iterate();
  }
}

/**
 * Runs the operator control code.
 *   left stick drive, right stick turn
 *   R1 / R2   claw close (grab) / open (release)
 *   X         claw ready stance (half)
 *   L1 / L2   cascade up / down (hold)
 *   up / down spin the toggle by hand (hold)
 *   A         failsafe latch for the optical sensor
 *   B + DOWN  run the selected auton, practice only
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);     // drive coasts, feels better to drive on
  cascade.set_brake_mode(pros::MotorBrake::hold); // cascade holds so it does not sag

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade

    // ---- claw ----
    // each press fires one move_absolute, the motor's own controller holds there until the
    // next press, so no per-loop state machine. the current limit keeps a grip from cooking
    if (master.get_digital_new_press(DIGITAL_R1)) {
      clawMove(CLAW_CLOSED); // grab
    } else if (master.get_digital_new_press(DIGITAL_R2)) {
      clawMove(CLAW_OPEN); // release
    } else if (master.get_digital_new_press(DIGITAL_X)) {
      clawMove(CLAW_HALF); // ready stance
    }

    // ---- cascade ----
    // hold L1 up, hold L2 down, crawl inside CASCADE_SLOW_ZONE of either end stop
    const double pos = cascade.get_position();

    if (master.get_digital(DIGITAL_L1)) {
      const double room = CASCADE_MAX - pos; // slow zone at the TOP as well as the bottom
      if (room <= 0) {
        cascade.brake();
      } else if (room < CASCADE_SLOW_ZONE) {
        cascade.move(CASCADE_SLOW_POWER);
      } else {
        cascade.move(127);
      }

    } else if (master.get_digital(DIGITAL_L2)) {
      const double room = pos - CASCADE_DOWN;
      if (room <= 0) {
        cascade.brake();
      } else if (room < CASCADE_SLOW_ZONE) {
        cascade.move(-CASCADE_SLOW_POWER);
      } else {
        cascade.move(-127);
      }

    } else {
      cascade.brake(); // brake mode is hold, so this holds the height
    }

    // ---- toggle ----
    // A latches the optical sensor out (for a dead/lying sensor), held arrows always win,
    // otherwise the sensor spins to the alliance color on its own
    if (master.get_digital_new_press(DIGITAL_A)) {
      toggleManual = !toggleManual;
      optical.set_led_pwm(toggleManual ? 0 : 100); // light off while overridden
      master.rumble(toggleManual ? "-" : ".");
    }

    if (master.get_digital(DIGITAL_UP)) {
      toggle.move(TOGGLE_SPEED); // a held arrow beats the sensor
    } else if (master.get_digital(DIGITAL_DOWN)) {
      toggle.move(-TOGGLE_SPEED);
    } else if (toggleManual) {
      toggle.move(0); // sensor latched out, no arrow held
    } else {
      autoToggle(allianceColor());
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}