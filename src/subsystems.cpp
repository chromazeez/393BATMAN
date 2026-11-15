#include "main.h"
#include <cmath>

// mechanism helpers. devices themselves are built in main.cpp, every tunable number
// lives in subsystems.hpp

// ============================================================================
// cascade
// ============================================================================

// the height the last cascadeMove asked for, so cascadeWait knows what it is waiting on
static double cascadeTarget = CASCADE_DOWN;

// move_absolute hands the target to the motor's OWN position controller, so the motor keeps
// driving there after this returns. that is what lets the cascade climb during a drive
void cascadeMove(double target, int rpm) {
    if (target < CASCADE_DOWN) target = CASCADE_DOWN; // clamped, so it never pushes into a stop
    if (target > CASCADE_MAX) target = CASCADE_MAX;

    cascadeTarget = target;
    cascade.move_absolute(target, rpm);
}

static bool cascadeArrived() {
    return std::fabs(cascadeTarget - cascade.get_position()) <= CASCADE_TOLERANCE;
}

void cascadeWait(std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();

    while (pros::millis() - start < timeout) {
        if (cascadeArrived()) return; // got there
        pros::delay(ez::util::DELAY_TIME);
    }

    cascade.brake(); // out of time, something is binding. stop pushing, let the route carry on
}

// one pid, vary speed. the higher it sits the more top heavy the robot is, so cap the
// drive/turn speed by height instead of tuning a second set of constants
int driveSpeedForHeight() {
    // whichever is higher, where it is or where it is heading. a drive that overlaps a raise
    // has to be slow from the start, not once the lift finally gets there
    double pos = cascade.get_position();
    if (cascadeTarget > pos) pos = cascadeTarget;

    if (pos > CASCADE_TIER_SLOW_ABOVE) return CASCADE_TIER_SLOW;
    if (pos > CASCADE_TIER_MID_ABOVE) return CASCADE_TIER_MID;
    return CASCADE_TIER_FULL;
}

// ============================================================================
// toggle (optical color sensing)
// ============================================================================

static double hueOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return BLUE_HUE;
        case ToggleColor::yellow: return YELLOW_HUE;
        default: return RED_HUE;
    }
}

// hue wraps at 360, so 359 and 1 are 2 apart, not 358. fold the difference before comparing
bool seesColor(ToggleColor color) {
    double diff = std::fabs(optical.get_hue() - hueOf(color));
    if (diff > 180) diff = 360 - diff;
    return diff <= HUE_TOLERANCE;
}

// one driver control pass. only spin when a toggle is actually in front AND it is the wrong
// color. nothing there, or already our color, means stop
void autoToggle(ToggleColor color) {
    const bool somethingClose = optical.get_proximity() > TOGGLE_MIN_PROXIMITY;
    if (somethingClose && !seesColor(color)) {
        toggle.move(TOGGLE_SPEED);
    } else {
        toggle.move(0);
    }
}

// auton version, blocks. no proximity gate: the route has lined the robot up, so a toggle is
// there. just spin until the right color shows or the timeout runs out
void autoToggle(ToggleColor color, std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();
    while (!seesColor(color) && pros::millis() - start < timeout) {
        toggle.move(TOGGLE_SPEED);
        pros::delay(10);
    }
    toggle.move(0);
}

// which alliance the selected slot is, so driver control aims the toggle at the right color
// TODO reconcile these page numbers with the final selector built in main.cpp / autons.cpp
ToggleColor allianceColor() {
    const int page = ez::as::auton_selector.auton_page_current;
    return (page == 4 || page == 5) ? ToggleColor::blue : ToggleColor::red; // blue right/left slots
}

// ============================================================================
// claw (grip, no ratchet)
// ============================================================================

// the position the last clawMove asked for, so clawWait knows what it is waiting on
static double clawTarget = CLAW_OPEN;

void clawMove(double target) {
    if (target < CLAW_OPEN) target = CLAW_OPEN; // clamped, so it never drives into a hard stop
    if (target > CLAW_CLOSED) target = CLAW_CLOSED;

    clawTarget = target;
    claw.move_absolute(target, CLAW_RPM); // position controller holds here on its own
}

static bool clawArrived() {
    return std::fabs(clawTarget - claw.get_position()) <= CLAW_TOLERANCE;
}

void clawWait(std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();

    while (pros::millis() - start < timeout) {
        if (clawArrived()) return; // reached the position
        pros::delay(ez::util::DELAY_TIME);
    }

    // timed out = gripping an object it cannot fully close on. leave move_absolute holding,
    // current-limited, that IS the grip. do NOT brake, there is no ratchet to hold it
}

// a grab finishes when the claw stops moving against the object (gripped), not when it
// reaches CLOSED (an object stops it short of that). return the instant it grips, so the
// route can move on immediately, but never before the grip has set
static void clawGripWait() {
    const std::uint32_t start = pros::millis();

    while (pros::millis() - start < CLAW_TIMEOUT) {
        if (clawArrived()) return; // empty close, actually reached CLOSED
        if (pros::millis() - start > CLAW_MIN_MS &&
            std::fabs(claw.get_actual_velocity()) < CLAW_STALL_VEL) {
            return; // stopped moving = gripped
        }
        pros::delay(ez::util::DELAY_TIME);
    }
    // timed out, leave move_absolute holding (current-limited). that is still the grip
}

void clawOpen() { clawMove(CLAW_OPEN); clawWait(CLAW_TIMEOUT); }
void clawClose() { clawMove(CLAW_CLOSED); clawGripWait(); }
void clawHalf() { clawMove(CLAW_HALF); clawWait(CLAW_TIMEOUT); }
