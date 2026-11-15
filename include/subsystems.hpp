#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include <cstdint>

extern Drive chassis;

// ============================================================================
// quick reference, how to use these in a route and what to tune
// ============================================================================
//
// POWER-ON RULE (both tare here, so the whole season depends on it)
//   cascade MUST start fully down, claw MUST start fully open
//   wrong start position = every height and every claw position is off
//
// --- cascade ---  async raise/lower that overlaps the drive
//   cascadeMove(CASCADE_HEIGHT[3]);                      // start raising to 3-pin, returns now
//   chassis.pid_drive_set(26_in, driveSpeedForHeight()); // drive slow because it is going up
//   chassis.pid_wait_quick();
//   cascadeWait(500);                                    // only blocks if the drive was too short
//   cascadeMove(CASCADE_HEIGHT[0], 150);                 // lower gently, lower rpm = soft set down
//   fire cascadeMove BEFORE the drive it should slow, driveSpeedForHeight reads it once
//   tune: CASCADE_HEIGHT[1..5] heights, CASCADE_MAX top limit, the TIER numbers for drive speed
//
// --- claw ---  grip, no ratchet, holds by position
//   clawClose();  clawOpen();  clawHalf();   // named, each blocks until there (or gripping)
//   a close onto an object "times out" on purpose and keeps holding, that is the grip
//   tune: CLAW_CLOSED (set JUST past contact, runs cooler), CLAW_HALF, CLAW_CURRENT_LIMIT if warm
//
// --- toggle ---  optical color match
//   autoToggle(color, 5000);   // route: spin until it sees color or 5s, blocks
//   autoToggle(allianceColor); // driver: one pass per loop, handled in opcontrol already
//   tune: RED/BLUE/YELLOW_HUE off the real toggles, HUE_TOLERANCE, TOGGLE_MIN_PROXIMITY
//
// default every drive/turn to pid_wait_quick, pid_wait only right before a grab or final score
// ============================================================================

// ---- devices, built in main.cpp ----

extern pros::MotorGroup cascade; // two motors, 9 faces the other way
extern pros::Motor toggle;
extern pros::Optical optical;
extern pros::Motor claw;

// ============================================================================
// cascade
// ============================================================================
// position = motor degrees from power on, so the cascade MUST start fully down
// or every height below is wrong. initialize() tares it

constexpr int CASCADE_RPM = 200; // green cartridge tops out here

constexpr double CASCADE_DOWN = 30;  // stowed, a hair above the bottom hard stop
constexpr double CASCADE_MAX = 2800; // top of real travel, MEASURE ME

// scoring heights, index = pin count, so CASCADE_HEIGHT[3] is the 3-pin height
// seeded from the old route's multipliers, all of these want a tape measure
constexpr double CASCADE_HEIGHT[6] = {
    CASCADE_DOWN, // 0  stowed
    855,          // 1-pin
    940,          // 2-pin
    1450,         // 3-pin
    2000,         // 4-pin  MEASURE ME, was never a real number
    2650,         // 5-pin
};

constexpr double CASCADE_TOLERANCE = 10;  // degrees, close enough to call it arrived
constexpr double CASCADE_SLOW_ZONE = 200; // ease off this close to either end stop
constexpr int CASCADE_SLOW_POWER = 40;    // crawl power inside that zone, out of 127

// speed tiers: a raised cascade is top heavy, so DRIVE slower instead of retuning pid
// pid constants never change, only the speed argument
constexpr double CASCADE_TIER_MID_ABOVE = 300;   // above this height, medium drive speed
constexpr double CASCADE_TIER_SLOW_ABOVE = 1200; // above this, slow drive speed
constexpr int CASCADE_TIER_FULL = 110;
constexpr int CASCADE_TIER_MID = 85;
constexpr int CASCADE_TIER_SLOW = 60;

// async: hand the target to the motor and return right away, so it climbs during a drive
// the target is clamped into [CASCADE_DOWN, CASCADE_MAX], so it can never push into a stop
void cascadeMove(double target, int rpm = CASCADE_RPM);

// block until the last cascadeMove gets there, or the timeout runs out
void cascadeWait(std::uint32_t timeout);

// drivetrain speed cap for however high the cascade is sitting/heading. does NOT move the
// cascade, it only reads its height. pass the result to pid_drive_set / pid_turn_set
int driveSpeedForHeight();

// ============================================================================
// toggle (optical color sensing)
// ============================================================================
// an optical sensor stares at a colored toggle, the motor spins it until the target color
// faces the sensor. deliberately SLOW: spin fast and the color blows past before the sensor
// updates. set_integration_time(20) in initialize() is the other half of that

constexpr int TOGGLE_SPEED = 60;            // out of 127
constexpr int TOGGLE_MIN_PROXIMITY = 100;   // 0-255, below this nothing is in front of the sensor

// hue = angle round the color wheel, 0-360, measured off the real toggles
constexpr double RED_HUE = 353;
constexpr double BLUE_HUE = 195;
constexpr double YELLOW_HUE = 51;
constexpr double HUE_TOLERANCE = 25; // within this many degrees counts as a match

enum class ToggleColor { red, blue, yellow };

// color name for the brain screen
inline const char* nameOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return "BLUE";
        case ToggleColor::yellow: return "YELLOW";
        default: return "RED";
    }
}

bool seesColor(ToggleColor color); // is the sensor looking at that color right now

void autoToggle(ToggleColor color);                        // one driver control pass, returns right away
void autoToggle(ToggleColor color, std::uint32_t timeout); // blocks until it sees the color, for routes

bool toggleManualMode();     // true once the driver latched the sensor out with A
ToggleColor allianceColor(); // the selected slot's color, so driver control aims the toggle right

// ============================================================================
// claw (grip, no ratchet)
// ============================================================================
// position = motor degrees from power on, so the claw MUST start fully OPEN, 0 = open.
// no ratchet means the motor actively holds the grip. it moves to a reachable position and
// holds there with move_absolute, it NEVER drives into a hard stop (that burns the motor).
// the current limit is the real safety: even gripping an object all match, current is capped

constexpr int CLAW_RPM = 100;            // move speed, green cartridge
constexpr int CLAW_CURRENT_LIMIT = 2000; // mA, backstop so a held grip can never cook the motor

constexpr double CLAW_OPEN = 0;     // start / release, the tare point
constexpr double CLAW_HALF = 210;   // ready stance  MEASURE ME
constexpr double CLAW_CLOSED = 420; // grip          MEASURE ME, set just past object contact to run cool

constexpr double CLAW_TOLERANCE = 8;          // degrees, close enough to call it arrived
constexpr std::uint32_t CLAW_TIMEOUT = 600;   // give up waiting after this, the grip keeps holding

// async: go to a position and hold there, returns right away. target clamped to [OPEN, CLOSED]
void clawMove(double target);

// block until the last clawMove arrives. on timeout it just returns, leaving the motor holding
// (that is the grip on an object it cannot fully close on). never brakes, there is no ratchet
void clawWait(std::uint32_t timeout);

// named blocking actions for routes: move and wait. driver control calls clawMove directly
void clawOpen();   // release
void clawClose();  // grab
void clawHalf();   // ready stance
