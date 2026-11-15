# 393BATMAN — VEX V5 robot code (Team 393B)

PROS 4 + EZ-Template. **No odometry, no tracking wheels** — pure PID plus distance sensors for
correction. I'm beginner-to-intermediate C++: **explain the why and help me understand as you go,
don't just dump code.** I have to defend all of this to competition judges.

## Season goals / philosophy

- **Insanely good PID is the foundation.** Everything else is built on it. Tune it first, then freeze it.
- **No odom, no tracking pods.** Relative PID moves for everything; **distance sensors (DSR /
  drive-to-distance) off the walls** for field correction. (Extropy-style: elite PID + encoder
  dynamics + DSR, zero pods.)
- **Three priorities: correction, consistency, speed.**
- **Target: first 5 pins in 15 seconds.** The skills opening is the same setup as a normal match,
  so one build serves both — two birds, one stone.

## The mental model (don't relearn this)

- **PID = consistent on one field. Walls (DSR) = consistent across different fields.** Two different
  jobs. The "hit it 3x in practice, missed it in the match" problem is *field variation* — only wall
  references fix that, not better PID and not odom.
- Internal sensors (drive encoders, IMU) only know "distance from where I started" — they drift and
  are blind to the real field. External sensors (distance off walls) know the actual field.
- **Delays / inconsistent wait times are a *symptom* of an inaccurate move**, not a thing to tune
  directly. Tune PID so moves land clean → `pid_wait_quick` exits instantly → no delay, repeatable.
- **Speed comes from overlap, not from shaving wait numbers.** The cascade raises/lowers *while
  driving*, the claw grabs on arrival. The robot should almost never stand still — the only real
  "wait" is the drive/turn itself.

## The game (how scoring works)

- Robot starts with a **preload = pin 1**.
- Scoring unit = **cup + pin** stacked; tallest stack = **5-pin**.
- **First goal:** preload + 2 (cup+pin) are already on the field; **pins 4 and 5 come from the
  matchloader** (at a wall).
- **Second goal:** the matchloader gives a single pin, then 4 more stacks.
- The **cascade lift** raises to a scoring height per pin count (1-pin up to 5-pin).

## PID tuning procedure (first thing, with the robot)

Tune **lateral first** (drive 24" = one tile), then **angular** (90°, then 180°). Start D = 0, I = 0.

1. **P:** stalls short → raise P. Reaches the target with a *small* overshoot → **stop raising P**
   (that overshoot is the signal P is strong enough). **Do NOT lower P to kill overshoot** — that
   makes it weak and it stalls short, which is exactly the delay problem. Kill overshoot with D.
2. **D:** overshoot / end-jitter → raise D until it settles clean on the target. Sluggish or buzzy →
   back D off.
3. Optional: nudge P up for speed, re-raise D. Add a little I only if it *consistently* stops a hair
   short every run (steady-state error).

Run each test **3–5 times** (tuned = same spot every run), at **match speed**. Once dialed, **freeze
it** — don't re-tune mid-route.

## Wait functions (conventions)

- **Default every move to `pid_wait_quick()`.**
- **`pid_wait()`** only at precision spots (the turn right before a grab, the final scoring approach).
- **Never `pid_wait_quick_chain()`** — it exits *before* the target and flings momentum into the next
  move; it wrecks accuracy and needs chain-constant tuning. Skip it entirely.
- To cut one move's delay surgically, wrap *just that move* in a looser exit condition
  (`pid_turn_exit_condition_set` / `pid_drive_exit_condition_set` before it, restore after). But the
  real fix is PID landing clean.

## Cascade (variable height) — ONE PID, vary SPEED

- A raised cascade = top-heavy = different dynamics. **Do NOT tune multiple PIDs.**
- Tune **one** PID (cascade down, full speed). When the cascade is raised, **drive/turn slower**
  (lower the speed argument) — slower = less momentum = the same PID stays stable.
- Use 2–3 **speed tiers** by height (down = full, partway = medium, high = slow). **PID constants
  never change — only the speed.**
- Lean **slightly more D** on the one tune for cross-condition stability (also helps vs battery/field).
- **Overlap:** raise the cascade *async during the approach drive*, lower it *async during the return*.
  Never park the robot to move the cascade.

## Code conventions

- **Every tunable number in one named block at the top** — drive/turn/cascade speeds, cascade
  heights, exit conditions. Tuning = change one named number, never hunt through the route.
- Consistent movement helpers; **speed and wait behavior explicit** on every move.
- **Mechanisms fire async** (cascade, claw) and run *during* drives; block only when the next step
  physically needs them finished.
- Cascade: **named positions**, async raise/lower, one blocking "wait till it's there"
  (`cascadeMoveTo` with a timeout) for when you must.
- Route step values (distances, angles) are inlined; values shared across files are named constants.
- No clever code — basic and explainable beats elegant.
- Comments: lowercase fragments, no trailing periods, no full sentences; end-of-line where they fit.

## Consistency = process, not just code

- **Consistent start placement** — square against the field perimeter or use a jig. This is the #1
  cause of "it broke and I didn't change code."
- **Battery discipline** — test at a consistent charge.
- **Build the route in segments**, with a **wall re-anchor** (wall-square or DSR) between each, so
  early error can't compound into the later segments.

## Hardware (from the previous robot — VERIFY against the current build)

- Drive: left `{19, 17, 6}`, right `{-20, -18, -3}`, blue cartridges with the **middle motor of each
  side green** (set in `initialize`). IMU port 5. Wheel 2.75", ~450 wheel rpm (verify by tape measure).
- Cascade: 2 motors `{-9, 10}` green. **Position is motor degrees from power-on → the cascade MUST
  start fully down** or every height is wrong.
- Claw: port 2. Grip claw, no ratchet — **hold position, never stall against the stop** (burns the motor).
- Optical / toggle: optical port 16 — **color sensing, same behavior as before** (hue match within a
  tolerance, proximity gate, slow spin so it doesn't blow past the color). Toggle motor port 8.
- Distance sensor(s): for DSR / drive-to-distance off walls (port 7 on the old bot).
- (Old rotation pod was on port 4 — dropping odom, so unused.)

## Multi-machine workflow

- One GitHub repo (**393BATMAN**) is the hub; home computer, Surface Pro, and Claude sessions each
  have their own clone.
- **Pull before you work, commit + push when done, ONE machine at a time.** Never leave uncommitted
  changes on two machines at once — that's what causes merge messes.

## Build

- PROS 4. Build with the PROS VS Code extension's Build button, or `pros m` / `pros mu` in the PROS
  terminal.
