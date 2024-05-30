# Thrust Stand

## Setup Instructions

Minimum hardware requirements:
- Toggle Switch - Configured as button 1 in AP button library.
- 10 Addressable LEDs for throttle bar. (not used in Thrust_Stand_Simple.Lua)

Firmware requirements:
- Thrust_Stand.Lua requires building from this branch
- Thrust_Stand_Simple.Lua should run on stable copter (4.4 onwards)

## Checklists


### Power on Checks
-	Power Supply set to standby.
-	Switch on Power Supply.
-	Check voltage and current limits.
-	Remove standby.

### “Pre-Flight” Checks
-	Thrust load cell reporting correctly.
-	Torque Load cell reporting correctly.
-	Set THST_CUR_LIM
-	Check RPM1_SCALE = 1
-	Set MOT_SPIN_MIN (motor test) [THST_ENBL_TEST = 1]
-	Check for linearisation enables [THST_USE_LIN]
-	Check MOT_SPIN_MAX = 1.0
-	Check MOT_PWM_MIN (usually set 1000)
-	Check MOT_PWM_MAX (usually set 2000)
-	Run up to 50% throttle check.
-	Calibrate Thrust and Torque

### Test Checks

- Set test mode.

#### Throttle Ramp

-	Set THST_HOLD_S (s) to the length of time to hold throttle at each step.
-	Set THST_MAX_THR (%) [0 - 100] to set the max throttle.
-	Set THST_RAMP_STEP [0.01 - 1.0]
Step Change
-	Set THST_HOLD_S (s) to the length of time to hold the step change.
-	Set THST_MAX_THR (%) [0 - 100] to the max throttle to set the step change to.
-	Set MOT_THST_HOVER [0.0 - 1.0]
Note: Throttle will step hover throttle +/- (max throttle – hover).  Ensure set to allow symmetric steps.
Frequency Sweep
-	Throttle/thrust output is between SPIN_MIN and SPIN_MAX
-	Set THST_HOLD_S (s) to the length of time to hold the initial trim state. Usually set 3 s.
-	Set THST_MAX_THR (%) [0 - 100] to the max throttle to set the step change to.
-	Set MOT_THST_HOVER [0.0 - 1.0] this is the centre/mean of the sin wave
-	Set THST_NSE_RAT – the noise ratio or amplitude.
-	Set THST_OMEGA_MIN (rad/s) (default = 0.3)
-	Set THST_OMEGA_MAX (rad/s) (default = 12)


Test Procedure
-	Log Temp, RH, and Pressure



## Test Modes

Copter flight modes are used as a proxy for thrust stand modes. The below describes which copter modes correlates to the thrust stand modes.

- **Stabilise:** Stepped throttle ramp for expo calculations
- **Alt_Hold** - Throttle mode hold, ramps to THST_MAX_THR and holds for THST_HOLD_S
- **Acro** - Throttle steps, ramps to MOT_THST_HOVER, and then step changes to THST_MAX_THR
- **Guided_NoGPS** – (System ID) Throttle chirps/frequency sweeps


## Parameter Definitions
- **THST_ENBL_TEST:** Set to 1 (allow) to enable copters motor test from the GCS.  Whenever a thrust stand run is started this param is automatically returned to 0 (disabled) with a set and save.  This is required to stop the script from overriding the output when copter is trying to do its motor test.
- **THST_MAX_THR:** The maximum throttle allowed on test.  0 to max throttle is within spin min and max.
- ** THST_USE_LIN:** – 0 = throttle control, 1 = thrust control.  When enabled thrust to throttle linearisation from multi-rotor code is used. **(Not enabled in Thrust_Stand_Simple.lua)**











