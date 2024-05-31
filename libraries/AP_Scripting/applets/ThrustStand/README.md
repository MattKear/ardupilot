# Thrust Stand
<span style="color:red">
The scripts described here, utilise ArduPilot's Lua scripting enviroment to effectively convert ArduCopter into a Thrust Stand.
</span>
<span style="color:red">
This has the advantage that the thrust stand will work with any ESC/protocol that ArduCopter supports.  Also, it allows for the script/workflow for to be easily customized to utilize all peripherals and sensors that are supported by ArduCopter.  For example, to measure RPM, just add an RPM sensor/ESC telemtry and the data will be added to the ArduCopter log.  Want to use WiFi telemetry to wirelessly operate the thrust stand? simply add an already supported wifi radio.
</span>

## Setup Instructions

Minimum hardware requirements:
- Toggle Switch - Configured as button 1 in AP button library.
- 10 Addressable LEDs for throttle bar. (not used in Thrust_Stand_Simple.Lua)
- SparkFun Qwiic Scale - NAU7802.  Only 1 is required for Thrust only measurements. A second is required for the optional torque measurement.  Note: to use two of these load cells either your flight controller must have 2 I2C buses available or you need an MUX, as the addresses are not configurable on those devices.

Firmware requirements:
- Thrust_Stand.Lua requires building copter from this branch.
- Thrust_Stand_Simple.Lua should run on stable copter (4.4 onwards).


## Connections & Setup
- Thrust load cell amplifier must be connected to I2C bus 1.
- Optional Torque load cell amplifier must be connecter to I2C bus 2 and THST_ENAB_TORQ must be set to 1.
- <span style="color:red">The ESC should be plugged into the flight controller output that you would use normally for flight (e.g. Servo output, CAN, etc) and configured as the Motor1 (33) output. As this script is using ArduCopter, all throttle protocols supported by ArduCopter are supported by this script. </span>
- A toggle switch must be configured as button 1, to act as a safety switch (see: https://ardupilot.org/copter/docs/common-buttons.html)

<img src="./toggleswitch.png" alt="Example toggle switch" width="150"/>

## Calibrations

The load cell outputs a unitless number that requires calibration parameters to convert to loads in your desired units.  To calibrate, apply known loads to the load cells to derive the necessary calibration parameters.  For each load cell a gradient and an offset are required.  The calibration parameters to be set are:
- **THST_CAL0_THST:** Thrust load cell calibration, zero offset.
- **THST_CALM_THST:** Thrust load cell calibration, gradient.
- **THST_CAL0_TORQ:** Torque load cell calibration, zero offset.
- **THST_CALM_TORQ:** Torque load cell calibration, gradient.

Apply a series of different loads (weights are often the easiest) and look at the corresponding MAV_THST_RAW or MAV_TORQ_RAW telemetry streams on the GCS.  Plotting raw output vs applied load should give you a straight line.  Calculate the gradient and zero offset using a line of best fit (linear regression).  Enter the obtained values into the parameters mentioned above.  Finally, re-apply a load and check either MAV_THRUST or MAV_TORQUE on the GCS to make sure that the reported value matches the applied load.

-  Raw measurement from the thrust load cell amplifier.
- **MAV_TORQ_RAW:** Raw measurement from the torque load cell amplifier.

## Example Test Run

<span style="color:red">
1. Power on the flight controller.
</span>

<span style="color:red">
2. Connect the GCS to the flight controller.  This can be in any means you would like, just like with a normal copter.  e.g. You can use USB, WiFi, or some other telemetry radio.
</span>

<span style="color:red">
3. Power on your propulsion system with either a battery or power supply.
</span>

<span style="color:red">
4.Set the flight mode that corresponds to the thrust stand mode that you would like to use.
</span>

<span style="color:red">
5. Perform the remaining checks as suggested below to make sure your sensors are all reporting correctly.
</span>

<span style="color:red">
6. When you are ready to start your test, toggle the safety switch to prepare the system for test.
</span>

<span style="color:red">
7. Arm copter from the GCS and the test will begin.
</span>

<span style="color:red">
8. Once the test has completed the throttle will hold at MOT_SPIN_MIN.  Toggle the safety switch.  This will both disarm ArduCopter and "Safe" the system. 
</span>


## Saftey Switch

<span style="color:red">
The safety switch is there to provide a quick and easy way to stop the test in an emergency.  If at any point during the test there is an issue or you simply want to stop the test, flick the switch and the throttle will be instantly zeroed and ArduCopter will be disarmed.
</span>

<span style="color:red">
Note: If you arm the system with the switch still in the "Safe" position, the test will not begin until the switch has been moved to the "Active" position.
</span>


## Checklists

### Power on Checks
- Power on propulsion system (Battery or power supply)
- Check for correct voltage measurement.
- Check/set current limit (THST_CUR_LIM).

### “Pre-Flight” Checks
- Check thrust load cell reporting correctly (number is moving and in correct direction).
- if applicable, check torque Load cell reporting correctly (number is moving and in correct direction).
- If applicable, check RPM1_SCALE = 1 is set for eRPM  measurement,
- Set MOT_SPIN_MIN (use Copter's motor test) [THST_ENBL_TEST = 1]
- Check/set for thrust linearisation enabled [THST_USE_LIN].  Pick whichever is appropriate for the test.
- Check MOT_SPIN_MAX = 1.0
- Check MOT_PWM_MIN (usually set 1000)
- Check MOT_PWM_MAX (usually set 2000)
- Run up to 50% throttle check.
- Calibrate Thrust and Torque load cells if necessary.

### Test Checks

- Set test mode.


#### Throttle Ramp Mode

-	Set THST_HOLD_S (s) to the length of time to hold throttle at each step.
-	Set THST_MAX_THR (%) [0 - 100] to set the max throttle.
-	Set THST_RAMP_STEP [0.01 - 1.0]


#### Transients / Step-Change Mode

-	Set THST_HOLD_S (s) to the length of time to hold the step change.
-	Set THST_MAX_THR (%) [0 - 100] to the max throttle to set the step change to.
-	Set MOT_THST_HOVER [0.0 - 1.0]
Note: Throttle will step hover throttle +/- (max throttle – hover).  Ensure set to allow symmetric steps.


#### Frequency Sweep / Throttle Chirp Mode
-	Throttle/thrust output is between MOT_SPIN_MIN and MOT_SPIN_MAX
-	Set THST_HOLD_S (s) to the length of time to hold the initial trim state. Usually set 3 s.
-	Set THST_MAX_THR (%) [0 - 100] to the max throttle to set the step change to.
-	Set MOT_THST_HOVER [0.0 - 1.0] this is the centre/mean of the sin wave
-	Set THST_NSE_RAT – the noise ratio or amplitude.
-	Set THST_OMEGA_MIN (rad/s) (default = 0.3)
-	Set THST_OMEGA_MAX (rad/s) (default = 12)


##### Test Procedure:

- Log air temperature, relative humidity, and pressure.
- Arm ArduCopter via GCS.
- Switch toggle safety switch.
- Throttle will ramp up and test will begin.
- Test will be performed  automatically, as per configuration.
- Throttle will ramp down and sit at MOT_SPIN_MIN once test has completed.
- Switch toggle safety switch to both safe and disarm system.


## Test Modes

Copter flight modes are used as a proxy for thrust stand modes. The below describes which copter modes correlates to the thrust stand modes.

You must set the flight modes below, via the GCS, to use the corresponding thrust stand modes described below.

- <span style="color:red">Ensure that you have set the mode via the GCS before you arm the thrust stand, to perform the desired test.</span>

- **Stabilise:** Stepped throttle ramp for expo calculations
- **Alt_Hold** - Throttle mode hold, ramps to THST_MAX_THR and holds for THST_HOLD_S
- **Acro** - Throttle steps, ramps to MOT_THST_HOVER, and then step changes to THST_MAX_THR
- **Guided_NoGPS** – (System ID) Throttle chirps/frequency sweeps


## Parameter Definitions

- **THST_CAL0_THST:** Thrust load cell calibration, zero offset.
- **THST_CALM_THST:** Thrust load cell calibration, gradient.
- **THST_CAL0_TORQ:** Torque load cell calibration, zero offset.
- **THST_CALM_TORQ:** Torque load cell calibration, gradient.
- **THST_CUR_LIM:** Current limit. If battery monitor measures a current above this value the throttle will be instantly zeroed and the test will stop.  Safety switch has to be cycled to allow another test to start again.
- **THST_MAX_THR:** Maximum throttle used. In a ramp test the throttle will ramp between 0 and max.  In all other test modes the throttle will be constrained by this value. 0 to max throttle is within spin min and max.
- **THST_HOLD_S:** Hold time in seconds.  In ramp test, this is the step dwell time.  In the transient, step test, this is the dwell time to hold the throttle at between step inputs.
- **THST_ENBL_TORQ:** Used to enable use of 2nd load cell for torque measurements. 0 = disabled, 1 = enable torque.
- **THST_ENBL_TEST:** Set to 1 (allow) to enable copters motor test from the GCS.  Whenever a thrust stand run is started this param is automatically returned to 0 (disabled) with a set and save.  This is required to stop the script from overriding the output when copter is trying to do its motor test.
- **THST_OMEGA_MIN:** Minimum frequency (rad/s) for throttle chirp test.
- **THST_OMEGA_MAX:** Maximum frequency (rad/s) for throttle chirp test.
- **THST_NSE_RAT:** Noise ratio to be applied during chirp test. Ratio of max throttle.
- **THST_RAMP_STEP:** Spacing between throttle steps in ramp test mode.  E.g. 0.1 = 10% throttle steps.
- **THST_USE_LIN:** – 0 = throttle control, 1 = thrust control.  When enabled thrust to throttle linearisation from multi-rotor code is used. **(Not enabled in Thrust_Stand_Simple.lua)**

- **MOT_PWM_MIN:** Copter MOT_PWM_MIN used in throttle output calculation. See here for definition: https://ardupilot.org/copter/docs/parameters.html#mot-pwm-min-pwm-output-minimum
- **MOT_PWM_MAX:** Copter MOT_PWM_MAX used in throttle output calculation. See here for definition: https://ardupilot.org/copter/docs/parameters.html#mot-pwm-max-pwm-output-maximum
- **MOT_SPIN_MIN:** Copter MOT_SPIN_MIN used in throttle output calculation. See here for definition: https://ardupilot.org/copter/docs/parameters.html#mot-spin-min-motor-spin-minimum
- **MOT_SPIN_MAX:** Copter MOT_PWM_MIN used in throttle output calculation. See here for definition: https://ardupilot.org/copter/docs/parameters.html#mot-spin-max-motor-spin-maximum

- **MOT_THST_EXPO:** Used in thrust linearisation calculation, if THST_USE_LIN is enabled.

- **MOT_THST_HOVER:** used for throttle step transient mode.  This is the middle throttle setting and the the throttle is stepped +/- THST_RAMP_STEP about this value.


## Live Outputs to GCS

Live telemetry data can be seen from the script. The following are output at 2 Hz:
- **MAV_THST_RAW:** Raw measurement from the thrust load cell amplifier.
- **MAV_TORQ_RAW:** Raw measurement from the torque load cell amplifier.
- **MAV_THRUST:** Calibrated thrust calculation from thrust load cell.
- **MAV_TORQUE:** Calibrated thrust calculation from torque load cell.

- <span style="color:red">This data can be viewed on the GCS just like any other telemetry data from Copter.  For example, in mission planner, the "Quick" tab will give numerical read outs and the "Tuning" graphs will plot the data trend vs time. </span>

## Logging

All data is stored in ArduCopter *.bin log files. The thrust and torque data is stored in the **THST** log field.  All other data is stored in the usual log fields. E.g Battery monitor stores current and voltage data, RPM for rpm data etc.
