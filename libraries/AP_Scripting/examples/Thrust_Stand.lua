-- Port of Sparkfun Qwiic Scale library for NAU7802
-- https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

local THRUST = 1 -- Index for calibration values relating to thrust sensor
local TORQUE = 2 -- Index for calibration values relating to torque sensor

-- Setup param table
-- key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 73
-- generate table
assert(param:add_table(PARAM_TABLE_KEY, "THST_", 9), 'could not add param table')

------------------------------------------------------------------------
-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end
------------------------------------------------------------------------

-- generate params and their defaults
assert(param:add_param(PARAM_TABLE_KEY, 1, 'CAL0_THST', 0.0), 'could not add param CAL0_THST')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'CAL0_TORQ', 0.0), 'could not add param CAL0_TORQ')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'CAL_M_THST', 1.0), 'could not add param CAL_M_THST')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'CAL_M_TORQ', 1.0), 'could not add param CAL_M_TORQ')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'CUR_LIM', 20.0), 'could not add param CUR_LIM')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'MAX_THR', 100.0), 'could not add param MAX_THR') -- (%) constrained to 0 to 100
assert(param:add_param(PARAM_TABLE_KEY, 7, 'HOLD_S', 3), 'could not add param HOLD_S')
assert(param:add_param(PARAM_TABLE_KEY, 8, 'ENBL_TORQ', 1), 'could not add param ENBL_TORQ')
assert(param:add_param(PARAM_TABLE_KEY, 9, 'ENBL_TEST', 0), 'could not add param ENBL_TEST')

-- setup param bindings
local ZERO_OFFSET_PARAM = {0,0}
local CAL_FACT_PARAM = {0,0}
ZERO_OFFSET_PARAM[THRUST] = bind_param("THST_CAL0_THST")
ZERO_OFFSET_PARAM[TORQUE] = bind_param("THST_CAL0_TORQ")
CAL_FACT_PARAM[THRUST] = bind_param("THST_CAL_M_THST")
CAL_FACT_PARAM[TORQUE] = bind_param("THST_CAL_M_TORQ")
local CURRENT_LIMIT = bind_param("THST_CUR_LIM")
local MAX_THR_PARAM = bind_param("THST_MAX_THR")
local THROTTLE_HOLD_TIME_PARAM = bind_param("THST_HOLD_S")
local USE_TORQUE = bind_param("THST_ENBL_TORQ")
local ENABLE_MOTOR_TEST = bind_param("THST_ENBL_TEST")

------------------------------------------------------------------------
local function torque_enabled()
  return (USE_TORQUE:get() > 0)
end
------------------------------------------------------------------------

-- Set pointer to load cell address
local i2c_thrust = i2c.get_device(0,42)
i2c_thrust:set_retries(10)

local i2c_torque = i2c.get_device(1,42)
i2c_torque:set_retries(10)

-- Time (ms) between samples
local _samp_dt_ms = 0

-- Motor function to override
_motor_channel = SRV_Channels:find_channel(33)
local MOTOR_TIMEOUT = 1000 --(ms)

-- Variables needed for setup and init of NAU7802
local _nau7802_setup_started_ms = 0
local _nau7802_i2c_dev
local FAST_SAMPLE = 3
local SLOW_SAMPLE = 0

-- Script vars used in calibration
local _dev_initialised = {false, false}
local _dev_name = {"Thrust", "Torque"}

-- Buttons
local SAFE_BUTTON = 1        -- Button number to set safety state
local CAL_BUTTON = 2         -- Button number for initiating calibration
local AUX1_BUTTON = 4 -- Button number for Aux 1
-- AUX2 = 5                  -- Button number for Aux 2, Additional btn instance needs adding to AP before this can be used

-- Update throttle
local _flag_hold_throttle = false
local _ramp_rate = 0.03 -- (%/s) How quickly throttle is advanced
local _hold_time = 3 --(s) How long the thottle is held at each discreate step
local _last_thr_update = 0 -- (ms) The last time the throttle was updated
local _current_thr = 0 --(%)
local _hold_thr_last_time = 0
local _next_thr_step = 0 --(%)
local _max_throttle = 1 --(%)
local _thr_inc_dec = 1 --(-) Used to switch the motor from increment to decrement
-- Transient step response modes
local _hover_point_achieved = false -- Has the controller achieved the hover throttle
local _throttle_step_index = 1 -- Index in the table of throttle steps to work through

-- Read in PWM if we are using an external throttle
local pwm_in = PWMSource()

-- setup the fast access pwm_min parameter
local mot_pwm_min_param = Parameter()
assert(mot_pwm_min_param:init('MOT_PWM_MIN'), 'failed get MOT_PWM_MIN')
-- init the local variable from the param value
local mot_pwm_min = mot_pwm_min_param:get()

-- setup the fast access pwm_max parameter
local mot_pwm_max_param = Parameter()
assert(mot_pwm_max_param:init('MOT_PWM_MAX'), 'failed get MOT_PWM_MAX')
-- init the local variable from the param value
local mot_pwm_max = mot_pwm_max_param:get()

-- setup the fast access pwm_max parameter
local mot_spin_min_param = Parameter()
assert(mot_spin_min_param:init('MOT_SPIN_MIN'), 'failed get MOT_SPIN_MIN')
-- init the local variable from the param value
local mot_spin_min = mot_spin_min_param:get()

-- setup the fast access pwm_max parameter
local mot_spin_max_param = Parameter()
assert(mot_spin_max_param:init('MOT_SPIN_MAX'), 'failed get MOT_SPIN_MAX')
-- init the local variable from the param value
local mot_spin_max = mot_spin_max_param:get()

-- Sys state - The current state of the system
local DISARMED = 1                      -- Ready to go, not running
local ARMED = 2                         -- Armed and motor running
local CURRENT_PROTECTION = 10           -- Error State

local _sys_state = DISARMED
local _last_sys_state = -1

-- mode
local THROTTLE_MODE_RAMP = 0 -- (copter stabilise) Throttle gradual ramp up and down
local THROTTLE_MODE_STEP = 1 -- (copter acro) Step inputs around hover throttle
local THROTTLE_MODE_HOLD = 2 -- (copter mode alt hold) Ramp to max and hold for defined time
local _throttle_mode = THROTTLE_MODE_RAMP
local _last_mode

-- LEDs
local NUM_LEDS = 10
local _led_chan = 0
local _last_led_update = 0

-- Measurements
local _current = 0.0
local _thrust_raw = 0.0
local _torque_raw = 0.0
local _thrust_time_ms = 0.0
local _torque_time_ms = 0.0

-- telemetry rate
local _last_telem_sent = 0.0

-- Forward declare functions
local init_nau7802

------------------------------------------------------------------------
-- Mask & set a given bit within a register
local function setBit(i2c_dev, bitNumber, registerAddress)
  if i2c_dev == nil then
    return false
  end

  local value = i2c_dev:read_registers(registerAddress)

  if value == nil then
    return false
  end

  value = value | (1 << bitNumber) -- Set this bit
  return i2c_dev:write_register(registerAddress, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Mask & clear a given bit within a register
local function clearBit(i2c_dev, bitNumber, registerAddress)
  if i2c_dev == nil then
    return false
  end

  local value = i2c_dev:read_registers(registerAddress)

  if value then
    value = value & (255 - (1 << bitNumber)) -- Set this bit
    return i2c_dev:write_register(registerAddress, value)
  end

  return false
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Return a given bit within a register
local function getBit(i2c_dev, bitNumber, registerAddress)
  if i2c_dev == nil then
    return 0
  end

  local value = i2c_dev:read_registers(registerAddress)

  if value == nil then
    return 0
  end

  -- Clear all but this bit
  value = value & (1 << bitNumber)
  return value > 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns true if Cycle Ready bit is set (conversion is complete)
local function available(i2c_dev)
  return getBit(i2c_dev, 5, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Check calibration status.
local function calAFEStatus(i2c_dev)
  if getBit(i2c_dev, 2, 2) then
    -- Calibration in progress
    return 1
  end

  if getBit(i2c_dev, 3, 2) then
    -- Calibration failure
    return 2
  end

  -- Calibration passed
  return 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Function to return the samples per second output by the qwiic scale
local function get_sps(rate)
  if rate == 0 then return 10 end
  if rate == 1 then return 20 end
  if rate == 2 then return 40 end
  if rate == 3 then return 80 end
  if rate == 7 then return 320 end
  -- Error value
  return false
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the readings per second
-- 10, 20, 40, and 80 samples per second is available
local function setSampleRate(i2c_dev, rate)
  if i2c_dev == nil then
    return false
  end
  if rate > 7 then
    rate = 7
  end

  local value = i2c_dev:read_registers(2)

  if value == nil then
    return false
  end

  value = value & 143 -- Clear CRS bits
  value = value | (rate << 4)  -- Mask in new CRS bits

  -- set sample delta time in lua
  _samp_dt_ms = math.ceil(1000/get_sps(rate))

  -- handle both error cases of not writting to the register and not getting a valid sample rate
  return i2c_dev:write_register(2, value) and _samp_dt_ms > 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Power up digital and analog sections of scale
local function powerUp(i2c_dev)
  setBit(i2c_dev, 1, 0)
  setBit(i2c_dev, 2, 0)

  -- Wait for Power Up bit to be set - takes approximately 200us
  local counter = 0
  while (true) do
    if getBit(i2c_dev, 3, 0) then
      break -- Good to go
    end
    if counter > 100 then
      return false -- Error
    end
    counter = counter + 1
  end
  return true
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Resets all registers to Power Off Defaults
local function reset(i2c_dev)
  -- Set RR
  setBit(i2c_dev, 0, 0)
  -- Clear RR to leave reset state
  return clearBit(i2c_dev, 0, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the onboard Low-Drop-Out (LDO) voltage regulator to a given value
-- 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
local function setLDO(i2c_dev, ldoValue)
  -- Error check
  if i2c_dev == nil then
    return false
  end
  if ldoValue > 7 then
    ldoValue = 7
  end

  -- Set the value of the LDO
  local value = i2c_dev:read_registers(1)

  if value == nil then
    return false
  end

  -- Clear LDO bits
  value = value & 199
  -- Mask in new LDO bits
  value = value | (ldoValue << 3)
  i2c_dev:write_register(1, value)

  -- Enable the internal LDO
  return setBit(i2c_dev, 7, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the gain
-- x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
local function setGain(i2c_dev, gainValue)
  if i2c_dev == nil then
    return false
  end

  if gainValue > 7 then
    gainValue = 7 -- Error check
  end

  local value = i2c_dev:read_registers(1)

  if value == nil then
    return false
  end

  value = value & 248 -- Clear gain bits
  value = value | gainValue -- Mask in new bits

  return i2c_dev:write_register(1, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns 24-bit reading
-- Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
local function getReading(i2c_dev)
  if i2c_dev == nil then
    return nil
  end

  local MSB = i2c_dev:read_registers(18) --ADC_OUT[23:16]
  local MID = i2c_dev:read_registers(19) --ADC_OUT[15:8]
  local LSB = i2c_dev:read_registers(20) --ADC_OUT[7:0]

  if MSB and MID and LSB then

    local value = (MSB << 16) | (MID << 8) | LSB
    local sign = (value & (1 << 23)) > 0

    if sign then
        value = 16777215 ~ value -- 24 bit XOR to flip the bits for two's compliment
        value = (value * -1) - 1
    end

    return value
  else
    return nil
  end

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- function to set device for functions that are called back
function set_device(i2c_dev, index)
  _nau7802_i2c_dev = i2c_dev
  _nau7802_index = index -- thrust or torque index
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns the instentanious raw output from the amplifer
local function get_raw_reading(i2c_dev)

  local on_scale = nil
    if available(i2c_dev) then
      on_scale = getReading(i2c_dev)
    end

  return on_scale
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns the last retrieved instentanious load on a given load cell applying the calibration
local function get_load(index)

  local on_scale = 0.0
  if index == THRUST then
    on_scale = _thrust_raw
  elseif index == TORQUE then
    on_scale = _torque_raw
  end

  zero = ZERO_OFFSET_PARAM[index]:get()
  grad = CAL_FACT_PARAM[index]:get()
  if grad <= 1 and grad >= -1 then
    grad = 1
  end

  return (on_scale - zero) / CAL_FACT_PARAM[index]:get()

end
------------------------------------------------------------------------

------------------------------------------------------------------------
-- poll the load cell amplifiers to see if they have a new value and store
-- the values in globals for access later in the script
local function fetch_measurements(now)

  local t = get_raw_reading(i2c_thrust)
  if t ~= nil then
    _thrust_raw = t
    _thrust_time_ms = now
  end

  if torque_enabled() then
    local q = get_raw_reading(i2c_torque)
    if q ~= nil then
      _torque_raw = q
      _torque_time_ms = now
    end
  end
end
------------------------------------------------------------------------

------------------------------------------------------------------------
local function begin(i2c_dev)

  local result = true

  result = result and reset(i2c_dev)                      -- Reset all registers
  result = result and powerUp(i2c_dev)                    -- Power on analog and digital sections of the scale
  result = result and setLDO(i2c_dev, 4)                  -- Set LDO to 3.3V
  result = result and setGain(i2c_dev, 7)                 -- Set gain to 128
  result = result and setSampleRate(i2c_dev, SLOW_SAMPLE) -- Set samples per second to 40.  Sample rate must be the same for both devices
  result = result and i2c_dev:write_register(21, 48)      -- Turn off CLK_CHP. From 9.1 power on sequencing.
  result = result and setBit(i2c_dev, 7, 28)              -- Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
  result = result and setBit(i2c_dev, 2, 2)               -- Begin asynchronous calibration of the analog front end.
  return result

end
------------------------------------------------------------------------


----------------------------------------------------------------------------------------------------------
--                                  Thrust test stand code                                              --
----------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------
-- Constrain a value between limits
function constrain(v, vmin, vmax)
  v = math.max(v,vmin)
  v = math.min(v,vmax)
  return v
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- set_device(i2c_dev, index) must be called before this function
init_nau7802 = function()

  if (_nau7802_setup_started_ms == 0) or (millis() - _nau7802_setup_started_ms >= 5000) then
    if begin(_nau7802_i2c_dev) then
      gcs:send_text(0,"Setup done: " .. _dev_name[_nau7802_index])
      _nau7802_setup_started_ms = millis()

    else
      error("Setup failed: " .. _dev_name[_nau7802_index], 1)
      return
    end

  else
    local cal_status = calAFEStatus(_nau7802_i2c_dev)
    if cal_status == 0 then
      gcs:send_text(0,"Inialization done: " .. _dev_name[_nau7802_index])
      _dev_initialised[_nau7802_index] = true

      -- Reset _nau7802 calibration variable
      _nau7802_setup_started_ms = 0

      -- Device initalised, return to main code
      return init, 100
    end

    if cal_status == 1 then
      -- Cal in progress, wait
      gcs:send_text(4, "amp setup in progress")
      return init_nau7802, 100
    end

    -- If we got this far then calibration failed
    gcs:send_text(0,"Initialisation failed: " .. _dev_name[_nau7802_index])
    gcs:send_text(0,"NAU7802 calibration status: " .. tostring(cal_status))
    return
  end

  return init_nau7802, 100
end
------------------------------------------------------------------------

------------------------------------------------------------------------
function init()

  -- Init thrust load cell amplifier
  if not(_dev_initialised[THRUST]) then
      set_device(i2c_thrust, THRUST)
      return init_nau7802, 100
  end

  -- Init torque load cell amplifier if using
  if not(_dev_initialised[TORQUE]) and torque_enabled() then
      set_device(i2c_torque, TORQUE)
      return init_nau7802, 100
  end

  -- Set first throttle step.  This is appropriate for the throttle mode we boot into
  _next_thr_step = 0.1

  -- Init neopixles
  _led_chan = SRV_Channels:find_channel(98)
  if not _led_chan then
    gcs:send_text(6, "LEDs: channel not set")
    return
  end

  -- Find_channel returns 0 to 15, convert to 1 to 16
  _led_chan = _led_chan + 1

  -- Added an extra LED to account for logic level shifter
  if not serialLED:set_num_neopixel(_led_chan,  NUM_LEDS+1) then
    gcs:send_text(6, "LEDs: neopixle not set")
  end

  -- init PWM in
  -- F765 Wing PIN 52 is PWM 3
  assert(pwm_in:set_pin(52), "Failed to setup PWM in (52)")

  -- Now main loop can be started
  return protected_update, 100

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update()

  -- Bodgy conversion from userdata to number
  local now = tonumber(tostring(millis()))

  -- Get state of inputs
  local safe_button_state = button:get_button_state(SAFE_BUTTON)

  -- must be called before update_lights()
  update_throttle_max()
  update_state_msg()
  update_lights(now)
  fetch_measurements(now)

  -----------------------------------------
  --- --- ---       State       --- --- ---
  -----------------------------------------

  -- Check if we should arm the system
  if safe_button_state and _sys_state == DISARMED and arming:is_armed() then
    _sys_state = ARMED
  end

  -- Change to local lua disarm state
  if (not safe_button_state or not arming:is_armed()) then
    _sys_state = DISARMED
  end

  -- Disarm copter if we are not 'armed' in lua.
  -- This handles the copter disarm when we enter an error state e.g. over current protection
  if _sys_state ~= ARMED and _last_sys_state >= ARMED then
    -- we zero throttle once, so that we do not have to keep overiding the motors channel
    zero_throttle()

    local has_disarmed = false
    while (has_disarmed == false) do
      has_disarmed = arming:disarm()
    end
  end

  -- Check if we need to reset after an over-current protection event
  -- We hold the error state until the safe switch has been closed
  if _sys_state == CURRENT_PROTECTION and not safe_button_state then
    _sys_state = DISARMED
  end

  -----------------------------------------
  --- --- ---      Control      --- --- ---
  -----------------------------------------

  -- If the sys state is disarmed then run non-critical updates
  if _sys_state == DISARMED then
    update_while_disarmed()

    -- The only time we do not want to maintain throttle override is when we allow motor test
    if not ENABLE_MOTOR_TEST:get() then
      zero_throttle()
    end
  end

  -- If system is armed update the throttle
  if _sys_state == ARMED and safe_button_state then
    -- If we ever begin to run a test we reset motor test enabled just to make sure it doesn't trip us up
    if ENABLE_MOTOR_TEST:get() then
      ENABLE_MOTOR_TEST:set_and_save(0)
    end

    if (CURRENT_LIMIT:get() <= 0) or (_current < CURRENT_LIMIT:get()) then
      -- Update the output throttle if we are within the current limits
      if _throttle_mode == THROTTLE_MODE_STEP then
        update_throttle_transient(now)

      else --_throttle_mode == THROTTLE_MODE_RAMP or _throttle_mode == THROTTLE_MODE_HOLD
        update_throttle_ramp(now)
      end

    else
      -- Activate current protection
      zero_throttle()
      _sys_state = CURRENT_PROTECTION
    end
  end

  -----------------------------------------
  --- --- ---    Measurement    --- --- ---
  -----------------------------------------

  -- Get current to check for over-current protection
  if battery:healthy(0) then
    _current = battery:current_amps(0)
  end

  local thrust = nil
  if (now - _thrust_time_ms) < (_samp_dt_ms*1.5) then
    thrust = get_load(THRUST)
  end

  local torque = 0.0
  if torque_enabled() and ((now - _torque_time_ms) < (_samp_dt_ms*1.5)) then
    torque = get_load(TORQUE)
  end

  -- Don't log or send thrust or torque are nil/out of date
  if thrust == nil or torque == nil then
    return
  end

  -- Log values
  if _sys_state == ARMED then
      -- Log to data flash logs
      -- We only log a few variables as the rest is already logged within the cpp
      logger.write('THST','ThO,Thst,Torq,TRaw,QRaw,ExTh','ffffff','------','------',_current_thr, thrust, torque, _thrust_raw, _torque_raw, pwm_in:get_pwm_us())
  end

  -- send telem values of thrust and torque to GCS at 2 hz
  if (now - _last_telem_sent) > 500 or (_last_telem_sent <= 0) then
    _last_telem_sent = now

    gcs:send_named_float('thrust', thrust)
    gcs:send_named_float('torque', torque)

    -- send info via send named text to allow for calibration
    gcs:send_named_float('thst_raw', _thrust_raw)
    gcs:send_named_float('torq_raw', _torque_raw)

  end

  -- Normal re-schedules of update are handled in protected_update()

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_state_msg()

  if _sys_state == _last_sys_state then
      return
  end

  if _sys_state == DISARMED then
    gcs:send_text(4,"Disarmed: Ready for test")
    _state_str = "Disarmed"
  end

  if _sys_state == ARMED then
    gcs:send_text(4,"Armed")
    _state_str = "Armed"
  end

  if _sys_state == CURRENT_PROTECTION then
    gcs:send_text(4,"Over current protection activated ")
    _state_str = "Curr Protect"
  end

  _last_sys_state = _sys_state
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_while_disarmed()
  -- Update throttle mode.
  -- Get flight mode. We use AP flight mode as a hack to have easy thrust stand modes that can be changed via mavlink
  local mode = vehicle:get_mode()
  if _last_mode ~= mode then
     -- Only allow throttle mode to be changed in Disarmed State
    _throttle_mode = mode

    if _throttle_mode == THROTTLE_MODE_STEP then
      update_sample_rate(FAST_SAMPLE)

    else --_throttle_mode == THROTTLE_MODE_RAMP or _throttle_mode == THROTTLE_MODE_HOLD
      update_sample_rate(SLOW_SAMPLE)
    end

    _last_mode = mode
  end

  -- update mot pwm min, max and spin, min max, ensuring valid values are obtained
  local min_pwm = mot_pwm_min_param:get()
  if min_pwm then
    mot_pwm_min = min_pwm
  end

  local max_pwm = mot_pwm_max_param:get()
  if max_pwm then
    mot_pwm_max = max_pwm
  end

  local spin_min = mot_spin_min_param:get()
  if spin_min then
    mot_spin_min = spin_min
  end

  local spin_max = mot_spin_max_param:get()
  if spin_max then
    mot_spin_max = spin_max
  end

  -- always reset the throttle step
  reset_thr_step()

end
------------------------------------------------------------------------

------------------------------------------------------------------------
function update_throttle_ramp(time)
  -- set update time when starting the throttle ramp
  if _last_thr_update <= 0 then
    _last_thr_update = time
    _hold_thr_last_time = time
  end

  -- Check whether to throttle hold or advance throttle
  if (time - _hold_thr_last_time) > (THROTTLE_HOLD_TIME_PARAM:get() * 1000) then
    -- timer has passed, advance throttle
    _current_thr = (_current_thr + _ramp_rate * (time - _last_thr_update) * 0.001 * _thr_inc_dec)

    -- constrain the current throttle
    if _thr_inc_dec > 0 then
      -- ramp up
      _current_thr = constrain(_current_thr, 0, _next_thr_step)
    else
      -- ramp down
      _current_thr = constrain(_current_thr, _next_thr_step, _max_throttle)
    end
  end

  -- See if we are at a throttle hold point
  -- now check if we are pausing at increments up the throttle ramp
  if ((_current_thr >= _next_thr_step) and (_thr_inc_dec > 0)) or ((_current_thr <= _next_thr_step) and (_thr_inc_dec < 0)) then
    _hold_thr_last_time = time

    -- Calculate new throttle step
    set_next_thr_step()
  end

  -- update throttle overide
  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_linear_throttle_pwm(_current_thr), MOTOR_TIMEOUT)
  _last_thr_update = time
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle_transient(time)
  local throttle_steps = {-0.05, 0, 0.05, -0.05, 0, 0.05, 0, -0.1, 0, 0.1}

  local hover_throttle = param:get('MOT_THST_HOVER')
  local step_hold_time = 1 -- (s)

  -- Check whether to switch throttle hold off
  if (time - _hold_thr_last_time) > (step_hold_time * 1000) and (_flag_hold_throttle == true) then
      _flag_hold_throttle = false
  end

  if not(_flag_hold_throttle) and (_last_thr_update > 0) then
    -- Gradually ramp throttle to hover throttle
    if not _hover_point_achieved then
      _current_thr = constrain((_current_thr + _ramp_rate * (time - _last_thr_update) * 0.001 * _thr_inc_dec), 0, _max_throttle)
      _last_thr_update = time
      SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_linear_throttle_pwm(_current_thr), MOTOR_TIMEOUT)
    else
      -- Step change throttle
      _current_thr = constrain((hover_throttle + throttle_steps[_throttle_step_index]), 0, _max_throttle)

      -- Set throttle hold
      _hold_thr_last_time = time
      _flag_hold_throttle = true

      -- Output to motor
      SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_linear_throttle_pwm(_current_thr), MOTOR_TIMEOUT)

      -- Increment index for next throttle step
      _throttle_step_index = _throttle_step_index + 1

      -- Check if we have reached the end of our planned steps
      if _throttle_step_index > #throttle_steps then
        -- Ramp the throttle back down
        _throttle_step_index = 1
        _thr_inc_dec = -1
        _hover_point_achieved = false
        -- Ensure current throttle is below hover throttle to ensure throttle rampsback down
        _current_thr = hover_throttle*0.95
      end
    end

    -- Check if initial ramp is complete
    if _current_thr >= hover_throttle then
      _hover_point_achieved = true
      _hold_thr_last_time = time
      _flag_hold_throttle = true
    end

  else
    _last_thr_update = time
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function zero_throttle()
  _current_thr = 0
  reset_thr_step()
  _flag_hold_throttle = false
  _last_thr_update = 0
  _hover_point_achieved = false
  _throttle_step_index = 1
  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle_max()
  if _sys_state == ARMED then
    -- Do not allow max throttle to be changed when armed
    return
  end

  _max_throttle = constrain(MAX_THR_PARAM:get()*0.01,0,1)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Calculate linear throttle
function calc_linear_throttle_pwm(throttle_pct)
  local thr = throttle_pct * (mot_spin_max - mot_spin_min) + mot_spin_min
  thr = constrain(thr, mot_spin_min, mot_spin_max)
  return calc_pwm(thr)
end
------------------------------------------------------------------------

------------------------------------------------------------------------
-- Calculate linear throttle
function calc_pwm(thr)
  return math.floor((thr *  (mot_pwm_max - mot_pwm_min)) + mot_pwm_min)
end
------------------------------------------------------------------------

------------------------------------------------------------------------
function set_next_thr_step()
  -- check if we need to start stepping down through the throttle
  if _current_thr >= _max_throttle then
    _thr_inc_dec = -1
  end

  -- Update next throttle step
  if _throttle_mode == THROTTLE_MODE_HOLD then
    -- this could set a negative throttle but it just gets contstained to 0 below
    _next_thr_step = _max_throttle*_thr_inc_dec

  else
    _next_thr_step = _next_thr_step + (0.1*_thr_inc_dec)

  end
  _next_thr_step = constrain(_next_thr_step, 0.0, _max_throttle)
end
------------------------------------------------------------------------

------------------------------------------------------------------------
function reset_thr_step()
  -- set ramp direction
    _thr_inc_dec = 1

  -- Update next throttle step
  if _throttle_mode == THROTTLE_MODE_HOLD then
    _next_thr_step = _max_throttle
  else
    _next_thr_step = 0.1
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set sample rate appropriate to mode
function update_sample_rate(rate)
  assert(setSampleRate(i2c_thrust, rate), 'Thrust rate set fail')

  if torque_enabled() then
    assert(setSampleRate(i2c_torque, rate), 'Torque rate set fail')
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_lights(now_ms)

  if (now_ms - _last_led_update <= 250) then
    -- limit the led update rate
    return
  end

  -- Define colours for addressable led display
  local colour = {}
  colour['act'] = {0,0.3,0} -- colour assigned to throttle actual value
  colour['error'] = {1,0,0} -- Colour assignment when in over current protection

  if _throttle_mode == THROTTLE_MODE_STEP then
    colour['max'] = {0.15,0,0.2} -- colour assigned to throttle range
    colour['disarm'] = {0.2,0.4,0.1} -- colour assigned to throttle range when disarmed
  else
    colour['max'] = {0,0,0.3} -- colour assigned to throttle range
    colour['disarm'] = {0.2,0.2,0} -- colour assigned to throttle range when disarmed
  end

  -- Array to keep track of state of each led
  local led_state = {}
  for i = 1, NUM_LEDS do
    led_state[i] = {} --create a new row
    led_state[i]['act'] = 0
    led_state[i]['max'] = 0
  end

  -- Calculate the throttle percentage that each led is worth
  if NUM_LEDS <= 0 then
    NUM_LEDS = 1
  end
  local throttle_to_led_pct = 1/NUM_LEDS
  local r, g, b = 0, 0, 0
  local remain = 0

  -- Update state array for full throttle range showing where max throttle is
  remain = _max_throttle
  for i = 1, NUM_LEDS do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- We have found the led that needs to be partially on
      led_state[i]['max'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- This led is fully on
      led_state[i]['max'] = 1

    else
      -- Set led to off
      led_state[i]['max'] = 0
    end
  end


  -- Update state array for full throttle range showing where current throttle is
  remain = _current_thr
  for i = 1, NUM_LEDS do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- We have found the led that needs to be partially on
      led_state[i]['act'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- This led is fully on
      led_state[i]['act'] = 1

    else
      -- Set led to off
      led_state[i]['act'] = 0
    end
  end

  -- Reset the neopixles everytime - this is due to a bug in current master
  serialLED:set_RGB(_led_chan, -1, 0, 0, 0)

  if _sys_state <= DISARMED then
    -- Update led setting
    for i = 1, NUM_LEDS do
      r = colour['disarm'][1]*(led_state[i]['max'])
      g = colour['disarm'][2]*(led_state[i]['max'])
      b = colour['disarm'][3]*(led_state[i]['max'])
      set_LED_colour(i, r, g, b)
    end

  elseif _sys_state == ARMED then
    -- update led setting
    for i = 1, NUM_LEDS do
      r = colour['max'][1]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][1]*led_state[i]['act']
      g = colour['max'][2]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][2]*led_state[i]['act']
      b = colour['max'][3]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][3]*led_state[i]['act']
      set_LED_colour(i, r, g, b)
    end

  else --CURRENT_PROTECTION
    for i = 1, NUM_LEDS do
      r = colour['error'][1]
      g = colour['error'][2]
      b = colour['error'][3]
      set_LED_colour(i, r, g, b)
    end
  end

  serialLED:send(_led_chan)

  _last_led_update = now_ms
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the led colour brightness using ratio from 0 to 1
function set_LED_colour(led, r, g, b)
  local r_byte = math.floor(constrain(r*255,0,255))
  local g_byte = math.floor(constrain(g*255,0,255))
  local b_byte = math.floor(constrain(b*255,0,255))
  serialLED:set_RGB(_led_chan, led, r_byte, g_byte, b_byte)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function protected_update()
  -- Normal operation uses a pcall incase we hit an error whilst the motor is running
  -- we still need to be able to stop it safely
  local success, err = pcall(update)
  if not success then
    gcs:send_text(0, "Internal Error: " .. err)
    return protected_update, 1000
  end

    return protected_update, 10
end
------------------------------------------------------------------------

-- Wait a while before starting so we have a better chance of seeing any error messages
return init, 2000
