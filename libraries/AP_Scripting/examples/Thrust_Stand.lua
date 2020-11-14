-- Port of Sparkfun Qwiic Scale library for NAU7802
-- https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

-- Set pointer to load cell address
local i2c_bus = i2c:get_device_direct(1,0x2A)

-- Time (ms) between samples
local _samp_dt_ms = 0

-- Motor function to override
local SERVO_FUNCTION = 94

-- Variables needed for setup and init of NAU7802
local _nau7802_setup_started = false
local _nau7802_initalised = false

-- Script vars used in calibration
local _calibration_factor = 1
local _zero_offset = 0
local _calibration_mass = 100 -- The known mass value that the sensor will be calibrated with

-- Script vars used in average calculation
local _ave_total = 0 -- Average value obtained
local _ave_n_samp_aquired = 0 -- Number of samples recorded in this average calculation
local _ave_last_samp_ms = 0 -- Time that the last sample was retrieved
local _ave_callback = 0 -- Number of times the average dunction is called back
local _ave_n_samples = 10 -- The number of samples that are requested in the average

-- Average thrust
local _ave_thrust = 0
local _flag_have_thrust = false

-- Buttons
local CAL_BUTTON = 2    -- button number for initiating calibration
local ARMING_BUTTON = 1 -- button number to start arm and sweep motor
local DEAD_MAN = 3      -- button number for dead mans switch

-- Save to file details
local file_name = "thrust_test.csv"
local format_string = "%s, %.4f, %04i, %.0f, %.4f, %.4f, %.5f\n" -- Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage (V), Current (A), Thrust (g)

-- Update throttle
local _flag_hold_throttle = false
local _ramp_rate = 0.03 -- (%/s) How quickly throttle is advanced
local _hold_time = 3 --(s) how long the thottle is held at each discreate step
local _n_throttle_steps = 4 -- Number of discrete steps that the throttle is held at
local _last_thr_update = 0 -- (ms) The last time the throttle was updated
local _current_thr = 0 --(%)
local _hold_thr_last_time = 0
local _next_thr_step = 0 --(%)
local _max_throttle = 0.65 --(%)

-- Sys state - The current state of the system
local REQ_CAL_ZERO_OFFSET = 1    -- Requires calibration, needs zero offset
local REQ_CAL_FACTOR = 2         -- In calibration, needs calibration factor
local DISARMED = 3               -- Ready to go, not running
local ARMED = 4                  -- Armed and motor running
local ERROR = 10                 -- Error State

local _sys_state = REQ_CAL_ZERO_OFFSET
local _last_sys_state = REQ_CAL_ZERO_OFFSET


-- LEDs
local _num_leds = 10
local _led_chan = 0
local _last_max_throttle = 0
local colour = {}
colour['act'] = {0,0.3,0} -- colour assigned to throttle actual value
colour['max'] = {0,0,0.3} -- colour assigned to throttle range

-- array to keep track of state of each led
local led_state = {}
for i = 1, _num_leds do
  led_state[i] = {} --create a new row
  led_state[i]['act'] = 0
  led_state[i]['max'] = 0
end

-- Lua param allocation
local ZERO_OFFSET_PARAM = "SCR_USER1"
local CAL_FACT_PARAM = "SCR_USER2"


------------------------------------------------------------------------
-- Mask & set a given bit within a register
local function setBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  value = value | (1 << bitNumber) -- Set this bit
  return i2c_bus:write_register(registerAddress, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Mask & clear a given bit within a register
local function clearBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  if value then
    value = value & (255 - (1 << bitNumber)) -- Set this bit
    return i2c_bus:write_register(registerAddress, value)
  end
  return false
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Return a given bit within a register
local function getBit(bitNumber, registerAddress)
  local value = i2c_bus:read_registers(registerAddress)
  value = value & (1 << bitNumber) -- Clear all but this bit
  return value > 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns true if Cycle Ready bit is set (conversion is complete)
local function available()
  return getBit(5, 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Check calibration status.
local function calAFEStatus()
  if getBit(2, 2) then
    -- Calibration in progress
    return 1
  end

  if getBit(3, 2) then
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
  -- Error value
  local ret_val = -1
  if rate == 0 then ret_val = 10 end
  if rate == 1 then ret_val = 20 end
  if rate == 2 then ret_val = 40 end
  if rate == 3 then ret_val = 80 end
  if rate == 7 then ret_val = 320 end

  return ret_val
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the readings per second
-- 10, 20, 40, 80, and 320 samples per second is available
local function setSampleRate(rate)
  if rate > 7 then
    rate = 7
  end

  local value = i2c_bus:read_registers(2)
  value = value & 143 -- Clear CRS bits
  value = value | (rate << 4)  -- Mask in new CRS bits

  -- set sample delta time in lua
  _samp_dt_ms = 1000/get_sps(rate)

  return i2c_bus:write_register(2, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Select between 1 and 2
local function setChannel(channelNumber)
  if (channelNumber == 0) then
    return clearBit(7, 2) -- Channel 1 (default)
  else
    return setBit(7, 2) -- Channel 2
  end
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Power up digital and analog sections of scale
local function powerUp()
  setBit(1, 0)
  setBit(2, 0)

  -- Wait for Power Up bit to be set - takes approximately 200us
  local counter = 0
  while (true) do
    if getBit(3, 0) then
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
-- Puts scale into low-power mode
local function powerDown()
  clearBit(1, 0)
  return (clearBit(2, 0))
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Resets all registers to Power Of Defaults
local function reset()
  setBit(0, 0) -- Set RR
  return clearBit(0, 0) -- Clear RR to leave reset state
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the onboard Low-Drop-Out voltage regulator to a given value
-- 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
local function setLDO(ldoValue)
  if ldoValue > 7 then
    ldoValue = 7 -- Error check
  end

  -- Set the value of the LDO
  local value = i2c_bus:read_registers(1)
  value = value & 199 -- Clear LDO bits
  value = value | (ldoValue << 3) -- Mask in new LDO bits
  i2c_bus:write_register(1, value)

  return setBit(7, 0) -- Enable the internal LDO
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the gain
-- x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
local function setGain(gainValue)
  if gainValue > 7 then
    gainValue = 7 -- Error check
  end

  local value = i2c_bus:read_registers(1)
  value = value & 248 -- Clear gain bits
  value = value | gainValue -- Mask in new bits

  return i2c_bus:write_register(1, value)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Get the revision code of this IC
local function getRevisionCode()
  revisionCode = i2c_bus:read_registers(31)
  return revisionCode & 0x0F
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns 24-bit reading
-- Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
local function getReading()

  local MSB = i2c_bus:read_registers(18)
  local MID = i2c_bus:read_registers(19)
  local LSB = i2c_bus:read_registers(20)

  if MSB and MID and LSB then

    local value = (MSB << 16) | (MID << 8) | LSB
  -- Quite likely to be a signed vs unsigned error here !! need to check
  
  --  local sign = (value & (1 << 23)) > 0
  --  if sign then
  --    value = 16777215 ~ value -- 23 bit not
  --   end
  -- gcs:send_text(0,string.format("%u, %u, %u, %u",MSB,MID,LSB,value))

    return value
  else
    return false
  end
  
end
------------------------------------------------------------------------


------------------------------------------------------------------------
local function reset_ave_var()
    _ave_n_samp_aquired = 0
    _ave_last_samp_ms = 0
    _ave_callback = 0
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- function is called recursivley based on sample rate set in script.  Will do a max of 50 callbacks
-- before force stoping, computing the average obtained at that time.
-- returns true on average calculation complete
function calc_average()

    -- if this is the first call back then reset the average value
    if _ave_callback < 1 then
        _ave_total = 0
    end

    local now = millis()
    _ave_callback = _ave_callback + 1

    if ((now - _ave_last_samp_ms) > _samp_dt_ms  or  _ave_last_samp_ms == 0) and available() then
        -- Add new reading to average
        _ave_total = _ave_total + getReading()
        _ave_n_samp_aquired = _ave_n_samp_aquired + 1

        -- Record time of last measurement
        _ave_last_samp_ms = now
    end

    if (_ave_n_samp_aquired == _ave_n_samples) or (_ave_callback > 50) then
        -- Compute average
        if _ave_n_samp_aquired > 0  then
            _ave_total = _ave_total/_ave_n_samp_aquired
        else
            -- Return error value
            _ave_total = -1
        end

        -- reset average script variables ready for next inital call to get_average
        reset_ave_var()

        -- calculation complete
        return true
    end

    -- if we have got this far we haven't finished computing the average
    return false

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Call when scale is setup, level, at running temperature, with nothing on it
local function calculate_zero_offset()

    if calc_average() then
        _zero_offset = _ave_total
        gcs:send_text(4,"Zero offset set: " .. tostring(_zero_offset))

        -- Save offset to EEPROM with param
        if not(param:set_and_save(ZERO_OFFSET_PARAM, _zero_offset)) then
            gcs:send_text(4,"Zero offset param set fail")
        end

        -- Advance state to next step in calibration process
        _sys_state = REQ_CAL_FACTOR
        return update, _samp_dt_ms
    end

    -- carry on doing average calculation
    return calculate_zero_offset, _samp_dt_ms

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
local function calculate_calibration_factor()

    if calc_average() then
        -- we have got average, calc claibration gradient
        _calibration_factor = (_ave_total - _zero_offset) / _calibration_mass
        gcs:send_text(4,"Scale calibrated factor: " .. tostring(_calibration_factor))

        if _calibration_factor <= 0 then
            -- Set error value
            _calibration_factor = -1
            gcs:send_text(0,"Calibration error")

            -- return to update without advancing system state
            return update, _samp_dt_ms

        else
            -- valid calibration factor so save offset to EEPROM with param
            if not(param:set_and_save(CAL_FACT_PARAM, _calibration_factor)) then
              gcs:send_text(4,"Cal factor param set fail")
            end
        end

        -- Advance system state and exit calibration
        _sys_state = DISARMED
        return update, _samp_dt_ms

    end

    -- Carry on doing average calculation
    return calculate_calibration_factor, _samp_dt_ms

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Returns the instentanious thrust
local function get_inst_thrust()

    -- Prevent divide by zero, issue persistant warning
    if _calibration_factor <= 0 then
        gcs:send_text(0,"WARN: cal fact <= 0")
        return -1
    end

    local on_scale = getReading()

    -- Prevent the current reading from being less than zero offset
    -- This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
    -- causing the weight to be negative or jump to millions of grams
    if on_scale < _zero_offset then
        on_scale = _zero_offset -- Force reading to zero
    end

    -- Calc and thrust
    return (on_scale - _zero_offset) / _calibration_factor

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set Int pin to be high when data is ready (default)
local function setIntPolarityHigh()
  return clearBit(7, 1) -- 0 = CRDY pin is high active (ready when 1)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set Int pin to be low when data is ready
local function setIntPolarityLow()
  return setBit(7, 1) -- 1 = CRDY pin is low active (ready when 0)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
local function begin()

  local result = true

  result = result and reset() -- Reset all registers
  result = result and powerUp() -- Power on analog and digital sections of the scale
  result = result and setLDO(4) -- Set LDO to 3.3V
  result = result and setGain(7) -- Set gain to 128
  result = result and setSampleRate(0) -- Set samples per second to 10
  result = result and i2c_bus:write_register(21, 48) -- Turn off CLK_CHP. From 9.1 power on sequencing.
  result = result and setBit(7, 28) -- Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
  result = result and setBit(2, 2) -- Begin asynchronous calibration of the analog front end.
                                   -- Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
  return result

end
------------------------------------------------------------------------




----------------------------------------------------------------------------------------------------------------
-- Thrust test stand code --------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------
-- constrain a value between limits
function constrain(v, vmin, vmax)
  if v < vmin then
     v = vmin
  end
  if v > vmax then
     v = vmax
  end
  return v
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function init()

    -- Always have throttle at zero to begin with
    --zero_throttle()

    if not _nau7802_setup_started then
        if begin() then
            gcs:send_text(0,"Setup done")
            _nau7802_setup_started = true
        else
            gcs:send_text(0,"Setup failed")
            return
        end

    elseif not _nau7802_initalised then
        local cal_status = calAFEStatus()
        if cal_status == 0 then
            gcs:send_text(0,"Inialization done")
            _nau7802_initalised = true

            -- Setup file to record data to
            local file = assert(io.open(file_name, "w"),"Could not make file: " .. file_name)
            local header = 'Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage(V), Current (A), Thrust (g)\n'
            file:write(header)
            file:close()

            -- Set first throttle step
            set_next_thr_step()

            load_calibration()

            -- Init neopixles
            _led_chan = SRV_Channels:find_channel(95)
            if not _led_chan then
              gcs:send_text(6, "LEDs: channel not set")
              return
            end
            -- find_channel returns 0 to 15, convert to 1 to 16
            _led_chan = _led_chan + 1
            serialLED:set_num_neopixel(_led_chan,  _num_leds)

            -- Now regular update can be started
            return update, 500
        end

        if cal_status == 1 then
            -- Cal in progress, wait
            return init, 500
        end

        -- If we got this far then calibration failed
        gcs:send_text(0,"Initialisation failed ")
        gcs:send_text(0,"NAU7802 calibration status: " .. tostring(cal_status))
        return
    end

    return init, 500
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update()

    -- Bodgy conversion from userdata to number
    local now = tonumber(tostring(millis()))

    update_state_msg()

    update_lights()

    -- Get state of inputs
    local cal_button_state = button:get_button_state(CAL_BUTTON)
    local arm_button_state = button:get_button_state(ARMING_BUTTON)
    local run_button_state = button:get_button_state(DEAD_MAN)

    --- --- --- state machine --- --- ---
    -- Reset calibration 
    if _sys_state == DISARMED and cal_button_state then
        _sys_state = REQ_CAL_ZERO_OFFSET

        -- Reset calibration in memory
        if not(param:set_and_save(ZERO_OFFSET_PARAM, 0)) then
            gcs:send_text(4,"Zero offset param set fail")
        end
        if not(param:set_and_save(CAL_FACT_PARAM, 0)) then
            gcs:send_text(4,"Cal factor param set fail")
        end

        return update, _samp_dt_ms
    end

    if arm_button_state and _sys_state == DISARMED then
        -- Arm copter so that we can use the batt failsafes
        if arming:arm() then
            _sys_state = ARMED
        end
    end

    if not arm_button_state and (_sys_state ~= REQ_CAL_ZERO_OFFSET) and (_sys_state ~= REQ_CAL_FACTOR) then
        _sys_state = DISARMED
    end



    if _sys_state ~= ARMED and _last_sys_state == ARMED then
        local has_disarmed = false
        while (has_disarmed == false) do
            has_disarmed = arming:disarm()
        end
    end

    if _sys_state == REQ_CAL_ZERO_OFFSET and cal_button_state then
        return calculate_zero_offset, 500
    end

    if _sys_state == REQ_CAL_FACTOR and cal_button_state then
        return calculate_calibration_factor, 500
    end

    if _sys_state == ARMED and run_button_state then
        -- update the output throttle
        update_throttle(now)

        local thrust = get_inst_thrust()

        -- Update rpm
        local rpm = RPM:get_rpm(0)
        if (rpm == nil) then
            rpm = -1
        end

        -- Update voltage and current
        -- Init to error state
        local voltage = -1
        local current = -1
        if battery:healthy(0) then
            voltage = battery:voltage(0)
            current = battery:current_amps(0)
        end

        -- Log values
        file = io.open(file_name, "a")
        file:write(string.format(format_string, tostring(now), _current_thr, calc_pwm(_current_thr), rpm, voltage, current, thrust))
        file:close()

    else
        -- Do not run motor without valid calibration or when disarmed
        zero_throttle()
    end

    --gcs:send_text(4,"Sys State = " .. tostring(_sys_state))

    return update, _samp_dt_ms

end
------------------------------------------------------------------------



------------------------------------------------------------------------
function update_state_msg()

    if _sys_state == _last_sys_state then
        return
    end

    if _sys_state == REQ_CAL_ZERO_OFFSET then
        gcs:send_text(4,"In calibration")
        gcs:send_text(4,"Unload sensor and press cal button")
    end

    if _sys_state == REQ_CAL_FACTOR then
      gcs:send_text(4,"In calibration")
      gcs:send_text(4,"Apply cal mass and press cal button")
    end

    if _sys_state == DISARMED then
      gcs:send_text(4,"Disarmed: Ready for test")
    end

    if _sys_state == ARMED then
      gcs:send_text(4,"Armed")
    end

    if _sys_state == ERROR then
      gcs:send_text(4,"Error")
    end

    _last_sys_state = _sys_state
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function load_calibration()

    local temp = param:get(ZERO_OFFSET_PARAM)
    gcs:send_text(4,"param 0 off: " .. tostring(temp))
    if (temp > 0) then
        -- Set value from storage
        _zero_offset = temp
    else
        -- Don't have valid calibration stored
        return
    end

    temp = param:get(CAL_FACT_PARAM)
    gcs:send_text(4,"param fact: " .. tostring(temp))
    if (temp > 0) then
      -- Set value from storage
      _calibration_factor = temp
    else
        -- Don't have valid calibration stored
        return
    end

    -- If we got this far then we have loaded a valid calibration
    gcs:send_text(4,"Calibration values loaded ")
    _sys_state = DISARMED

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_throttle(time)

    -- Check whether to switch throttle hold off
    if (time - _hold_thr_last_time) > (_hold_time * 1000) and (_flag_hold_throttle == true) then
        _flag_hold_throttle = false
    end

    -- Dont advance throttle if _last_update < 0
    if not(_flag_hold_throttle) and (_last_thr_update > 0) then
        -- Calculate throttle if it is to be increased
        _current_thr = constrain((_current_thr + _ramp_rate * (time - _last_thr_update) * 0.001),0,1)
        _last_thr_update = time
        SRV_Channels:set_output_pwm(SERVO_FUNCTION, calc_pwm(_current_thr))

        -- See if we are at a throttle hold point
        if _current_thr >= _next_thr_step then
            _hold_thr_last_time = time
            _flag_hold_throttle = true

            -- calculate new throttle step
            set_next_thr_step()
        end
    else
      _last_thr_update = time
    end

end
------------------------------------------------------------------------


------------------------------------------------------------------------
function zero_throttle()
    _current_thr = 0
    -- Ensure minimum of 2 throttle steps
    if _n_throttle_steps < 2 then 
        _n_throttle_steps = 2
    end
    -- Reset throttle step
    _next_thr_step = _max_throttle/_n_throttle_steps
    _flag_hold_throttle = false
    _last_thr_update = 0
    SRV_Channels:set_output_pwm(SERVO_FUNCTION, calc_pwm(_current_thr))
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function calc_pwm(throttle_pct)
    -- Calc pwm from throttle as a % (0-1)
    local pwm = math.floor((throttle_pct * 1000) + 1000)
    return pwm
end
------------------------------------------------------------------------



------------------------------------------------------------------------
function set_next_thr_step()
    -- Ensure minimum of 2 throttle steps
    if _n_throttle_steps < 2 then 
        _n_throttle_steps = 2
    end

    -- Update next throttle step
    _next_thr_step = _next_thr_step + (_max_throttle)/_n_throttle_steps
    _next_thr_step = constrain(_next_thr_step,0,_max_throttle)
end
------------------------------------------------------------------------


------------------------------------------------------------------------
function update_lights()

  -- calculate the throttle percentage that each led is worth
  if _num_leds <= 0 then
    return
  end
  local throttle_to_led_pct = 1/_num_leds

  local r, g, b = 0, 0, 0
  local remain = 0


  -- update state array for full throttle range
  remain = _max_throttle
  for i = 1, _num_leds do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- we have found the led that needs to be partially on
      led_state[i]['max'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- this led is fully on
      led_state[i]['max'] = 1

    else
      -- set led to off
      led_state[i]['max'] = 0
    end
  end


  -- update state array for full throttle range
  remain = _current_thr
  for i = 1, _num_leds do
    remain = remain - throttle_to_led_pct

    if (remain <= 0) and (remain >= -throttle_to_led_pct) then
      -- we have found the led that needs to be partially on
      led_state[i]['act'] = (throttle_to_led_pct+remain)/throttle_to_led_pct

    elseif (remain > 0) then
      -- this led is fully on
      led_state[i]['act'] = 1

    else
      -- set led to off
      led_state[i]['act'] = 0
    end
  end


  -- update led setting
  remain = _current_thr
  for i = 1, _num_leds do
    r = colour['max'][1]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][1]*led_state[i]['act']
    g = colour['max'][2]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][2]*led_state[i]['act']
    b = colour['max'][3]*(led_state[i]['max']-led_state[i]['act']) + colour['act'][3]*led_state[i]['act']
    set_LED_colour(i, r, g, b)
  end

  serialLED:send(_led_chan)

end
------------------------------------------------------------------------


------------------------------------------------------------------------
-- Set the led colour brightness using percentages
function set_LED_colour(led, r, g, b)
      -- set leds colours
      local r_byte = math.floor(constrain(r*255,0,255))
      local g_byte = math.floor(constrain(g*255,0,255))
      local b_byte = math.floor(constrain(b*255,0,255))
      serialLED:set_RGB(_led_chan, led, r_byte, g_byte, b_byte)
end
------------------------------------------------------------------------


-- Wait a while before starting so we have a better chance of seeing any error messages
return init, 5000