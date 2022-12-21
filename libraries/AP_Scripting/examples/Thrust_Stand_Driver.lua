-- Script to Drive the motor output as a seperate isolated system to the logging unit

-- Setup param table
-- key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 73
-- generate table
assert(param:add_table(PARAM_TABLE_KEY, "THST_", 8), 'could not add param table')

------------------------------------------------------------------------
-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end
------------------------------------------------------------------------

-- generate params and their defaults
assert(param:add_param(PARAM_TABLE_KEY, 6, 'MAX_THR', 100.0), 'could not add param MAX_THR') -- (%) constrained to 0 to 100
assert(param:add_param(PARAM_TABLE_KEY, 7, 'HOLD_S', 3), 'could not add param HOLD_S')

-- setup param bindings
local MAX_THR_PARAM = bind_param("THST_MAX_THR")
local THROTTLE_HOLD_TIME_PARAM = bind_param("THST_HOLD_S")

-- Motor function to override
_motor_channel = SRV_Channels:find_channel(33)
_to_logger_channel = SRV_Channels:find_channel(34)
local MOTOR_TIMEOUT = 1000 --(ms)

-- Update throttle
local _ramp_rate = 0.03 -- (%/s) How quickly throttle is advanced
local _hold_time = 3 --(s) How long the thottle is held at each discreate step
local _last_thr_update = 0 -- (ms) The last time the throttle was updated
local _current_thr = 0 --(%)
local _hold_thr_last_time = 0
local _next_thr_step = 0.1 --(%)
local _max_throttle = 1 --(%)
local _thr_inc_dec = 1 --(-) Used to switch the motor from increment to decrement

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

-- Sys state - The current state of the system
local DISARMED = 1                      -- Ready to go, not running
local ARMED = 2                         -- Armed and motor running

local _sys_state = DISARMED
local _last_sys_state = -1

-- mode
local THROTTLE_MODE_RAMP = 0 -- (copter stabilise) Throttle gradual ramp up and down
local THROTTLE_MODE_STEP = 1 -- (copter acro) Step inputs around hover throttle
local THROTTLE_MODE_HOLD = 2 -- (copter mode alt hold) Ramp to max and hold for defined time
local _throttle_mode = THROTTLE_MODE_RAMP
local _last_mode

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
function update()

  -- Bodgy conversion from userdata to number
  local now = tonumber(tostring(millis()))

  update_throttle_max()
  update_state_msg()

  -----------------------------------------
  --- --- ---       State       --- --- ---
  -----------------------------------------

  -- Check if we should arm the system
  if arming:is_armed() then
    _sys_state = ARMED
  end

  -- Change to local lua disarm state
  if not arming:is_armed() then
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

  -----------------------------------------
  --- --- ---      Control      --- --- ---
  -----------------------------------------

  -- If the sys state is disarmed then run non-critical updates
  if _sys_state == DISARMED then
    update_while_disarmed()
  end

  -- If system is armed update the throttle
  if _sys_state == ARMED
    -- Update the output throttle
    update_throttle_ramp(now)
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

  -- update mot pwm min and max
  local min_pwm = mot_pwm_min_param:get()
  if min_pwm then
    mot_pwm_min = min_pwm
  end

  local max_pwm = mot_pwm_max_param:get()
  if max_pwm then
    mot_pwm_max = max_pwm
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
  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(_to_logger_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
  _last_thr_update = time
end
------------------------------------------------------------------------

------------------------------------------------------------------------
function zero_throttle()
  _current_thr = 0
  reset_thr_step()
  _last_thr_update = 0
  SRV_Channels:set_output_pwm_chan_timeout(_motor_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
  SRV_Channels:set_output_pwm_chan_timeout(_to_logger_channel, calc_pwm(_current_thr), MOTOR_TIMEOUT)
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
function calc_pwm(throttle_pct)
  -- Calc pwm from throttle as a % (0-1)
  return math.floor((throttle_pct * (mot_pwm_max - mot_pwm_min)) + mot_pwm_min)
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
return protected_update, 2000
