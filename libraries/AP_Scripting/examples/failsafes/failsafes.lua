-- Lua Failsafe Script V1.0

-- Script to add failsafe functionaility in the event of the following:
   -- Loss of GPS
   -- Loss of GCS
   -- Geofence Breach
   -- Engine Failure

-- For all failsafes:
   -- first action (on short timer) is to change to RTL and slow speed to safe deployable chute speed
   -- second action (on long timer) is to just deplot chute

-- Bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- Use ICE param for the minimum RPM to be conisdered running
local ICE_RPM_THRESH = Parameter()
if not ICE_RPM_THRESH:init('ICE_RPM_THRESH') then
   gcs:send_text(6, 'get ICE_RPM_THRESH failed')
end

local TECS_SPDWEIGHT = Parameter()
if not TECS_SPDWEIGHT:init('TECS_SPDWEIGHT') then
   gcs:send_text(6, 'get TECS_SPDWEIGHT failed')
end
local _init_spdweight = TECS_SPDWEIGHT:get()

-- Create param table for "Lua FailSafes"
local PARAM_TABLE_KEY = 75 -- Does not clash with ready_take_off script table (key = 74)
assert(param:add_table(PARAM_TABLE_KEY, "LFS_", 12), 'could not add param table')

-- Add params to table
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RPM_SHORT', 5), 'could not add LFS_RPM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'RPM_LONG', 30), 'could not add LFS_RPM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'CHUTE_SPD', 20), 'could not add LFS_CHUTE_SPD')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'FENCE_SHORT', 5), 'could not add LFS_FENCE_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'FENCE_LONG', 30), 'could not add LFS_FENCE_LONG')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'GPS_SHORT', 10), 'could not add LFS_GPS_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 7, 'GPS_LONG', 60), 'could not add LFS_GPS_LONG')
assert(param:add_param(PARAM_TABLE_KEY, 8, 'TELEM_SHORT', 10), 'could not add LFS_TELEM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 9, 'TELEM_LONG', 60), 'could not add LFS_TELEM_LONG')
assert(param:add_param(PARAM_TABLE_KEY, 10, 'HOME_DIST', 50), 'could not add LFS_HOME_DIST')
assert(param:add_param(PARAM_TABLE_KEY, 11, 'HOME_ALT', 20), 'could not add LFS_HOME_ALT')
assert(param:add_param(PARAM_TABLE_KEY, 12, 'RPM_CHAN', 1), 'could not add LFS_RPM_CHAN')


local LFS_RPM_SHORT = bind_param("LFS_RPM_SHORT")
local LFS_RPM_LONG = bind_param("LFS_RPM_LONG")
local LFS_CHUTE_SPD = bind_param("LFS_CHUTE_SPD")
local LFS_FENCE_SHORT = bind_param("LFS_FENCE_SHORT")
local LFS_FENCE_LONG = bind_param("LFS_FENCE_LONG")
local LFS_GPS_SHORT = bind_param("LFS_GPS_SHORT")
local LFS_GPS_LONG = bind_param("LFS_GPS_LONG")
local LFS_TELEM_SHORT = bind_param("LFS_TELEM_SHORT")
local LFS_TELEM_LONG = bind_param("LFS_TELEM_LONG")
local LFS_HOME_DIST = bind_param("LFS_HOME_DIST")
local LFS_HOME_ALT = bind_param("LFS_HOME_ALT")
local LFS_RPM_CHAN = bind_param("LFS_RPM_CHAN")

local MODE_RTL = 11
local MODE_MANUAL = 0
local REASON_RC = 1

local _in_failsafe_short = false
local _in_failsafe_long = false
local _have_set_mode = false
local _in_home_bubble = false
local _armed_and_flying = false
local _in_manual_on_rc = false

local arm_check_auth_id = arming:get_aux_auth_id()

local _last_lua_fs_msg_sent = uint32_t(0)

local _rpm = 0

local REASON_RPM_FS = 1
local REASON_GPS_FS = 2
local REASON_GEOFENCE_FS = 3
local REASON_TELEM_FS = 4
local _fs_reason_str = {}
_fs_reason_str[REASON_RPM_FS] = "RPM Lost"
_fs_reason_str[REASON_GPS_FS] = "GPS Lost"
_fs_reason_str[REASON_GEOFENCE_FS] = "Fence Breach"
_fs_reason_str[REASON_TELEM_FS] = "GCS Lost"


local _set_speed_complete = false
function do_fs_short_action(now_ms, reason)
   -- protection to prevent mode changes if we have already popped the chute
   if _in_failsafe_long then
      return
   end

   -- Tell user that we are in a failsafe
   if ((now_ms - _last_lua_fs_msg_sent):tofloat() > 5000.0) or (_last_lua_fs_msg_sent == 0) then
      gcs:send_text(2, "LFS: " .. _fs_reason_str[reason] .. ", Short")
      _last_lua_fs_msg_sent = now_ms
   end

   -- return early if we are inside the home bubble, disarmed, or not flying so that we have notified the user but do not enact the failsafe action
   if (not _armed_and_flying) or (_in_home_bubble) then
      return
   end

   -- Change to RTL without spamming to allow user to take back control
   if (vehicle:get_mode() ~= MODE_RTL) and (not _have_set_mode) then
      vehicle:set_mode(MODE_RTL)
      _have_set_mode = true
   end

   if _have_set_mode and not _set_speed_complete then
      local airspeed_ms = LFS_CHUTE_SPD:get()
      _set_speed_complete = vehicle:do_change_airspeed(airspeed_ms)

      if (reason == REASON_RPM_FS) then
         -- Change TECS_SPDWEIGHT to ensure we prioritise the safe flying airspeed with pitch control
         TECS_SPDWEIGHT:set(2)
      end
   end
end


function do_fs_long_action(now_ms, reason)
   -- Notify the user that we are in long failsafe
   if ((now_ms - _last_lua_fs_msg_sent):tofloat() > 5000.0) or (_last_lua_fs_msg_sent == 0) then
      gcs:send_text(2, "LFS: " .. _fs_reason_str[reason] .. ", Long")
      _last_lua_fs_msg_sent = now_ms
   end

   -- release the chute if not in the home bubble and flying
   if (_armed_and_flying) and (not _in_home_bubble) and (not _in_manual_on_rc) then
      parachute:release()
   end

end


function reset_fs()
   _last_lua_fs_msg_sent = 0
   _in_failsafe_short = false
   _in_failsafe_long = false
   _have_set_mode = false
   _confirm_counter = 0
   _set_speed_complete = false
   TECS_SPDWEIGHT:set(_init_spdweight)
   _breach_bm = 0
   _in_manual_on_rc = false
end


local _rpm_failed_ms = 0
function check_engine(now_ms)

   _rpm = RPM:get_rpm(math.max(LFS_RPM_CHAN:get() - 1, 0))

   if (_rpm < ICE_RPM_THRESH:get()) and (_rpm_failed_ms == 0) then
      _rpm_failed_ms = now_ms
      -- Only just set the time on this loop so cannot failsafe yet
      return false
   elseif (_rpm >= ICE_RPM_THRESH:get()) then
      _rpm_failed_ms = 0
      return false
   end

   -- Long failsafe timer
   if ((now_ms - _rpm_failed_ms):tofloat() >= LFS_RPM_LONG:get()*1000) and (_rpm_failed_ms > 0) and (_in_failsafe_short) then
      _in_failsafe_long = true
      return true
   end

   -- Short failsafe timer
   if ((now_ms - _rpm_failed_ms):tofloat() >= LFS_RPM_SHORT:get()*1000) and (_rpm_failed_ms > 0) then
      _in_failsafe_short = true
      return true
   end

   -- if we got this far we are not in failsafe yet
   return false

end


local _breach_bm = 0
local _time_since_breach = 0
function check_fence(now_ms)

   _breach_bm = fence:get_breaches()

   if _breach_bm == 0 then
      return false
   end

   -- Note: Fence only returns to the first break time so do not have to worry about multiple breaches reseting the timer
   local _last_fence_breach_time = fence:get_breach_time()

   local _time_since_breach = 0
   -- Protect against race conditions incase _last_gps_time is larger than now_ms leading to a uint32 wrap.
   if (_last_fence_breach_time < now_ms) then
      _time_since_breach = (now_ms - _last_fence_breach_time):tofloat()
   end

   -- Long failsafe timer
   if (_time_since_breach >= LFS_FENCE_LONG:get()*1000) and (_in_failsafe_short) then
      _in_failsafe_long = true
      return true
   end

   -- Short failsafe timer
   if _time_since_breach >= LFS_FENCE_SHORT:get()*1000 then
      _in_failsafe_short = true
      return true
   end

   -- if we got this far we are not in failsafe yet
   return false
end

-- track if we have seen a given instance of GPS scince boot
local _last_gps_time = uint32_t(0)
local _time_since_last_gps = 0
function check_gps(now_ms)

   -- Check both instances of physical gps devices
   for i = 0, 1, 1 do
      -- If GPS status is FIX_2D or better then we get time updates
      local last_gps = gps:last_fix_time_ms(i)

      if last_gps > uint32_t(0) then
         -- we have seen this instance of GPS before
         if last_gps > _last_gps_time then
            -- use the latest update
            _last_gps_time = last_gps
         end
      end
   end

   _time_since_last_gps = 0
   -- Protect against race conditions incase _last_gps_time is larger than now_ms leading to a uint32 wrap.
   if (_last_gps_time < now_ms) then
      _time_since_last_gps = (now_ms - _last_gps_time):tofloat()
   end

   -- Long failsafe timer
   if (_time_since_last_gps >= LFS_GPS_LONG:get()*1000) and (_in_failsafe_short) then
      _in_failsafe_long = true
      return true
   end

   -- Short failsafe timer
   if _time_since_last_gps >= LFS_GPS_SHORT:get()*1000 then
      _in_failsafe_short = true
      return true
   end

   -- if we got this far we are not in failsafe yet
   return false
end


-- track if we have seen a given instance of GPS scince boot
local _last_gcs_time = uint32_t(0)
local _time_since_last_gcs = 0
function check_gcs(now_ms)

   _last_gcs_time = gcs:last_seen()

   if _last_gcs_time == 0 then
      -- we haven't seen a GCS yet
      return true
   end

   _time_since_last_gcs = 0
   -- Protect against race conditions incase _last_gcs_time is larger than now_ms leading to a uint32 wrap.
   if (_last_gcs_time < now_ms) then
      _time_since_last_gcs = (now_ms - _last_gcs_time):tofloat()
   end

   -- Long failsafe timer
   if (_time_since_last_gcs >= LFS_TELEM_LONG:get()*1000) and (_in_failsafe_short) then
      _in_failsafe_long = true
      return true
   end

   -- Short failsafe timer
   if _time_since_last_gcs >= LFS_TELEM_SHORT:get()*1000 then
      _in_failsafe_short = true
      return true
   end

   -- if we got this far we are not in failsafe yet
   return false
end


-- example main loop function
function update()

   _armed_and_flying = arming:is_armed() and vehicle:get_likely_flying()

   local now_ms = millis()

   -- Update gps failsafe check irrelevent of whether we are within the home bubble or not
   local in_gps_fs = check_gps(now_ms)

   -- Ensure we have a current GPS data packet, independant of the LFS_GPS_* times
   _in_home_bubble = false
   if (not in_gps_fs) and ((now_ms - _last_gps_time):tofloat() < 1000) and (ahrs:home_is_set())then
      -- check if we are within the home bubble before checking the rest of the failsafes
      -- We do not do this check if we are already in a failsafe
      local pos = ahrs:get_relative_position_NED_home()
      if pos and (pos:xy():length() < LFS_HOME_DIST:get()) and (-pos:z() < LFS_HOME_ALT:get()) then
         -- don't update failsafes within the home bubble
         _in_home_bubble = true
      end
   end

   local in_rpm_fs = check_engine(now_ms)
   local in_fence_fs = check_fence(now_ms)
   local in_gcs_fs = check_gcs(now_ms)

   -- Check if we can clear the failsafes
   if (not in_rpm_fs) and (not in_fence_fs) and (not in_gps_fs) and (not in_gcs_fs) then
      -- Tell the user the failsafe has cleared
      if (_in_failsafe_short) then
         gcs:send_text(2, "LFS: FS Cleared")
      end
      reset_fs()
   end

   -- pre-arm checks
   if arm_check_auth_id then
      -- we do not check rpm for a prearm as we may want to arm without the engine started
      if (not in_fence_fs) and (not in_gps_fs) and (not in_gcs_fs) then
         arming:set_aux_auth_passed(arm_check_auth_id)
      else
         arming:set_aux_auth_failed(arm_check_auth_id, "In LFS failsafe")
      end
   end

   -- FS Reason bitmask
   local reason = 0
   local reason_bm = 0
   if (in_fence_fs) then
      reason = REASON_GEOFENCE_FS
      reason_bm = reason_bm | 1 << REASON_GEOFENCE_FS
   end
   if (in_gps_fs) then
      reason = REASON_GPS_FS
      reason_bm = reason_bm | 1 << REASON_GPS_FS
   end
   if (in_gcs_fs) then
      reason = REASON_TELEM_FS
      reason_bm = reason_bm | 1 << REASON_TELEM_FS
   end
   if (in_rpm_fs) then
      -- RPM reason should always be last as this can result in an engine failure and we want short action to modify speed weight for safe gliding
      reason = REASON_RPM_FS
      reason_bm = reason_bm | 1 << REASON_RPM_FS
   end

   -- update logs
   -- R = failsafe Reason
   -- rpm = RPM from provided instance
   -- FR = Fence breach Reason
   -- FS = Time since fence breach (ms)
   -- GT = last Gps Time
   -- GS = Time since last GPS was 2D fix or better (ms)
   -- TT = last Telemetry Time
   -- TS = Time scince last gcs heartbeat (ms)
   -- HB = bool, true if in Home Bubble
   local HB = 0
   if _in_home_bubble then
      HB = 1
   end
   logger.write('LFS','R,rpm,FR,FS,GT,GS,TT,TS,HB','IfIfIfIfI', uint32_t(reason_bm), _rpm, uint32_t(_breach_bm), _time_since_breach, _last_gps_time, _time_since_last_gps, _last_gcs_time, _time_since_last_gcs, uint32_t(HB))

   -- Check if the user has changed to manual via the transmitter. We will use this as a sign to block shute_deployment
   if (_in_failsafe_short) then
      _in_manual_on_rc = (vehicle:get_mode() == MODE_MANUAL) and (vehicle:get_control_mode_reason() == REASON_RC)
   end

   if _in_failsafe_long then
      do_fs_long_action(now_ms, reason)
   end

   if _in_failsafe_short then
      do_fs_short_action(now_ms, reason)
   end
end


function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "FS Script Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  -- Run normally at 5 Hz
  return protected_wrapper, 200
end

-- delay start of running 
return protected_wrapper, 5000
