-- Script to add failsafe functionaility in the event of the following:
   -- Loss of GPS
   -- Loss of GCS
   -- Geofence Breach
   -- Engine Failure

-- For all failsafes:
   -- first action (on short timer) is to change to RTL and slow speed to safe deployable chute speed
   -- second action (on long timer) is to just deplot chute

-- Use ICE param for the minimum RPM to be conisdered running
local ICE_RPM_THRESH = Parameter()
if not ICE_RPM_THRESH:init('ICE_RPM_THRESH') then
   gcs:send_text(6, 'get ICE_RPM_THRESH failed')
end

-- Need to know what instance of RPM the ICE library is using
local ICE_RPM_CHAN = Parameter()
if not ICE_RPM_CHAN:init('ICE_RPM_CHAN') then
   gcs:send_text(6, 'get ICE_RPM_CHAN failed')
end

local TECS_SPDWEIGHT = Parameter()
if not TECS_SPDWEIGHT:init('TECS_SPDWEIGHT') then
   gcs:send_text(6, 'get TECS_SPDWEIGHT failed')
end
local _init_spdweight = TECS_SPDWEIGHT:get()

-- Create param table for "Lua FailSafes"
local PARAM_TABLE_KEY = 75 -- Does not clash with ready_take_off script table (key = 74)
assert(param:add_table(PARAM_TABLE_KEY, "LFS_", 3), 'could not add param table')

-- Add params to table
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RPM_SHORT', 5), 'could not add LFS_RPM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'RPM_LONG', 30), 'could not add LFS_RPM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'CHUTE_SPD', 20), 'could not add LFS_CHUTE_SPD')

local LFS_RPM_SHORT = Parameter("LFS_RPM_SHORT")
local LFS_RPM_LONG = Parameter("LFS_RPM_LONG")
local LFS_CHUTE_SPD = Parameter("LFS_CHUTE_SPD")

local MODE_RTL = 11

local _in_failsafe_short = false
local _in_failsafe_long = false
local _have_set_mode = false

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
      gcs:send_text(2, " LFS: " .. _fs_reason_str[reason] .. ", Short")
      _last_lua_fs_msg_sent = now_ms
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
   -- release the chute
   parachute:release()

   if ((now_ms - _last_lua_fs_msg_sent):tofloat() > 5000.0) or (_last_lua_fs_msg_sent == 0) then
      gcs:send_text(2, " LFS: " .. _fs_reason_str[reason] .. ", Long")
      _last_lua_fs_msg_sent = now_ms
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
end


local _rpm_failed_ms = 0
function check_engine(now_ms)

   if (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
      return false
   end

   _rpm = RPM:get_rpm(math.max(ICE_RPM_CHAN:get() - 1, 0))

   if (_rpm < ICE_RPM_THRESH:get()) and (_rpm_failed_ms == 0) then
      _rpm_failed_ms = now_ms
   elseif (_rpm >= ICE_RPM_THRESH:get()) then
      _rpm_failed_ms = 0
      return false
   end

   -- Long failsafe timer
   if ((now_ms - _rpm_failed_ms):tofloat() >= LFS_RPM_LONG:get()*1000) and (_rpm_failed_ms > 0) then
      _in_failsafe_long = true
   end

   -- Short failsafe timer
   if ((now_ms - _rpm_failed_ms):tofloat() >= LFS_RPM_SHORT:get()*1000) and (_rpm_failed_ms > 0) then
      _in_failsafe_short = true
   end

   return true

end




-- example main loop function
function update()

   -- TODO: add in check distance from home

   local now_ms = millis()

   local in_rpm_fs = check_engine(now_ms)

   -- Check if we can clear the failsafes
   if (not in_rpm_fs) then
      -- TODO: Do we want to cancel the failesafes here.  For example, a dodgey RPM sensor cutting in and out will may keep cancelling the long failsafe mean while the aircraft is loosing height
      reset_fs()
   end

   -- FS Reason bitmask
   local reason = 0
   local reason_bm = 0
   if (in_rpm_fs) then
      -- RPM reason should always be last as this can result in an engine failure and we want short action to modify speed weight for safe gliding
      reason = REASON_RPM_FS
      reason_bm = reason_bm | 1 << REASON_RPM_FS
   end

   -- update logs
   logger.write('LFS','R,rpm','If', uint32_t(reason_bm), _rpm)

   -- update telem
   gcs:send_named_float("FSRP", _rpm)


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













