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

-- Create param table for "Lua FailSafes"
local PARAM_TABLE_KEY = 75 -- Does not clash with ready_take_off script table (key = 74)
assert(param:add_table(PARAM_TABLE_KEY, "LFS_", 2), 'could not add param table')

-- Add params to table
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RPM_SHORT', 5), 'could not add LFS_RPM_SHORT')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'RPM_LONG', 30), 'could not add LFS_RPM_SHORT')

local LFS_RPM_SHORT = Parameter("LFS_RPM_SHORT")
local LFS_RPM_LONG = Parameter("LFS_RPM_LONG")

local MODE_RTL = 11

local _in_failsafe_short = false
local _in_failsafe_long = false
local _have_set_mode = false

local _last_lua_fs_msg_sent = 0

function do_fs_short_action(now_ms)
   -- protection to prevent mode changes if we have already popped the chute
   if _in_failsafe_long then
      return
   end

   -- Tell user that we are in a failsafe
   if ((now_ms - _last_lua_fs_msg_sent):to_float() < 5000.0) or (_last_lua_fs_msg_sent == 0) then
      gcs:send_text(2, "In Scr Short FS")
      _last_lua_fs_msg_sent = now_ms
   end

   -- Change to RTL without spamming to allow user to take back control
   if (vehicle:get_mode() ~= MODE_RTL) and (not _have_set_mode) then
      vehicle:set_mode(MODE_RTL)
      _have_set_mode = true
   end
end

function do_fs_long_action(now_ms)
   -- release the chute
   parachute:release()

   if ((now_ms - _last_lua_fs_msg_sent):to_float() < 5000.0) or (_last_lua_fs_msg_sent == 0) then
      gcs:send_text(2, "Chute Release from Scr")
      _last_lua_fs_msg_sent = now_ms
   end

end

function reset_fs()
   _last_lua_fs_msg_sent = 0
   _in_failsafe_short = false
   _in_failsafe_long = false
   _have_set_mode = false
end


local _last_rpm_fail = 0
function check_engine(now_ms)

   if (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
      return false
   end

   gcs:send_text(2, "DB " .. tostring(math.max(ICE_RPM_CHAN:get() - 1, 0)))
   local rpm = RPM:get_rpm(math.max(ICE_RPM_CHAN:get() - 1, 0))

   if (rpm > 0) and (rpm < ICE_RPM_THRESH:get()) then
      _last_rpm_fail = now_ms
   else
      _last_rpm_fail = 0
      return false
   end

   -- Long failsafe timer
   if ((now_ms - _last_rpm_fail):to_float() >= LFS_RPM_LONG:get()*1000) and (_last_rpm_fail > 0) then
      _in_failsafe_long = true
   end

   -- Short failsafe timer
   if ((now_ms - _last_rpm_fail):to_float() >= LFS_RPM_SHORT:get()*1000) and (_last_rpm_fail > 0) then
      _in_failsafe_short = true
   end

   return true

end




-- example main loop function
function update()

   -- TODO add in check distance from home

   local now_ms = millis()

   local in_rpm_fs = check_engine(now_ms)


   -- Check if we can clear the failsafes
   if (not in_rpm_fs) then
      reset_fs()
   end


   if _in_failsafe_long then
      do_fs_long_action(now_ms)
      return
   end

   if _in_failsafe_short then
      do_fs_short_action()
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













