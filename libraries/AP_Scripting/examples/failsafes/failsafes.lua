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

-- need the sysid of this vehicle so we can send MAVLink commands to it
local SYSID_THISMAV = Parameter()
if not SYSID_THISMAV:init('SYSID_THISMAV') then
   gcs:send_text(6, 'get SYSID_THISMAV failed')
end

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

-- Mavlink setup for sending DO_CHANGE_SPEED commands
local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink.init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_ACK_ID)


local _confirm_counter = 0
local _set_speed_complete = false
-- Send DO_CHANGE_SPEED mavlink command to bring the aircraft to the safe chute release speed
function do_set_speed()
   local MAV_CMD_DO_CHANGE_SPEED = 178 -- msg enum

   local msg, chan = mavlink:receive_chan()

   -- See if we got an ack
   if (_confirm_counter > 0) and (msg ~= nil) then
      local parsed_msg = mavlink_msgs.decode(msg, msg_map)
      if (parsed_msg ~= nil) then
         gcs:send_text(2, "DB: " .. tostring(parsed_msg.command))
         if (parsed_msg.command == COMMAND_ACK_ID) then
            gcs:send_text(2, "DB: Got cmd ACK")
            _set_speed_complete = true
         elseif (parsed_msg.command == MAV_CMD_DO_CHANGE_SPEED) then
            gcs:send_text(2, "DB: Got change speed ACK")
            _set_speed_complete = true
         end
      end
   end

   -- We do not need to send another change speed command if complete
   if (_set_speed_complete) then
      return
   end

   -- Build do change speed command and send
   local cmd = {}
   cmd.param1 = 0 -- speed type is airspeed
   cmd.param2 = LFS_CHUTE_SPD:get() -- (m/s)
   cmd.param3 = -2 -- default value
   cmd.param4 = 0 -- not used
   cmd.param5 = 0 -- not used
   cmd.param6 = 0 -- not used
   cmd.param7 = 0 -- not used
   cmd.command = MAV_CMD_DO_CHANGE_SPEED
   cmd.target_system = SYSID_THISMAV:get()
   cmd.target_component = 0
   cmd.confirmation = 0
   mavlink:send_chan(0, mavlink_msgs.encode("COMMAND_LONG", cmd))

   _confirm_counter = _confirm_counter + 1

   if _confirm_counter >= 255 then
      gcs:send_text(3, "DO_CHANGE_SPEED cmd failed to send")
      _confirm_counter = 0
   end
end


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
      do_set_speed()
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













