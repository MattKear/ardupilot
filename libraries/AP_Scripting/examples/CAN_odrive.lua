local CMD_ID_MASK = 0x1F
local NODE_ID_MASK = 0x7E0
local NODE_ID_SHIFT = 5

local STATE_UNKOWN = 0
local STATE_IDLE = 1
local STATE_CLOSEDLOOP = 8

local CMD_HEARTBEAT = 0x1
local CMD_SET_AXIS_STATE = 0x7
local CMD_SET_INPUT_POS = 0x0C
local CMD_CLEAR_ERRORS = 0x18

local LOCAL_STATE_DISAMED = 0
local LOCAL_STATE_ARMED = 1
local LOCAL_STATE_ERROR = 10
local local_state = LOCAL_STATE_DISAMED

local odrive_status = {
   axis_errors = 0,
   axis_state = 0,
   procedure_result = 0,
   trajectory_done_flag = 0
}

local target_node_id = 10

local have_heartbeat = false
local had_error = false
local last_heartbeat_ms = millis()
local HEARTBEAT_TIMEOUT = uint32_t(5000)

local CAN_BUFFER_SIZE = 20

-- Load CAN driver. The first will attach to a protocol of 10
local driver = CAN:get_device(CAN_BUFFER_SIZE)

if not driver then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

-- Helper to pack 11-bit ID format used by ODrive
function get_id(cmd)
   return (target_node_id << NODE_ID_SHIFT) | cmd
end

-- Read data from can buffer
function read_data()

   if not driver then
      gcs:send_text(0, "No Driver")
      return
   end

   for _ = 1, CAN_BUFFER_SIZE do
      local frame = driver:read_frame()

      if not frame then
         return
      end

      local cmd_id = (frame:id() & CMD_ID_MASK):toint()
      local node_id = ((frame:id() & NODE_ID_MASK) >> NODE_ID_SHIFT):toint()

      if (node_id ~= target_node_id) then
         gcs:send_text(0, "Got ID = " .. tostring(node_id) .. " only want " .. tostring(target_node_id))
         goto next_frame
      end

      if (cmd_id == CMD_HEARTBEAT) then
         update_heartbeat(frame)

      -- elseif (cmd_id == CMD_SET_INPUT_POS) then
      --    gcs:send_text(0,string.format("cmd: " ..  tostring(cmd_id) .." from node " .. tostring(node_id) .. ": %i, %i, %i, %i, %i, %i, %i, %i", frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
      else
         gcs:send_text(0,string.format("cmd: " ..  tostring(cmd_id) .." from node " .. tostring(node_id) .. ": %i, %i, %i, %i, %i, %i, %i, %i", frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
      end

      ::next_frame::
   end

end

-- Parse heartbeat from ODrive and store state
function update_heartbeat(frame)

   odrive_status.axis_errors = uint32_t(frame:data(0) | (frame:data(1) << 8) | (frame:data(2) << 16) | (frame:data(3) << 24))
   odrive_status.axis_state = frame:data(4)
   odrive_status.procedure_result = frame:data(5)
   odrive_status.trajectory_done_flag = frame:data(6)

   --gcs:send_named_float("AErr", odrive_status.axis_errors:toint())

   -- We have a valid heartbeat, update timer and state
   last_heartbeat_ms = millis()
   have_heartbeat = true
   if (odrive_status.axis_errors) and (not had_error) then
      had_error = odrive_status.axis_errors > 0
   end
end

-- Set control mode on odrive. This is needed before we can drive the motor.
function set_odrive_state(arm)
   msg = CANFrame()

   msg:id(get_id(CMD_SET_AXIS_STATE))

   local state = STATE_IDLE
   if arm then 
      state = STATE_CLOSEDLOOP
   end

   -- requested state is a uint32_t
    msg:data(0, state)
    msg:data(1, 0)
    msg:data(2, 0)
    msg:data(3, 0)

   -- sending 4 bytes of data
   msg:dlc(4)

   local timeout = 500
   driver:write_frame(msg, timeout)
end

-- send position input commands to odrive
function send_position_command(pos)
   -- For future reference, we will need to set the reference frame using this:
   -- https://docs.odriverobotics.com/v/latest/manual/control.html#homed-reference-frame

   msg = CANFrame()

   msg:id(get_id(CMD_SET_INPUT_POS))

   local vel_ff = 0
   local torque_ff = 0

   -- pack payload
   payload = string.pack("<fhh", pos, vel_ff, torque_ff)
   for i = 1, #payload do
      msg:data(i - 1, string.byte(payload, i))
   end

   msg:dlc(#payload)

   -- timeout of 1000us
   driver:write_frame(msg, 1000)
end

-- send position input commands to odrive
function send_clear_error()
   -- For future reference, we will need to set the reference frame using this:
   -- https://docs.odriverobotics.com/v/latest/manual/control.html#homed-reference-frame

   msg = CANFrame()

   msg:id(get_id(CMD_CLEAR_ERRORS))

   -- pack payload - identify led blink = true
   msg:data(0, 0)

   msg:dlc(1)

   -- timeout of 1000us
   driver:write_frame(msg, 1000)
end


local position_des = 0.0
local position_inc = 0.01
local pos_max = 20.0

function update()

   -- read data sent from the ODrive
   read_data()

   -- update timeout on heartbeat state
   if millis() - last_heartbeat_ms > HEARTBEAT_TIMEOUT then
      have_heartbeat = false
   end

   -- update_logging()

   if not have_heartbeat then
      -- we are not speaking to the odrive, no point in continuing
      return update, 10
   end

   -- See if we should arm the odrive
   if (not SRV_Channels:get_safety_state()) and (odrive_status.axis_state == STATE_IDLE) and (not had_error) and (local_state == LOCAL_STATE_DISAMED) then
      gcs:send_text(2, "Arming ODRIVE")
      set_odrive_state(true)
      local_state = LOCAL_STATE_ARMED
   end

   -- see if we should send a disarm command to the odrive
   if SRV_Channels:get_safety_state() and ((odrive_status.axis_state ~= STATE_IDLE) or local_state == LOCAL_STATE_ERROR or (odrive_status.axis_state == STATE_UNKOWN)) then
      gcs:send_text(2, "Disarming ODRIVE")

      -- Send command to odrive
      set_odrive_state(false)

      -- Reset system, clearing errors, if we have safety on
      if (local_state == LOCAL_STATE_ERROR) then
         send_clear_error()
         had_error = false
      end

      local_state = LOCAL_STATE_DISAMED
   end

   if had_error and (local_state < LOCAL_STATE_ERROR) then
      set_odrive_state(false)
      local_state = LOCAL_STATE_ERROR
      gcs:send_text(2, "In Error State")
   end

   gcs:send_named_float("Stat", local_state)
   if had_error then
      gcs:send_named_float("Err", 1)
   else
      gcs:send_named_float("Err", 0)
   end

   if arming:is_armed() and (odrive_status.axis_state == STATE_CLOSEDLOOP) then
      -- move the motor
      send_position_command(position_des)

      -- update position for next call
      position_des = position_des + position_inc

      if position_des > pos_max then
         position_inc = -math.abs(position_inc)
         position_des = pos_max
      end
      if position_des < -pos_max then
         position_inc = math.abs(position_inc)
         position_des = -pos_max
      end

   end

   gcs:send_named_float("DPos", position_des)

   return update, 10

end

return update()
