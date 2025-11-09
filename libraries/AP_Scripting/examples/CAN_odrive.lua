local CMD_ID_MASK = 0x1F
local NODE_ID_MASK = 0x7E0
local NODE_ID_SHIFT = 5

local STATE_UNKOWN = 0
local STATE_IDLE = 1
local STATE_CLOSEDLOOP = 8

local CMD_HEARTBEAT = 1
local CMD_SET_AXIS_STATE = 7

local odrive_status = {
   axis_errors = 0,
   axis_state = 0,
   procedure_result = 0,
   trajectory_done_flag = 0
}

local target_node_id = 10

-- Load CAN driver. The first will attach to a protocol of 10
local driver = CAN:get_device(20)

if not driver then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

function get_id(cmd)
   return (target_node_id << NODE_ID_SHIFT) | cmd
end

function read_heartbeat()
   if not driver then
      return
   end

   frame = driver:read_frame()

   if not frame then
      return
   end

   local cmd_id = frame:id() & CMD_ID_MASK
   local node_id = (frame:id() & NODE_ID_MASK) >> NODE_ID_SHIFT
   odrive_status.axis_errors = frame:data(0) << 24 + frame:data(1) << 16 + frame:data(2) << 8 + frame:data(3)
   odrive_status.axis_state = frame:data(4)
   odrive_status.procedure_result = frame:data(5)
   odrive_status.trajectory_done_flag = frame:data(6)
   -- gcs:send_text(0,string.format("cmd: " ..  tostring(cmd_id) .." from node " .. tostring(node_id) .. ": %i, %i, %i, %i, %i, %i, %i, %i", frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
   gcs:send_text(4,string.format("Node: " .. tostring(node_id) .. ": Err: %i, State: %i, Res: %i, Done: %i,", odrive_status.axis_errors, odrive_status.axis_state, odrive_status.procedure_result, odrive_status.trajectory_done_flag))
end

function set_odrive_state(arm)
   msg = CANFrame()

   local target_id = get_id(CMD_SET_AXIS_STATE)
   gcs:send_text(4,"id = " .. tostring(target_id))
   msg:id(get_id(CMD_SET_AXIS_STATE))

   local state = STATE_IDLE
   if arm then 
      state = STATE_CLOSEDLOOP
   end

   -- requested state is a uint32_t. I only want to send either 1 or 8 so i am being lazy and only packing the 1st bit
    msg:data(0, state)
   --  msg:data(1, 0)
   --  msg:data(2, 0)
   --  msg:data(3, 0)

   -- sending 4 bytes of data
   msg:dlc(4)

   -- timeout of 1000us
   driver:write_frame(msg, 1000)
end


function update()

   read_heartbeat()

   -- Tie odrive state to safety state of vehicle
   if not SRV_Channels:get_safety_state() and odrive_status.axis_state == STATE_IDLE then
      gcs:send_text(2, "Arming ODRIVE")
      set_odrive_state(true)
   elseif (SRV_Channels:get_safety_state() and odrive_status.axis_state ~= STATE_IDLE) or odrive_status.axis_state == STATE_UNKOWN then
      gcs:send_text(2, "Disarming ODRIVE")
      set_odrive_state(false)
   end

   return update, 10

end

return update()
