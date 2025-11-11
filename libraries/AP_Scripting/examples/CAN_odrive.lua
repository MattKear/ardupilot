local CMD_ID_MASK = 0x1F
local NODE_ID_MASK = 0x7E0
local NODE_ID_SHIFT = 5

local STATE_UNKOWN = 0X00
local STATE_IDLE = 0X01
local STATE_CLOSEDLOOP = 0X08
local STATE_HOMING = 0x0B

local CMD_HEARTBEAT = 0x1
local CMD_SET_AXIS_STATE = 0x7
local CMD_SET_INPUT_POS = 0x0C
local CMD_CLEAR_ERRORS = 0x18
local CMD_RXSDO = 0x04
local CMD_TXSDO = 0x05
local CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
local CMD_GET_TEMPERATURE = 0x15

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
local odrive_configured = false
local configured = false
local last_heartbeat_ms = millis()
local HEARTBEAT_TIMEOUT = uint32_t(5000)

local OPCODE_READ = 0x00
local OPCODE_WRITE = 0x01


-- format_lookup = {
--     'bool': '?',
--     'uint8': 'B', 'int8': 'b',
--     'uint16': 'H', 'int16': 'h',
--     'uint32': 'I', 'int32': 'i',
--     'uint64': 'Q', 'int64': 'q',
--     'float': 'f'
-- }

-- ODrive settings as found from: https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/firmware/P5x-2epyHO8DXkyYEYQCpBsdw9skZ1GP04WKg4RVIjo/flat_endpoints.json
local axis0 = {}

axis0.is_homed = {
    id = 227,
    type = "B" -- is actually a bool but sending a byte
}

axis0.config = {}
axis0.config.can = {}
axis0.config.can.encoder_msg_rate_ms = {
    id = 275,
    type = "I" -- uint32
}
axis0.config.can.iq_msg_rate_ms = {
    id = 276,
    type = "I" -- uint32
}
axis0.config.can.error_msg_rate_ms = {
    id = 277,
    type = "I" -- uint32
}
axis0.config.can.temperature_msg_rate_ms = {
    id = 278,
    type = "I" -- uint32
}
axis0.config.can.bus_voltage_msg_rate_ms = {
    id = 279,
    type = "I", -- uint32
}
axis0.config.can.torques_msg_rate_ms = {
    id = 280,
    type = "I", -- uint32
}
axis0.config.can.powers_msg_rate_ms = {
    id = 281,
    type = "I", -- uint32
}
axis0.config.can.input_vel_scale = {
    id = 282,
    type = "I", -- uint32
}
axis0.config.can.input_torque_scale = {
    id = 283,
    type = "I", -- uint32
}

axis0.controller = {}
axis0.controller.config = {}
axis0.controller.config.homing_speed = {
    id = 395,
    type = "f"
}
axis0.controller.config.vel_ramp_rate = {
    id = 386,
    type = "f"
}



-- Load CAN driver. The first will attach to a protocol of 10
local CAN_BUFFER_SIZE = 20
local driver = CAN:get_device(CAN_BUFFER_SIZE)

if not driver then
   gcs:send_text(0,"No scripting CAN interfaces found")
   return
end

-- Helper to pack 11-bit ID format used by ODrive
function get_id(cmd)
   return (target_node_id << NODE_ID_SHIFT) | cmd
end

-- Helper to parse data from can frames
function unpack_data(frame, start_bit, end_bit, format_str)
   local packed_str = ""
   for i = start_bit, end_bit do
      packed_str = packed_str .. string.char(frame:data(i))
   end
   return string.unpack("<" .. format_str, packed_str) -- Always little-endian for ODrive
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
      elseif (cmd_id == CMD_TXSDO) then
         read_TxSdo(frame)
      elseif (cmd_id == CMD_GET_BUS_VOLTAGE_CURRENT) then
         update_volt_curr_telem(frame)
      elseif (cmd_id == CMD_GET_TEMPERATURE) then
         update_temp_telem(frame)
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

-- parse data from CMD_GET_BUS_VOLTAGE_CURRENT and stuff in ESC telem
local esc_telem_data = ESCTelemetryData()
function update_volt_curr_telem(frame)
   local bus_voltage = unpack_data(frame, 0, 3, "f") -- float
   local bus_current = unpack_data(frame, 4, 7, "f") -- float

   -- update esc telem data
   esc_telem_data:voltage(bus_voltage)
   esc_telem_data:current(bus_current)
   -- 0x0C is mask for voltage and current data
   esc_telem:update_telem_data(0, esc_telem_data, 0x0C)
end

-- parse data from CMD_GET_TEMPERATURE and stuff in esc telem
function update_temp_telem(frame)
   local fet_temp = unpack_data(frame, 0, 3, "f") -- float
   local motor_temp = unpack_data(frame, 4, 7, "f") -- float

   -- convert to cdeg
   fet_temp = math.floor(fet_temp * 100)
   motor_temp = math.floor(motor_temp * 100)

   -- update esc telem data
   esc_telem_data:temperature_cdeg(fet_temp)
   esc_telem_data:motor_temp_cdeg(motor_temp)
   -- 0x03 is mask for temperature and motor temperature
   esc_telem:update_telem_data(0, esc_telem_data, 0x03)
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

-- setup a fixed command for starting homing
local set_state_homing = CANFrame()
set_state_homing:id(get_id(CMD_SET_AXIS_STATE))
set_state_homing:data(0, STATE_HOMING)
set_state_homing:dlc(4) -- requested state is a uint32_t


-- send position input commands to odrive
function send_position_command(pos)
   -- For future reference, we will need to set the reference frame using this:
   -- https://docs.odriverobotics.com/v/latest/manual/control.html#homed-reference-frame

   msg = CANFrame()

   msg:id(get_id(CMD_SET_INPUT_POS))

   local vel_ff = 0
   local torque_ff = 0

   -- pack payload
   local payload = string.pack("<fhh", pos, vel_ff, torque_ff)
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

-- Read/Write an endpoint value
function send_RxSdo(opcode, endpoint, value)
   msg = CANFrame()

   msg:id(get_id(CMD_RXSDO))

   -- pack payload
   local format = "<BHB" .. endpoint.type
   local payload = string.pack(format, opcode, endpoint.id, 0, value)
   for i = 1, #payload do
      msg:data(i - 1, string.byte(payload, i))
   end

   msg:dlc(#payload)

   -- timeout of 1000us
   driver:write_frame(msg, 1000)
end

-- Function to recursively search for an endpoint by ID
local function find_endpoint(tbl, id)
    for k, v in pairs(tbl) do
        if type(v) == "table" then
            if v.id == id then
                return v, k  -- return both the endpoint table and its name
            else
                local found, name = find_endpoint(v, id)
                if found then return found, name end
            end
        end
    end
    return nil
end

-- Read the endpoint data sent by the odrive after we sent the RxSdo command
function read_TxSdo(frame)

    -- Extract endpoint ID (little endian)
    local endpt_id = frame:data(1) | (frame:data(2) << 8)

    -- Find endpoint metadata
    local endpoint, name = find_endpoint(axis0, endpt_id)
    if not endpoint then
        gcs:send_text(0, string.format("Unknown endpoint ID: %d", endpt_id))
        return
    end

    if (frame:dlc() <= 4) then
      -- No payload to read
      return nil
    end

    -- Read payload data bytes starting from byte 4, to number of bytes - 1
    local value = unpack_data(frame, 4, frame:dlc() - 1, endpoint.type)

   -- generic print if we haven't handled it
   gcs:send_text(0, string.format(
      "Endpoint %s (ID %d): %s = %d",
      name, endpt_id, endpoint.type, value
   ))
end


-- Send all required settings to odrive when we first start talking to it
-- returns true when all setup has complete
function run_setup()

   -- set message rates for cyclic telem
   send_RxSdo(OPCODE_WRITE, axis0.config.can.bus_voltage_msg_rate_ms, 500)
   send_RxSdo(OPCODE_WRITE, axis0.config.can.temperature_msg_rate_ms, 500)

   -- set kinematic limits
   send_RxSdo(OPCODE_WRITE, axis0.controller.config.homing_speed, -10.0) -- rev/s
   send_RxSdo(OPCODE_WRITE, axis0.controller.config.vel_ramp_rate, 10.0) -- rev/s/s

   return true

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

   if not odrive_configured then
      odrive_configured = run_setup()
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

   -- When armed, output position commands
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
