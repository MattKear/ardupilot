local CMD_ID_MASK = 0x1F
local NODE_ID_MASK = 0x7E0
local NODE_ID_SHIFT = 5

local STATE_UNKOWN = 0X00
local STATE_IDLE = 0X01
local STATE_CLOSEDLOOP = 0X08
local STATE_HOMING = 0x0B

local CMD_HEARTBEAT = 0x1
local CMD_RXSDO = 0x04
local CMD_TXSDO = 0x05
local CMD_SET_AXIS_STATE = 0x7
local CMD_GET_ENCODER_ESTIMATES = 0x9
local CMD_SET_INPUT_POS = 0x0C
local CMD_GET_TEMPERATURE = 0x15
local CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
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

local had_error = false
local odrive_configured = false
local hit_endstop = {
   max = false,
   min = false
}

local last_heartbeat_ms = millis()
local HEARTBEAT_TIMEOUT = uint32_t(5000)
local position_est = 0

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
    type = "f" -- float
}
axis0.controller.config.vel_ramp_rate = {
    id = 386,
    type = "f" -- float
}

axis0.min_endstop = {}
axis0.min_endstop.state = {
   id = 415,
   type = "B" -- is actually a bool but sending a byte
}

axis0.max_endstop = {}
axis0.max_endstop.state = {
   id = 421,
   type = "B" -- is actually a bool but sending a byte
}

-- Make a type table from all of the info above that is a lookup of type from id once at boot
function build_lookup_table(tbl, lookup)
    -- if no lookup table passed, create a new one
    lookup = lookup or {}

    for k, v in pairs(tbl) do
        if type(v) == "table" then
            if v.id and v.type then
                -- add entry
                lookup[v.id] = v.type
            else
                -- recurse into nested tables
                build_lookup_table(v, lookup)
            end
        end
    end

    return lookup
end
local endpoint_types = build_lookup_table(axis0)


local PARAM_TABLE_KEY = 2
local PARAM_TABLE_PREFIX = "OD_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 50), 'could not add param table')

local POS_MAX = bind_add_param('POS_MAX', 1, 10) -- Max endpoint position, turns from centre
local POS_MIN = bind_add_param('POS_MIN', 2, -10) -- Min endpoint position, turns from centre
local POT_MAX_VOLT = bind_add_param('POT_MAX_VOLT', 3, 3.0) -- Potentiometer voltage reading corresponding to max position endpoint position
local POT_MIN_VOLT = bind_add_param('POT_MIN_VOLT', 4, 0.3) -- Potentiometer voltage reading corresponding to min position endpoint position


-- Load CAN driver. The first will attach to a protocol of 10
local driver = assert(CAN:get_device(20), "No scripting CAN interfaces found")

-- Helper to pack 11-bit ID format used by ODrive
function get_id(cmd)
   return (target_node_id << NODE_ID_SHIFT) | cmd
end

-- Only accept data from the target node id
driver:add_filter(uint32_t(0x3F) << NODE_ID_SHIFT, uint32_t(target_node_id) << NODE_ID_SHIFT)

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

   for _ = 1, 40 do
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
      elseif (cmd_id == CMD_GET_ENCODER_ESTIMATES) then
         update_position_est(frame)
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

   -- We have a valid heartbeat, update timer and state
   last_heartbeat_ms = millis()
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

-- update the reported position from the odrive
function update_position_est(frame)
   position_est = unpack_data(frame, 0, 3, "f") -- float
   -- Note: We also get vel estimate from this message but we just throw it away
end

-- Set control mode on odrive. This is needed before we can drive the motor.
local state_msg = CANFrame()
state_msg:id(get_id(CMD_SET_AXIS_STATE))
state_msg:dlc(4)
function set_odrive_state(arm)

   local state = STATE_IDLE
   if arm then 
      state = STATE_CLOSEDLOOP
   end

   -- requested state is a uint32_t
   state_msg:data(0, state)

   local timeout = 500
   driver:write_frame(msg, timeout)
end

-- setup a fixed command for starting homing
local set_state_homing = CANFrame()
set_state_homing:id(get_id(CMD_SET_AXIS_STATE))
set_state_homing:data(0, STATE_HOMING)
set_state_homing:dlc(4) -- requested state is a uint32_t


-- send position input commands to odrive
function send_position_command(input_pos)
   -- For future reference, we will need to set the reference frame using this:
   -- https://docs.odriverobotics.com/v/latest/manual/control.html#homed-reference-frame

   -- calculate the desired position from an input (-1 to 1)
   -- linear interpolation between min and max position
   local scaled_input = (input_pos + 1.0) * 0.5
   local des_pos = (POS_MAX:get() - POS_MIN:get()) * scaled_input + POS_MIN:get()

   -- Do not allow position to push past end stops
   if hit_endstop.min and (des_pos < position_est) then
      des_pos = position_est
   end
   if hit_endstop.max and (des_pos > position_est) then
      des_pos = position_est
   end

   des_pos = constrain(des_pos, POS_MIN:get(), POS_MAX:get())

   -- send position command to odrive
   local msg = CANFrame()
   msg:id(get_id(CMD_SET_INPUT_POS))

   -- pack payload
   local vel_ff = 0
   local torque_ff = 0
   local payload = string.pack("<fhh", des_pos, vel_ff, torque_ff)
   for i = 1, #payload do
      msg:data(i - 1, string.byte(payload, i))
   end
   msg:dlc(#payload)

   -- timeout of 500us
   driver:write_frame(msg, 500)

   -- report on telem
   gcs:send_named_float("DPos", des_pos) -- desired position
   gcs:send_named_float("MPos", position_est) -- measured position
end

-- send position input commands to odrive
local clear_err_msg = CANFrame()
clear_err_msg:id(get_id(CMD_CLEAR_ERRORS))
clear_err_msg:data(0, 0) -- pack payload - identify led blink = true
clear_err_msg:dlc(1)
function send_clear_error()
   -- timeout of 1000us
   driver:write_frame(clear_err_msg, 1000)
end

-- Read/Write an endpoint value
function send_RxSdo(opcode, endpoint, value)
   local msg = CANFrame()

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

-- Read the endpoint data sent by the odrive after we sent the RxSdo command
function read_TxSdo(frame)

    -- Extract endpoint ID
    local endpt_id = frame:data(1) | (frame:data(2) << 8)

    local endpoint_type = endpoint_types[endpt_id]
    if not endpoint then
        -- we have somehow managed to receive an id we didn't ask for and don't know about
        return
    end

    if (frame:dlc() <= 4) then
      -- No payload to read
      return
    end

    -- Read payload data bytes starting from byte 4, to number of bytes - 1
    local value = unpack_data(frame, 4, frame:dlc() - 1, endpoint_type)

    if endpt_id == axis0.min_endstop.state.id then
      -- update min endstop state
      hit_endstop.min = value > 0

    elseif endpt_id == axis0.max_endstop.state.id then
      -- update max endstop state
      hit_endstop.max = value > 0

    else
      -- generic print if we haven't handled it
      gcs:send_text(0, string.format(
         "Endpoint %s (ID %d): %s = %d",
         name, endpt_id, endpoint_type, value
      ))
   end
end

local function constrain(v, vmin, vmax)
   return math.max(math.min(v, vmax), vmin)
end

-- Send all required settings to odrive when we first start talking to it
-- returns true when all setup has complete
function run_setup()

   -- set message rates for cyclic telem
   send_RxSdo(OPCODE_WRITE, axis0.config.can.bus_voltage_msg_rate_ms, 500)
   send_RxSdo(OPCODE_WRITE, axis0.config.can.temperature_msg_rate_ms, 500)
   send_RxSdo(OPCODE_WRITE, axis0.config.can.encoder_msg_rate_ms, 250)

   -- set kinematic limits
   send_RxSdo(OPCODE_WRITE, axis0.controller.config.vel_ramp_rate, 10.0) -- rev/s/s

   -- For improved safety, it is also recommended to set <axis>.controller.config.absolute_setpoints to True.
   -- This makes the ODrive reject position control commands after startup until <axis>.pos_estimate has been set.

   return true

end


-- What is needed for absolute position control: https://docs.odriverobotics.com/v/0.6.11/manual/control.html#absolute-encoder-reference-frame


-- rate limited function for regularly requesting endpoint data
local last_checked_endpoints_ms = millis()
function request_endpoints(now)
   -- Update at 4 Hz
   if (now - last_checked_endpoints_ms) < 250 then
      return
   end

   -- request endstop states
   send_RxSdo(OPCODE_READ, axis0.min_endstop.state, 0)
   send_RxSdo(OPCODE_READ, axis0.max_endstop.state, 0)

   last_checked_endpoints_ms = now
end

local position_des = 0.0
local position_inc = 0.005
local pos_max = 1.0
function update()

   local now = millis()

   -- request endpoint data that we will regularly want updating
   request_endpoints(now)

   -- read data sent from the ODrive
   read_data()

   -- update timeout on heartbeat state
   if now - last_heartbeat_ms > HEARTBEAT_TIMEOUT then
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

      -- send command to actuator
      send_position_command(position_des)
   end

   return update, 10

end

return update()
