-- Script for sending a go/no-go for take-off signal to GCS
-- Script checks motor rpm and throttle output against user defined param values and sends
-- a named value float via mavlink, to be interpreted by the GCS as go/no-go

local READY_FOR_TAKEOFF = 1
local NOT_READY = 0
local PARAM_TABLE_KEY = 74
local MAX_RPM_INSTANCE = 1
local K_THROTTLE = 70

-- Create param table
assert(param:add_table(PARAM_TABLE_KEY, "TOR_", 3), 'could not add param table')

-- Add params to table
assert(param:add_param(PARAM_TABLE_KEY, 1, 'RPM_INST', 1), 'could not add TOR_RPM_INST')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'RPM_PASS', 2000), 'could not add TOR_RPM_PASS')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'TH_PASS', 80), 'could not add TOR_TH_PASS')

local _rpm_instance = Parameter("TOR_RPM_INST")
local _min_rpm = Parameter("TOR_RPM_PASS")
local _min_throttle = Parameter("TOR_TH_PASS")


function update()

    local state = NOT_READY

    local rpm_instance = math.floor(math.min(math.max(_rpm_instance:get() - 1, 0), MAX_RPM_INSTANCE))
    local rpm = RPM:get_rpm(rpm_instance)

    local current_throttle = SRV_Channels:get_output_scaled(K_THROTTLE)

    if (rpm < 0) and (arming:is_armed()) then
        gcs:send_text(3, 'TOR: RPM sensor unhealthy')
    end

    if (rpm >= _min_rpm:get()) and (current_throttle >= _min_throttle:get()) and (arming:is_armed()) then
        state = READY_FOR_TAKEOFF
    end

    -- Send named float to gcs
    gcs:send_named_float('TO_STATE', state)

    -- Add to log
    logger.write('TOFF', 'sta', 'f', state)

    -- 1.0 Hz update
    return update, 1000

end

-- 2 Sec delay on boot
return update, 2000
