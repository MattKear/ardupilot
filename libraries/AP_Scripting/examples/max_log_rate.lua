

local file_name = "log_test.csv"
local format_string = "%s, %.4f, %04i, %.0f, %.4f, %.4f, %.5f, %.5f, %.2f, %.2f, %05i, %03i\n" -- Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage (V), Current (A), Thrust (g), Torque (g.cm), ESC Volt (V), ESC Current (A), ESC RPM (rpm), ESC Temperature (deg C)


local file = assert(io.open(file_name, "w"),"Could not make file: " .. file_name)
local header = 'Time (ms), Throttle (), RC Out (us), Motor Commutations (1/min), Voltage(V), Current (A), Thrust (g), Torque (kg.cm), ESC Volt (V), ESC Current (A), ESC RPM (rpm), ESC Temperature (deg C)\n'
file:write(header)
file:close()

local count = 0

function update()

    local now = tonumber(tostring(millis()))

    file = io.open(file_name, "a")
    file:write(string.format(format_string, tostring(now), 0.1, 1100, 240, 21.3, 5.1, 1200.1, 1400.0, 21.2, 5.2, 242, 25))
    file:close()

    if count < 1000 then
        count = count + 1
        return update, 12.5
    end
end

return update, 2000
