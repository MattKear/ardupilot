-- This script will set / reset params when leaving or entering a bubble around home location.
-- We assume we are in a bubble if we are on ground and disarmed
-- The bubble is a sphere around home.

-- TODO add check around rally point location aswell

local _bubble_radius = 25 -- bubble around home in Meters
local features_active = true -- when we first boot the features will be 'as set' in the params.  We will deactivate on the first run as we will be on ground

-- Setup param objects for fast access
local WVANE_ENABLE = Parameter()
assert(WVANE_ENABLE:init('WVANE_ENABLE'),"Bubble: get WVANE_ENABLE fail")

local AVD_F_ACTION = Parameter()
assert(AVD_F_ACTION:init('AVD_F_ACTION'),"Bubble: get AVD_F_ACTION fail")

-- get the parameter settings as we don't wan't to assume what there are or will be
local last_set_val = {}
last_set_val['WVANE_ENABLE'] = WVANE_ENABLE:get()
last_set_val['AVD_F_ACTION'] = AVD_F_ACTION:get()


-- Determines if the vehicle is inside the bubble hemi-sphere
function inside_home_bubble()

    -- get vector realtive to home
    local vec_to_home = ahrs:get_relative_position_NED_home()

    -- It is safer to assume we are outside the bubble if we don't have a valid distance from home
    if not vec_to_home then
        return false
    end

    if vec_to_home:length() > _bubble_radius then
        return false
    end
    return true
end

-- Function to deactivate the features
function deactivate()
    -- Update what the param settings were before we disbale the features
    last_set_val['WVANE_ENABLE'] = WVANE_ENABLE:get()
    last_set_val['AVD_F_ACTION'] = AVD_F_ACTION:get()

    -- disable weathervane when inside the bubble
    assert(WVANE_ENABLE:set(0),"Bubble: fail set WVANE_ENABLE=0")

    -- set avoidance to report only
    assert(AVD_F_ACTION:set(1),"Bubble: fail set AVD_F_ACTION=1")

    features_active = false
end

-- Function to activate the features to there 'as set values'
function activate()
     -- reset params back to there previous values
     assert(WVANE_ENABLE:set(last_set_val['WVANE_ENABLE']),"Bubble: fail to reset WVANE_ENABLE")
     assert(AVD_F_ACTION:set(last_set_val['AVD_F_ACTION']),"Bubble: fail to reset AVD_F_ACTION")

     features_active = true
end

-- main update function
function update()

    -- do not run if home has not been set.
    if not ahrs:home_is_set() then
        return
    end

    -- Disable features either if we have moved inside the bubble or if we are on ground or disarmed
    if (not arming:is_armed() and not vehicle:get_likely_flying()) or inside_home_bubble() then
        if features_active then
            gcs:send_text(6, 'Inside bubble')
            deactivate()
        end
        return
    end

    -- Check if we have just moved outside the bubble
    if not inside_home_bubble() and not features_active then
        gcs:send_text(6, 'Exited bubble')
        activate()
    end
end

-- wrapper around update(). This calls update() at 2Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 500
end

-- start running update loop
return protected_wrapper()
