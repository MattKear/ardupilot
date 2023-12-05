local BOOT_DELAY_MS = 3000
local UPDATE_DELAY = 500
local MOTOR_RUN_TIME = 2000
local MOTOR_TEST_PWM = 1170
local mot_num = 1
local has_been_init = false

-- create an array of the motor functions to be tested.  You have previously wanted to test up to 4 motors so i have put the four motor functions in here
-- The order of the array matters if you want the motors to test in the order that the insequence motor test works
local motor_functions_to_test = {33,36,34,35} -- Quad X
-- local motor_functions_to_test = {35,33,36,34} -- Quad +
-- local motor_functions_to_test = {37,33,36,38,34,35} -- Hex X

-- This is an array that we will store channel numbers
local channels_to_test = {}

-- I have added an example of a pre-arm check that forces your operators to have completed a motor test before you are allowed to arm
-- just delete the bit indicated with "PREARM OPTION" if you do not want this.
local auth_id = arming:get_aux_auth_id()

function update()

    -- PREARM OPTION: delete block if not wanted
    if auth_id then
        -- default to pre-arm failed as this script will no longer be running if we have done the motor test
        arming:set_aux_auth_failed(auth_id, "Motor test not run")
    end

    if arming:is_armed() then
        -- return early if system is armed, we do not want to motor test
        return update, UPDATE_DELAY
    end

    -- Initialise the script by finding the motor channels that we need
    if not has_been_init then
        -- Assume we succesfully found all of the motor channels and fail it if later if we did not
        has_been_init = true
        -- Initialise the script by finding the motor outputs that you want to test.  This allows the script to be generic should you ever need to move the motor outputs to other channels
        for i = 1, #motor_functions_to_test do
            local chan = SRV_Channels:find_channel(motor_functions_to_test[i])
            if chan ~= nil then
                channels_to_test[i] = chan
            else
                gcs:send_text(1,"Mot Test Fail: Function " .. tostring(motor_functions_to_test[i]) .. " not found")
                -- fail the init as we did not find all of the motor channels
                has_been_init = false
            end
        end
    end

    -- We did not manage to init the script properly. Restart the script after delay.  This will keep flashing the failure message to the GCS so that the operator knows there is a problem
    if not has_been_init then
        return update, UPDATE_DELAY
    end

    -- Add your additional conditions for running the motor test here.  In this example i am checking that the RC input that is set to Scripting 1 is high to allow the test to run.  The rc input must be kept high for the whole test
    local switch = assert(rc:find_channel_for_option(300), "Mot Test: Could not find Scripting 1 RC Switch")
    if switch:norm_input() < 0.6 then
        -- RC switch is low so we will not conduct the motor test
        -- reset the motor number to restart the test next time
        mot_num = 1
        return update, UPDATE_DELAY
    end

    if mot_num > #channels_to_test then
        gcs:send_text(4, "Motor Test Complete")

        -- PREARM OPTION: delete the nest two lines if you do not want this
        -- pass the arming checks
        arming:set_aux_auth_passed(auth_id)

        -- end the script, we only want to test once, reboot or restarting scripting required to be able to run the test again
        return
    end

    -- Tell the GCS which motor we are testing
    gcs:send_text(4, "Mot Test: Testing Motor " .. tostring(motor_functions_to_test[mot_num] - 32)) -- This math only works up to motor 8

    -- Output to motors and increment the motor to test
    SRV_Channels:set_output_pwm_chan_timeout(channels_to_test[mot_num], MOTOR_TEST_PWM, MOTOR_RUN_TIME)
    mot_num = mot_num + 1

    return update, MOTOR_RUN_TIME

end

return update, BOOT_DELAY_MS 
