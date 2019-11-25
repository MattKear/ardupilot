-- This script is a test of led override

local count = 0

function update() -- this is the loop which periodically runs

  count = count + 1

  if count == 1 then
    -- solid red for 15 seconds
    -- red,green,blue,rate hz, duration ms
    notify:handle_rgb_override(255,0,0,0,15000)
  elseif count == 2 then
    -- solid green for 15 seconds
    notify:handle_rgb_override(0,255,0,0,15000)
  elseif count == 3 then
    -- solid blue for 15 seconds
    notify:handle_rgb_override(0,0,255,0,15000)
  elseif count == 4 then
    -- 1hz red  + green for 15 seconds
    notify:handle_rgb_override(255,255,0,1,15000)
  elseif count == 5 then
    -- 1hz green + blue for 15 seconds
    notify:handle_rgb_override(0,255,255,1,15000)
  elseif count == 6 then
    -- 1hz red + blue for 15 seconds
    notify:handle_rgb_override(255,0,255,1,15000)
  elseif count == 7 then
    -- fast white for 15 seconds
    notify:handle_rgb_override(255,255,255,10,15000)
    count = 0
  end

  return update, 30000 -- reschedules the loop in 30 seconds

end

return update() -- run immediately before starting to reschedule
