-- This script is a test of param set and get
local count = 0

function update() -- this is the loop which periodically runs

  -- get and print all the scripting paramerters
  gcs:send_text(0, string.format("LUA: SCR_ENABLE: %f",param:get_by_name('SCR_ENABLE')))
  gcs:send_text(0, string.format("LUA: SCR_VM_I_COUNT: %f",param:get_by_name('SCR_VM_I_COUNT')))
  gcs:send_text(0, string.format("LUA: SCR_HEAP_SIZE: %f",param:get_by_name('SCR_HEAP_SIZE')))
  gcs:send_text(0, string.format("LUA: SCR_DEBUG_LVL: %f",param:get_by_name('SCR_DEBUG_LVL')))

  -- increment the script heap size by one
  local new_heap_size = param:get_by_name('SCR_HEAP_SIZE') + 1;
  local success = param:set_by_name('SCR_HEAP_SIZE',new_heap_size)
  if not(success) then
    gcs:send_text(0, 'LUA: param set failed')
  end

  count = count + 1;

  -- self terminate after 30 loops
  if count > 30 then
    gcs:send_text(0, 'LUA: goodby, world')
    param:set_by_name('SCR_ENABLE',0)
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
