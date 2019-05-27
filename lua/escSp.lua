local scriptVersion = '0.1'
local refresh = 0
local tsReceiveConfig = 0
local tsSendConfig = 0
local sendConfigIntents = 0
local receiveConfigOk = false
local sendConfigOk = true
local lcdChange = false
local config =
   {firmwareVersion = '',
    protocol = {selected = 4, list = {'HW V3', 'HW V4/V5', 'PWM', ''}, elements = 3},
    battery = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    pwm = {selected = 3, list = {'Off', 'On', ''}, elements = 2}}
local selection = {selected = 1, state = false, list = {'protocol', 'battery', 'pwm'}, elements = 3}

local function getFlags(element)
  if selection.selected ~= element then return SMLSIZE end
  if selection.selected == element and selection.state == false then return SMLSIZE + INVERS end
  if selection.selected == element and selection.state == true then return SMLSIZE + INVERS + BLINK end
  return
end

local function increase(data)
  data.selected = data.selected + 1
  if data.selected == data.elements + 1 then data.selected = 1 end
end

local function decrease(data)
  data.selected = data.selected - 1
  if data.selected == 0 then data.selected = data.elements end
end

local function sendConfig()
  local value = 0
  value = bit32.bor(value, config.protocol.selected - 1)                 -- bits 1,2
  value = bit32.bor(value, bit32.lshift(config.battery.selected - 1, 2)) -- bit 3
  value = bit32.bor(value, bit32.lshift(config.pwm.selected - 1, 3))     -- bit 4
  sportTelemetryPush(10, 0x10, 0x5002, value)
  sendConfigOk = false
end

local function init_func()
end

local function bg_func(event)
  if refresh < 5 then refresh = refresh + 1 end
end

local function run_func(event)
  if refresh == 5 or lcdChange == true or selection.state == true then
    lcd.clear()
    lcd.drawScreenTitle('ESC SmartPort v' .. scriptVersion, 1, 1)
    lcd.drawText(1, 9, 'Firmware', SMLSIZE)
    lcd.drawText(1, 17, 'Protocol', SMLSIZE)
    lcd.drawText(1, 25, 'Battery', SMLSIZE)
    lcd.drawText(1, 33, 'PWM out', SMLSIZE)
    lcd.drawText(50, 9, config.firmwareVersion, SMLSIZE)
    lcd.drawText(1, 54, 'Long press [MENU] to update', SMLSIZE)
    lcd.drawText(50, 17, config.protocol.list[config.protocol.selected], getFlags(1))
    lcd.drawText(50, 25, config.battery.list[config.battery.selected], getFlags(2))
    lcd.drawText(50, 33, config.pwm.list[config.pwm.selected], getFlags(3))
    if receiveConfigOk == false then lcd.drawText(35, 43, 'Connecting...', SMLSIZE) end
    lcdChange = false;
  end

  if receiveConfigOk == false or sendConfigOk == false then
    local physicalId, primId, dataId, value = sportTelemetryPop()          -- frsky/lua: phys_id/sensor id, type/frame_id, sensor_id/data_id
    if physicalId == 9 and dataId == 0x5001 then
      config.protocol.selected = bit32.extract(value,0,2) + 1                      -- bits 1,2
      config.battery.selected = bit32.extract(value,2) + 1                         -- bit 3
      config.pwm.selected = bit32.extract(value,3) + 1                             -- bit 4
      config.firmwareVersion = bit32.extract(value,4,4) .. '.' .. bit32.extract(value,8,4) -- bits 5-8.9-12
      lcdChange = true
      receiveConfigOk = true
    end
    if physicalId == 9 and dataId == 0x5003 then
      sendConfigOk = true
      popupWarning('Updated', EVT_EXIT_BREAK)
    end
    if receiveConfigOk == false and getTime() - tsReceiveConfig > 100 then
      sportTelemetryPush(10, 0x10, 0x5000, 0)

      tsReceiveConfig = getTime()
    end
    if sendConfigOk == false and getTime() - tsSendConfig > 100 then
      sendConfig()
      tsSendConfig = getTime()
      sendConfigIntents = sendConfigIntents + 1
    end
    if sendConfigIntents == 5 then
      sendConfigOk = true
      sendConfigIntents = 0
      popupWarning('Update failed', EVT_EXIT_BREAK)
    end
  end
-- left = up/decrease right = down/increase
  if selection.state == false then
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK then
      decrease(selection)
      lcdChange = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK then
      increase(selection)
      lcdChange = true
    end
  end
  if selection.state == true then
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK then
      decrease(config[selection.list[selection.selected]])
      lcdChange = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK then
      increase(config[selection.list[selection.selected]])
      lcdChange = true
    end
  end
  if event == EVT_ENTER_BREAK and receiveConfigOk == true then
    selection.state = not selection.state
    lcdChange = true
  end
  if event == EVT_EXIT_BREAK then
    selection.state = false
    lcdChange = true
  end
  if event == EVT_MENU_BREAK then
    if receiveConfigOk and sendConfigOk then
      sendConfig()
      tsSendConfig = getTime()
    else
      if receiveConfigOk == false then popupWarning('Not connected', EVT_EXIT_BREAK) end
    end
  end
  refresh = 0
end

return {run=run_func, background=bg_func, init=init_func}
