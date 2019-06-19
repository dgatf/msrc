local scriptVersion = '0.3'
local refresh = 0
local tsReceiveConfig = 0
local tsSendConfig = 0
local sendConfigIntents = 0
local receiveConfigOk = false
local sendConfigOk = true
local lcdChange = true
local config =
   {firmwareVersion = '',
    protocol = {selected = 4, list = {'HW V3', 'HW V4/V5', 'PWM', ''}, elements = 3},
    voltage1 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    voltage2 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    ntc1 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    ntc2 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    current = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    pwm = {selected = 3, list = {'Off', 'On', ''}, elements = 2}}
local selection = {selected = 1, state = false, list = {'protocol', 'voltage1', 'voltage2', 'ntc1', 'ntc2', 'current', 'pwm'}, elements = 8}

local function getFlags(element)
  if selection.selected ~= element then return SMLSIZE end
  if selection.selected == element and selection.state == false then return SMLSIZE + INVERS end
  if selection.selected == element and selection.state == true then return SMLSIZE + INVERS + BLINK end
  return
end

local function increase(data)
  data.selected = data.selected + 1
  if data.selected > data.elements then data.selected = 1 end
end

local function decrease(data)
  data.selected = data.selected - 1
  if data.selected < 1 then data.selected = data.elements end
end

local function sendConfig()
  local value = 0
  value = bit32.bor(value, config.protocol.selected - 1)                 -- bits 1,2
  value = bit32.bor(value, bit32.lshift(config.voltage1.selected - 1, 2)) -- bit 3
  value = bit32.bor(value, bit32.lshift(config.voltage2.selected - 1, 3)) -- bit 4
  value = bit32.bor(value, bit32.lshift(config.current.selected - 1, 4)) -- bit 5
  value = bit32.bor(value, bit32.lshift(config.ntc1.selected - 1, 5))     -- bit 6
  value = bit32.bor(value, bit32.lshift(config.ntc2.selected - 1, 6))     -- bit 7
  value = bit32.bor(value, bit32.lshift(config.pwm.selected - 1, 7))     -- bit 8
  sportTelemetryPush(10, 0x10, 0x5002, value)
  sendConfigOk = false
end

local function init_func()
end

local function bg_func(event)
  if refresh < 5 then refresh = refresh + 1 end
end

local function refreshHorus()
  lcd.drawText(1, 1, 'ESC SmartPort v' .. scriptVersion)
  lcd.drawText(1, 20, 'Firmware', SMLSIZE)
  lcd.drawText(1, 40, 'Protocol', SMLSIZE)
  lcd.drawText(1, 60, 'Voltage1', SMLSIZE)
  lcd.drawText(240, 60, 'Voltage2', SMLSIZE)
  lcd.drawText(1, 80, 'Ntc1', SMLSIZE)
  lcd.drawText(240, 80, 'Ntc2', SMLSIZE)
  lcd.drawText(1, 100, 'Current', SMLSIZE)
  lcd.drawText(240, 100, 'PWM out', SMLSIZE)
  lcd.drawText(120, 20, config.firmwareVersion, SMLSIZE)
  lcd.drawText(120, 40, config.protocol.list[config.protocol.selected], getFlags(1))
  lcd.drawText(120, 60, config.voltage1.list[config.voltage1.selected], getFlags(2))
  lcd.drawText(360, 60, config.voltage2.list[config.voltage2.selected], getFlags(3))
  lcd.drawText(120, 80, config.ntc1.list[config.ntc1.selected], getFlags(4))
  lcd.drawText(360, 80, config.ntc2.list[config.ntc2.selected], getFlags(5))
  lcd.drawText(120, 100, config.current.list[config.current.selected], getFlags(6))
  lcd.drawText(360, 100, config.pwm.list[config.pwm.selected], getFlags(7))
  if receiveConfigOk == false then lcd.drawText(80, 200, 'Connecting...', INVERS) end
  lcd.drawText(1, 150, 'Long press [ENTER] to update', SMLSIZE)
end

local function refreshTaranis()
  lcd.drawScreenTitle('ESC SmartPort v' .. scriptVersion, 1, 1)
  lcd.drawText(1, 9, 'Firmware', SMLSIZE)
  lcd.drawText(1, 17, 'Protocol', SMLSIZE)
  lcd.drawText(1, 25, 'Voltage1', SMLSIZE)
  lcd.drawText(64, 25, 'Voltage2', SMLSIZE)
  lcd.drawText(1, 33, 'Ntc1', SMLSIZE)
  lcd.drawText(64, 33, 'Ntc2', SMLSIZE)
  lcd.drawText(1, 41, 'Current', SMLSIZE)
  lcd.drawText(64, 41, 'PWM out', SMLSIZE)
  lcd.drawText(50, 9, config.firmwareVersion, SMLSIZE)
  lcd.drawText(50, 17, config.protocol.list[config.protocol.selected], getFlags(1))
  lcd.drawText(45, 25, config.voltage1.list[config.voltage1.selected], getFlags(2))
  lcd.drawText(109, 25, config.voltage2.list[config.voltage2.selected], getFlags(3))
  lcd.drawText(45, 33, config.ntc1.list[config.ntc1.selected], getFlags(4))
  lcd.drawText(109, 33, config.ntc2.list[config.ntc2.selected], getFlags(5))
  lcd.drawText(45, 41, config.current.list[config.current.selected], getFlags(6))
  lcd.drawText(109, 41, config.pwm.list[config.pwm.selected], getFlags(7))
  if receiveConfigOk == false then lcd.drawText(35, 28, 'Connecting...', INVERS) end
  lcd.drawText(1, 57, 'Long press [ENTER] to update', SMLSIZE)
end

local function run_func(event)

  if refresh == 5 or lcdChange == true or selection.state == true then
    lcd.clear()
    lcdChange = false
    if LCD_W == 480 then refreshHorus() else refreshTaranis() end
  end

  if receiveConfigOk == false or sendConfigOk == false then
    local physicalId, primId, dataId, value = sportTelemetryPop()          -- frsky/lua: phys_id/sensor id, type/frame_id, sensor_id/data_id
    if physicalId == 9 and dataId == 0x5001 then
      if bit32.extract(value,0,2) + 1 >= 1 and bit32.extract(value,0,2) + 1 <= 3 then
        config.protocol.selected = bit32.extract(value,0,2) + 1                      -- bits 1,2
      end
      if bit32.extract(value,2) + 1 >= 1 or bit32.extract(value,2) + 1 <= 2 then
        config.voltage1.selected = bit32.extract(value,2) + 1                         -- bit 3
      end
      if bit32.extract(value,3) + 1 >= 1 or bit32.extract(value,3) + 1 <= 2 then
        config.voltage2.selected = bit32.extract(value,3) + 1                         -- bit 4
      end
      if bit32.extract(value,4) + 1 >= 1 or bit32.extract(value,4) + 1 <= 2 then
        config.current.selected = bit32.extract(value,4) + 1                         -- bit 5
      end
      if bit32.extract(value,5) + 1 >= 1 or bit32.extract(value,5) + 1 <= 2 then
        config.ntc1.selected = bit32.extract(value,5) + 1                             -- bit 6
      end
      if bit32.extract(value,6) + 1 >= 1 or bit32.extract(value,6) + 1 <= 2 then
        config.ntc2.selected = bit32.extract(value,6) + 1                             -- bit 7
      end
      if bit32.extract(value,7) + 1 >= 1 or bit32.extract(value,7) + 1 <= 2 then
        config.pwm.selected = bit32.extract(value,7) + 1                             -- bit 8
      end
      config.firmwareVersion = bit32.extract(value,24,8) .. '.' .. bit32.extract(value,16,8) -- bits 32-25-24-17
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
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK or event == EVT_DOWN_BREAK then
      decrease(selection)
      lcdChange = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK or event == EVT_UP_BREAK then
      increase(selection)
      lcdChange = true
    end
  end
  if selection.state == true then
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK or event == EVT_DOWN_BREAK then
      decrease(config[selection.list[selection.selected]])
      lcdChange = true
    end
    if event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK or event == EVT_UP_BREAK then
      increase(config[selection.list[selection.selected]])
      lcdChange = true
    end
  end
  if event == EVT_ENTER_BREAK and receiveConfigOk == true and sendConfigOk == true then
    selection.state = not selection.state
    lcdChange = true
  end
  if event == EVT_EXIT_BREAK then
    selection.state = false
    lcdChange = true
  end
  if event == EVT_ENTER_LONG then
    -- killEvents(EVT_ENTER_LONG) -- not working
    if receiveConfigOk == true and sendConfigOk == true then
      sendConfig()
      tsSendConfig = getTime()
    else
      if receiveConfigOk == false then popupWarning('Not connected', EVT_EXIT_BREAK) end
    end
  end
  refresh = 0
  return 0
end

return {run=run_func, background=bg_func, init=init_func}
