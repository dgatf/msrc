local scriptVersion = '0.3'
local tsReadConfig = 0
local tsSendConfig = 0
local readConfigState = 0
local sendConfigState = 60
local refresh = 0
local lcdChange = true
local scroll = 0
local config =
   {firmwareVersion = '',
    protocol = {selected = 4, list = {'HW V3', 'HW V4/V5', 'PWM', ''}, elements = 3},
    voltage1 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    voltage2 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    ntc1 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    ntc2 = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    current = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    pwm = {selected = 3, list = {'Off', 'On', ''}, elements = 2},
    refreshRpm = {selected = 1, elements = 16},
    refreshVolt = {selected = 1, elements = 16},
    refreshCurr = {selected = 1, elements = 16},
    refreshTemp = {selected = 1, elements = 16},
    queueRpm = {selected = 1, elements = 16},
    queueVolt = {selected = 1, elements = 16},
    queueCurr = {selected = 1, elements = 16},
    queueTemp = {selected = 1, elements = 16},
    queuePwm = {selected = 1, elements = 16},
   }
local selection = {selected = 1, state = false, list = {'protocol', 'voltage1', 'voltage2', 'ntc1', 'ntc2', 'current', 'pwm', 'refreshRpm', 'refreshVolt', 'refreshCurr', 'refreshTemp', 'queueRpm', 'queueVolt', 'queueCurr', 'queueTemp', 'queuePwm', 'btnUpdate'}, elements = 17}

local function getFlags(element)
  if string.find(selection.list[element], 'btn') == 1 and selection.selected == element then return INVERS + BLINK end
  if string.find(selection.list[element], 'btn') == 1 and selection.selected ~= element then return INVERS end
  if selection.selected ~= element then return 0 end
  if selection.selected == element and selection.state == false then return INVERS end
  if selection.selected == element and selection.state == true then return INVERS + BLINK end
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

local function readConfig()
  if readConfigState < 10 then
    sportTelemetryPush(10, 0x21, 0xFFFF, 0x80)
    readConfigState = readConfigState + 2
  elseif readConfigState == 10 then
    sportTelemetryPush(10, 0x10, 0x5000, 0)
  elseif readConfigState >= 20 then
    sportTelemetryPush(10, 0x20, 0xFFFF, 0x80)
    readConfigState = readConfigState + 2
    lcdChange = true
  end
  tsReadConfig = getTime()
end

local function sendConfig()
  if sendConfigState < 10 then
    sportTelemetryPush(10, 0x21, 0xFFFF, 0x80)
    sendConfigState = sendConfigState + 2
  elseif sendConfigState == 10 then
    local value = 0
    value = bit32.bor(value, config.protocol.selected - 1)                      -- bits 1-2
    value = bit32.bor(value, bit32.lshift(config.voltage1.selected - 1, 2))     -- bit 3
    value = bit32.bor(value, bit32.lshift(config.voltage2.selected - 1, 3))     -- bit 4
    value = bit32.bor(value, bit32.lshift(config.current.selected - 1, 4))      -- bit 5
    value = bit32.bor(value, bit32.lshift(config.ntc1.selected - 1, 5))         -- bit 6
    value = bit32.bor(value, bit32.lshift(config.ntc2.selected - 1, 6))         -- bit 7
    value = bit32.bor(value, bit32.lshift(config.pwm.selected - 1, 7))          -- bit 8
    value = bit32.bor(value, bit32.lshift(config.refreshRpm.selected - 1, 8))   -- bits 9-12
    value = bit32.bor(value, bit32.lshift(config.refreshVolt.selected - 1, 12)) -- bits 13-16
    value = bit32.bor(value, bit32.lshift(config.refreshCurr.selected - 1, 16)) -- bits 17-20
    value = bit32.bor(value, bit32.lshift(config.refreshTemp.selected - 1, 20)) -- bits 21-24
    sportTelemetryPush(10, 0x10, 0x5011, value)
  elseif sendConfigState == 20 then
    local value = 0
    value = bit32.bor(value, config.queueRpm.selected)                          -- bits 1-4
    value = bit32.bor(value, bit32.lshift(config.queueVolt.selected, 4))        -- bits 5-8
    value = bit32.bor(value, bit32.lshift(config.queueCurr.selected, 8))        -- bits 9-12
    value = bit32.bor(value, bit32.lshift(config.queueTemp.selected, 12))       -- bits 13-16
    value = bit32.bor(value, bit32.lshift(config.queuePwm.selected, 16))        -- bits 17-20
    sportTelemetryPush(10, 0x10, 0x5012, value)
  elseif sendConfigState >= 30 then
    sportTelemetryPush(10, 0x20, 0xFFFF, 0x80)
    sendConfigState = sendConfigState + 2
  end
  tsSendConfig = getTime()
end

local function refreshHorus()
  lcd.drawRectangle(40, 30, 400, 217)
  lcd.drawText(150, 5, 'ESC SmartPort v' .. scriptVersion, INVERS)
  lcd.drawText(50, 40, 'Firmware', 0)
  lcd.drawText(170, 40, config.firmwareVersion, 0)
  lcd.drawText(50, 60, 'Protocol', 0)
  lcd.drawText(170, 60, config.protocol.list[config.protocol.selected], getFlags(1))
  lcd.drawText(50, 80, 'Voltage1', 0)
  lcd.drawText(170, 80, config.voltage1.list[config.voltage1.selected], getFlags(2))
  lcd.drawText(290, 80, 'Voltage2', 0)
  lcd.drawText(410, 80, config.voltage2.list[config.voltage2.selected], getFlags(3))
  lcd.drawText(50, 100, 'Ntc1', 0)
  lcd.drawText(170, 100, config.ntc1.list[config.ntc1.selected], getFlags(4))
  lcd.drawText(290, 100, 'Ntc2', 0)
  lcd.drawText(410, 100, config.ntc2.list[config.ntc2.selected], getFlags(5))
  lcd.drawText(50, 120, 'Current', 0)
  lcd.drawText(170, 120, config.current.list[config.current.selected], getFlags(6))
  lcd.drawText(290, 120, 'PWM out', 0)
  lcd.drawText(410, 120, config.pwm.list[config.pwm.selected], getFlags(7))

  lcd.drawText(50, 140, 'Ref RPM', 0)
  lcd.drawText(170, 140, (config.refreshRpm.selected - 1) * 100, getFlags(8))
  lcd.drawText(290, 140, 'Ref Volt', 0)
  lcd.drawText(410, 140, (config.refreshVolt.selected - 1) * 100, getFlags(9))

  lcd.drawText(50, 160, 'Ref Curr', 0)
  lcd.drawText(170, 160, (config.refreshCurr.selected - 1) * 100, getFlags(10))
  lcd.drawText(290, 160, 'Ref Temp', 0)
  lcd.drawText(410, 160, (config.refreshTemp.selected - 1) * 100, getFlags(11))

  lcd.drawText(50, 180, 'Avg RPM', 0)
  lcd.drawText(170, 180, config.queueRpm.selected, getFlags(12))
  lcd.drawText(290, 180, 'Avg Volt', 0)
  lcd.drawText(410, 180, config.queueVolt.selected, getFlags(13))

  lcd.drawText(50, 200, 'Avg Curr', 0)
  lcd.drawText(170, 200, config.queueCurr.selected, getFlags(14))
  lcd.drawText(290, 200, 'Avg Temp', 0)
  lcd.drawText(410, 200, config.queueTemp.selected, getFlags(15))

  lcd.drawText(50, 220, 'Avg PWM', 0)
  lcd.drawText(170, 220, config.queuePwm.selected, getFlags(16))

  if readConfigState < 30 then lcd.drawText(180, 155, 'Connecting...', INVERS) end
  lcd.drawText(200, 250, 'UPDATE', getFlags(17))
end

local function refreshTaranis()
  lcd.drawText(1, 9 - scroll * 8, 'Firmware', SMLSIZE)
  lcd.drawText(44, 9 - scroll * 8, config.firmwareVersion, SMLSIZE)
  lcd.drawText(1, 17 - scroll * 8, 'Protocol', SMLSIZE)
  lcd.drawText(44, 17 - scroll * 8, config.protocol.list[config.protocol.selected], SMLSIZE + getFlags(1))
  lcd.drawText(1, 25 - scroll * 8, 'Voltage1', SMLSIZE)
  lcd.drawText(44, 25 - scroll * 8, config.voltage1.list[config.voltage1.selected], SMLSIZE + getFlags(2))
  lcd.drawText(64, 25 - scroll * 8, 'Voltage2', SMLSIZE)
  lcd.drawText(108, 25 - scroll * 8, config.voltage2.list[config.voltage2.selected], SMLSIZE + getFlags(3))
  lcd.drawText(1, 33 - scroll * 8, 'Ntc1', SMLSIZE)
  lcd.drawText(44, 33 - scroll * 8, config.ntc1.list[config.ntc1.selected], SMLSIZE + getFlags(4))
  lcd.drawText(64, 33 - scroll * 8, 'Ntc2', SMLSIZE)
  lcd.drawText(108, 33 - scroll * 8, config.ntc2.list[config.ntc2.selected], SMLSIZE + getFlags(5))
  lcd.drawText(1, 41 - scroll * 8, 'Current', SMLSIZE)
  lcd.drawText(44, 41 - scroll * 8, config.current.list[config.current.selected], SMLSIZE + getFlags(6))
  lcd.drawText(64, 41 - scroll * 8, 'PWM out', SMLSIZE)
  lcd.drawText(108, 41 - scroll * 8, config.pwm.list[config.pwm.selected], SMLSIZE + getFlags(7))

  lcd.drawText(1, 49 - scroll * 8, 'Ref RPM', SMLSIZE)
  lcd.drawText(44, 49 - scroll * 8, (config.refreshRpm.selected - 1) * 100, SMLSIZE + getFlags(8))
  lcd.drawText(64, 49 - scroll * 8, 'Ref Volt', SMLSIZE)
  lcd.drawText(108, 49 - scroll * 8, (config.refreshVolt.selected - 1) * 100, SMLSIZE + getFlags(9))

  lcd.drawText(1, 57 - scroll * 8, 'Ref Curr', SMLSIZE)
  lcd.drawText(44, 57 - scroll * 8, (config.refreshCurr.selected - 1) * 100, SMLSIZE + getFlags(10))
  lcd.drawText(64, 57 - scroll * 8, 'Ref Temp', SMLSIZE)
  lcd.drawText(108, 57 - scroll * 8, (config.refreshTemp.selected - 1) * 100, SMLSIZE + getFlags(11))

  lcd.drawText(1, 65 - scroll * 8, 'Avg RPM', SMLSIZE)
  lcd.drawText(44, 65 - scroll * 8, config.queueRpm.selected, SMLSIZE + getFlags(12))
  lcd.drawText(64, 65 - scroll * 8, 'Avg Volt', SMLSIZE)
  lcd.drawText(108, 65 - scroll * 8, config.queueVolt.selected, SMLSIZE + getFlags(13))

  lcd.drawText(1, 73 - scroll * 8, 'Avg Curr', SMLSIZE)
  lcd.drawText(44, 73 - scroll * 8, config.queueCurr.selected, SMLSIZE + getFlags(14))
  lcd.drawText(64, 73 - scroll * 8, 'Avg Temp', SMLSIZE)
  lcd.drawText(108, 73 - scroll * 8, config.queueTemp.selected, SMLSIZE + getFlags(15))

  lcd.drawText(1, 81 - scroll * 8, 'Avg PWM', SMLSIZE)
  lcd.drawText(44, 81 - scroll * 8, config.queuePwm.selected, SMLSIZE + getFlags(16))

  if readConfigState < 30 then lcd.drawText(35, 28, 'Connecting...', INVERS) end

  lcd.drawText(1, 89 - scroll * 8, 'UPDATE', SMLSIZE + getFlags(17))
  if sendConfigState == 40 then lcd.drawText(35, 89 - scroll * 8, 'Ok', SMLSIZE)
  elseif sendConfigState == 50 then lcd.drawText(35, 89 - scroll * 8, 'ERROR', SMLSIZE) end
  lcd.drawScreenTitle('ESC SmartPort v' .. scriptVersion, 1, 1)
end

local function init_func()
end

local function bg_func(event)
  if refresh < 5 then refresh = refresh + 1 end
end

local function run_func(event)

  -- update lcd
  if refresh == 5 or lcdChange == true or selection.state == true or string.find(selection.list[selection.selected], 'btn') then
    lcd.clear()
    lcdChange = false
    if LCD_W == 480 then refreshHorus() else refreshTaranis() end
  end

  -- check incoming packets
  if readConfigState == 10 or sendConfigState == 10 or sendConfigState == 20 then
    local physicalId, primId, dataId, value = sportTelemetryPop()

    -- read config
    if physicalId == 9 and dataId == 0x5001 then
      config.firmwareVersion = bit32.extract(value,16,8) .. '.' .. bit32.extract(value,8,8) .. '.' .. bit32.extract(value,0,8)
    end
    if physicalId == 9 and dataId == 0x5002 then
      if bit32.extract(value,0,2) + 1 >= 1 and bit32.extract(value,0,2) + 1 <= 3 then
        config.protocol.selected = bit32.extract(value,0,2) + 1                             -- bits 1,2
      end
      if bit32.extract(value,2) + 1 >= 1 and bit32.extract(value,2) + 1 <= 2 then
        config.voltage1.selected = bit32.extract(value,2) + 1                               -- bit 3
      end
      if bit32.extract(value,3) + 1 >= 1 and bit32.extract(value,3) + 1 <= 2 then
        config.voltage2.selected = bit32.extract(value,3) + 1                               -- bit 4
      end
      if bit32.extract(value,4) + 1 >= 1 and bit32.extract(value,4) + 1 <= 2 then
        config.current.selected = bit32.extract(value,4) + 1                                -- bit 5
      end
      if bit32.extract(value,5) + 1 >= 1 and bit32.extract(value,5) + 1 <= 2 then
        config.ntc1.selected = bit32.extract(value,5) + 1                                   -- bit 6
      end
      if bit32.extract(value,6) + 1 >= 1 and bit32.extract(value,6) + 1 <= 2 then
        config.ntc2.selected = bit32.extract(value,6) + 1                                   -- bit 7
      end
      if bit32.extract(value,7) + 1 >= 1 and bit32.extract(value,7) + 1 <= 2 then
        config.pwm.selected = bit32.extract(value,7) + 1                                    -- bit 8
      end
      if bit32.extract(value,8,4) + 1 >= 1 and bit32.extract(value,8,4) + 1 <= 16 then
        config.refreshRpm.selected = bit32.extract(value,8,4) + 1                           -- bits 9-12
      end
      if bit32.extract(value,12,4) + 1 >= 1 and bit32.extract(value,12,4) + 1 <= 16 then
        config.refreshVolt.selected = bit32.extract(value,12,4) + 1                         -- bits 13-16
      end
      if bit32.extract(value,16,4) + 1 >= 1 and bit32.extract(value,16,4) + 1 <= 16 then
        config.refreshCurr.selected = bit32.extract(value,12,4) + 1                         -- bits 17-20
      end
      if bit32.extract(value,20,4) >= 1 and bit32.extract(value,20,4) <= 16 then
        config.refreshTemp.selected = bit32.extract(value,20,4) + 1                         -- bits 21-24
      end
    end
    if physicalId == 9 and dataId == 0x5003 then
      if bit32.extract(value,0,4) >= 1 and bit32.extract(value,0,4) <= 16 then
        config.queueRpm.selected = bit32.extract(value,0,4)                                -- bits 1-4
      end
      if bit32.extract(value,4,4) >= 1 and bit32.extract(value,4,4) <= 16 then
        config.queueVolt.selected = bit32.extract(value,4,4)                               -- bits 5-8
      end
      if bit32.extract(value,8,4) >= 1 and bit32.extract(value,8,4) <= 16 then
        config.queueCurr.selected = bit32.extract(value,8,4)                               -- bits 9-12
      end
      if bit32.extract(value,12,4) >= 1 and bit32.extract(value,12,4) <= 16 then
        config.queueTemp.selected = bit32.extract(value,12,4)                               -- bits 13-16
      end
      if bit32.extract(value,16,4) >= 1 and bit32.extract(value,16,4) <= 16 then
        config.queuePwm.selected = bit32.extract(value,16,4)                                 -- bits 17-20
      end
      lcdChange = true
      readConfigState = 20
    end

    -- send config status
    if physicalId == 9 and dataId == 0x5020 and sendConfigState == 10 then
      sendConfigState = 20
    end
    if physicalId == 9 and dataId == 0x5021 and sendConfigState == 20 then
      sendConfigState = 60
      lcdChange = false
    end

  end

  -- send packets
  if readConfigState < 30 and getTime() - tsReadConfig > 100 then
    readConfig()
  end
  if sendConfigState < 40 and getTime() - tsSendConfig > 100 then
    sendConfig()
  end

  -- key events (left = up/decrease right = down/increase)
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
  if event == EVT_ENTER_BREAK then
    if string.find(selection.list[selection.selected], 'btn') ~= 1 then
      if readConfigState == 30 and sendConfigState >= 40 then
        selection.state = not selection.state
        lcdChange = true
      end
    else
      if readConfigState == 30 then
        sendConfigState = 0
      end
    end
  end
  if event == EVT_EXIT_BREAK then
    selection.state = false
    lcdChange = true
  end

  --lcd scrolling
  scroll = 0
  if selection.selected > 7 then scroll = 1 end
  if selection.selected > 9 then scroll = 2 end
  if selection.selected > 11 then scroll = 3 end
  if selection.selected > 13 then scroll = 4 end

  refresh = 0

  return 0
end

return {run=run_func, background=bg_func, init=init_func}
