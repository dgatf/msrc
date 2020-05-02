--
-- Multi Sensor RC - MSRC
--
-- License https://www.gnu.org/licenses/gpl-3.0.en.html
--

local scriptVersion = "0.5"
local tsReadConfig = 0
local tsSendConfig = 0
local state = {
    INIT = {},
    MAINTENANCE_ON = {},
    CONFIG_REQUESTED = {},
    PACKET_1 = {},
    PACKET_2 = {},
    PACKET_3 = {},
    PACKET_4 = {},
    MAINTENANCE_OFF = {}
}
local readConfigState = state["INIT"]
local sendConfigState = state["MAINTENANCE_OFF"]
local refresh = 0
local lcdChange = true
local scroll = 0
local sensorIdTx = 17 -- sensorId 18
local config = {
    firmwareVersion = "",
    protocol = {selected = 6, list = {"NONE", "HW V3", "HW V4/V5", "PWM", "CASTLE", ""}, elements = 5},
    voltage1 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    voltage2 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    ntc1 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    ntc2 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    current = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    pwm = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    refreshRpm = {selected = 1, elements = 16},
    refreshVolt = {selected = 1, elements = 16},
    refreshCurr = {selected = 1, elements = 16},
    refreshTemp = {selected = 1, elements = 16},
    queueRpm = {selected = 1, elements = 16},
    queueVolt = {selected = 1, elements = 16},
    queueCurr = {selected = 1, elements = 16},
    queueTemp = {selected = 1, elements = 16},
    i2c1 = {selected = 4, list = {"NONE", "BMP180", "BMP280", ""}, elements = 3},
    i2c1Address = {selected = 1, elements = 128},
    i2c2 = {selected = 4, list = {"NONE", "BMP180", "BMP280", ""}, elements = 3},
    i2c2Address = {selected = 1, elements = 128}
}
local selection = {
    selected = 1,
    elements = 20,
    state = false,
    list = {
        "protocol",
        "voltage1",
        "voltage2",
        "ntc1",
        "ntc2",
        "current",
        "pwm",
        "refreshRpm",
        "refreshVolt",
        "refreshCurr",
        "refreshTemp",
        "queueRpm",
        "queueVolt",
        "queueCurr",
        "queueTemp",
        "i2c1",
        "i2c1Address",
        "i2c2",
        "i2c2Address",
        "btnUpdate"
    }
}

local function getFlags(element)
    if string.find(selection.list[element], "btn") == 1 and selection.selected == element then
        return INVERS + BLINK
    end
    if string.find(selection.list[element], "btn") == 1 and selection.selected ~= element then
        return INVERS
    end
    if selection.selected ~= element then
        return 0
    end
    if selection.selected == element and selection.state == false then
        return INVERS
    end
    if selection.selected == element and selection.state == true then
        return INVERS + BLINK
    end
    return
end

local function increase(data)
    data.selected = data.selected + 1
    if data.selected > data.elements then
        data.selected = 1
    end
end

local function decrease(data)
    data.selected = data.selected - 1
    if data.selected < 1 then
        data.selected = data.elements
    end
end

local function readConfig()
    if readConfigState ~= state["MAINTENANCE_OFF"] then
        if readConfigState == state["INIT"] then
            if sportTelemetryPush(sensorIdTx, 0x21, 0xFFFF, 0x80) then
                tsReadConfig = getTime()
                readConfigState = state["MAINTENANCE_ON"]
            end
        elseif readConfigState == state["MAINTENANCE_ON"] then
            if sportTelemetryPush(sensorIdTx, 0x30, 0x5000, 0) then
                readConfigState = state["CONFIG_REQUESTED"]
            end
        elseif readConfigState == state["PACKET_4"] then
            if sportTelemetryPush(sensorIdTx, 0x20, 0xFFFF, 0x80) then
                readConfigState = state["MAINTENANCE_OFF"]
                lcdChange = true
            end
        end
        local physicalId, primId, dataId, value = sportTelemetryPop()
        if primId == 0x32 and dataId == 0x5000 then
            if bit32.extract(value, 0, 8) == 0xF1 and readConfigState == state["CONFIG_REQUESTED"] then
                config.firmwareVersion =
                    bit32.extract(value, 24, 8) .. "." .. bit32.extract(value, 16, 8) .. "." .. bit32.extract(value, 8, 8)
                readConfigState = state["PACKET_1"]
            end
            if bit32.extract(value, 0, 8) == 0xF2 and readConfigState == state["PACKET_1"] then
                if bit32.extract(value, 10) + 1 >= 1 and bit32.extract(value, 10) + 1 <= 2 then
                    config.voltage1.selected = bit32.extract(value, 10) + 1 -- bit 11
                end
                if bit32.extract(value, 11) + 1 >= 1 and bit32.extract(value, 11) + 1 <= 2 then
                    config.voltage2.selected = bit32.extract(value, 11) + 1 -- bit 12
                end
                if bit32.extract(value, 12) + 1 >= 1 and bit32.extract(value, 12) + 1 <= 2 then
                    config.current.selected = bit32.extract(value, 12) + 1 -- bit 13
                end
                if bit32.extract(value, 13) + 1 >= 1 and bit32.extract(value, 13) + 1 <= 2 then
                    config.ntc1.selected = bit32.extract(value, 13) + 1 -- bit 14
                end
                if bit32.extract(value, 14) + 1 >= 1 and bit32.extract(value, 14) + 1 <= 2 then
                    config.ntc2.selected = bit32.extract(value, 14) + 1 -- bit 15
                end
                if bit32.extract(value, 15) + 1 >= 1 and bit32.extract(value, 15) + 1 <= 2 then
                    config.pwm.selected = bit32.extract(value, 15) + 1 -- bit 16
                end
                if bit32.extract(value, 16, 4) + 1 >= 1 and bit32.extract(value, 16, 4) + 1 <= 16 then
                    config.refreshRpm.selected = bit32.extract(value, 16, 4) + 1 -- bits 17-20
                end
                if bit32.extract(value, 20, 4) + 1 >= 1 and bit32.extract(value, 20, 4) + 1 <= 16 then
                    config.refreshVolt.selected = bit32.extract(value, 20, 4) + 1 -- bits 21-24
                end
                if bit32.extract(value, 24, 4) + 1 >= 1 and bit32.extract(value, 24, 4) + 1 <= 16 then
                    config.refreshCurr.selected = bit32.extract(value, 24, 4) + 1 -- bits 25-28
                end
                if bit32.extract(value, 28, 4) >= 1 and bit32.extract(value, 28, 4) <= 16 then
                    config.refreshTemp.selected = bit32.extract(value, 28, 4) + 1 -- bits 29-32
                end
                readConfigState = state["PACKET_2"]
            end
            if bit32.extract(value, 0, 8) == 0xF3 and readConfigState == state["PACKET_2"] then
                if bit32.extract(value, 8, 4) >= 1 and bit32.extract(value, 8, 4) <= 16 then
                    config.queueRpm.selected = bit32.extract(value, 8, 4) -- bits 9-12
                end
                if bit32.extract(value, 12, 4) >= 1 and bit32.extract(value, 12, 4) <= 16 then
                    config.queueVolt.selected = bit32.extract(value, 12, 4) -- bits 13-16
                end
                if bit32.extract(value, 16, 4) >= 1 and bit32.extract(value, 16, 4) <= 16 then
                    config.queueCurr.selected = bit32.extract(value, 16, 4) -- bits 17-20
                end
                if bit32.extract(value, 20, 4) >= 1 and bit32.extract(value, 20, 4) <= 16 then
                    config.queueTemp.selected = bit32.extract(value, 20, 4) -- bits 21-24
                end
                if bit32.extract(value, 24, 8) >= 0 and bit32.extract(value, 24, 8) <= 4 then
                    config.protocol.selected = bit32.extract(value, 24, 8) + 1 -- bits 25-32
                end
                readConfigState = state["PACKET_3"]
            end
            if bit32.extract(value, 0, 8) == 0xF4 and readConfigState == state["PACKET_3"] then
                if bit32.extract(value, 8, 4) >= 0 and bit32.extract(value, 8, 4) <= 3 then
                    config.i2c1.selected = bit32.extract(value, 8, 4) + 1 -- bits 9-12
                end
                if bit32.extract(value, 12, 4) >= 0 and bit32.extract(value, 12, 4) <= 3 then
                    config.i2c2.selected = bit32.extract(value, 12, 4) + 1 -- bits 13-16
                end
                if bit32.extract(value, 16, 8) >= 0 and bit32.extract(value, 16, 8) <= 127 then
                    config.i2c1Address.selected = bit32.extract(value, 16, 8) + 1 -- bits 17-24
                end
                if bit32.extract(value, 24, 8) >= 0 and bit32.extract(value, 24, 8) <= 127 then
                    config.i2c2Address.selected = bit32.extract(value, 24, 8) + 1 -- bits 25-32
                end
                readConfigState = state["PACKET_4"]
            end
        end
        -- timeout
        if getTime() - tsReadConfig > 100 then
            readConfigState = state["INIT"]
        end
    end
end

local function sendConfig()
    if sendConfigState ~= state["MAINTENANCE_OFF"] then
        if sendConfigState == state["INIT"] then
            if sportTelemetryPush(sensorIdTx, 0x21, 0xFFFF, 0x80) then
                tsSendConfig = getTime()
                sendConfigState = state["MAINTENANCE_ON"]
            end
        elseif sendConfigState == state["MAINTENANCE_ON"] then
            local value = 0xF1 -- bits 1-8
            value = bit32.bor(value, bit32.lshift(config.voltage1.selected - 1, 10)) -- bit 11
            value = bit32.bor(value, bit32.lshift(config.voltage2.selected - 1, 11)) -- bit 12
            value = bit32.bor(value, bit32.lshift(config.current.selected - 1, 12)) -- bit 13
            value = bit32.bor(value, bit32.lshift(config.ntc1.selected - 1, 13)) -- bit 14
            value = bit32.bor(value, bit32.lshift(config.ntc2.selected - 1, 14)) -- bit 15
            value = bit32.bor(value, bit32.lshift(config.pwm.selected - 1, 15)) -- bit 16
            value = bit32.bor(value, bit32.lshift(config.refreshRpm.selected - 1, 16)) -- bits 17-20
            value = bit32.bor(value, bit32.lshift(config.refreshVolt.selected - 1, 20)) -- bits 21-24
            value = bit32.bor(value, bit32.lshift(config.refreshCurr.selected - 1, 24)) -- bits 25-28
            value = bit32.bor(value, bit32.lshift(config.refreshTemp.selected - 1, 28)) -- bits 29-32
            if sportTelemetryPush(sensorIdTx, 0x31, 0x5000, value) then
                sendConfigState = state["PACKET_1"]
            end
        elseif sendConfigState == state["PACKET_1"] then
            local value = 0xF2 -- bits 1-8
            value = bit32.bor(value, bit32.lshift(config.queueRpm.selected, 8)) -- bits 9-12
            value = bit32.bor(value, bit32.lshift(config.queueVolt.selected, 12)) -- bits 13-16
            value = bit32.bor(value, bit32.lshift(config.queueCurr.selected, 16)) -- bits 17-20
            value = bit32.bor(value, bit32.lshift(config.queueTemp.selected, 20)) -- bits 21-24
            value = bit32.bor(value, bit32.lshift(config.protocol.selected - 1, 24)) -- bits 25-32
            if sportTelemetryPush(sensorIdTx, 0x31, 0x5000, value) then
                sendConfigState = state["PACKET_2"]
            end
        elseif sendConfigState == state["PACKET_2"] then
            local value = 0xF3 -- bits 1-8
            value = bit32.bor(value, bit32.lshift(config.i2c1.selected - 1, 8)) -- bits 9-12
            value = bit32.bor(value, bit32.lshift(config.i2c2.selected - 1, 12)) -- bits 13-16
            value = bit32.bor(value, bit32.lshift(config.i2c1Address.selected - 1, 16)) -- bits 17-24
            value = bit32.bor(value, bit32.lshift(config.i2c2Address.selected - 1, 24)) -- bits 25-32
            if sportTelemetryPush(sensorIdTx, 0x31, 0x5000, value) then
                sendConfigState = state["PACKET_3"]
            end
        elseif sendConfigState == state["PACKET_3"] then
            local physicalId, primId, dataId, value = sportTelemetryPop()
            if primId == 0x32 and dataId == 0x5000 and value == 0xFF then
                if sportTelemetryPush(sensorIdTx, 0x20, 0xFFFF, 0x80) then
                    sendConfigState = state["MAINTENANCE_OFF"]
                    lcdChange = true
                end
            end
        end
        -- timeout
        if getTime() - tsSendConfig > 100 then
            sendConfigState = state["INIT"]
        end
    end
end

local function refreshHorus()
    lcd.drawRectangle(40, 30, 400, 217)
    lcd.drawText(150, 5, "MSRC v" .. scriptVersion, INVERS)
    lcd.drawText(50, 40, "Firmware", 0)
    lcd.drawText(170, 40, config.firmwareVersion, 0)
    lcd.drawText(50, 60, "Protocol", 0)
    lcd.drawText(170, 60, config.protocol.list[config.protocol.selected], getFlags(1))
    lcd.drawText(50, 80, "Voltage1", 0)
    lcd.drawText(170, 80, config.voltage1.list[config.voltage1.selected], getFlags(2))
    lcd.drawText(290, 80, "Voltage2", 0)
    lcd.drawText(410, 80, config.voltage2.list[config.voltage2.selected], getFlags(3))
    lcd.drawText(50, 100, "Ntc1", 0)
    lcd.drawText(170, 100, config.ntc1.list[config.ntc1.selected], getFlags(4))
    lcd.drawText(290, 100, "Ntc2", 0)
    lcd.drawText(410, 100, config.ntc2.list[config.ntc2.selected], getFlags(5))
    lcd.drawText(50, 120, "Current", 0)
    lcd.drawText(170, 120, config.current.list[config.current.selected], getFlags(6))
    lcd.drawText(290, 120, "PWM out", 0)
    lcd.drawText(410, 120, config.pwm.list[config.pwm.selected], getFlags(7))

    lcd.drawText(50, 140, "Rate RPM", 0)
    lcd.drawText(170, 140, (config.refreshRpm.selected - 1) * 100, getFlags(8))
    lcd.drawText(290, 140, "Rate Volt", 0)
    lcd.drawText(410, 140, (config.refreshVolt.selected - 1) * 100, getFlags(9))

    lcd.drawText(50, 160, "Rate Curr", 0)
    lcd.drawText(170, 160, (config.refreshCurr.selected - 1) * 100, getFlags(10))
    lcd.drawText(290, 160, "Rate Temp", 0)
    lcd.drawText(410, 160, (config.refreshTemp.selected - 1) * 100, getFlags(11))

    lcd.drawText(50, 180, "Avg RPM", 0)
    lcd.drawText(170, 180, config.queueRpm.selected, getFlags(12))
    lcd.drawText(290, 180, "Avg Volt", 0)
    lcd.drawText(410, 180, config.queueVolt.selected, getFlags(13))

    lcd.drawText(50, 200, "Avg Curr", 0)
    lcd.drawText(170, 200, config.queueCurr.selected, getFlags(14))
    lcd.drawText(290, 200, "Avg Temp", 0)
    lcd.drawText(410, 200, config.queueTemp.selected, getFlags(15))

    lcd.drawText(50, 220, "I2C 1", 0)
    lcd.drawText(170, 220, config.i2c1.list[config.i2c1.selected], getFlags(16))
    lcd.drawText(290, 220, "Address", 0)
    lcd.drawText(410, 220, config.i2c1Address, getFlags(17))

    lcd.drawText(50, 240, "I2C 2", 0)
    lcd.drawText(170, 240, config.i2c2.list[config.i2c2.selected], getFlags(18))
    lcd.drawText(290, 240, "Address", 0)
    lcd.drawText(410, 240, config.i2c2Address, getFlags(19))

    if readConfigState < 30 then
        lcd.drawText(180, 155, "Connecting...", INVERS)
    end
    if sendConfigState < 40 then
        lcd.drawText(200, 250, "UPDATING", SMLSIZE + getFlags(20))
    else
        lcd.drawText(200, 250, "UPDATE", SMLSIZE + getFlags(20))
    end
    --if sendConfigState == 40 then lcd.drawText(250, 250, 'OK', 0)
    --elseif sendConfigState == 50 then lcd.drawText(250, 250, 'ERROR', 0) end
end

local function refreshTaranis()
    lcd.drawText(1, 9 - scroll * 8, "Firmware", SMLSIZE)
    lcd.drawText(44, 9 - scroll * 8, config.firmwareVersion, SMLSIZE)
    lcd.drawText(1, 17 - scroll * 8, "Protocol", SMLSIZE)
    lcd.drawText(44, 17 - scroll * 8, config.protocol.list[config.protocol.selected], SMLSIZE + getFlags(1))
    lcd.drawText(1, 25 - scroll * 8, "Voltage1", SMLSIZE)
    lcd.drawText(44, 25 - scroll * 8, config.voltage1.list[config.voltage1.selected], SMLSIZE + getFlags(2))
    lcd.drawText(64, 25 - scroll * 8, "Voltage2", SMLSIZE)
    lcd.drawText(108, 25 - scroll * 8, config.voltage2.list[config.voltage2.selected], SMLSIZE + getFlags(3))
    lcd.drawText(1, 33 - scroll * 8, "Ntc1", SMLSIZE)
    lcd.drawText(44, 33 - scroll * 8, config.ntc1.list[config.ntc1.selected], SMLSIZE + getFlags(4))
    lcd.drawText(64, 33 - scroll * 8, "Ntc2", SMLSIZE)
    lcd.drawText(108, 33 - scroll * 8, config.ntc2.list[config.ntc2.selected], SMLSIZE + getFlags(5))
    lcd.drawText(1, 41 - scroll * 8, "Current", SMLSIZE)
    lcd.drawText(44, 41 - scroll * 8, config.current.list[config.current.selected], SMLSIZE + getFlags(6))
    lcd.drawText(64, 41 - scroll * 8, "PWM out", SMLSIZE)
    lcd.drawText(108, 41 - scroll * 8, config.pwm.list[config.pwm.selected], SMLSIZE + getFlags(7))

    lcd.drawText(1, 49 - scroll * 8, "Rate RPM", SMLSIZE)
    lcd.drawText(44, 49 - scroll * 8, (config.refreshRpm.selected - 1) * 100, SMLSIZE + getFlags(8))
    lcd.drawText(64, 49 - scroll * 8, "Rate Volt", SMLSIZE)
    lcd.drawText(108, 49 - scroll * 8, (config.refreshVolt.selected - 1) * 100, SMLSIZE + getFlags(9))

    lcd.drawText(1, 57 - scroll * 8, "Rate Curr", SMLSIZE)
    lcd.drawText(44, 57 - scroll * 8, (config.refreshCurr.selected - 1) * 100, SMLSIZE + getFlags(10))
    lcd.drawText(64, 57 - scroll * 8, "Rate Temp", SMLSIZE)
    lcd.drawText(108, 57 - scroll * 8, (config.refreshTemp.selected - 1) * 100, SMLSIZE + getFlags(11))

    lcd.drawText(1, 65 - scroll * 8, "Avg RPM", SMLSIZE)
    lcd.drawText(44, 65 - scroll * 8, config.queueRpm.selected, SMLSIZE + getFlags(12))
    lcd.drawText(64, 65 - scroll * 8, "Avg Volt", SMLSIZE)
    lcd.drawText(108, 65 - scroll * 8, config.queueVolt.selected, SMLSIZE + getFlags(13))

    lcd.drawText(1, 73 - scroll * 8, "Avg Curr", SMLSIZE)
    lcd.drawText(44, 73 - scroll * 8, config.queueCurr.selected, SMLSIZE + getFlags(14))
    lcd.drawText(64, 73 - scroll * 8, "Avg Temp", SMLSIZE)
    lcd.drawText(108, 73 - scroll * 8, config.queueTemp.selected, SMLSIZE + getFlags(15))

    lcd.drawText(1, 81 - scroll * 8, "I2C 1", SMLSIZE)
    lcd.drawText(30, 81 - scroll * 8, config.i2c1.list[config.i2c1.selected], SMLSIZE + getFlags(16))
    lcd.drawText(64, 81 - scroll * 8, "Address", SMLSIZE)
    lcd.drawText(108, 81 - scroll * 8, config.i2c1Address.selected - 1, SMLSIZE + getFlags(17))

    lcd.drawText(1, 89 - scroll * 8, "I2C 2", SMLSIZE)
    lcd.drawText(30, 89 - scroll * 8, config.i2c2.list[config.i2c2.selected], SMLSIZE + getFlags(18))
    lcd.drawText(64, 89 - scroll * 8, "Address", SMLSIZE)
    lcd.drawText(108, 89 - scroll * 8, config.i2c2Address.selected - 1, SMLSIZE + getFlags(19))

    if readConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(35, 28, "Connecting...", INVERS)
    end

    if sendConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(1, 97 - scroll * 8, "UPDATING", SMLSIZE + getFlags(20))
    else
        lcd.drawText(1, 97 - scroll * 8, "UPDATE", SMLSIZE + getFlags(20))
    end
    lcd.drawScreenTitle("MSRC v" .. scriptVersion, 1, 1)
end

local function init_func()
end

local function bg_func(event)
    if refresh < 5 then
        refresh = refresh + 1
    end
end

local function run_func(event)
    -- update lcd
    if
        refresh == 5 or lcdChange == true or selection.state == true or
            string.find(selection.list[selection.selected], "btn")
     then
        lcd.clear()
        lcdChange = false
        if LCD_W == 480 then
            refreshHorus()
        else
            refreshTaranis()
        end
    end

    -- update state
    if readConfigState ~= state["MAINTENANCE_OFF"] then
        readConfig()
    end
    if sendConfigState ~= state["MAINTENANCE_OFF"] then
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
        if string.find(selection.list[selection.selected], "btn") ~= 1 then
            if readConfigState == state["MAINTENANCE_OFF"] and sendConfigState == state["MAINTENANCE_OFF"] then
                selection.state = not selection.state
                lcdChange = true
            end
        else
            if readConfigState == state["MAINTENANCE_OFF"] and sendConfigState == state["MAINTENANCE_OFF"] then
                sendConfigState = state["INIT"]
            end
        end
    end
    if event == EVT_EXIT_BREAK then
        selection.state = false
        lcdChange = true
    end

    --lcd scrolling
    scroll = 0
    if selection.selected > 11 then
        scroll = 1
    end
    if selection.selected > 13 then
        scroll = 2
    end
    if selection.selected > 15 then
        scroll = 3
    end
    if selection.selected > 17 then
        scroll = 4
    end
    if selection.selected > 19 then
        scroll = 5
    end

    refresh = 0

    return 0
end

return {run = run_func, background = bg_func, init = init_func}
