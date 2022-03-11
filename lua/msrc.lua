--
-- Multi Sensor RC - MSRC
--
-- License https://www.gnu.org/licenses/gpl-3.0.en.html
--

local scriptVersion = "0.9"
local dataIdSensor = 0x5000
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
local lcdChange = true
local scroll = 0
local sensorIdTx = 17 -- sensorId 18
local config = {
    firmwareVersion = "",
    protocol = {
        selected = 9,
        list = {"NONE", "HW V3", "HW V4", "PWM", "CASTLE", "KONTRONIK", "APD F", "APD HV", ""},
        elements = 8
    },
    voltage1 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    voltage2 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    ntc1 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    ntc2 = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    current = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    airspeed = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    pwm = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    gps = {selected = 3, list = {"Off", "On", ""}, elements = 2},
    refreshRpm = {selected = 1, elements = 16},
    refreshVolt = {selected = 1, elements = 16},
    refreshCurr = {selected = 1, elements = 16},
    refreshTemp = {selected = 1, elements = 16},
    queueRpm = {selected = 1, elements = 16},
    queueVolt = {selected = 1, elements = 16},
    queueCurr = {selected = 1, elements = 16},
    queueTemp = {selected = 1, elements = 16},
    i2c1 = {selected = 4, list = {"NONE", "BMP280", "MS5611", ""}, elements = 3},
    i2c1Address = {selected = 1, elements = 128},
    i2c2 = {selected = 4, list = {"NONE", "BMP280", "MS5611", ""}, elements = 3},
    i2c2Address = {selected = 1, elements = 128}
}
local selection = {
    selected = 1,
    elements = 22,
    state = false,
    list = {
        "protocol",
        "voltage1",
        "voltage2",
        "ntc1",
        "ntc2",
        "current",
        "airspeed",
        "pwm",
        "gps",
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
    if selection.selected ~= element and selection.list[element] ~= "btnUpdate" then
        return 0
    end
    if selection.list[element] == "btnUpdate" and selection.selected == element then
        return INVERS + BLINK
    end
    if selection.list[element] == "btnUpdate" and selection.selected ~= element then
        return INVERS
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
            if sportTelemetryPush(sensorIdTx, 0x30, dataIdSensor, 0) then
                readConfigState = state["CONFIG_REQUESTED"]
            end
        elseif readConfigState == state["PACKET_4"] then
            if sportTelemetryPush(sensorIdTx, 0x20, 0xFFFF, 0x80) then
                readConfigState = state["MAINTENANCE_OFF"]
                lcdChange = true
            end
        end
        local physicalId, primId, dataId, value = sportTelemetryPop()
        if primId == 0x32 and dataId == dataIdSensor then
            if bit32.extract(value, 0, 8) == 0xF1 and readConfigState == state["CONFIG_REQUESTED"] then
                config.firmwareVersion =
                    bit32.extract(value, 24, 8) ..
                    "." .. bit32.extract(value, 16, 8) .. "." .. bit32.extract(value, 8, 8)
                readConfigState = state["PACKET_1"]
            end
            if bit32.extract(value, 0, 8) == 0xF2 and readConfigState == state["PACKET_1"] then
                if bit32.extract(value, 8) + 1 >= 1 and bit32.extract(value, 8) + 1 <= 2 then
                    config.airspeed.selected = bit32.extract(value, 8) + 1 -- bit 9
                end
                if bit32.extract(value, 9) + 1 >= 1 and bit32.extract(value, 9) + 1 <= 2 then
                    config.gps.selected = bit32.extract(value, 9) + 1 -- bit 10
                end
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
                if bit32.extract(value, 24, 8) >= 0 and bit32.extract(value, 24, 8) <= 7 then
                    config.protocol.selected = bit32.extract(value, 24, 8) + 1 -- bits 25-32
                end
                readConfigState = state["PACKET_3"]
            end
            if bit32.extract(value, 0, 8) == 0xF4 and readConfigState == state["PACKET_3"] then
                if bit32.extract(value, 8, 4) >= 0 and bit32.extract(value, 8, 4) <= 3 then
                    config.i2c1.selected = bit32.extract(value, 8, 4) + 1 -- bits 9-12
                end
                if bit32.extract(value, 12, 8) >= 0 and bit32.extract(value, 12, 8) <= 127 then
                    config.i2c1Address.selected = bit32.extract(value, 12, 8) + 1 -- bits 13-20
                end
                if bit32.extract(value, 20, 4) >= 0 and bit32.extract(value, 20, 4) <= 3 then
                    config.i2c2.selected = bit32.extract(value, 20, 4) + 1 -- bits 21-24
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
            value = bit32.bor(value, bit32.lshift(config.airspeed.selected - 1, 8)) -- bit 9
            value = bit32.bor(value, bit32.lshift(config.gps.selected - 1, 9)) -- bit 10
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
            if sportTelemetryPush(sensorIdTx, 0x31, dataIdSensor, value) then
                sendConfigState = state["PACKET_1"]
            end
        elseif sendConfigState == state["PACKET_1"] then
            local value = 0xF2 -- bits 1-8
            value = bit32.bor(value, bit32.lshift(config.queueRpm.selected, 8)) -- bits 9-12
            value = bit32.bor(value, bit32.lshift(config.queueVolt.selected, 12)) -- bits 13-16
            value = bit32.bor(value, bit32.lshift(config.queueCurr.selected, 16)) -- bits 17-20
            value = bit32.bor(value, bit32.lshift(config.queueTemp.selected, 20)) -- bits 21-24
            value = bit32.bor(value, bit32.lshift(config.protocol.selected - 1, 24)) -- bits 25-32
            if sportTelemetryPush(sensorIdTx, 0x31, dataIdSensor, value) then
                sendConfigState = state["PACKET_2"]
            end
        elseif sendConfigState == state["PACKET_2"] then
            local value = 0xF3 -- bits 1-8
            value = bit32.bor(value, bit32.lshift(config.i2c1.selected - 1, 8)) -- bits 9-12
            value = bit32.bor(value, bit32.lshift(config.i2c1Address.selected - 1, 12)) -- bits 13-20
            value = bit32.bor(value, bit32.lshift(config.i2c2.selected - 1, 20)) -- bits 21-24
            value = bit32.bor(value, bit32.lshift(config.i2c2Address.selected - 1, 24)) -- bits 25-32
            if sportTelemetryPush(sensorIdTx, 0x31, dataIdSensor, value) then
                sendConfigState = state["PACKET_3"]
            end
        elseif sendConfigState == state["PACKET_3"] then
            local physicalId, primId, dataId, value = sportTelemetryPop()
            if primId == 0x32 and dataId == dataIdSensor and value == 0xFF then
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
    lcd.clear()
    lcd.drawRectangle(25, 23, 430, 231)
    lcd.drawText(200, 1, "MSRC v" .. scriptVersion, INVERS)

    lcd.drawText(40, 24, "Firmware", 0)
    lcd.drawText(170, 24, config.firmwareVersion, 0)

    lcd.drawText(40, 43, "Protocol", 0)
    lcd.drawText(170, 43, config.protocol.list[config.protocol.selected], getFlags(1))

    lcd.drawText(40, 62, "Voltage1", 0)
    lcd.drawText(170, 62, config.voltage1.list[config.voltage1.selected], getFlags(2))
    lcd.drawText(290, 62, "Voltage2", 0)
    lcd.drawText(410, 62, config.voltage2.list[config.voltage2.selected], getFlags(3))

    lcd.drawText(40, 81, "Ntc1", 0)
    lcd.drawText(170, 81, config.ntc1.list[config.ntc1.selected], getFlags(4))
    lcd.drawText(290, 81, "Ntc2", 0)
    lcd.drawText(410, 81, config.ntc2.list[config.ntc2.selected], getFlags(5))

    lcd.drawText(40, 100, "Current", 0)
    lcd.drawText(170, 100, config.current.list[config.current.selected], getFlags(6))
    lcd.drawText(290, 100, "AirSpeed", 0)
    lcd.drawText(410, 100, config.airspeed.list[config.airspeed.selected], getFlags(7))

    lcd.drawText(40, 119, "PWM out", 0)
    lcd.drawText(170, 119, config.pwm.list[config.pwm.selected], getFlags(8))
    lcd.drawText(290, 119, "GPS", 0)
    lcd.drawText(410, 119, config.gps.list[config.gps.selected], getFlags(9))

    lcd.drawText(40, 138, "Rate RPM", 0)
    lcd.drawText(170, 138, (config.refreshRpm.selected - 1) * 100, getFlags(10))
    lcd.drawText(290, 138, "Rate Volt", 0)
    lcd.drawText(410, 138, (config.refreshVolt.selected - 1) * 100, getFlags(11))

    lcd.drawText(40, 157, "Rate Curr", 0)
    lcd.drawText(170, 157, (config.refreshCurr.selected - 1) * 100, getFlags(12))
    lcd.drawText(290, 157, "Rate Temp", 0)
    lcd.drawText(410, 157, (config.refreshTemp.selected - 1) * 100, getFlags(13))

    lcd.drawText(40, 176, "Avg RPM", 0)
    lcd.drawText(170, 176, config.queueRpm.selected, getFlags(14))
    lcd.drawText(290, 176, "Avg Volt", 0)
    lcd.drawText(410, 176, config.queueVolt.selected, getFlags(15))

    lcd.drawText(40, 195, "Avg Curr", 0)
    lcd.drawText(170, 195, config.queueCurr.selected, getFlags(16))
    lcd.drawText(290, 195, "Avg Temp", 0)
    lcd.drawText(410, 195, config.queueTemp.selected, getFlags(17))

    lcd.drawText(40, 214, "I2C 1", 0)
    lcd.drawText(170, 214, config.i2c1.list[config.i2c1.selected], getFlags(18))
    lcd.drawText(290, 214, "Address", 0)
    lcd.drawText(410, 214, config.i2c1Address.selected - 1, getFlags(19))

    lcd.drawText(40, 233, "I2C 2", 0)
    lcd.drawText(170, 233, config.i2c2.list[config.i2c2.selected], getFlags(20))
    lcd.drawText(290, 233, "Address", 0)
    lcd.drawText(410, 233, config.i2c2Address.selected - 1, getFlags(21))

    if readConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(180, 155, "Connecting...", INVERS)
    end
    if sendConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(200, 253, "UPDATING", getFlags(22))
    else
        lcd.drawText(200, 253, "UPDATE", getFlags(22))
    end
end

local function refreshTaranis()
    lcdChange = false
    lcd.clear()
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
    lcd.drawText(64, 41 - scroll * 8, "AirSpeed", SMLSIZE)
    lcd.drawText(108, 41 - scroll * 8, config.airspeed.list[config.airspeed.selected], SMLSIZE + getFlags(7))

    lcd.drawText(1, 49 - scroll * 8, "PWM out", SMLSIZE)
    lcd.drawText(44, 49 - scroll * 8, config.pwm.list[config.pwm.selected], SMLSIZE + getFlags(8))
    lcd.drawText(64, 49 - scroll * 8, "GPS", SMLSIZE)
    lcd.drawText(108, 49 - scroll * 8, config.gps.list[config.gps.selected], SMLSIZE + getFlags(9))

    lcd.drawText(1, 57 - scroll * 8, "Rate RPM", SMLSIZE)
    lcd.drawText(44, 57 - scroll * 8, (config.refreshRpm.selected - 1) * 100, SMLSIZE + getFlags(10))
    lcd.drawText(64, 57 - scroll * 8, "Rate Volt", SMLSIZE)
    lcd.drawText(108, 57 - scroll * 8, (config.refreshVolt.selected - 1) * 100, SMLSIZE + getFlags(11))

    lcd.drawText(1, 65 - scroll * 8, "Rate Curr", SMLSIZE)
    lcd.drawText(44, 65 - scroll * 8, (config.refreshCurr.selected - 1) * 100, SMLSIZE + getFlags(12))
    lcd.drawText(64, 65 - scroll * 8, "Rate Temp", SMLSIZE)
    lcd.drawText(108, 65 - scroll * 8, (config.refreshTemp.selected - 1) * 100, SMLSIZE + getFlags(13))

    lcd.drawText(1, 73 - scroll * 8, "Avg RPM", SMLSIZE)
    lcd.drawText(44, 73 - scroll * 8, config.queueRpm.selected, SMLSIZE + getFlags(14))
    lcd.drawText(64, 73 - scroll * 8, "Avg Volt", SMLSIZE)
    lcd.drawText(108, 73 - scroll * 8, config.queueVolt.selected, SMLSIZE + getFlags(15))

    lcd.drawText(1, 81 - scroll * 8, "Avg Curr", SMLSIZE)
    lcd.drawText(44, 81 - scroll * 8, config.queueCurr.selected, SMLSIZE + getFlags(16))
    lcd.drawText(64, 81 - scroll * 8, "Avg Temp", SMLSIZE)
    lcd.drawText(108, 81 - scroll * 8, config.queueTemp.selected, SMLSIZE + getFlags(17))

    lcd.drawText(1, 89 - scroll * 8, "I2C 1", SMLSIZE)
    lcd.drawText(30, 89 - scroll * 8, config.i2c1.list[config.i2c1.selected], SMLSIZE + getFlags(18))
    lcd.drawText(64, 89 - scroll * 8, "Address", SMLSIZE)
    lcd.drawText(108, 89 - scroll * 8, config.i2c1Address.selected - 1, SMLSIZE + getFlags(19))

    lcd.drawText(1, 97 - scroll * 8, "I2C 2", SMLSIZE)
    lcd.drawText(30, 97 - scroll * 8, config.i2c2.list[config.i2c2.selected], SMLSIZE + getFlags(20))
    lcd.drawText(64, 97 - scroll * 8, "Address", SMLSIZE)
    lcd.drawText(108, 97 - scroll * 8, config.i2c2Address.selected - 1, SMLSIZE + getFlags(21))

    if readConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(35, 28, "Connecting...", INVERS)
    end

    if sendConfigState ~= state["MAINTENANCE_OFF"] then
        lcd.drawText(1, 105 - scroll * 8, "UPDATING", SMLSIZE + getFlags(22))
    else
        lcd.drawText(1, 105 - scroll * 8, "UPDATE", SMLSIZE + getFlags(22))
    end
    lcd.drawScreenTitle("MSRC v" .. scriptVersion, 1, 1)
end

local function run_func(event)
    if event == nil then
        error("Cannot run as a model script!")
        return 2
    end

    -- update lcd
    if LCD_W == 480 then
        refreshHorus()
    elseif lcdChange == true or selection.state == true or selection.list[selection.selected] == "btnUpdate" then
        refreshTaranis()
    end

    -- update state
    if readConfigState ~= state["MAINTENANCE_OFF"] then
        readConfig()
    end
    if sendConfigState ~= state["MAINTENANCE_OFF"] then
        sendConfig()
    end

    -- key events (left = up/decrease right = down/increase)
    if event == EVT_ROT_LEFT or event == EVT_MINUS_BREAK or event == EVT_DOWN_BREAK then
        if selection.state == false then
            decrease(selection)
            lcdChange = true
        else
            decrease(config[selection.list[selection.selected]])
            lcdChange = true
        end
    elseif event == EVT_ROT_RIGHT or event == EVT_PLUS_BREAK or event == EVT_UP_BREAK then
        if selection.state == false then
            increase(selection)
            lcdChange = true
        else
            increase(config[selection.list[selection.selected]])
            lcdChange = true
        end
    elseif event == EVT_ENTER_BREAK then
        if selection.list[selection.selected] ~= "btnUpdate" then
            if readConfigState == state["MAINTENANCE_OFF"] and sendConfigState == state["MAINTENANCE_OFF"] then
                selection.state = not selection.state
                lcdChange = true
            end
        else
            if readConfigState == state["MAINTENANCE_OFF"] and sendConfigState == state["MAINTENANCE_OFF"] then
                sendConfigState = state["INIT"]
            end
        end
    elseif event == EVT_EXIT_BREAK then
        selection.state = false
        lcdChange = true
    end

    --lcd scrolling
    if LCD_W ~= 480 then
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
        if selection.selected > 21 then
            scroll = 6
        end
    end

    return 0
end

return {run = run_func}
