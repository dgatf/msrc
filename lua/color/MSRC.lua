--[[
radio frameId: set = 0x33, get = 0x34
msrc frameId: 0x32. dataId DIY: 5100 - 52FF (to get into lua environment)

cmd        | sender | frameId | dataId  | value (4B)    | sensorId
-----------|--------|---------|---------|---------------|---------
set var    | radio  | 0x33    | 0x51nn  | value         |
get var    | radio  | 0x34    | 0x51nn  | 0             |
send var   | msrc   | 0x32    | 0x51nn  | value         |
ack        | msrc   | 0x32    | 0x5201  | ack=1, nack=0 |
ack        | radio  | 0x33    | 0x5201  | ack=1, nack=0 |
sensorid n | radio  | 0x35    | 0x5200  | 0             | 1 - 28 -> ack
start save | radio  | 0x35    | 0x5201  | 0             |
end save   | radio  | 0x35    | 0x5201  | 1             |
]]--

local scriptVersion = "v1.0"
local firmwareVersion
local page = 0
local pageLong = false
local pagePos = 1
local isSelected = false
local status = "searchSensorId"
local exit = false
local saveChanges = true
local tsRequest = 0
local newValue = true
local sensorIdIndex = 1
local newValueConfig = true
local posConfig = 1

local onOffStr = { "Off", "On" }

local pageName = {
	"Sensor Id",
	"Refresh rate",
	"Average elements",
	"ESC",
	"GPS",
	"Vario",
	"Fuel meter",
	"GPIO",
	"Analog rate",
	"Temperature analog",
	"Voltage analog",
	"Current analog",
	"Airspeed analog",
}

-- Page 1 - SensorId
--                 str, val, min, max, incr, dataId
local sensorId = { "Sensor Id", 1, 1, 28, 1, 0x5128 }
local page_sensorId = { sensorId }

-- Page 2 - Refresh interval (ms 1-2000)
local rateRpm = { "RPM", nil, 1, 2000, 1, 0x5112 }
local rateVolt = { "Voltage", nil, 1, 2000, 1, 0x5113 }
local rateCurr = { "Current", nil, 1, 2000, 1, 0x5114 }
local rateTemp = { "Temperature", nil, 1, 2000, 1, 0x5115 }
local rateGps = { "GPS", nil, 1, 2000, 1, 0x5116 }
local rateCons = { "Consumption", nil, 1, 2000, 1, 0x5117 }
local rateVario = { "Vario", nil, 1, 2000, 1, 0x5118 }
local rateAirspeed = { "Airspeed", nil, 1, 2000, 1, 0x5119 }

local page_rate = { rateRpm, rateVolt, rateCurr, rateTemp, rateGps, rateCons, rateVario, rateAirspeed }

-- Page 3 - Averaging elements (1-16)
local avgRpm = { "RPM", nil, 1, 16, 1, 0x510C }
local avgVolt = { "Voltage", nil, 1, 16, 1, 0x510D }
local avgCurr = { "Current", nil, 1, 16, 1, 0x510E }
local avgTemp = { "Temperature", nil, 1, 16, 1, 0x510F }
local avgVario = { "Vario", nil, 1, 16, 1, 0x5110 }
local avgAirspeed = { "Airspeed", nil, 1, 16, 1, 0x5111 }

local page_avg = { avgRpm, avgVolt, avgCurr, avgTemp, avgVario, avgAirspeed }

-- Page 4 - ESC
local pairPoles = { "Pair of Poles", nil, 1, 16, 1, 0x5122 }
local mainGear = { "Main Gear", nil, 1, 16, 1, 0x5123 }
local pinionGear = { "Pinion Gear", nil, 1, 16, 1, 0x5124 }
local escProtocolStr = {
    "None",
	"Hobbywing V3",
	"Hobbywing V4",
	"PWM",
	"Castle Link",
	"Kontronic",
	"Kiss",
	"APD HV",
	"HobbyWing V5",
	"Smart ESC/BAT",
	"OMP M4",
	"ZTW"
}
local escProtocol = { "Protocol", nil, 0, 11, 1, 0x5103 }
local hw4Thresold = { "Thr thresold", nil, 1, 100, 1, 0x512F }
local hw4VoltDiv = { "Volt divisor", nil, 1, 1000, 0.1, 0x5131 }
local hw4CurrMult = { "Curr mult", nil, 1, 1000, 0.1, 0x5132 }
local hw4MaxCurr = { "Max curr", nil, 1, 1000, 1, 0x5130 }
local hw4InitDelay = { "Init Delay", nil, 1, 2, 1, 0x512E }
local hw4AutoOffset = { "Auto offset", nil, 1, 2, 1, 0x5135 }
local smartEscConsumption = { "Calc cons", nil, 0, 1, 1, 0x5145 }

local page_esc = { pairPoles, mainGear, pinionGear, escProtocol, hw4Thresold, hw4VoltDiv, hw4CurrMult, hw4MaxCurr, smartEscConsumption }

-- Page 5 - GPS
local gpsEnable = { "Enable", nil, 0, 1, 1, 0x5104 }
local gpsProtocolStr = { "NMEA", "UBLOX" }
local gpsProtocol = { "Protocol", nil, 0, 1, 1, 0x5149 }
local gpsBaudrateVal = { 115200, 57600, 38400, 9600 }
local gpsBaudrate = { "Baudrate", nil, 0, 3, 1, 0x5105 }
local gpsRateVal = { 1, 5, 10, 20 }
local gpsRate = { "Rate", nil, 0, 3, 1, 0x5147 }

local page_gps = { gpsEnable, gpsProtocol, gpsBaudrate, gpsRate }

-- Page 6 - Vario
local varioModelStr = { "None", "BMP280", "MS5611", "BMP180" }
local varioModel = { "Model", nil, 0, 3, 1, 0x510A }
local varioAddress = { "Address", nil, 0x76, 0x77, 1, 0x510B }
local varioFilterStr = { "Low", "Medium", "High" }
local varioFilter = { "Filter", nil, 0, 1, 1, 0x5126 }

local page_vario = { varioModel, varioAddress, varioFilter }

-- Page 7 - Fuel meter
local fuelMeter = { "Enable", nil, 0, 1, 1, 0x5142 }
local mlPulse = { "ml/pulse", nil, 1, 100, 0.1, 0x5141 }

local page_fuelmeter = { fuelMeter, mlPulse }

-- Page 8 - GPIO
local gpioInterval = { "Interval(ms)", nil, 10, 10000, 1, 0x511D }
local gpio17 = { "17", nil, 0, 1, 1, 0x5138 }
local gpio18 = { "18", 0, 0, 1, 1, 0 }
local gpio19 = { "19", 0, 0, 1, 1, 0 }
local gpio20 = { "20", 0, 0, 1, 1, 0 }
local gpio21 = { "21", 0, 0, 1, 1, 0 }
local gpio22 = { "22", 0, 0, 1, 1, 0 }

local page_gpio = { gpioInterval, gpio17, gpio18, gpio19, gpio20, gpio21, gpio22 }

-- Page 9 - Analog rate
local analogRate = { "Rate(Hz)", nil, 1, 100, 1, 0x5136 }

local page_analogRate = { analogRate }

-- Page 10 - Temperature analog
local analogTemp = { "Enable", nil, 0, 1, 1, 0x5108 }

local page_analogTemp = { analogTemp }

-- Page 11 - Voltage analog
local analogVolt = { "Enable", nil, 0, 1, 1, 0x5106 }
local analogVoltMult = { "Multiplier", nil, 1, 1000, 0.1, 0x511B }

local page_analogVolt = { analogVolt, analogVoltMult }

-- Page 12 - Current analog
local analogCurr = { "Enable", nil, 0, 1, 1, 0x5107 }
local analogCurrType = { "Type", nil, 0, 1, 1, 0x512D }
local analogCurrTypeStr = { "Hall Effect", "Shunt Resistor" }
local analogCurrMult = { "Mult", nil, 0, 100, 0.1, 0x512E }
local analogCurrSens = { "Sens(mV/A)", 0, 0, 100, 0.1, 0 }
local analogCurrAutoOffset = { "Auto Offset", nil, 0, 1, 1, 0x5121 }
local analogCurrOffset  = { "Offset", nil, 0, 99, 0.1, 0x5120 }

local page_analogCurr = { analogCurr, analogCurrType, analogCurrMult, analogCurrAutoOffset, analogCurrOffset }

-- Page 13 - Airspeed analog
local analogAirspeed = { "Enable", nil, 0, 1, 1, 0x5109 }
local analogAirspeedSlope = { "Slope", nil, -2, 2, 0.1, 0x5140 }
local analogAirspeedOffset = { "Offset", nil, -1, 1, 0.1, 0x513F }

local page_analogAirspeed = { analogAirspeed, analogAirspeedSlope, analogAirspeedOffset }

local vars = {
	page_sensorId,
	page_rate,
	page_avg,
	page_esc,
	page_gps,
	page_vario,
	page_fuelmeter,
	page_gpio,
	page_analogRate,
	page_analogTemp,
	page_analogVolt,
	page_analogCurr,
	page_analogAirspeed,
}

local function getTextFlags(itemPos)
	local value = 0
	if itemPos == pagePos then
		value = INVERS
		if isSelected == true then
			value = value + BLINK
		end
	end
	return value
end

local function getValue(isIncremented)
	if isIncremented == true and vars[page][pagePos][2] < vars[page][pagePos][4] then
		vars[page][pagePos][2] = vars[page][pagePos][2] + vars[page][pagePos][5]
	end
	if isIncremented == false and vars[page][pagePos][2] > vars[page][pagePos][3] then
		vars[page][pagePos][2] = vars[page][pagePos][2] - vars[page][pagePos][5]
	end
end

local function getString(strList, value)
    if value > #strList then 
        return strList[1]
    else
        return strList[value]
    end
end

local function drawTitle(str, page, pages)
    lcd.drawText(1, 1 ,str)
    if page ~= 0 and pages ~= 0 then
        lcd.drawText(200, 1 ,page .. "/" .. pages)
    end
end

local function drawPage()
    lcd.clear()
    if exit == true then
        drawTitle("Exit", 0, 0)
		lcd.drawText(1, 20, "Save changes?", 0)
		lcd.drawText(1, 60, "Reboot MSRC to apply changes", 0)
        if saveChanges == true then
            lcd.drawText(1, 40, "Yes", INVERS)
		    lcd.drawText(200, 40, "Cancel", 0)
        else
            lcd.drawText(1, 40, "Yes", 0 )
		    lcd.drawText(200, 40, "Cancel", INVERS)
        end
        return
    end
    drawTitle(pageName[page], page, #vars)
	if pagePos > #vars[page] then
		pagePos = 1
	end
	if pagePos < 1 then
		pagePos = #vars[page]
	end
	-- 1 Connection
	if page == 1 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
	-- 2 Refresh rate
	elseif page == 2 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
		lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
		lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
		lcd.drawText(200, 65, vars[page][4][2], SMLSIZE + getTextFlags(4))
		lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
		lcd.drawText(200, 80, vars[page][5][2], SMLSIZE + getTextFlags(5))
		lcd.drawText(1, 95, vars[page][6][1], SMLSIZE)
		lcd.drawText(200, 95, vars[page][6][2], SMLSIZE + getTextFlags(6))
		lcd.drawText(1, 110, vars[page][7][1], SMLSIZE)
		lcd.drawText(200, 110, vars[page][7][2], SMLSIZE + getTextFlags(7))
		lcd.drawText(1, 125, vars[page][8][1], SMLSIZE)
		lcd.drawText(200, 125, vars[page][8][2], SMLSIZE + getTextFlags(8))
	-- 3 Average elements
	elseif page == 3 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
		lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
		lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
		lcd.drawText(200, 65, vars[page][4][2], SMLSIZE + getTextFlags(4))
		lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
		lcd.drawText(200, 80, vars[page][5][2], SMLSIZE + getTextFlags(5))
		lcd.drawText(1, 95, vars[page][6][1], SMLSIZE)
		lcd.drawText(200, 95, vars[page][6][2], SMLSIZE + getTextFlags(6))
	-- 4 ESC
	elseif page == 4 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
		lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
		lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
		lcd.drawText(200, 65, getString(escProtocolStr, vars[page][4][2] + 1), SMLSIZE + getTextFlags(4))
		if escProtocolStr[vars[page][4][2] + 1] == "Hobbywing V4" then
            vars[page] = { pairPoles, mainGear, pinionGear, escProtocol, hw4Thresold, hw4VoltDiv, hw4CurrMult, hw4MaxCurr }
			lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
			lcd.drawText(200, 80, vars[page][5][2], SMLSIZE + getTextFlags(5))
			lcd.drawText(1, 95, vars[page][6][1], SMLSIZE)
			lcd.drawText(200, 95, vars[page][6][2], SMLSIZE + getTextFlags(6))
			lcd.drawText(1, 110, vars[page][7][1], SMLSIZE)
			lcd.drawText(200, 110, vars[page][7][2], SMLSIZE + getTextFlags(7))
			lcd.drawText(1, 125, vars[page][8][1], SMLSIZE)
			lcd.drawText(200, 125, vars[page][8][2], SMLSIZE + getTextFlags(8))
        elseif escProtocolStr[vars[page][4][2] + 1] == "Smart ESC/BAT" then
            vars[page] = { pairPoles, mainGear, pinionGear, escProtocol, smartEscConsumption }
            lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
			lcd.drawText(200, 80, getString(onOffStr, vars[page][5][2] + 1), SMLSIZE + getTextFlags(5))
		else
            vars[page] = { pairPoles, mainGear, pinionGear, escProtocol }
        end
	-- 5 GPS
	elseif page == 5 then
        lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
        lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, getString(gpsProtocolStr, vars[page][2][2] + 1), SMLSIZE + getTextFlags(2))
        lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, getString(gpsBaudrateVal, vars[page][3][2] + 1), SMLSIZE + getTextFlags(3))
        lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
		lcd.drawText(200, 65, getString(gpsRateVal, vars[page][4][2] + 1), SMLSIZE + getTextFlags(4))
	-- 6 Vario
	elseif page == 6 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(varioModelStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
		if varioModelStr[vars[page][1][2] + 1] == "BMP280" then
            vars[page] = { varioModel, varioAddress, varioFilter }
			lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
			lcd.drawText(200, 50, getString(varioFilterStr, vars[page][3][2]), SMLSIZE + getTextFlags(3))
		else
            vars[page] = { varioModel, varioAddress }
		end
	-- 7 Fuel meter
	elseif page == 7 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
        lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
	-- 8 GPIO
	elseif page == 8 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, getString(onOffStr, vars[page][2][2] + 1), SMLSIZE + getTextFlags(2))
		lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, getString(onOffStr, vars[page][3][2] + 1), SMLSIZE + getTextFlags(3))
		lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
		lcd.drawText(200, 65, getString(onOffStr, vars[page][4][2] + 1), SMLSIZE + getTextFlags(4))
		lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
		lcd.drawText(200, 80, getString(onOffStr, vars[page][5][2] + 1), SMLSIZE + getTextFlags(5))
		lcd.drawText(1, 95, vars[page][6][1], SMLSIZE)
		lcd.drawText(200, 95, getString(onOffStr, vars[page][6][2] + 1), SMLSIZE + getTextFlags(6))
		lcd.drawText(1, 110, vars[page][7][1], SMLSIZE)
		lcd.drawText(200, 110, getString(onOffStr, vars[page][7][2] + 1), SMLSIZE + getTextFlags(7))
	-- 9 Analog rate
	elseif page == 9 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, vars[page][1][2], SMLSIZE + getTextFlags(1))
	-- 10 Analog temperature
	elseif page == 10 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
	-- 11 Analog voltage
	elseif page == 11 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
	-- 12 Analog current
	elseif page == 12 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, getString(analogCurrTypeStr, vars[page][2][2] + 1), SMLSIZE + getTextFlags(2))
		if analogCurrTypeStr[vars[page][2][2] + 1] == "Hall Effect" then
			vars[page] = { analogCurr, analogCurrType, analogCurrSens, analogCurrAutoOffset }
            if getString(onOffStr, vars[page][4][2] + 1) == "On" then
                lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
                lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
                lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
                lcd.drawText(200, 65, getString(onOffStr, vars[page][4][2] + 1), SMLSIZE + getTextFlags(4))
            else
			    vars[page] = { analogCurr, analogCurrType, analogCurrSens, analogCurrAutoOffset, analogCurrOffset }
                lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
                lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
                lcd.drawText(1, 65, vars[page][4][1], SMLSIZE)
                lcd.drawText(200, 65, getString(onOffStr, vars[page][4][2] + 1), SMLSIZE + getTextFlags(4))
                lcd.drawText(1, 80, vars[page][5][1], SMLSIZE)
                lcd.drawText(200, 80, vars[page][5][2], SMLSIZE + getTextFlags(5))
            end
		else
			vars[page] = { analogCurr, analogCurrType, analogCurrMult }
			lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
			lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
		end
	-- 13 Analog airspeed
	elseif page == 13 then
		lcd.drawText(1, 20, vars[page][1][1], SMLSIZE)
		lcd.drawText(200, 20, getString(onOffStr, vars[page][1][2] + 1), SMLSIZE + getTextFlags(1))
		lcd.drawText(1, 35, vars[page][2][1], SMLSIZE)
		lcd.drawText(200, 35, vars[page][2][2], SMLSIZE + getTextFlags(2))
		lcd.drawText(1, 50, vars[page][3][1], SMLSIZE)
		lcd.drawText(200, 50, vars[page][3][2], SMLSIZE + getTextFlags(3))
	end
end

local function handleEvents(event)
	-- Check one-time script
	if event == nil then
		error("Cannot run as a model script!")
		return 2
	end
	-- Handle events
	if event == EVT_EXIT_BREAK then
		if isSelected == true then
			isSelected = false
		else
		    exit = true
		end
	elseif event == EVT_PAGE_BREAK and exit == false then
		if pageLong then
			page = page - 1
		else
			page = page + 1
		end
		if page > #vars then
			page = 1
		elseif page < 1 then
			page = #vars
		end
		pageLong = false
		pagePos = 1
		isSelected = false
        status = "getConfig"
        posConfig = 1
	elseif event == EVT_PAGE_LONG then
		pageLong = true
	elseif event == EVT_ROT_RIGHT then
        if exit == true then
            saveChanges = not saveChanges
        elseif isSelected == false then
			pagePos = pagePos + 1
			if pagePos > #vars[page] then
				pagePos = 1
			end
		else
			getValue(true)
		end
	elseif event == EVT_ROT_LEFT then
        if exit == true then
            saveChanges = not saveChanges
            return
        end
		if isSelected == false then
			pagePos = pagePos - 1
			if pagePos < 1 then
				pagePos = #vars[page]
			end
		else
			getValue(false)
		end
	elseif event == EVT_ROT_BREAK then
        if exit == true then
            if saveChanges == true then
                status = "saveConfig"
                page = 1
                posConfig = 1
            else
                exit = false
                saveChanges = true
            end
        else
            isSelected = not isSelected
        end
	end
	if page > 0 and status == "config" then
		drawPage()
	end
end

local function searchSensorId()
	local sensorIdFound, frameId, dataId, value = sportTelemetryPop()
	if frameId == 0x32 then
		sensorId[2] = sensorIdFound + 1
        page = 1
		status = "getConfig"
		lcd.clear()
        drawTitle("MSRC "  .. scriptVersion, 0, 0)
        lcd.drawText(1, 20, "Found MSRC at sensorId  " .. sensorId[2], SMLSIZE)
		return
	end
	if sensorIdIndex <= 28 then
		if sportTelemetryPush(sensorIdIndex - 1, 0x35, 0x5200, 0) then
			lcd.clear()
            drawTitle("MSRC "  .. scriptVersion, 0, 0)
            lcd.drawText(1, 20, "Search MSRC at sensorId " .. sensorIdIndex, SMLSIZE)
			sensorIdIndex = sensorIdIndex + 1
		end
	end
end

local function getConfig()
	local sensor, frameId, dataId, value = sportTelemetryPop()
    if dataId ~= nil then
        lcd.clear()
        drawTitle(pageName[page], page, #vars)
		lcd.drawText(60, 30, posConfig .. "/" .. #vars[page], 0)
        if dataId == 0x5101 then
            firmwareVersion = "v" .. bit32.rshift(value, 16) .. "." .. bit32.band(bit32.rshift(value, 8), 0xF) .. "." .. bit32.band(value, 0xF) 
        elseif dataId == 0x5131 or dataId == 0x5132 or dataId == 0x513F or dataId == 0x5140 or dataId == 0x511B or dataId == 0x5120 then
            vars[page][posConfig][2] = value / 100
        elseif dataId == 0x5141 then
            vars[page][posConfig][2] = value / 10000
        elseif dataId == 0x5132 then
            analogCurrSens[2] = 1000 / value
        elseif dataId == 0x5105 then 
            if value == 115200 then 
                vars[page][posConfig][2] = 1
            elseif value == 57600 then 
                vars[page][posConfig][2] = 2
            elseif value == 38400 then 
                vars[page][posConfig][2] = 3
            else
                vars[page][posConfig][2] = 4
            end
        elseif dataId == 0x5147 then 
            if value == 1 then 
                vars[page][posConfig][2] = 1
            elseif value == 5 then 
                vars[page][posConfig][2] = 2
            elseif value == 10 then 
                vars[page][posConfig][2] = 3
            else
                vars[page][posConfig][2] = 4
            end
        elseif dataId == 0x5138 then 
            gpio17[2] = bit32.extract(value, 0)
            gpio18[2] = bit32.extract(value, 1)
            gpio19[2] = bit32.extract(value, 2)
            gpio20[2] = bit32.extract(value, 3)
            gpio21[2] = bit32.extract(value, 4)
            gpio22[2] = bit32.extract(value, 5)
        else
            vars[page][posConfig][2] = value
        end
		posConfig = posConfig + 1
        if posConfig > #vars[page] then
			status = "config"
		end
		newValueConfig = true
    elseif newValueConfig == true or (getTime() - tsRequest > 200) then
        if firmwareVersion == nil then
            if sportTelemetryPush(sensorId[2] - 1, 0x34, 0x5101, 0) then 
                tsRequest = getTime()
                newValueConfig = false
            end
        elseif vars[page][posConfig][2] ~= nil then
            posConfig = posConfig + 1
            if posConfig > #vars[page] then
			    status = "config"
		    end
        elseif sportTelemetryPush(sensorId[2] - 1, 0x34, vars[page][posConfig][6], 0) then
			tsRequest = getTime()
			newValueConfig = false
		end
	end
end

local function saveConfig()
    if status == "saveConfig" then
        if sportTelemetryPush(sensorId[2] - 1, 0x35, 0x5201, 0) then
            status = "startSave"
        end
        return
	end
    if page > #vars then
        if sportTelemetryPush(sensorId[2] - 1, 0x35, 0x5201, 1) then
            status = "exit"
		end 
        return  
    end
    if vars[page][posConfig][2] ~= nil and vars[page][posConfig][6] ~= 0 then
        local value = vars[page][posConfig][2]
        local dataId = vars[page][posConfig][6]
        if dataId == 0x5131 or dataId == 0x5132 or dataId == 0x513F or dataId == 0x5140 or dataId == 0x511B then
            value = value * 100
        elseif dataId == 0x5141 then
            value = value * 10000
        elseif dataId == 0x5105 then 
            if vars[page][posConfig][2] == 1 then 
                value = 115200
            elseif vars[page][posConfig][2] == 2 then 
                value = 57600
            elseif vars[page][posConfig][2] == 3  then 
                value = 38400
            else
                value = 9600
            end
        elseif dataId == 0x5147 then 
            if vars[page][posConfig][2] == 1 then 
                value = 1
            elseif vars[page][posConfig][2] == 2 then 
                value = 5
            elseif vars[page][posConfig][2] == 3 then 
                value = 10
            else
                value = 20
            end
        elseif dataId == 0x5138 then 
            value = gpio17[2] -- bit 1
            value = bit32.bor(value, bit32.lshift(gpio18[2], 1)) -- bit 2
            value = bit32.bor(value, bit32.lshift(gpio19[2], 2)) -- bit 3
            value = bit32.bor(value, bit32.lshift(gpio20[2], 3)) -- bit 4
            value = bit32.bor(value, bit32.lshift(gpio21[2], 4)) -- bit 5
            value = bit32.bor(value, bit32.lshift(gpio22[2], 5)) -- bit 6
        end
        lcd.clear()
        if sportTelemetryPush(sensorId[2] - 1, 0x33, vars[page][posConfig][6], value) then
            lcd.drawText(1, 20, "Send dataId " .. vars[page][posConfig][6], SMLSIZE)
            posConfig = posConfig + 1
        end
    else
        posConfig = posConfig + 1
    end
    if posConfig > #vars[page] then
        page = page + 1
        posConfig = 1
    end
end

local function init_func(event)
	lcd.clear()
    drawTitle("MSRC "  .. scriptVersion, 0, 0)
end

local function run_func(event)
	if status == "searchSensorId" then
        searchSensorId()
    elseif status == "getConfig" then
		getConfig()
	elseif status == "config" then
        handleEvents(event)
    elseif status == "saveConfig" or status == "startSave" then
		saveConfig()
	elseif status == "exit" then
        lcd.clear()
		drawTitle("MSRC " .. scriptVersion, 0, 0)
		lcd.drawText(1, 20, "Completed!", SMLSIZE)
		lcd.drawText(1, 40, "Reboot MSRC to apply changes.", SMLSIZE)
    end
	return 0
end

return { init = init_func, run = run_func }
