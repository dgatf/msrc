local toolName = "TNS|MSRC config|TNE"

local scriptVersion = "v1.4"

local statusEnum = {
	start = 1,
	getConfig = 2,
	config = 3,
	exitScr = 4,
	saveConfig = 5,
	startSave = 6,
	maintOff = 7,
	exit = 8,
}
local pageEnum = {
	sensorId = 1,
	rate = 2,
	avg = 3,
	esc = 4,
	gps = 5,
	vario = 6,
	fuelmeter = 7,
	gpio = 8,
	analogRate = 9,
	analogTemp = 10,
	analogVolt = 11,
	analogCurr = 12,
	analogAirspeed = 13,
	gyro = 14,
}
local varEnum = {
	str = 1,
	val = 2,
	min = 3,
	max = 4,
	incr = 5,
	dataId = 6,
	list = 7,
}
local firmwareVersion
local sensorIdTx = 18
local page = 0
local pageLong = false
local pageItem = 1
local isSelected = false
local status = statusEnum.start
local saveChanges = true
local ts = 0
local vars = {}

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
    "Gyro",
}

-- Page 1 - SensorId
local sensorId = { "Sensor Id", nil, 1, 28, 1, 0x5128 } -- str, val, min, max, incr, dataId
vars[pageEnum.sensorId] = { sensorId }

-- Page 2 - Refresh interval (ms 1-2000)
local rateRpm = { "RPM", nil, 1, 2000, 1, 0x513A }
local rateVolt = { "Voltage", nil, 1, 2000, 1, 0x5113 }
local rateCurr = { "Current", nil, 1, 2000, 1, 0x5114 }
local rateTemp = { "Temperature", nil, 1, 2000, 1, 0x5115 }
local rateGps = { "GPS", nil, 1, 2000, 1, 0x5116 }
local rateCons = { "Consumption", nil, 1, 2000, 1, 0x5117 }
local rateVario = { "Vario", nil, 1, 2000, 1, 0x5118 }
local rateAirspeed = { "Airspeed", nil, 1, 2000, 1, 0x5119 }
vars[pageEnum.rate] = { rateRpm, rateVolt, rateCurr, rateTemp, rateGps, rateCons, rateVario, rateAirspeed }

-- Page 3 - Averaging elements (1-16)
local avgRpm = { "RPM", nil, 1, 16, 1, 0x510C }
local avgVolt = { "Voltage", nil, 1, 16, 1, 0x510D }
local avgCurr = { "Current", nil, 1, 16, 1, 0x510E }
local avgTemp = { "Temperature", nil, 1, 16, 1, 0x510F }
local avgVario = { "Vario", nil, 1, 16, 1, 0x5110 }
local avgAirspeed = { "Airspeed", nil, 1, 16, 1, 0x5111 }
vars[pageEnum.avg] = { avgRpm, avgVolt, avgCurr, avgTemp, avgVario, avgAirspeed }

-- Page 4 - ESC
local pairPoles = { "Pair of Poles", nil, 1, 20, 1, 0x5122 }
local mainGear = { "Main Gear", nil, 1, 1000, 1, 0x5123 }
local pinionGear = { "Pinion Gear", nil, 1, 1000, 1, 0x5124 }
local escProtocolStr = {
	"None",
	"Hobbywing V3",
	"Hobbywing V4",
	"PWM",
	"Castle Link",
	"Kontronik",
	"Kiss",
	"APD HV",
	"HobbyWing V5",
	"Smart ESC/BAT",
	"OMP M4",
	"ZTW",
    "OpenYGE",
}
local escProtocol = { "Protocol", nil, 0, 11, 1, 0x5103, escProtocolStr }
local hw4InitDelay = { "Init Delay", nil, 0, 1, 1, 0x512E, onOffStr }
local hw4AutoDetect = { "Auto detect", nil, 0, 1, 1, 0x514B, onOffStr }
local hw4VoltMult = { "Volt mult", nil, 0, 100000, 1, 0x5131 }
local hw4CurrMult = { "Curr mult", nil, 0, 100000, 1, 0x5132 }
local hw4AutoOffset = { "Auto offset", nil, 0, 1, 1, 0x5135, onOffStr }
local hw4Offset = { "Curr offset", nil, 0, 2000, 1, 0x5139 }
local smartEscConsumption = { "Calc cons", nil, 0, 1, 1, 0x5145, onOffStr }
vars[pageEnum.esc] = {
	pairPoles,
	mainGear,
	pinionGear,
	escProtocol,
	hw4InitDelay,
	hw4AutoDetect,
	hw4VoltMult,
	hw4CurrMult,
	hw4AutoOffset,
	hw4Offset,
	smartEscConsumption,
}

-- Page 5 - GPS
local gpsEnable = { "Enable", nil, 0, 1, 1, 0x5104, onOffStr }
local gpsProtocolStr = { "UBLOX", "NMEA" }
local gpsProtocol = { "Protocol", nil, 0, 1, 1, 0x5149, gpsProtocolStr }
local gpsBaudrateVal = { 9600, 38400, 57600, 115200 }
local gpsBaudrate = { "Baudrate", nil, 0, 3, 1, 0x5105, gpsBaudrateVal }
local gpsRateVal = { 1, 5, 10, 20 }
local gpsRate = { "Rate", nil, 0, 3, 1, 0x5147, gpsRateVal }
vars[pageEnum.gps] = { gpsEnable, gpsProtocol, gpsBaudrate, gpsRate }

-- Page 6 - Vario
local varioModelStr = { "None", "BMP280", "MS5611", "BMP180" }
local varioModel = { "Model", nil, 0, 3, 1, 0x510A, varioModelStr }
local varioAddress = { "Address", nil, 0x76, 0x77, 1, 0x510B }
local varioFilterStr = { "Low", "Medium", "High" }
local varioFilter = { "Filter", nil, 1, 3, 1, 0x5126, varioFilterStr }
vars[pageEnum.vario] = { varioModel, varioAddress, varioFilter }

-- Page 7 - Fuel meter
local fuelMeter = { "Enable", nil, 0, 1, 1, 0x5142, onOffStr }
local mlPulse = { "ml/pulse", nil, 0, 1, 0.0001, 0x5141 }
vars[pageEnum.fuelmeter] = { fuelMeter, mlPulse }

-- Page 8 - GPIO
local gpioInterval = { "Interval(ms)", nil, 10, 10000, 1, 0x511D }
local gpio = { "", nil, 0, 0x3F, 1, 0x5138 }
local gpio17 = { "17", 0, 0, 1, 1, 0, onOffStr }
local gpio18 = { "18", 0, 0, 1, 1, 0, onOffStr }
local gpio19 = { "19", 0, 0, 1, 1, 0, onOffStr }
local gpio20 = { "20", 0, 0, 1, 1, 0, onOffStr }
local gpio21 = { "21", 0, 0, 1, 1, 0, onOffStr }
local gpio22 = { "22", 0, 0, 1, 1, 0, onOffStr }
vars[pageEnum.gpio] = { gpioInterval, gpio }

-- Page 9 - Analog rate
local analogRate = { "Rate(Hz)", nil, 1, 100, 1, 0x5136 }
vars[pageEnum.analogRate] = { analogRate }

-- Page 10 - Temperature analog
local analogTemp = { "Enable", nil, 0, 1, 1, 0x5108, onOffStr }
vars[pageEnum.analogTemp] = { analogTemp }

-- Page 11 - Voltage analog
local analogVolt = { "Enable", nil, 0, 1, 1, 0x5106, onOffStr }
local analogVoltMult = { "Multiplier", nil, 1, 1000, 0.01, 0x511B }
vars[pageEnum.analogVolt] = { analogVolt, analogVoltMult }

-- Page 12 - Current analog
local analogCurr = { "Enable", nil, 0, 1, 1, 0x5107, onOffStr }
local analogCurrTypeStr = { "Hall Effect", "Shunt Resistor" }
local analogCurrType = { "Type", nil, 0, 1, 1, 0x511C, analogCurrTypeStr }
local analogCurrMult = { "Mult", nil, 0, 100, 0.01, 0x511F }
local analogCurrSens = { "Sens(mV/A)", 0, 0, 100, 0.01, 0 }
local analogCurrAutoOffset = { "Auto Offset", nil, 0, 1, 1, 0x5121, onOffStr }
local analogCurrOffset = { "Offset", nil, 0, 3.3, 0.01, 0x5120 }
vars[pageEnum.analogCurr] = { analogCurr, analogCurrType, analogCurrMult, analogCurrAutoOffset, analogCurrOffset }

-- Page 13 - Airspeed analog
local analogAirspeed = { "Enable", nil, 0, 1, 1, 0x5109, onOffStr }
local analogAirspeedVcc = { "Vcc(V)", nil, 3, 6, 0.01, 0x5140 }
local analogAirspeedOffset = { "Offset(mV)", nil, -1000, 1000, 1, 0x513F }
vars[pageEnum.analogAirspeed] = { analogAirspeed, analogAirspeedVcc, analogAirspeedOffset }

-- Page 14 - Gyro
local gyro = { "Enable", nil, 0, 1, 1, 0x514F, onOffStr }
local gyroAddress = { "Address", nil, 0x68, 0x69, 1, 0x5150 }
local gyroAccSens = { "Acc sens", nil, 0, 3, 1, 0x514C, { 2, 4, 8, 16 } }
local gyroGyroSens = { "Gyro sens", nil, 0, 3, 1, 0x514D, { 250, 500, 1000, 2000 } }
local gyroGyroWeight = { "Gyro weight", nil, 0, 100, 1, 0x514E }
local gyroFilter = { "Filter", nil, 0, 6, 1, 0x5151 }
vars[pageEnum.gyro] = { gyro, gyroAddress, gyroAccSens, gyroGyroSens, gyroGyroWeight, gyroFilter }

local function getTextFlags(item)
	local value = 0
	if item == pageItem then
		value = INVERS
		if isSelected == true then
			value = value + BLINK
		end
	end
	return value
end

local function changeValue(isIncremented)
	local mult = 1
	if getRotEncSpeed() == ROTENC_MIDSPEED then
		mult = 10
	elseif getRotEncSpeed() == ROTENC_HIGHSPEED then
		mult = 100
	end
	if isIncremented == true then
		vars[page][pageItem][varEnum.val] = vars[page][pageItem][varEnum.val]
			+ vars[page][pageItem][varEnum.incr] * mult
		if vars[page][pageItem][varEnum.val] > vars[page][pageItem][varEnum.max] then
			vars[page][pageItem][varEnum.val] = vars[page][pageItem][varEnum.max]
		end
	else
		vars[page][pageItem][varEnum.val] = vars[page][pageItem][varEnum.val]
			- vars[page][pageItem][varEnum.incr] * mult
		if vars[page][pageItem][varEnum.val] < vars[page][pageItem][varEnum.min] then
			vars[page][pageItem][varEnum.val] = vars[page][pageItem][varEnum.min]
		end
	end
end

local function getValue(list, index) -- index starts at 1
	if index > #list then
		return list[1]
	end
	return list[index]
end

local function getIndex(list, value)
	for i = 1, #list do
		if value == list[i] then
			return i
		end
	end
	return 1
end

local function handleEvents(event)
	-- Check one-time script
	if event == nil then
		return 2
	end
	-- Handle events
	if page == 0 then
		if event == EVT_PAGE_BREAK or event == 513 then
			page = 1
			status = statusEnum.getConfig
		end
	elseif event == EVT_EXIT_BREAK then
		if isSelected == true then
			isSelected = false
		else
			status = statusEnum.exitScr
		end
	elseif (event == EVT_PAGE_BREAK or event == 513) and (status ~= statusEnum.exitScr) then
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
		pageItem = 1
		isSelected = false
		status = statusEnum.getConfig
		ts = 0
	elseif event == EVT_PAGE_LONG or event == 2049 then
		pageLong = true
	elseif event == EVT_ROT_RIGHT then
		if status == statusEnum.exitScr then
			saveChanges = not saveChanges
		elseif isSelected == false then
			pageItem = pageItem + 1
			if pageItem > #vars[page] then
				pageItem = 1
			end
		else
			changeValue(true)
		end
	elseif event == EVT_ROT_LEFT then
		if status == statusEnum.exitScr then
			saveChanges = not saveChanges
		end
		if isSelected == false then
			pageItem = pageItem - 1
			if pageItem < 1 then
				pageItem = #vars[page]
			end
		else
			changeValue(false)
		end
	elseif event == EVT_ROT_BREAK then
		if status == statusEnum.exitScr then
			pageItem = 1
			if saveChanges == true then
				status = statusEnum.saveConfig
				page = 1
				vars[pageEnum.gpio] = { gpioInterval, gpio }
			else
				saveChanges = true
				status = statusEnum.getConfig
			end
		else
			isSelected = not isSelected
		end
	end
end

local function getConfig()
	local sensor, frameId, dataId, value = sportTelemetryPop()
	if firmwareVersion == nil then
		if dataId ~= nil and dataId == 0x5101 then
			firmwareVersion = "v"
				.. bit32.rshift(value, 16)
				.. "."
				.. bit32.band(bit32.rshift(value, 8), 0xF)
				.. "."
				.. bit32.band(value, 0xF)
			status = statusEnum.config
		elseif (getTime() - ts > 200) and sportTelemetryPush(sensorIdTx - 1, 0x30, 0x5101, 1) then
			ts = getTime()
		end
	elseif dataId ~= nil and dataId == vars[page][pageItem][varEnum.dataId] then
		if dataId == 0x511E or dataId == 0x5140 or dataId == 0x511B or dataId == 0x5120 then
			value = value / 100.0
		elseif dataId == 0x5141 then
			value = value / 10000.0
		elseif dataId == 0x511F then
			value = value / 100.0
			if analogCurrTypeStr[analogCurrType[varEnum.val] + 1] == "Hall Effect" then
				value = 1000.0 / value
				analogCurrSens[varEnum.val] = math.floor(value * 100) / 100.0
			end
		elseif dataId == 0x5105 then
			value = getIndex(gpsBaudrateVal, value) - 1
		elseif dataId == 0x5147 then
			value = getIndex(gpsRateVal, value) - 1
		elseif dataId == 0x5138 then
			gpio17[varEnum.val] = bit32.extract(value, 0)
			gpio18[varEnum.val] = bit32.extract(value, 1)
			gpio19[varEnum.val] = bit32.extract(value, 2)
			gpio20[varEnum.val] = bit32.extract(value, 3)
			gpio21[varEnum.val] = bit32.extract(value, 4)
			gpio22[varEnum.val] = bit32.extract(value, 5)
		elseif dataId == 0x5126 then
			value = value - 1
		elseif dataId == 0x5135 then
			if value == 0 then
				value = 1
			else
				value = 0
			end
		elseif dataId == 0x513F then
			value = value - 1000
		end
		if value < vars[page][pageItem][varEnum.min] then
			value = vars[page][pageItem][varEnum.min]
		end
		if value > vars[page][pageItem][varEnum.max] then
			value = vars[page][pageItem][varEnum.max]
		end
		vars[page][pageItem][varEnum.val] = value
		pageItem = pageItem + 1
		ts = 0
		if pageItem > #vars[page] then
			pageItem = 1
			status = statusEnum.config
		end
	elseif vars[page][pageItem][varEnum.val] ~= nil then
		pageItem = pageItem + 1
		ts = 0
		if pageItem > #vars[page] then
			pageItem = 1
			status = statusEnum.config
		end
	elseif
		(getTime() - ts > 200) and sportTelemetryPush(sensorIdTx - 1, 0x30, vars[page][pageItem][varEnum.dataId], 1)
	then
		ts = getTime()
	end
	if page == pageEnum.gpio and status == statusEnum.config then
		vars[pageEnum.gpio] = { gpioInterval, gpio17, gpio18, gpio19, gpio20, gpio21, gpio22 }
	end
end

local function saveConfig()
	if status == statusEnum.saveConfig then
		if sportTelemetryPush(sensorIdTx - 1, 0x31, 0x5201, 0) then
			status = statusEnum.startSave
		end
		return
	end
	if page > #vars then
		if sportTelemetryPush(sensorIdTx - 1, 0x31, 0x5201, 1) then
			status = statusEnum.maintOff
		end
		return
	end
	local value = vars[page][pageItem][varEnum.val]
	local dataId = vars[page][pageItem][varEnum.dataId]
	if value ~= nil and dataId ~= 0 then
		if dataId == 0x511E or dataId == 0x511F or dataId == 0x5140 or dataId == 0x511B then
			value = value * 100
		elseif dataId == 0x5141 then
			value = value * 10000
		elseif dataId == 0x5105 then
			value = getValue(gpsBaudrateVal, gpsBaudrate[varEnum.val] + 1)
		elseif dataId == 0x5147 then
			value = getValue(gpsRateVal, gpsRate[varEnum.val] + 1)
		elseif dataId == 0x5138 then
			value = gpio17[varEnum.val] -- bit 1
			value = bit32.bor(value, bit32.lshift(gpio18[varEnum.val], 1)) -- bit 2
			value = bit32.bor(value, bit32.lshift(gpio19[varEnum.val], 2)) -- bit 3
			value = bit32.bor(value, bit32.lshift(gpio20[varEnum.val], 3)) -- bit 4
			value = bit32.bor(value, bit32.lshift(gpio21[varEnum.val], 4)) -- bit 5
			value = bit32.bor(value, bit32.lshift(gpio22[varEnum.val], 5)) -- bit 6
		elseif dataId == 0x5126 then
			value = vars[page][pageItem][varEnum.val] + 1
		elseif dataId == 0x5135 then
			if vars[page][pageItem][varEnum.val] == 1 then
				value = 0
			else
				value = 1
			end
		elseif dataId == 0x513F then
			value = vars[page][pageItem][varEnum.val] + 1000
		end
		value = math.floor(value)
		if sportTelemetryPush(sensorIdTx - 1, 0x31, dataId, value) then
			pageItem = pageItem + 1
		end
	else
		pageItem = pageItem + 1
	end
	if pageItem > #vars[page] then
		page = page + 1
		pageItem = 1
	end
end

local function setPageItems()
	if page == pageEnum.esc then
		if escProtocol[varEnum.val] + 1 > #escProtocolStr then
			escProtocol[varEnum.val] = 0
		end
		if hw4AutoDetect[varEnum.val] + 1 > #onOffStr then
			hw4AutoDetect[varEnum.val] = 0
		end
		if hw4AutoOffset[varEnum.val] + 1 > #onOffStr then
			hw4AutoOffset[varEnum.val] = 0
		end
		vars[page] = { pairPoles, mainGear, pinionGear, escProtocol }
		if escProtocolStr[escProtocol[varEnum.val] + 1] == "Hobbywing V4" then
			vars[page] = {
				pairPoles,
				mainGear,
				pinionGear,
				escProtocol,
				hw4InitDelay,
				hw4AutoDetect,
			}
			if getValue(onOffStr, hw4AutoDetect[varEnum.val] + 1) == "Off" then
				vars[page] = {
					pairPoles,
					mainGear,
					pinionGear,
					escProtocol,
					hw4InitDelay,
					hw4AutoDetect,
					hw4VoltMult,
					hw4CurrMult,
					hw4AutoOffset,
				}
				if getValue(onOffStr, hw4AutoOffset[varEnum.val] + 1) == "Off" then
					vars[page] = {
						pairPoles,
						mainGear,
						pinionGear,
						escProtocol,
						hw4InitDelay,
						hw4AutoDetect,
						hw4VoltMult,
						hw4CurrMult,
						hw4AutoOffset,
						hw4Offset,
					}
				end
			end
		elseif escProtocolStr[escProtocol[varEnum.val] + 1] == "Smart ESC/BAT" then
			vars[page] = { pairPoles, mainGear, pinionGear, escProtocol, smartEscConsumption }
		end
	elseif page == pageEnum.vario then
		if varioModel[varEnum.val] + 1 > #varioModelStr then
			varioModel[varEnum.val] = 0
		end
		if varioModelStr[varioModel[varEnum.val] + 1] == "BMP280" then
			vars[page] = { varioModel, varioAddress, varioFilter }
		else
			vars[page] = { varioModel, varioAddress }
		end
	elseif page == pageEnum.analogCurr then
		if analogCurrType[varEnum.val] + 1 > #analogCurrTypeStr then
			analogCurrType[varEnum.val] = 0
		end
		if analogCurrAutoOffset[varEnum.val] + 1 > #onOffStr then
			analogCurrAutoOffset[varEnum.val] = 0
		end
		if analogCurrTypeStr[analogCurrType[varEnum.val] + 1] == "Hall Effect" then
			vars[page] = { analogCurr, analogCurrType, analogCurrSens, analogCurrAutoOffset }
			if getValue(onOffStr, analogCurrAutoOffset[varEnum.val] + 1) == "Off" then
				vars[page] = { analogCurr, analogCurrType, analogCurrSens, analogCurrAutoOffset, analogCurrOffset }
			end
		else
			vars[page] = { analogCurr, analogCurrType, analogCurrMult }
		end
	end
end

local function drawTitle(str, page, pages)
	if LCD_H == 64 then
		lcd.drawScreenTitle(str, page, pages)
	else
		lcd.drawText(1, 1, str)
		if page ~= 0 and pages ~= 0 then
			lcd.drawText(200, 1, page .. "/" .. pages)
		end
	end
end

local function drawPage()
	lcd.clear()
	if page == 0 then
		drawTitle("MSRC " .. scriptVersion, 0, 0)
		if firmwareVersion ~= nil then
			lcd.drawText(1, 20, "Firmware " .. firmwareVersion, SMLSIZE)
			lcd.drawText(1, 35, "Press Page", SMLSIZE)
		end
	elseif status == statusEnum.getConfig then
		drawTitle(pageName[page], page, #vars)
		lcd.drawText(60, 30, pageItem .. "/" .. #vars[page], 0)
	elseif status == statusEnum.config then
		drawTitle(pageName[page], page, #vars)
		local scroll, fileHeight, fileStart
		if LCD_H == 64 then
			scroll = pageItem - 8
			if scroll < 0 then
				scroll = 0
			end
			fileStart = 9
			fileHeight = 7
		else
			scroll = 0
			fileStart = 20
			fileHeight = 15
		end
		for i = 1, #vars[page] - scroll do
			lcd.drawText(1, fileStart + fileHeight * (i - 1), vars[page][i + scroll][1], SMLSIZE)
			if #vars[page][i + scroll] == 6 then
				local val = vars[page][i + scroll][varEnum.val]
				if val == nil then
					val = -1
				end
				lcd.drawText(LCD_W / 2, fileStart + fileHeight * (i - 1), val, SMLSIZE + getTextFlags(i + scroll))
			elseif #vars[page][i + scroll] == 7 then
				local val = 1
				if vars[page][i + scroll][varEnum.val] ~= nil then
					val = vars[page][i + scroll][varEnum.val] + 1
				end
				lcd.drawText(
					LCD_W / 2,
					fileStart + fileHeight * (i - 1),
					getValue(vars[page][i + scroll][varEnum.list], val),
					SMLSIZE + getTextFlags(i + scroll)
				)
			end
		end
	elseif status == statusEnum.exitScr then
		drawTitle("Exit", 0, 0)
		lcd.drawText(1, 20, "Save changes?", SMLSIZE)
		local flag_yes = 0
		local flag_cancel = 0
		if saveChanges == true then
			flag_yes = INVERS
		else
			flag_cancel = INVERS
		end
		lcd.drawText(1, 30, "Yes", SMLSIZE + flag_yes)
		lcd.drawText(60, 30, "Cancel", SMLSIZE + flag_cancel)
	elseif status == statusEnum.startSave then
		drawTitle("Saving ", 0, 0)
		lcd.drawText(1, 20, "Update " .. page .. "/" .. #vars, SMLSIZE)
	elseif status == statusEnum.exit then
		drawTitle("MSRC " .. scriptVersion, 0, 0)
		lcd.drawText(1, 20, "Completed!", SMLSIZE)
		lcd.drawText(1, 40, "Reboot MSRC to apply changes.", SMLSIZE)
	end
end

local function run_func(event)
	if status == statusEnum.start then
		if sportTelemetryPush(sensorIdTx - 1, 0x21, 0xFFFF, 0x80) then
			status = statusEnum.getConfig
		end
	elseif status == statusEnum.getConfig then
		handleEvents(event)
		getConfig()
	elseif status == statusEnum.config or status == statusEnum.exitScr then
		handleEvents(event)
		if status == statusEnum.config then
			setPageItems()
		end
	elseif status == statusEnum.saveConfig or status == statusEnum.startSave then
		saveConfig()
	elseif status == statusEnum.maintOff then
		if sportTelemetryPush(sensorIdTx - 1, 0x20, 0xFFFF, 0x80) then
			status = statusEnum.exit
		end
	end
	drawPage()
	return 0
end

return { run = run_func }
