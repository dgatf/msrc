--
-- Multi Sensor RC - MSRC
--
-- License https://www.gnu.org/licenses/gpl-3.0.en.html
--

local scriptVersion = "0.1"
local signature = ""
local rawCurrent = ""

local function refreshHorus()
    lcd.clear()
    lcd.drawRectangle(25, 23, 430, 231)
    lcd.drawText(40, 24, "ESC Signature", 0)
    lcd.drawText(170, 24, signature, 0)
    lcd.drawText(40, 53, "Raw Current", 0)
    lcd.drawText(170, 53, rawCurrent)
end

local function refreshTaranis()
    lcdChange = false
    lcd.clear()
    lcd.drawText(1, 9, "ESC Signature", SMLSIZE)
    lcd.drawText(1, 17, signature, SMLSIZE)
    lcd.drawText(1, 31, "Raw Current", SMLSIZE)
    lcd.drawText(1, 40, rawCurrent, SMLSIZE)
end

local function run_func(event)
    if event == nil then
        error("Cannot run as a model script!")
        return 2
    end

    local value1 = getValue(0x5100)
    local value2 = getValue(0x5101)
    local value3 = getValue(0x5102)

    local bit1 = bit32.extract(value1, 0, 8)
    local bit2 = bit32.extract(value1, 8, 8)
    local bit3 = bit32.extract(value1, 16, 8)
    local bit4 = bit32.extract(value1, 24, 8)
    local bit5 = bit32.extract(value2, 0, 8)
    local bit6 = bit32.extract(value2, 8, 8)
    local bit7 = bit32.extract(value2, 16, 8)
    local bit8 = bit32.extract(value2, 24, 8)
    local bit9 = bit32.extract(value3, 0, 8)
    local bit10 = bit32.extract(value3, 8, 8)
    signature = string.format("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", bit1, bit2, bit3, bit4, bit5, bit6, bit7, bit8, bit9, bit10)

    rawCurrent = bit32.extract(value3, 16, 16)

    -- update lcd
    if LCD_W == 480 then
        refreshHorus()
    else
        refreshTaranis()
    end

    return 0
end

return {run = run_func}