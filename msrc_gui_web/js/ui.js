/**
 * ui.js — Bidirectional mapping between config_t and DOM widgets.
 *         Also handles protocol-dependent widget visibility.
 */

/* ── Helpers ─────────────────────────────────────────── */

function $(id) { return document.getElementById(id); }

function alphaToElements(alpha) {
    if (alpha <= 0) return 1;
    return Math.round(2 / alpha - 1) || 1;
}
function elementsToAlpha(n) {
    return 2.0 / (n + 1);
}

function selectByValue(sel, val) {
    const s = typeof val === 'number' ? String(val) : val;
    for (let i = 0; i < sel.options.length; i++) {
        if (sel.options[i].value === s || sel.options[i].text === s) {
            sel.selectedIndex = i;
            return;
        }
    }
}

function selectedValue(sel) { return sel.options[sel.selectedIndex]?.value ?? ''; }
function selectedText(sel) { return sel.options[sel.selectedIndex]?.text ?? ''; }

function show(el) { if (el) el.style.display = ''; }
function hide(el) { if (el) el.style.display = 'none'; }
function showIf(el, cond) { if (el) el.style.display = cond ? '' : 'none'; }

/* ── Config → UI ─────────────────────────────────────── */

function setUiFromConfig(cfg) {
    /* Receiver protocol (value is stored as userData in the option) */
    selectByValue($('cbReceiver'), cfg.rx_protocol);

    /* Serial Monitor */
    selectByValue($('cbSerialMonitorGpio'), cfg.serial_monitor_gpio);
    selectByValue($('cbBaudrate'), cfg.serial_monitor_baudrate);
    $('cbParity').selectedIndex = cfg.serial_monitor_parity > 2 ? 0 : cfg.serial_monitor_parity;
    $('cbStopbits').selectedIndex = Math.max(0, cfg.serial_monitor_stop_bits - 1);
    $('sbTimeout').value = Math.min(cfg.serial_monitor_timeout_ms, 100);
    $('cbInverted').checked = cfg.serial_monitor_inverted;
    selectByValue($('cbSerialFormat'), cfg.serial_monitor_format);

    /* ESC – combo index = esc_protocol - 1 */
    if (cfg.esc_protocol === ESC_PROTOCOL.ESC_NONE) {
        $('chkEsc').checked = false;
    } else {
        $('chkEsc').checked = true;
        $('cbEsc').selectedIndex = cfg.esc_protocol - 1;
    }

    /* GPS */
    $('chkGps').checked = cfg.enable_gps;
    selectByValue($('cbGpsBaudrate'), cfg.gps_baudrate);
    selectByValue($('cbGpsRate'), cfg.gps_rate);
    $('cbGpsProtocol').selectedIndex = cfg.gps_protocol;

    /* Analog rate */
    $('sbAnalogRate').value = cfg.analog_rate;

    /* Voltage */
    $('chkVoltage').checked = cfg.enable_analog_voltage;
    $('sbVoltage1Mult').value = cfg.analog_voltage_multiplier;
    $('ckSbusBattery').checked = cfg.sbus_battery_slot;
    $('ckSbusExtVolt').checked = !cfg.sbus_battery_slot;

    /* Temperature */
    $('cbTemperature1').checked = cfg.enable_analog_ntc;

    /* Current – Hall vs Shunt */
    $('chkCurrent').checked = cfg.enable_analog_current;
    $('cbCurrentSensorType').selectedIndex = cfg.analog_current_type;
    $('cbCurrentAutoOffset').checked = cfg.analog_current_autoffset;
    $('sbQuiescentVoltage').value = cfg.analog_current_quiescent_voltage;
    if (cfg.analog_current_type === 0) { /* Hall */
        $('sbCurrentSens').value = cfg.analog_current_multiplier > 0 ? (1000 / cfg.analog_current_multiplier) : 1;
    } else { /* Shunt */
        $('sbAnalogCurrentMultiplier').value = cfg.analog_current_multiplier;
    }

    /* Airspeed */
    $('chkAirspeed').checked = cfg.enable_analog_airspeed;
    $('sbAirspeedVcc').value = cfg.airspeed_vcc / 100;
    $('sbAirspeedOffset').value = cfg.airspeed_offset;

    /* Vario / Altitude */
    if (cfg.i2c_module === 0) { /* I2C_NONE */
        $('chkAltitude').checked = false;
    } else {
        $('chkAltitude').checked = true;
        $('cbBarometerType').selectedIndex = cfg.i2c_module - 1;
    }
    if (cfg.bmp280_filter >= 1 && cfg.bmp280_filter <= 3) {
        $('cbAltitudeFilter').selectedIndex = cfg.bmp280_filter - 1;
    }
    $('cbVarioAutoOffset').checked = cfg.vario_auto_offset;

    /* Refresh rates */
    $('sbRpmRate').value = cfg.refresh_rate_rpm;
    $('sbVoltageRate').value = cfg.refresh_rate_voltage;
    $('sbCurrentRate').value = cfg.refresh_rate_current;
    $('sbTemperatureRate').value = cfg.refresh_rate_temperature;
    $('sbGpsRate').value = cfg.refresh_rate_gps;
    $('sbConsumptionRate').value = cfg.refresh_rate_consumption;
    $('sbVarioRate').value = cfg.refresh_rate_vario;
    $('sbAirspeedRate').value = cfg.refresh_rate_airspeed;

    /* Averaging – alpha → elements */
    $('sbRpmAvg').value = alphaToElements(cfg.alpha_rpm);
    $('sbVoltageAvg').value = alphaToElements(cfg.alpha_voltage);
    $('sbCurrentAvg').value = alphaToElements(cfg.alpha_current);
    $('sbTemperatureAvg').value = alphaToElements(cfg.alpha_temperature);
    $('sbVarioAvg').value = alphaToElements(cfg.alpha_vario);
    $('sbAirspeedAvg').value = alphaToElements(cfg.alpha_airspeed);

    /* Analog voltage multiplier */
    $('sbVoltage1Mult').value = cfg.analog_voltage_multiplier;

    /* RPM multipliers */
    $('sbPairOfPoles').value = cfg.pairOfPoles;
    $('sbMainTeeth').value = cfg.mainTeeth;
    $('sbPinionTeeth').value = cfg.pinionTeeth;

    /* PWM out */
    $('cbPwmOut').checked = cfg.enable_pwm_out;

    /* SmartPort */
    $('sbSensorId').value = cfg.smartport_sensor_id;

    /* XBUS */
    $('cbClockStretch').checked = cfg.xbus_clock_stretch;
    $('cbAlternativePacket').checked = cfg.xbus_use_alternative_volt_temp;

    /* IBUS */
    $('cbAlternativeCoordinates').checked = cfg.ibus_alternative_coordinates;

    /* Jeti Ex */
    selectByValue($('cbSpeedUnitsGps'), cfg.jeti_gps_speed_units_kmh ? '1' : '0');

    /* SBUS */
    $('ckSbusBattery').checked = cfg.sbus_battery_slot;
    $('ckSbusExtVolt').checked = !cfg.sbus_battery_slot;

    /* FPort / FBUS */
    $('cbFPortInverted').checked = cfg.fport_inverted;
    $('cbFbusInverted').checked = cfg.fbus_inverted;

    /* HW V4/V5 ESC parameters */
    $('cbInitDelay').checked = cfg.enable_esc_hw4_init_delay;
    $('cbEscAutoOffset').checked = !cfg.esc_hw4_is_manual_offset;
    $('sbEscOffset').value = cfg.esc_hw4_offset;
    $('sbVoltageMultiplier').value = Math.round(cfg.esc_hw4_voltage_multiplier * 100000);
    $('sbCurrentMultiplier').value = Math.round(cfg.esc_hw4_current_multiplier * 100000);
    $('cbHw4AutoDetect').checked = cfg.esc_hw4_auto_detect;

    /* Smart ESC */
    $('cbCalculateConsumption').checked = cfg.smart_esc_calc_consumption;

    /* Fuel flow */
    $('chkFuelmeter').checked = cfg.enable_fuel_flow;
    $('sbMlPulse').value = cfg.fuel_flow_ml_per_pulse;

    /* Fuel pressure */
    $('chkFuelPressure').checked = cfg.enable_fuel_pressure;
    selectByValue($('cbMaxPressure'), cfg.xgzp68xxd_k);

    /* GPIOs bitmask */
    $('cbGpio17').checked = !!(cfg.gpio_mask & 0x01);
    $('cbGpio18').checked = !!(cfg.gpio_mask & 0x02);
    $('cbGpio19').checked = !!(cfg.gpio_mask & 0x04);
    $('cbGpio20').checked = !!(cfg.gpio_mask & 0x08);
    $('cbGpio21').checked = !!(cfg.gpio_mask & 0x10);
    $('cbGpio22').checked = !!(cfg.gpio_mask & 0x20);
    $('sbGpioInterval').value = cfg.gpio_interval;

    /* Gyro */
    $('chkGyro').checked = cfg.enable_gyro;
    $('cbGyroAccSens').selectedIndex = cfg.mpu6050_acc_scale;
    $('cbGyroSens').selectedIndex = cfg.mpu6050_gyro_scale;
    $('sbGyroWeight').value = cfg.mpu6050_gyro_weighting;
    $('sbGyroFilter').value = cfg.mpu6050_filter;

    /* LiPo INA3221 */
    selectByValue($('cbIna3221Filter'), cfg.ina3221_filter);
    $('sbLipoCells').value = cfg.lipo_cells;
    $('chkLipo').checked = cfg.enable_lipo;

    /* Trigger visibility update */
    updateVisibility();
    updateEscOptions();
    updateCurrentSensorUI();
    updateToggleables();
}

/* ── UI → Config ─────────────────────────────────────── */

function getConfigFromUi() {
    const cfg = configDefault();

    cfg.version = CONFIG_VERSION;
    cfg.rx_protocol = parseInt(selectedValue($('cbReceiver')));

    /* Serial Monitor */
    cfg.serial_monitor_baudrate = parseInt(selectedText($('cbBaudrate')));
    cfg.serial_monitor_gpio = parseInt(selectedText($('cbSerialMonitorGpio')));
    cfg.serial_monitor_stop_bits = parseInt(selectedText($('cbStopbits')));
    cfg.serial_monitor_parity = parseInt(selectedValue($('cbParity')));
    cfg.serial_monitor_timeout_ms = parseInt($('sbTimeout').value);
    cfg.serial_monitor_inverted = $('cbInverted').checked;
    cfg.serial_monitor_format = parseInt(selectedValue($('cbSerialFormat')));

    /* ESC */
    if ($('chkEsc').checked)
        cfg.esc_protocol = $('cbEsc').selectedIndex + 1;
    else
        cfg.esc_protocol = ESC_PROTOCOL.ESC_NONE;

    /* GPS */
    cfg.enable_gps = $('chkGps').checked;
    cfg.gps_baudrate = parseInt(selectedText($('cbGpsBaudrate')));
    cfg.gps_rate = parseInt(selectedText($('cbGpsRate')));
    cfg.gps_protocol = $('cbGpsProtocol').selectedIndex;

    /* Voltage */
    cfg.enable_analog_voltage = $('chkVoltage').checked;
    cfg.analog_voltage_multiplier = parseFloat($('sbVoltage1Mult').value);
    cfg.sbus_battery_slot = $('ckSbusBattery').checked;

    /* Current */
    cfg.enable_analog_current = $('chkCurrent').checked;
    cfg.analog_current_type = parseInt(selectedValue($('cbCurrentSensorType')));
    cfg.analog_current_quiescent_voltage = parseFloat($('sbQuiescentVoltage').value);

    if (cfg.analog_current_type === 0) { /* Hall */
        if ($('cbCurrentAutoOffset').checked) {
            cfg.analog_current_autoffset = true;
            cfg.analog_current_offset = 0;
        } else {
            cfg.analog_current_autoffset = false;
            cfg.analog_current_offset = parseFloat($('sbQuiescentVoltage').value);
        }
        const sens = parseFloat($('sbCurrentSens').value);
        cfg.analog_current_multiplier = sens > 0 ? 1000 / sens : 1;
    } else { /* Shunt */
        cfg.analog_current_autoffset = false;
        cfg.analog_current_offset = 0;
        cfg.analog_current_multiplier = parseFloat($('sbAnalogCurrentMultiplier').value);
    }

    /* Temperature */
    cfg.enable_analog_ntc = $('cbTemperature1').checked;

    /* Airspeed */
    cfg.enable_analog_airspeed = $('chkAirspeed').checked;
    cfg.airspeed_vcc = Math.round(parseFloat($('sbAirspeedVcc').value) * 100);
    cfg.airspeed_offset = parseInt($('sbAirspeedOffset').value);

    /* Vario */
    if ($('chkAltitude').checked)
        cfg.i2c_module = $('cbBarometerType').selectedIndex + 1;
    else
        cfg.i2c_module = 0;
    cfg.bmp280_filter = $('cbAltitudeFilter').selectedIndex + 1;
    cfg.vario_auto_offset = $('cbVarioAutoOffset').checked;

    /* Refresh rates */
    cfg.refresh_rate_rpm = parseInt($('sbRpmRate').value);
    cfg.refresh_rate_voltage = parseInt($('sbVoltageRate').value);
    cfg.refresh_rate_current = parseInt($('sbCurrentRate').value);
    cfg.refresh_rate_temperature = parseInt($('sbTemperatureRate').value);
    cfg.refresh_rate_gps = parseInt($('sbGpsRate').value);
    cfg.refresh_rate_consumption = parseInt($('sbConsumptionRate').value);
    cfg.refresh_rate_vario = parseInt($('sbVarioRate').value);
    cfg.refresh_rate_airspeed = parseInt($('sbAirspeedRate').value);

    /* Averaging – elements → alpha */
    cfg.alpha_rpm = elementsToAlpha(parseInt($('sbRpmAvg').value));
    cfg.alpha_voltage = elementsToAlpha(parseInt($('sbVoltageAvg').value));
    cfg.alpha_current = elementsToAlpha(parseInt($('sbCurrentAvg').value));
    cfg.alpha_temperature = elementsToAlpha(parseInt($('sbTemperatureAvg').value));
    cfg.alpha_vario = elementsToAlpha(parseInt($('sbVarioAvg').value));
    cfg.alpha_airspeed = elementsToAlpha(parseInt($('sbAirspeedAvg').value));

    /* Analog rate */
    cfg.analog_rate = parseInt($('sbAnalogRate').value);

    /* RPM multipliers */
    cfg.pairOfPoles = parseInt($('sbPairOfPoles').value);
    cfg.mainTeeth = parseInt($('sbMainTeeth').value);
    cfg.pinionTeeth = parseInt($('sbPinionTeeth').value);

    /* PWM out */
    cfg.enable_pwm_out = $('cbPwmOut').checked;

    /* SmartPort */
    cfg.smartport_sensor_id = parseInt($('sbSensorId').value);

    /* XBUS */
    cfg.xbus_clock_stretch = $('cbClockStretch').checked;
    cfg.xbus_use_alternative_volt_temp = $('cbAlternativePacket').checked;

    /* IBUS */
    cfg.ibus_alternative_coordinates = $('cbAlternativeCoordinates').checked;

    /* Jeti Ex */
    cfg.jeti_gps_speed_units_kmh = selectedValue($('cbSpeedUnitsGps')) === '1';

    /* FPort / FBUS */
    cfg.fport_inverted = $('cbFPortInverted').checked;
    cfg.fbus_inverted = $('cbFbusInverted').checked;

    /* HW V4/V5 ESC parameters */
    cfg.enable_esc_hw4_init_delay = $('cbInitDelay').checked;
    cfg.esc_hw4_is_manual_offset = !$('cbEscAutoOffset').checked;
    cfg.esc_hw4_offset = parseFloat($('sbEscOffset').value);
    cfg.esc_hw4_voltage_multiplier = parseInt($('sbVoltageMultiplier').value) / 100000;
    cfg.esc_hw4_current_multiplier = parseInt($('sbCurrentMultiplier').value) / 100000;
    cfg.esc_hw4_auto_detect = $('cbHw4AutoDetect').checked;

    /* Smart ESC */
    cfg.smart_esc_calc_consumption = $('cbCalculateConsumption').checked;

    /* Fuel flow */
    cfg.enable_fuel_flow = $('chkFuelmeter').checked;
    cfg.fuel_flow_ml_per_pulse = parseFloat($('sbMlPulse').value);

    /* Fuel pressure */
    cfg.enable_fuel_pressure = $('chkFuelPressure').checked ? 1 : 0;
    cfg.xgzp68xxd_k = parseInt(selectedValue($('cbMaxPressure')));

    /* GPIOs bitmask */
    cfg.gpio_mask = 0;
    if ($('cbGpio17').checked) cfg.gpio_mask |= 0x01;
    if ($('cbGpio18').checked) cfg.gpio_mask |= 0x02;
    if ($('cbGpio19').checked) cfg.gpio_mask |= 0x04;
    if ($('cbGpio20').checked) cfg.gpio_mask |= 0x08;
    if ($('cbGpio21').checked) cfg.gpio_mask |= 0x10;
    if ($('cbGpio22').checked) cfg.gpio_mask |= 0x20;
    cfg.gpio_interval = parseInt($('sbGpioInterval').value);

    /* Gyro */
    cfg.enable_gyro = $('chkGyro').checked;
    cfg.mpu6050_acc_scale = parseInt(selectedValue($('cbGyroAccSens')));
    cfg.mpu6050_gyro_scale = parseInt(selectedValue($('cbGyroSens')));
    cfg.mpu6050_gyro_weighting = parseInt($('sbGyroWeight').value);
    cfg.mpu6050_filter = parseInt($('sbGyroFilter').value);

    /* LiPo INA3221 */
    cfg.enable_lipo = $('chkLipo').checked ? 1 : 0;
    cfg.ina3221_filter = parseInt(selectedValue($('cbIna3221Filter')));
    cfg.lipo_cells = parseInt($('sbLipoCells').value);

    /* Debug always off from GUI */
    cfg.debug = 0;

    return cfg;
}

/* ── Protocol-dependent visibility ───────────────────── */

const P = RX_PROTOCOL; // shorthand

function updateVisibility() {
    const proto = parseInt(selectedValue($('cbReceiver')));
    const txt = selectedText($('cbReceiver'));
    const isSerial = proto === P.SERIAL_MONITOR;

    /* Serial Monitor fieldset */
    showIf($('fsSerialMonitor'), isSerial);

    /* Hide all sensors and averaging when Serial Monitor */
    const sensorArea = document.querySelector('.config-column:nth-child(2)');
    showIf(sensorArea, !isSerial);
    showIf($('gbAverage'), !isSerial);

    /* Refresh rates: SmartPort, Frsky D, FPort, FBUS */
    const hasRates = [P.RX_SMARTPORT, P.RX_FRSKY_D, P.RX_FPORT, P.RX_FBUS].includes(proto);
    showIf($('gbRate'), hasRates);

    /* SmartPort sensor ID: SmartPort, FBUS */
    showIf($('optSmartportSensorId'), proto === P.RX_SMARTPORT || proto === P.RX_FBUS);

    /* Protocol-specific options */
    showIf($('optXbusClockStretch'), proto === P.RX_XBUS);
    showIf($('optIbus'), proto === P.RX_IBUS);
    showIf($('optJeti'), proto === P.RX_JETIEX || proto === P.RX_JETIEX_SENSOR);
    showIf($('optSbus'), proto === P.RX_SBUS);
    showIf($('optFportInverted'), proto === P.RX_FPORT);
    showIf($('optFbusInverted'), proto === P.RX_FBUS);

    /* Fuel meter: SmartPort, JetiEx, JetiEx Sensor, XBUS, HOTT, FPort, FBUS */
    const hasFuel = [P.RX_SMARTPORT, P.RX_JETIEX, P.RX_JETIEX_SENSOR, P.RX_XBUS, P.RX_HOTT, P.RX_FPORT, P.RX_FBUS].includes(proto);
    showIf($('gbFuelmeter'), hasFuel && !isSerial);

    /* Fuel pressure: SRXL, SRXL2, JetiEx, JetiEx Sensor, XBUS, HOTT */
    const hasFuelPressure = [P.RX_SRXL, P.RX_SRXL2, P.RX_JETIEX, P.RX_JETIEX_SENSOR, P.RX_XBUS, P.RX_HOTT].includes(proto);
    showIf($('gbFuelPressure'), hasFuelPressure && !isSerial);

    /* GPIO: SmartPort, FPort, FBUS */
    const hasGpio = [P.RX_SMARTPORT, P.RX_FPORT, P.RX_FBUS].includes(proto);
    showIf($('gbGpio'), hasGpio && !isSerial);

    /* Airspeed: hidden for Sanwa, GHST */
    showIf($('gbAirspeed'), ![P.RX_SANWA, P.RX_GHST].includes(proto) && !isSerial);

    /* Temperature: hidden for GHST */
    showIf($('grpTemperature'), proto !== P.RX_GHST && !isSerial);

    /* GPS, Current, Vario: hidden for Sanwa */
    const sanwaHidden = proto === P.RX_SANWA;
    showIf($('gbGps'), !sanwaHidden && !isSerial);
    showIf($('gbCurrent'), !sanwaHidden && !isSerial);
    showIf($('gbAltitude'), !sanwaHidden && !isSerial);

    /* LiPo: CRSF, SmartPort, FPort, FBUS, HOTT, SRXL, SRXL2, JetiEx, JetiEx Sensor */
    const hasLipo = [P.RX_CRSF, P.RX_SMARTPORT, P.RX_FPORT, P.RX_FBUS, P.RX_HOTT,
                     P.RX_SRXL, P.RX_SRXL2, P.RX_JETIEX, P.RX_JETIEX_SENSOR].includes(proto);
    showIf($('gbLipo'), hasLipo && !isSerial);

    /* Averaging visibility per protocol */
    const allAvg = !isSerial;
    showIf($('grpRpmAvg'), allAvg && !([P.RX_GHST].includes(proto)));
    showIf($('grpVoltageAvg'), allAvg);
    showIf($('grpCurrentAvg'), allAvg && !([P.RX_SANWA].includes(proto)));
    showIf($('grpTemperatureAvg'), allAvg && !([P.RX_GHST].includes(proto)));
    showIf($('grpVarioAvg'), allAvg && !([P.RX_SANWA].includes(proto)));
    showIf($('grpAirspeedAvg'), allAvg && !([P.RX_SANWA, P.RX_GHST].includes(proto)));

    /* Sanwa: only RPM, Voltage, Temperature in avg */
    if (proto === P.RX_SANWA) {
        show($('grpRpmAvg'));
        show($('grpVoltageAvg'));
        show($('grpTemperatureAvg'));
        hide($('grpCurrentAvg'));
        hide($('grpVarioAvg'));
        hide($('grpAirspeedAvg'));
    }
    /* GHST: only Voltage, Current, Vario in avg */
    if (proto === P.RX_GHST) {
        hide($('grpRpmAvg'));
        show($('grpVoltageAvg'));
        show($('grpCurrentAvg'));
        hide($('grpTemperatureAvg'));
        show($('grpVarioAvg'));
        hide($('grpAirspeedAvg'));
    }

    /* Regenerate circuit diagram */
    if (typeof generateCircuit === 'function') generateCircuit();
}

/* ── ESC-specific options ────────────────────────────── */

function updateEscOptions() {
    const idx = $('cbEsc').selectedIndex;
    const escProto = $('chkEsc').checked ? idx + 1 : 0;

    /* Smart ESC option */
    showIf($('optSmartEsc'), escProto === ESC_PROTOCOL.ESC_SMART);

    /* HW4/HW5 parameter block */
    const isHw4or5 = escProto === ESC_PROTOCOL.ESC_HW4 || escProto === ESC_PROTOCOL.ESC_HW5;
    showIf($('gbEscParameters'), isHw4or5);

    /* HW4 Auto Detect + PWM Out */
    showIf($('optHw4AutoDetect'), isHw4or5);

    /* ESC Offset: shown when manual (autoOffset unchecked) */
    showIf($('grpEscOffset'), isHw4or5 && !$('cbEscAutoOffset').checked);

    /* RPM Multipliers: always visible when ESC enabled */
    showIf($('gbRpmMultipliers'), $('chkEsc').checked);

    /* GPS Rate: only visible when GPS protocol is UBLOX */
    showIf($('grpGpsRate'), $('cbGpsProtocol').selectedIndex === 0);

    /* Baro filter: only for BMP280 (index 0) */
    showIf($('grpAltitudeFilter'), $('cbBarometerType').selectedIndex === 0);

    if (typeof generateCircuit === 'function') generateCircuit();
}

/* ── Current sensor type UI toggle ───────────────────── */

function updateCurrentSensorUI() {
    const isHall = $('cbCurrentSensorType').selectedIndex === 0;
    showIf($('optHallEffect'), isHall);
    showIf($('optShuntResistor'), !isHall);

    /* Quiescent Voltage: only if Hall and not Auto Offset */
    showIf($('grpQuiescentVoltage'), isHall && !$('cbCurrentAutoOffset').checked);
}

/* ── Toggleable fieldsets (checkbox in legend) ───────── */

function updateToggleables() {
    document.querySelectorAll('.toggleable').forEach(fs => {
        const chk = fs.querySelector('legend input[type=checkbox]');
        const content = fs.querySelector('.fieldset-content');
        if (chk && content) {
            content.style.display = chk.checked ? '' : 'none';
        }
    });
}

/* ── Bind all event listeners for reactive UI ────────── */

function bindUIEvents() {
    $('cbReceiver').addEventListener('change', updateVisibility);
    $('cbEsc').addEventListener('change', updateEscOptions);
    $('chkEsc').addEventListener('change', () => { updateEscOptions(); updateToggleables(); });
    $('cbEscAutoOffset').addEventListener('change', updateEscOptions);
    $('cbCurrentSensorType').addEventListener('change', updateCurrentSensorUI);
    $('cbCurrentAutoOffset').addEventListener('change', updateCurrentSensorUI);
    $('cbBarometerType').addEventListener('change', updateEscOptions);
    $('cbGpsProtocol').addEventListener('change', updateEscOptions);

    /* Toggleable fieldsets */
    document.querySelectorAll('.toggleable legend input[type=checkbox]').forEach(chk => {
        chk.addEventListener('change', () => { updateToggleables(); if (typeof generateCircuit === 'function') generateCircuit(); });
    });

    /* Temperature checkbox triggers circuit redraw */
    $('cbTemperature1').addEventListener('change', () => { if (typeof generateCircuit === 'function') generateCircuit(); });

    /* Clock stretch triggers circuit redraw */
    $('cbClockStretch').addEventListener('change', () => { if (typeof generateCircuit === 'function') generateCircuit(); });
}
