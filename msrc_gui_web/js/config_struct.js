/**
 * config_struct.js — Binary serialization of config_t matching the C struct layout in include/shared.h.
 *
 * The firmware sends/receives config_t as raw bytes. We must pack/unpack identical binary
 * layout here. ARM Cortex-M0+ uses little-endian. Struct is packed with natural alignment
 * (no explicit __attribute__((packed)) on config_t itself, so compiler padding applies).
 *
 * We manually lay out every field with its exact byte offset, matching GCC ARM output.
 */

const CONFIG_VERSION = 2;

/* Enum mappings (same values as shared.h) */
const RX_PROTOCOL = {
    RX_SMARTPORT: 0, RX_FRSKY_D: 1, RX_XBUS: 2, RX_SRXL: 3, RX_IBUS: 4,
    RX_SBUS: 5, RX_MULTIPLEX: 6, RX_JETIEX: 7, RX_HITEC: 8, RX_SRXL2: 9,
    SERIAL_MONITOR: 10, RX_CRSF: 11, RX_HOTT: 12, RX_SANWA: 13, RX_JR_PROPO: 14,
    RX_FPORT: 15, RX_FBUS: 16, RX_GHST: 17, RX_JETIEX_SENSOR: 18
};

const ESC_PROTOCOL = {
    ESC_NONE: 0, ESC_HW3: 1, ESC_HW4: 2, ESC_PWM: 3, ESC_CASTLE: 4,
    ESC_KONTRONIK: 5, ESC_APD_F: 6, ESC_APD_HV: 7, ESC_HW5: 8, ESC_SMART: 9,
    ESC_OMP_M4: 10, ESC_ZTW: 11, ESC_OPENYGE: 12
};

/**
 * Build the field layout descriptor. Each entry is [name, type, byteOffset].
 * Types: 'u8','i8','bool','u16','i16','u32','i32','f32'
 * 
 * We compute offsets by walking through the struct exactly as GCC ARM would lay it out
 * with default alignment rules (natural alignment for each type).
 */
function buildLayout() {
    const fields = [];
    let offset = 0;

    function align(off, alignment) {
        return Math.ceil(off / alignment) * alignment;
    }

    function add(name, type) {
        let size, al;
        switch (type) {
            case 'u8': case 'i8': case 'bool': size = 1; al = 1; break;
            case 'u16': case 'i16': size = 2; al = 2; break;
            case 'u32': case 'i32': case 'f32': size = 4; al = 4; break;
            default: throw new Error('Unknown type: ' + type);
        }
        offset = align(offset, al);
        fields.push({ name, type, offset, size });
        offset += size;
    }

    // config_t fields in exact order from shared.h
    add('version', 'u16');                       // 0x5101
    add('rx_protocol', 'u8');                    // 0x5102
    add('esc_protocol', 'u8');                   // 0x5103
    add('enable_gps', 'bool');                   // 0x5104
    // padding to align gps_baudrate (u32) to 4-byte boundary
    add('gps_baudrate', 'u32');                  // 0x5105
    add('enable_analog_voltage', 'bool');        // 0x5106
    add('enable_analog_current', 'bool');        // 0x5107
    add('enable_analog_ntc', 'bool');            // 0x5108
    add('enable_analog_airspeed', 'bool');       // 0x5109
    add('i2c_module', 'u8');                     // 0x510A
    add('ina3221_filter', 'u8');                 // 0x510B
    add('alpha_rpm', 'f32');                     // 0x510C
    add('alpha_voltage', 'f32');                 // 0x510D
    add('alpha_current', 'f32');                 // 0x510E
    add('alpha_temperature', 'f32');             // 0x510F
    add('alpha_vario', 'f32');                   // 0x5110
    add('alpha_airspeed', 'f32');                // 0x5111
    add('refresh_rate_rpm', 'u16');              // 0x5112
    add('refresh_rate_voltage', 'u16');          // 0x5113
    add('refresh_rate_current', 'u16');          // 0x5114
    add('refresh_rate_temperature', 'u16');      // 0x5115
    add('refresh_rate_gps', 'u16');              // 0x5116
    add('refresh_rate_consumption', 'u16');      // 0x5117
    add('refresh_rate_vario', 'u16');            // 0x5118
    add('refresh_rate_airspeed', 'u16');         // 0x5119
    add('refresh_rate_default', 'u16');          // 0x511A
    add('analog_voltage_multiplier', 'f32');     // 0x511B
    add('analog_current_type', 'u8');            // 0x511C
    add('gpio_interval', 'u16');                 // 0x511D
    add('analog_current_quiescent_voltage', 'f32'); // 0x511E
    add('analog_current_multiplier', 'f32');     // 0x511F
    add('analog_current_offset', 'f32');         // 0x5120
    add('analog_current_autoffset', 'bool');     // 0x5121
    add('pairOfPoles', 'u8');                    // 0x5122
    add('mainTeeth', 'u8');                      // 0x5123
    add('pinionTeeth', 'u8');                    // 0x5124
    add('rpm_multiplier', 'f32');                // 0x5125
    add('bmp280_filter', 'u8');                  // 0x5126
    add('enable_pwm_out', 'bool');               // 0x5127
    add('smartport_sensor_id', 'u8');            // 0x5128
    add('smartport_data_id', 'u16');             // 0x5129
    add('vario_auto_offset', 'bool');            // 0x512A
    add('xbus_clock_stretch', 'bool');           // 0x512B
    add('jeti_gps_speed_units_kmh', 'bool');     // 0x512C
    add('enable_esc_hw4_init_delay', 'bool');    // 0x512D
    add('esc_hw4_init_delay_duration', 'u16');   // 0x512E
    add('esc_hw4_current_thresold', 'u8');       // 0x512F
    add('esc_hw4_current_max', 'u16');           // 0x5130
    add('esc_hw4_voltage_multiplier', 'f32');    // 0x5131
    add('esc_hw4_current_multiplier', 'f32');    // 0x5132
    add('ibus_alternative_coordinates', 'bool'); // 0x5133
    add('debug', 'u8');                          // 0x5134
    add('esc_hw4_is_manual_offset', 'bool');     // 0x5135
    add('analog_rate', 'u8');                    // 0x5136
    add('xbus_use_alternative_volt_temp', 'bool'); // 0x5137
    add('gpio_mask', 'u8');                      // 0x5138
    add('esc_hw4_offset', 'f32');                // 0x5139
    add('serial_monitor_baudrate', 'u32');       // 0x513A
    add('serial_monitor_stop_bits', 'u8');       // 0x513B
    add('serial_monitor_parity', 'u8');          // 0x513C
    add('serial_monitor_timeout_ms', 'u16');     // 0x513D
    add('serial_monitor_inverted', 'bool');      // 0x513E
    add('esc_hw4_auto_detect', 'bool');          // 0x514B
    add('airspeed_vcc', 'i16');                  // 0x5140
    add('fuel_flow_ml_per_pulse', 'f32');        // 0x5141
    add('enable_fuel_flow', 'bool');             // 0x5142
    add('xgzp68xxd_k', 'u16');                  // 0x5143
    add('enable_fuel_pressure', 'u8');           // 0x5144
    add('smart_esc_calc_consumption', 'bool');   // 0x5145
    add('serial_monitor_gpio', 'u8');            // 0x5146
    add('gps_rate', 'u8');                       // 0x5147
    add('serial_monitor_format', 'u8');          // 0x5148
    add('gps_protocol', 'u8');                   // 0x5149
    add('sbus_battery_slot', 'bool');            // 0x514A
    add('fport_inverted', 'bool');
    add('fbus_inverted', 'bool');
    add('airspeed_offset', 'i16');               // 0x513F
    add('mpu6050_acc_scale', 'u8');              // 0x514C
    add('mpu6050_gyro_scale', 'u8');             // 0x514D
    add('mpu6050_gyro_weighting', 'u8');         // 0x514E
    add('enable_gyro', 'bool');                  // 0x514F
    add('enable_lipo', 'u8');                    // 0x5150
    add('mpu6050_filter', 'u8');                 // 0x5151
    add('lipo_cells', 'u8');                     // 0x5152
    add('spare72', 'u8');
    add('spare73', 'u16');
    add('spare8', 'u32');
    add('spare9', 'u32');
    add('spare10', 'u32');
    add('spare11', 'u32');
    add('spare12', 'u32');
    add('spare13', 'u32');
    add('spare14', 'u32');
    add('spare15', 'u32');
    add('spare16', 'u32');
    add('spare17', 'u32');
    add('spare18', 'u32');
    add('spare19', 'u32');
    add('spare20', 'u32');

    return { fields, totalSize: align(offset, 4) }; // struct is padded to 4-byte alignment at end
}

const LAYOUT = buildLayout();
const CONFIG_SIZE = LAYOUT.totalSize;

/**
 * Decode a config_t from a Uint8Array (little-endian).
 * @param {Uint8Array} bytes
 * @returns {Object} Config object with named fields.
 */
function configDecode(bytes) {
    if (bytes.length < CONFIG_SIZE) {
        throw new Error(`Config buffer too small: ${bytes.length} < ${CONFIG_SIZE}`);
    }
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const cfg = {};
    for (const f of LAYOUT.fields) {
        switch (f.type) {
            case 'u8':   cfg[f.name] = dv.getUint8(f.offset); break;
            case 'i8':   cfg[f.name] = dv.getInt8(f.offset); break;
            case 'bool': cfg[f.name] = dv.getUint8(f.offset) !== 0; break;
            case 'u16':  cfg[f.name] = dv.getUint16(f.offset, true); break;
            case 'i16':  cfg[f.name] = dv.getInt16(f.offset, true); break;
            case 'u32':  cfg[f.name] = dv.getUint32(f.offset, true); break;
            case 'i32':  cfg[f.name] = dv.getInt32(f.offset, true); break;
            case 'f32':  cfg[f.name] = dv.getFloat32(f.offset, true); break;
        }
    }
    return cfg;
}

/**
 * Encode a config object to Uint8Array (little-endian).
 * @param {Object} cfg
 * @returns {Uint8Array}
 */
function configEncode(cfg) {
    const buf = new ArrayBuffer(CONFIG_SIZE);
    const dv = new DataView(buf);
    // Zero-fill already done by ArrayBuffer constructor
    for (const f of LAYOUT.fields) {
        const v = cfg[f.name] ?? 0;
        switch (f.type) {
            case 'u8':   dv.setUint8(f.offset, v & 0xFF); break;
            case 'i8':   dv.setInt8(f.offset, v); break;
            case 'bool': dv.setUint8(f.offset, v ? 1 : 0); break;
            case 'u16':  dv.setUint16(f.offset, v & 0xFFFF, true); break;
            case 'i16':  dv.setInt16(f.offset, v, true); break;
            case 'u32':  dv.setUint32(f.offset, v >>> 0, true); break;
            case 'i32':  dv.setInt32(f.offset, v, true); break;
            case 'f32':  dv.setFloat32(f.offset, v, true); break;
        }
    }
    return new Uint8Array(buf);
}

/**
 * Create a default config object.
 */
function configDefault() {
    return {
        version: CONFIG_VERSION,
        rx_protocol: RX_PROTOCOL.RX_SMARTPORT,
        esc_protocol: ESC_PROTOCOL.ESC_NONE,
        enable_gps: false,
        gps_baudrate: 9600,
        enable_analog_voltage: false,
        enable_analog_current: false,
        enable_analog_ntc: false,
        enable_analog_airspeed: false,
        i2c_module: 0,
        ina3221_filter: 1,
        alpha_rpm: 2.0 / (1 + 1),
        alpha_voltage: 2.0 / (1 + 1),
        alpha_current: 2.0 / (1 + 1),
        alpha_temperature: 2.0 / (1 + 1),
        alpha_vario: 2.0 / (1 + 1),
        alpha_airspeed: 2.0 / (1 + 1),
        refresh_rate_rpm: 1000,
        refresh_rate_voltage: 1000,
        refresh_rate_current: 1000,
        refresh_rate_temperature: 1000,
        refresh_rate_gps: 1000,
        refresh_rate_consumption: 1000,
        refresh_rate_vario: 1000,
        refresh_rate_airspeed: 1000,
        refresh_rate_default: 1000,
        analog_voltage_multiplier: 7.8,
        analog_current_type: 0,
        gpio_interval: 1000,
        analog_current_quiescent_voltage: 0,
        analog_current_multiplier: 1,
        analog_current_offset: 0,
        analog_current_autoffset: true,
        pairOfPoles: 1,
        mainTeeth: 1,
        pinionTeeth: 1,
        rpm_multiplier: 1,
        bmp280_filter: 3,
        enable_pwm_out: false,
        smartport_sensor_id: 15,
        smartport_data_id: 0x5100,
        vario_auto_offset: false,
        xbus_clock_stretch: false,
        jeti_gps_speed_units_kmh: true,
        enable_esc_hw4_init_delay: false,
        esc_hw4_init_delay_duration: 10000,
        esc_hw4_current_thresold: 10,
        esc_hw4_current_max: 250,
        esc_hw4_voltage_multiplier: 0.0088,
        esc_hw4_current_multiplier: 0.3,
        ibus_alternative_coordinates: false,
        debug: 0,
        esc_hw4_is_manual_offset: false,
        analog_rate: 10,
        xbus_use_alternative_volt_temp: false,
        gpio_mask: 0,
        esc_hw4_offset: 0,
        serial_monitor_baudrate: 19200,
        serial_monitor_stop_bits: 1,
        serial_monitor_parity: 0,
        serial_monitor_timeout_ms: 1,
        serial_monitor_inverted: false,
        esc_hw4_auto_detect: false,
        airspeed_vcc: 500,
        fuel_flow_ml_per_pulse: 0.01,
        enable_fuel_flow: false,
        xgzp68xxd_k: 64,
        enable_fuel_pressure: 0,
        smart_esc_calc_consumption: false,
        serial_monitor_gpio: 5,
        gps_rate: 1,
        serial_monitor_format: 0,
        gps_protocol: 0,
        sbus_battery_slot: true,
        fport_inverted: false,
        fbus_inverted: false,
        airspeed_offset: 0,
        mpu6050_acc_scale: 0,
        mpu6050_gyro_scale: 0,
        mpu6050_gyro_weighting: 96,
        enable_gyro: false,
        enable_lipo: 0,
        mpu6050_filter: 0,
        lipo_cells: 3,
        spare72: 0, spare73: 0,
        spare8: 0, spare9: 0, spare10: 0, spare11: 0, spare12: 0,
        spare13: 0, spare14: 0, spare15: 0, spare16: 0, spare17: 0,
        spare18: 0, spare19: 0, spare20: 0,
    };
}
