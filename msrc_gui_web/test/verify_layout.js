// Test: verifies JS config_t layout matches C++ offsets.
// Run: docker run --rm -v $(pwd):/app -w /app node:20-alpine node test/verify_layout.js
const fs = require('fs');

const src = fs.readFileSync('js/config_struct.js', 'utf8');
const fn = new Function(src + '\nreturn { CONFIG_VERSION, RX_PROTOCOL, ESC_PROTOCOL, LAYOUT, CONFIG_SIZE, configDecode, configEncode, configDefault };');
const { LAYOUT, CONFIG_SIZE, configDecode, configEncode, configDefault } = fn();

const checks = {
    'version': 0, 'rx_protocol': 2, 'esc_protocol': 3, 'enable_gps': 4,
    'gps_baudrate': 8, 'enable_analog_voltage': 12, 'alpha_rpm': 20,
    'refresh_rate_rpm': 44, 'analog_voltage_multiplier': 64,
    'analog_current_type': 68, 'gpio_interval': 70,
    'analog_current_quiescent_voltage': 72, 'rpm_multiplier': 88,
    'smartport_data_id': 96, 'esc_hw4_init_delay_duration': 102,
    'esc_hw4_voltage_multiplier': 108, 'esc_hw4_offset': 124,
    'serial_monitor_baudrate': 128, 'serial_monitor_timeout_ms': 134,
    'airspeed_vcc': 138, 'fuel_flow_ml_per_pulse': 140,
    'enable_fuel_flow': 144, 'airspeed_offset': 158,
    'enable_gyro': 163, 'lipo_cells': 166, 'spare72': 167,
    'spare73': 168, 'spare8': 172, 'spare20': 220
};

const fm = {};
LAYOUT.fields.forEach(function(f) { fm[f.name] = f.offset; });

var pass = 0, fail = 0;
Object.keys(checks).forEach(function(name) {
    var expected = checks[name];
    var actual = fm[name];
    if (actual === expected) pass++;
    else { fail++; console.log('FAIL: ' + name + ' expected=' + expected + ' got=' + actual); }
});
console.log('CONFIG_SIZE=' + CONFIG_SIZE + ' expected=224 ' + (CONFIG_SIZE === 224 ? 'OK' : 'FAIL'));
console.log(pass + ' passed, ' + fail + ' failed');

// Roundtrip test
var cfg = configDefault();
var encoded = configEncode(cfg);
var decoded = configDecode(encoded);
var rt_pass = 0, rt_fail = 0;
LAYOUT.fields.forEach(function(f) {
    var orig = cfg[f.name] !== undefined ? cfg[f.name] : 0;
    var got = decoded[f.name];
    var origN = typeof orig === 'boolean' ? (orig ? 1 : 0) : orig;
    var gotN = typeof got === 'boolean' ? (got ? 1 : 0) : got;
    if (f.type === 'f32') {
        if (Math.abs(origN - gotN) < 0.0001) rt_pass++;
        else { rt_fail++; console.log('RT FAIL: ' + f.name + ' orig=' + origN + ' got=' + gotN); }
    } else {
        if (origN === gotN) rt_pass++;
        else { rt_fail++; console.log('RT FAIL: ' + f.name + ' orig=' + origN + ' got=' + gotN); }
    }
});
console.log('Roundtrip: ' + rt_pass + ' passed, ' + rt_fail + ' failed');
