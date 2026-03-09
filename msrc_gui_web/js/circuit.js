/**
 * circuit.js — Canvas-based circuit diagram that composites PNG overlay images,
 *              matching the Qt generateCircuit() logic from mainwindow.cpp.
 */

const CIRCUIT_IMAGES = {};
const CIRCUIT_IMAGE_LIST = [
    'rp2040_zero.png', 'current_rp2040_zero.png', 'voltage_rp2040_zero.png',
    'ntc_rp2040_zero.png', 'airspeed_rp2040_zero.png', 'gps_rp2040_zero.png',
    'esc_rp2040_zero.png', 'pwm_rp2040_zero.png', 'castle_rp2040_zero.png',
    'smart_esc.png', 'vario_rp2040_zero.png', 'fuel_pressure.png', 'fuel_meter.png',
    'receiver_frsky_d_rp2040_zero.png', 'receiver_xbus_rp2040_zero.png',
    'receiver_hitec_rp2040_zero.png', 'receiver_serial_rp2040_zero.png',
    'clock_stretch_xbus_rp2040_zero.png'
];

let circuitZoom = 1;
let imagesLoaded = false;

function preloadCircuitImages() {
    let loaded = 0;
    return new Promise(resolve => {
        CIRCUIT_IMAGE_LIST.forEach(name => {
            const img = new Image();
            img.onload = img.onerror = () => {
                loaded++;
                if (loaded >= CIRCUIT_IMAGE_LIST.length) {
                    imagesLoaded = true;
                    resolve();
                }
            };
            img.src = 'res/' + name;
            CIRCUIT_IMAGES[name] = img;
        });
    });
}

function generateCircuit() {
    const canvas = document.getElementById('circuitCanvas');
    if (!canvas || !imagesLoaded) return;
    const ctx = canvas.getContext('2d');

    const base = CIRCUIT_IMAGES['rp2040_zero.png'];
    if (!base || !base.naturalWidth) return;

    canvas.width = base.naturalWidth * circuitZoom;
    canvas.height = base.naturalHeight * circuitZoom;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.scale(circuitZoom, circuitZoom);

    // Base image always drawn
    ctx.drawImage(base, 0, 0);

    const proto = parseInt(selectedValue(document.getElementById('cbReceiver')));
    const isSerial = proto === RX_PROTOCOL.SERIAL_MONITOR;

    if (!isSerial) {
        // Analog sensors
        if (document.getElementById('chkCurrent')?.checked)
            drawOverlay(ctx, 'current_rp2040_zero.png');
        if (document.getElementById('chkVoltage')?.checked)
            drawOverlay(ctx, 'voltage_rp2040_zero.png');
        if (document.getElementById('cbTemperature1')?.checked)
            drawOverlay(ctx, 'ntc_rp2040_zero.png');
        if (document.getElementById('chkAirspeed')?.checked)
            drawOverlay(ctx, 'airspeed_rp2040_zero.png');
        if (document.getElementById('chkGps')?.checked)
            drawOverlay(ctx, 'gps_rp2040_zero.png');

        // ESC
        if (document.getElementById('chkEsc')?.checked) {
            const escIdx = document.getElementById('cbEsc').selectedIndex;
            const escProto = escIdx + 1;
            if (escProto === ESC_PROTOCOL.ESC_PWM) {
                drawOverlay(ctx, 'pwm_rp2040_zero.png');
            } else if (escProto === ESC_PROTOCOL.ESC_CASTLE) {
                drawOverlay(ctx, 'castle_rp2040_zero.png');
            } else if (escProto === ESC_PROTOCOL.ESC_SMART) {
                drawOverlay(ctx, 'smart_esc.png');
            } else {
                drawOverlay(ctx, 'esc_rp2040_zero.png');
            }
        }

        // I2C sensors (vario, gyro, lipo all share same I2C bus image)
        if (document.getElementById('chkAltitude')?.checked)
            drawOverlay(ctx, 'vario_rp2040_zero.png');
        if (document.getElementById('chkGyro')?.checked)
            drawOverlay(ctx, 'vario_rp2040_zero.png');
        if (document.getElementById('chkLipo')?.checked)
            drawOverlay(ctx, 'vario_rp2040_zero.png');

        // Fuel
        if (document.getElementById('chkFuelPressure')?.checked)
            drawOverlay(ctx, 'fuel_pressure.png');
        if (document.getElementById('chkFuelmeter')?.checked)
            drawOverlay(ctx, 'fuel_meter.png');

        // Receiver overlay
        if (proto === RX_PROTOCOL.RX_FRSKY_D || proto === RX_PROTOCOL.RX_CRSF || proto === RX_PROTOCOL.RX_JETIEX_SENSOR) {
            drawOverlay(ctx, 'receiver_frsky_d_rp2040_zero.png');
        } else if (proto === RX_PROTOCOL.RX_XBUS) {
            drawOverlay(ctx, 'receiver_xbus_rp2040_zero.png');
        } else if (proto === RX_PROTOCOL.RX_HITEC) {
            drawOverlay(ctx, 'receiver_hitec_rp2040_zero.png');
        } else {
            drawOverlay(ctx, 'receiver_serial_rp2040_zero.png');
        }

        // XBUS clock stretch
        if (proto === RX_PROTOCOL.RX_XBUS && document.getElementById('cbClockStretch')?.checked) {
            drawOverlay(ctx, 'clock_stretch_xbus_rp2040_zero.png');
        }
    }

    ctx.restore();
}

function drawOverlay(ctx, name) {
    const img = CIRCUIT_IMAGES[name];
    if (img && img.naturalWidth) {
        ctx.drawImage(img, 0, 0);
    }
}

function initCircuit() {
    document.getElementById('btZoomIn')?.addEventListener('click', () => {
        circuitZoom = Math.min(circuitZoom + 0.25, 3);
        generateCircuit();
    });
    document.getElementById('btZoomOut')?.addEventListener('click', () => {
        circuitZoom = Math.max(circuitZoom - 0.25, 0.5);
        generateCircuit();
    });
}
