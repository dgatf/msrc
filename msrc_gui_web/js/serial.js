/**
 * serial.js — Web Serial API communication with MSRC board.
 *
 * Protocol: all messages start with header byte 0x30.
 * Commands:
 *   GUI→Board 0x30 0x30 + config_t bytes  → write config to flash
 *   GUI→Board 0x30 0x31                    → request current config
 *   Board→GUI 0x30 0x32 + config_t bytes   → config response
 *   GUI→Board 0x30 0x33                    → enable debug output
 *   GUI→Board 0x30 0x34                    → disable debug output
 *   GUI→Board 0x30 0x35                    → force write default config
 */

const HEADER = 0x30;
const CMD_WRITE_CONFIG = 0x30;
const CMD_REQUEST_CONFIG = 0x31;
const CMD_CONFIG_RESPONSE = 0x32;
const CMD_DEBUG_ON = 0x33;
const CMD_DEBUG_OFF = 0x34;
const CMD_DEFAULT_CONFIG = 0x35;

class SerialConnection {
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.readableStreamClosed = null;
        this.writableStreamClosed = null;
        this.connected = false;
        this.debugMode = false;
        this._rxBuffer = new Uint8Array(0);
        this._onConfigReceived = null;
        this._onDebugData = null;
        this._onDisconnect = null;
        this._reading = false;
    }

    static isSupported() {
        return 'serial' in navigator;
    }

    async requestPort() {
        this.port = await navigator.serial.requestPort();
        return this.port;
    }

    async connect() {
        if (!this.port) throw new Error('No port selected');
        await this.port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none' });
        this.connected = true;
        this._startReading();
    }

    async disconnect() {
        this._reading = false;
        if (this.reader) {
            try { await this.reader.cancel(); } catch (e) { /* ignore */ }
            this.reader = null;
        }
        if (this.port) {
            try { await this.port.close(); } catch (e) { /* ignore */ }
        }
        this.connected = false;
        this._rxBuffer = new Uint8Array(0);
    }

    async _startReading() {
        this._reading = true;
        while (this._reading && this.port?.readable) {
            this.reader = this.port.readable.getReader();
            try {
                while (this._reading) {
                    const { value, done } = await this.reader.read();
                    if (done) break;
                    if (value) this._processBytes(value);
                }
            } catch (e) {
                if (this._reading) {
                    console.error('Serial read error:', e);
                }
            } finally {
                try { this.reader.releaseLock(); } catch (e) { /* ignore */ }
                this.reader = null;
            }
        }
        if (this.connected) {
            this.connected = false;
            if (this._onDisconnect) this._onDisconnect();
        }
    }

    _processBytes(chunk) {
        if (this.debugMode) {
            if (this._onDebugData) {
                this._onDebugData(new TextDecoder().decode(chunk));
            }
            return;
        }

        // Accumulate bytes for config response
        const merged = new Uint8Array(this._rxBuffer.length + chunk.length);
        merged.set(this._rxBuffer);
        merged.set(chunk, this._rxBuffer.length);
        this._rxBuffer = merged;

        // Config response = 0x30 0x32 + CONFIG_SIZE bytes
        const expectedLen = CONFIG_SIZE + 2;
        if (this._rxBuffer.length >= expectedLen) {
            // Search for header
            for (let i = 0; i <= this._rxBuffer.length - expectedLen; i++) {
                if (this._rxBuffer[i] === HEADER && this._rxBuffer[i + 1] === CMD_CONFIG_RESPONSE) {
                    const configBytes = this._rxBuffer.slice(i + 2, i + 2 + CONFIG_SIZE);
                    this._rxBuffer = this._rxBuffer.slice(i + 2 + CONFIG_SIZE);
                    if (this._onConfigReceived) {
                        this._onConfigReceived(configBytes);
                    }
                    return;
                }
            }
            // If buffer is way too large without finding pattern, trim
            if (this._rxBuffer.length > expectedLen * 3) {
                this._rxBuffer = this._rxBuffer.slice(-expectedLen);
            }
        }
    }

    async _write(bytes) {
        if (!this.port?.writable) throw new Error('Port not writable');
        const writer = this.port.writable.getWriter();
        try {
            await writer.write(new Uint8Array(bytes));
        } finally {
            writer.releaseLock();
        }
    }

    async requestConfig() {
        this._rxBuffer = new Uint8Array(0);
        await this._write([HEADER, CMD_REQUEST_CONFIG]);
    }

    async writeConfig(configBytes) {
        const packet = new Uint8Array(2 + configBytes.length);
        packet[0] = HEADER;
        packet[1] = CMD_WRITE_CONFIG;
        packet.set(configBytes, 2);
        await this._write(packet);
    }

    async enableDebug() {
        this.debugMode = true;
        this._rxBuffer = new Uint8Array(0);
        await this._write([HEADER, CMD_DEBUG_ON]);
    }

    async disableDebug() {
        this.debugMode = false;
        await this._write([HEADER, CMD_DEBUG_OFF]);
    }

    async writeDefaultConfig() {
        await this._write([HEADER, CMD_DEFAULT_CONFIG]);
    }

    set onConfigReceived(fn) { this._onConfigReceived = fn; }
    set onDebugData(fn) { this._onDebugData = fn; }
    set onDisconnect(fn) { this._onDisconnect = fn; }
}

/**
 * WebUSB fallback for platforms without Web Serial (e.g. Android).
 * Talks to the RP2040 USB CDC ACM device at the raw USB level.
 * Same public interface as SerialConnection.
 */

// USB CDC Class requests
const CDC_SET_LINE_CODING = 0x20;
const CDC_SET_CONTROL_LINE_STATE = 0x22;

// RP2040 default Pico SDK USB VID/PID
const PICO_VID = 0x2E8A;
const PICO_PID_CDC = 0x000A;

class WebUSBConnection {
    constructor() {
        this.device = null;
        this.interfaceNumber = -1;
        this.endpointIn = -1;
        this.endpointOut = -1;
        this.connected = false;
        this.debugMode = false;
        this._rxBuffer = new Uint8Array(0);
        this._onConfigReceived = null;
        this._onDebugData = null;
        this._onDisconnect = null;
        this._reading = false;
    }

    static isSupported() {
        return 'usb' in navigator;
    }

    async requestPort() {
        this.device = await navigator.usb.requestDevice({
            filters: [{ vendorId: PICO_VID }]
        });
        return this.device;
    }

    async connect() {
        if (!this.device) throw new Error('No device selected');
        await this.device.open();

        // Select configuration 1 if not already
        if (this.device.configuration === null) {
            await this.device.selectConfiguration(1);
        }

        // Find the CDC data interface (class 0x0A = CDC Data)
        this._findEndpoints();

        await this.device.claimInterface(this.interfaceNumber);

        // SET_LINE_CODING: 115200 baud, 1 stop bit, no parity, 8 data bits
        const lineCoding = new ArrayBuffer(7);
        const dv = new DataView(lineCoding);
        dv.setUint32(0, 115200, true); // dwDTERate
        dv.setUint8(4, 0);             // bCharFormat: 1 stop bit
        dv.setUint8(5, 0);             // bParityType: none
        dv.setUint8(6, 8);             // bDataBits

        // Control interface is typically interfaceNumber - 1
        const controlInterface = this.interfaceNumber > 0 ? this.interfaceNumber - 1 : 0;
        await this.device.controlTransferOut({
            requestType: 'class',
            recipient: 'interface',
            request: CDC_SET_LINE_CODING,
            value: 0,
            index: controlInterface
        }, new Uint8Array(lineCoding));

        // SET_CONTROL_LINE_STATE: activate DTR
        await this.device.controlTransferOut({
            requestType: 'class',
            recipient: 'interface',
            request: CDC_SET_CONTROL_LINE_STATE,
            value: 0x01, // DTR = 1
            index: controlInterface
        });

        this.connected = true;
        this._startReading();
    }

    _findEndpoints() {
        for (const iface of this.device.configuration.interfaces) {
            for (const alt of iface.alternates) {
                // CDC Data class = 0x0A
                if (alt.interfaceClass === 0x0A) {
                    this.interfaceNumber = iface.interfaceNumber;
                    for (const ep of alt.endpoints) {
                        if (ep.direction === 'in') this.endpointIn = ep.endpointNumber;
                        if (ep.direction === 'out') this.endpointOut = ep.endpointNumber;
                    }
                    return;
                }
            }
        }
        // Fallback: try any interface with bulk endpoints
        for (const iface of this.device.configuration.interfaces) {
            for (const alt of iface.alternates) {
                let hasIn = false, hasOut = false;
                for (const ep of alt.endpoints) {
                    if (ep.type === 'bulk' && ep.direction === 'in') hasIn = true;
                    if (ep.type === 'bulk' && ep.direction === 'out') hasOut = true;
                }
                if (hasIn && hasOut) {
                    this.interfaceNumber = iface.interfaceNumber;
                    for (const ep of alt.endpoints) {
                        if (ep.type === 'bulk' && ep.direction === 'in') this.endpointIn = ep.endpointNumber;
                        if (ep.type === 'bulk' && ep.direction === 'out') this.endpointOut = ep.endpointNumber;
                    }
                    return;
                }
            }
        }
        throw new Error('No CDC data interface found on device');
    }

    async disconnect() {
        this._reading = false;
        if (this.device) {
            try {
                await this.device.releaseInterface(this.interfaceNumber);
                await this.device.close();
            } catch (e) { /* ignore */ }
        }
        this.connected = false;
        this._rxBuffer = new Uint8Array(0);
    }

    async _startReading() {
        this._reading = true;
        while (this._reading && this.device?.opened) {
            try {
                const result = await this.device.transferIn(this.endpointIn, 64);
                if (result.status === 'ok' && result.data && result.data.byteLength > 0) {
                    this._processBytes(new Uint8Array(result.data.buffer));
                }
                if (result.status === 'stall') {
                    await this.device.clearHalt('in', this.endpointIn);
                }
            } catch (e) {
                if (this._reading) {
                    console.error('WebUSB read error:', e);
                    break;
                }
            }
        }
        if (this.connected) {
            this.connected = false;
            if (this._onDisconnect) this._onDisconnect();
        }
    }

    _processBytes(chunk) {
        if (this.debugMode) {
            if (this._onDebugData) {
                this._onDebugData(new TextDecoder().decode(chunk));
            }
            return;
        }
        const merged = new Uint8Array(this._rxBuffer.length + chunk.length);
        merged.set(this._rxBuffer);
        merged.set(chunk, this._rxBuffer.length);
        this._rxBuffer = merged;

        const expectedLen = CONFIG_SIZE + 2;
        if (this._rxBuffer.length >= expectedLen) {
            for (let i = 0; i <= this._rxBuffer.length - expectedLen; i++) {
                if (this._rxBuffer[i] === HEADER && this._rxBuffer[i + 1] === CMD_CONFIG_RESPONSE) {
                    const configBytes = this._rxBuffer.slice(i + 2, i + 2 + CONFIG_SIZE);
                    this._rxBuffer = this._rxBuffer.slice(i + 2 + CONFIG_SIZE);
                    if (this._onConfigReceived) this._onConfigReceived(configBytes);
                    return;
                }
            }
            if (this._rxBuffer.length > expectedLen * 3) {
                this._rxBuffer = this._rxBuffer.slice(-expectedLen);
            }
        }
    }

    async _write(bytes) {
        if (!this.device?.opened) throw new Error('Device not open');
        await this.device.transferOut(this.endpointOut, new Uint8Array(bytes));
    }

    async requestConfig() {
        this._rxBuffer = new Uint8Array(0);
        await this._write([HEADER, CMD_REQUEST_CONFIG]);
    }

    async writeConfig(configBytes) {
        const packet = new Uint8Array(2 + configBytes.length);
        packet[0] = HEADER;
        packet[1] = CMD_WRITE_CONFIG;
        packet.set(configBytes, 2);
        await this._write(packet);
    }

    async enableDebug() {
        this.debugMode = true;
        this._rxBuffer = new Uint8Array(0);
        await this._write([HEADER, CMD_DEBUG_ON]);
    }

    async disableDebug() {
        this.debugMode = false;
        await this._write([HEADER, CMD_DEBUG_OFF]);
    }

    async writeDefaultConfig() {
        await this._write([HEADER, CMD_DEFAULT_CONFIG]);
    }

    set onConfigReceived(fn) { this._onConfigReceived = fn; }
    set onDebugData(fn) { this._onDebugData = fn; }
    set onDisconnect(fn) { this._onDisconnect = fn; }
}

/**
 * Factory: returns the best available connection for this platform.
 * - Desktop Chrome/Edge → SerialConnection (Web Serial API)
 * - Android Chrome → WebUSBConnection (WebUSB API)
 */
function createConnection() {
    if (SerialConnection.isSupported()) return new SerialConnection();
    if (WebUSBConnection.isSupported()) return new WebUSBConnection();
    return null;
}
