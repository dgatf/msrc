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
