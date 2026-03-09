/**
 * sw.js — Service Worker for offline PWA support.
 * Caches all app assets on install, serves from cache first.
 */

importScripts('./js/version.js');
const CACHE_NAME = 'msrc-link-' + APP_VERSION;

const ASSETS = [
    './',
    './index.html',
    './css/style.css',
    './js/version.js',
    './js/config_struct.js',
    './js/serial.js',
    './js/circuit.js',
    './js/ui.js',
    './js/app.js',
    './manifest.json',
    './res/msrc.png',
    './res/msrc-192.png',
    './res/msrc-512.png',
    './res/rp2040_zero.png',
    './res/current_rp2040_zero.png',
    './res/voltage_rp2040_zero.png',
    './res/ntc_rp2040_zero.png',
    './res/airspeed_rp2040_zero.png',
    './res/gps_rp2040_zero.png',
    './res/esc_rp2040_zero.png',
    './res/pwm_rp2040_zero.png',
    './res/castle_rp2040_zero.png',
    './res/smart_esc.png',
    './res/vario_rp2040_zero.png',
    './res/fuel_pressure.png',
    './res/fuel_meter.png',
    './res/receiver_frsky_d_rp2040_zero.png',
    './res/receiver_xbus_rp2040_zero.png',
    './res/receiver_hitec_rp2040_zero.png',
    './res/receiver_serial_rp2040_zero.png',
    './res/clock_stretch_xbus_rp2040_zero.png'
];

self.addEventListener('install', event => {
    event.waitUntil(
        caches.open(CACHE_NAME).then(cache => cache.addAll(ASSETS))
    );
    self.skipWaiting();
});

self.addEventListener('activate', event => {
    event.waitUntil(
        caches.keys().then(keys =>
            Promise.all(keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k)))
        )
    );
    self.clients.claim();
});

self.addEventListener('fetch', event => {
    event.respondWith(
        caches.match(event.request).then(cached => cached || fetch(event.request))
    );
});
