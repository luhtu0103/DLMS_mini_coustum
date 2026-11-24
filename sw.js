// DLMS Mini Service Worker - Version 7 (network-first, version-aware offline cache)
const CACHE_NAME = 'dlms-v7';
const BASE_PATH = '/DLMS_mini_coustum/';
const urlsToCache = [
  BASE_PATH,
  BASE_PATH + 'index.html',
  BASE_PATH + 'style.css?v=20261124-audit1',
  BASE_PATH + 'script.js?v=20261124-2',
  BASE_PATH + 'manifest.json',
  BASE_PATH + 'icon-192.png'
];

self.addEventListener('install', event => {
  console.log('[SW] Installing:', CACHE_NAME);
  self.skipWaiting();
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => cache.addAll(urlsToCache))
  );
});

self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(cacheNames => Promise.all(
      cacheNames.map(cacheName => cacheName !== CACHE_NAME ? caches.delete(cacheName) : Promise.resolve())
    )).then(() => self.clients.claim())
  );
});

// Network-first for fresh content; ignore query strings only as an offline fallback.
self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET') return;

  event.respondWith(
    fetch(event.request).then(response => {
      if (response && response.status === 200) {
        const responseClone = response.clone();
        caches.open(CACHE_NAME).then(cache => cache.put(event.request, responseClone));
      }
      return response;
    }).catch(() => caches.match(event.request, { ignoreSearch: true }).then(cached => {
      if (cached) return cached;
      if (event.request.mode === 'navigate') {
        return caches.match(BASE_PATH + 'index.html');
      }
      return new Response('Offline: resource is not cached.', {
        status: 503,
        statusText: 'Service Unavailable',
        headers: { 'Content-Type': 'text/plain; charset=utf-8' }
      });
    }))
  );
});
