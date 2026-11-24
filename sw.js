// DLMS Mini Service Worker - Version 12
const CACHE_NAME = 'dlms-v13';
const BASE_PATH = '/DLMS_mini_coustum/';
const ASSETS = [
  BASE_PATH,
  BASE_PATH + 'index.html',
  BASE_PATH + 'style.css?v=20261124-effects-compact1',
  BASE_PATH + 'script.js?v=20261124-effects-compact1',
  BASE_PATH + 'manifest.json'
];
self.addEventListener('install', event => {
  event.waitUntil((async () => {
    const cache = await caches.open(CACHE_NAME);
    await Promise.all(ASSETS.map(async url => {
      try {
        const response = await fetch(url, { cache: 'reload' });
        if (response.ok) await cache.put(url, response);
      } catch (error) { console.warn('[SW] Precache skipped:', url, error); }
    }));
    await self.skipWaiting();
  })());
});
self.addEventListener('activate', event => {
  event.waitUntil((async () => {
    const keys = await caches.keys();
    await Promise.all(keys.filter(key => key.startsWith('dlms-') && key !== CACHE_NAME).map(key => caches.delete(key)));
    await self.clients.claim();
  })());
});
self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET') return;
  const request = event.request;
  event.respondWith((async () => {
    try {
      const response = await fetch(request);
      if (response && response.ok && new URL(request.url).origin === self.location.origin) {
        const cache = await caches.open(CACHE_NAME);
        cache.put(request, response.clone()).catch(() => {});
      }
      return response;
    } catch (error) {
      const cached = await caches.match(request, { ignoreSearch: true });
      if (cached) return cached;
      if (request.mode === 'navigate') {
        const shell = await caches.match(BASE_PATH + 'index.html');
        if (shell) return shell;
      }
      return new Response('Offline: resource is not cached.', { status: 503, statusText: 'Service Unavailable', headers: { 'Content-Type': 'text/plain; charset=utf-8' } });
    }
  })());
});
