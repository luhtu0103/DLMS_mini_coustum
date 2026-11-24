// DLMS Mini Service Worker - Version 5 (Network-First Strategy)
const CACHE_NAME = 'dlms-v6';
const urlsToCache = [
  '/DLMS_mini_coustum/',
  '/DLMS_mini_coustum/index.html',
  '/DLMS_mini_coustum/style.css',
  '/DLMS_mini_coustum/script.js',
  '/DLMS_mini_coustum/manifest.json',
  '/DLMS_mini_coustum/icon-192.png'
];

// Install: Simpan file penting ke cache
self.addEventListener('install', event => {
  console.log('[SW] Installing new version:', CACHE_NAME);
  self.skipWaiting(); // Paksa SW baru langsung aktif
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => {
      console.log('[SW] Caching essential files');
      return cache.addAll(urlsToCache);
    })
  );
});

// Activate: Hapus cache lama
self.addEventListener('activate', event => {
  console.log('[SW] Activating new version:', CACHE_NAME);
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames.map(cacheName => {
          if (cacheName !== CACHE_NAME) {
            console.log('[SW] Deleting old cache:', cacheName);
            return caches.delete(cacheName);
          }
        })
      );
    }).then(() => {
      console.log('[SW] Taking control of all pages');
      return self.clients.claim(); // Ambil kontrol semua tab yang terbuka
    })
  );
});

// Fetch: Network-First Strategy (Prioritas file terbaru dari server)
self.addEventListener('fetch', event => {
  // Hanya handle request GET
  if (event.request.method !== 'GET') return;

  event.respondWith(
    fetch(event.request)
      .then(response => {
        // Jika dapat dari network, simpan ke cache untuk offline
        if (response.status === 200) {
          const responseClone = response.clone();
          caches.open(CACHE_NAME).then(cache => {
            cache.put(event.request, responseClone);
          });
        }
        return response;
      })
      .catch(() => {
        // Jika offline, ambil dari cache
        console.log('[SW] Network failed, serving from cache');
        return caches.match(event.request);
      })
  );
});
