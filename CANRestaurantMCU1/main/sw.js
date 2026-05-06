/* ─── Robot Control — Service Worker ──────────────────────────────────
   Caches index.html + manifest.json so the app works offline after the
   first visit.  Bump CACHE_NAME whenever you push a new version so
   Chrome fetches fresh files.
──────────────────────────────────────────────────────────────────── */
const CACHE_NAME = 'robot-v1';
const PRECACHE = [
  './',
  './index.html',
  './manifest.json',
];

self.addEventListener('install', function(e) {
  e.waitUntil(
    caches.open(CACHE_NAME).then(function(cache) {
      return cache.addAll(PRECACHE);
    }).then(function() {
      return self.skipWaiting();
    })
  );
});

self.addEventListener('activate', function(e) {
  e.waitUntil(
    caches.keys().then(function(keys) {
      return Promise.all(
        keys.filter(function(k) { return k !== CACHE_NAME; })
            .map(function(k) { return caches.delete(k); })
      );
    }).then(function() {
      return self.clients.claim();
    })
  );
});

/* Network-first for HTML so updates are seen immediately;
   cache fallback keeps it working offline. */
self.addEventListener('fetch', function(e) {
  e.respondWith(
    fetch(e.request).then(function(resp) {
      var clone = resp.clone();
      caches.open(CACHE_NAME).then(function(cache) {
        cache.put(e.request, clone);
      });
      return resp;
    }).catch(function() {
      return caches.match(e.request);
    })
  );
});
