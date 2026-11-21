const fs = require('node:fs');
const path = require('node:path');

const root = path.resolve(__dirname, '..');
const out = path.join(root, 'www');
const required = ['index.html', 'style.css', 'script.js'];
const optional = ['manifest.json', 'icon-192.png'];

for (const file of required) {
  if (!fs.existsSync(path.join(root, file))) {
    throw new Error('Required web asset missing: ' + file);
  }
}

fs.rmSync(out, { recursive: true, force: true });
fs.mkdirSync(out, { recursive: true });

for (const file of [...required, ...optional]) {
  const src = path.join(root, file);
  if (fs.existsSync(src) && fs.statSync(src).isFile()) {
    fs.copyFileSync(src, path.join(out, file));
  }
}

// Load the generated native BLE adapter only in the packaged Android app.
const htmlPath = path.join(out, 'index.html');
let html = fs.readFileSync(htmlPath, 'utf8');
const scriptTag = '<script src="script.js"></script>';
if (!html.includes(scriptTag)) {
  throw new Error('Could not find script.js tag in index.html');
}
html = html.replace(scriptTag, '<script src="native-ble-bundle.js"></script>\\n    ' + scriptTag);
fs.writeFileSync(htmlPath, html, 'utf8');

console.log('Prepared Capacitor web assets:', fs.readdirSync(out).join(', '));
