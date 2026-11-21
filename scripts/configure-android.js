const fs = require('node:fs');
const path = require('node:path');

const manifestPath = path.resolve(__dirname, '..', 'android', 'app', 'src', 'main', 'AndroidManifest.xml');
if (!fs.existsSync(manifestPath)) {
  throw new Error('AndroidManifest.xml not found: ' + manifestPath);
}

let xml = fs.readFileSync(manifestPath, 'utf8');
const permissions = [
  '<uses-permission android:name="android.permission.BLUETOOTH" android:maxSdkVersion="30" />',
  '<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" android:maxSdkVersion="30" />',
  '<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" android:maxSdkVersion="30" />',
  '<uses-permission android:name="android.permission.BLUETOOTH_SCAN" android:usesPermissionFlags="neverForLocation" />',
  '<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />',
  '<uses-permission android:name="android.permission.RECORD_AUDIO" />'
];

for (const permission of permissions) {
  const match = permission.match(/android:name="([^"]+)"/);
  if (!match || xml.includes('android:name="' + match[1] + '"')) continue;
  const manifestOpen = xml.match(/<manifest\b[^>]*>/);
  if (!manifestOpen) throw new Error('Could not locate manifest opening tag');
  xml = xml.replace(manifestOpen[0], manifestOpen[0] + '\n    ' + permission);
}

fs.writeFileSync(manifestPath, xml, 'utf8');
console.log('Configured Bluetooth and microphone permissions in AndroidManifest.xml');
