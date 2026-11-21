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
  '<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />',
  '<uses-permission android:name="android.permission.BLUETOOTH_SCAN" android:usesPermissionFlags="neverForLocation" />',
  '<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />',
  '<uses-permission android:name="android.permission.RECORD_AUDIO" />',
  '<uses-permission android:name="android.permission.MODIFY_AUDIO_SETTINGS" />'
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
function findMainActivity(dir) {
  if (!fs.existsSync(dir)) return null;
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      const found = findMainActivity(full);
      if (found) return found;
    } else if (entry.name === 'MainActivity.java') return full;
  }
  return null;
}

const javaRoot = path.resolve(__dirname, '..', 'android', 'app', 'src', 'main', 'java');
const mainActivityPath = findMainActivity(javaRoot);
if (!mainActivityPath) throw new Error('MainActivity.java not found under ' + javaRoot);
let mainActivity = fs.readFileSync(mainActivityPath, 'utf8');
const packageMatch = mainActivity.match(/^package\s+([\w.]+);/m);
if (!packageMatch) throw new Error('Could not determine MainActivity package');
const packageName = packageMatch[1];
const pluginPath = path.join(path.dirname(mainActivityPath), 'MicrophonePermissionPlugin.java');
const pluginSource = `package ${packageName};

import android.Manifest;
import com.getcapacitor.JSObject;
import com.getcapacitor.PermissionState;
import com.getcapacitor.Plugin;
import com.getcapacitor.PluginCall;
import com.getcapacitor.PluginMethod;
import com.getcapacitor.annotation.CapacitorPlugin;
import com.getcapacitor.annotation.Permission;
import com.getcapacitor.annotation.PermissionCallback;

@CapacitorPlugin(name = "MicrophonePermission", permissions = {
    @Permission(alias = "microphone", strings = { Manifest.permission.RECORD_AUDIO })
})
public class MicrophonePermissionPlugin extends Plugin {
    @PluginMethod
    public void request(PluginCall call) {
        if (getPermissionState("microphone") == PermissionState.GRANTED) {
            resolveGranted(call, true);
        } else {
            requestPermissionForAlias("microphone", call, "permissionCallback");
        }
    }

    @PermissionCallback
    private void permissionCallback(PluginCall call) {
        resolveGranted(call, getPermissionState("microphone") == PermissionState.GRANTED);
    }

    private void resolveGranted(PluginCall call, boolean granted) {
        JSObject result = new JSObject();
        result.put("granted", granted);
        call.resolve(result);
    }
}
`;
fs.writeFileSync(pluginPath, pluginSource, 'utf8');
if (!mainActivity.includes('MicrophonePermissionPlugin.class')) {
  if (!mainActivity.includes('import android.os.Bundle;')) {
    mainActivity = mainActivity.replace(packageMatch[0], packageMatch[0] + '\n\nimport android.os.Bundle;');
  }
  const classClose = mainActivity.lastIndexOf('}');
  if (classClose < 0) throw new Error('Could not locate MainActivity class closing brace');
  const onCreate = `
    @Override
    public void onCreate(Bundle savedInstanceState) {
        registerPlugin(MicrophonePermissionPlugin.class);
        super.onCreate(savedInstanceState);
    }
`;
  mainActivity = mainActivity.slice(0, classClose) + onCreate + mainActivity.slice(classClose);
  fs.writeFileSync(mainActivityPath, mainActivity, 'utf8');
}
console.log('Configured native Android microphone permission bridge');

