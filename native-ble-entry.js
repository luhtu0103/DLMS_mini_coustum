import { BleClient } from '@capacitor-community/bluetooth-le';
import { registerPlugin } from '@capacitor/core';

// Expose the native runtime microphone permission bridge to the web UI.
window.DLMSMicrophonePermission = registerPlugin('MicrophonePermission');

const SERVICE_UUID = '0000ffe0-0000-1000-8000-00805f9b34fb';
const CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb';

let initialized = false;
let device = null;
let connected = false;

async function initialize() {
  if (initialized) return;
  await BleClient.initialize();
  initialized = true;
}

export const nativeBLE = {
  isConnected() {
    return connected && !!device;
  },

  getDevice() {
    return device;
  },

  async connect(onData, onDisconnect) {
    await initialize();
    device = await BleClient.requestDevice({ services: [SERVICE_UUID] });
    await BleClient.connect(device.deviceId, () => {
      connected = false;
      if (typeof onDisconnect === 'function') onDisconnect();
    });
    connected = true;

    try {
      await BleClient.startNotifications(
        device.deviceId,
        SERVICE_UUID,
        CHARACTERISTIC_UUID,
        (value) => {
          if (typeof onData === 'function') onData(value);
        }
      );
    } catch (error) {
      try { await BleClient.disconnect(device.deviceId); } catch (_) {}
      connected = false;
      device = null;
      throw error;
    }

    return { deviceId: device.deviceId, name: device.name || 'ESP32-S3' };
  },

  async write(text) {
    if (!device || !connected) throw new Error('BLE belum terhubung');
    const bytes = new TextEncoder().encode(String(text));
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    await BleClient.write(device.deviceId, SERVICE_UUID, CHARACTERISTIC_UUID, view);
  },

  async disconnect() {
    if (device) {
      const id = device.deviceId;
      connected = false;
      device = null;
      try { await BleClient.disconnect(id); } catch (_) {}
    }
  }
};
