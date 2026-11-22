        let bleDevice, bleCharacteristic;
        let bleNativeConnected = false;

        function getNativeBLE() {
            if (!(window.Capacitor && typeof window.Capacitor.isNativePlatform === 'function' && window.Capacitor.isNativePlatform())) return null;
            return window.DLMSNativeBLEBundle && window.DLMSNativeBLEBundle.nativeBLE ? window.DLMSNativeBLEBundle.nativeBLE : null;
        }

        function isBLEConnected() {
            const nativeBLE = getNativeBLE();
            if (nativeBLE) return bleNativeConnected && nativeBLE.isConnected();
            return !!(bleDevice && bleDevice.gatt && bleDevice.gatt.connected);
        }
        const SERVICE_UUID = '0000ffe0-0000-1000-8000-00805f9b34fb';
        const CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb';

        let currentOut = 1;
        let currentInput = 'L';
        let saveMode = false;
        let dashActiveOut = 1;
        let graphMode = 'RTA'; 
        let isLinked = false; 
        let oscData = new Array(50).fill(50); 

        const bleCommandQueue = [];
        let bleQueueBusy = false;

        let globalState = {
            1: { hpfOn: true, hpfFreq: 80, hpfSlope: 24, lpfOn: false, lpfFreq: 20000, lpfSlope: 24, p1f: 100, p1g: 0, p2f: 1000, p2g: 0, p3f: 5000, p3g: 0, gain: 0, delay: 0, limitOn: true, limit: 0, routing: 0, isMute: false, phase180: false, compOn: false, compThresh: -20, compRatio: 4, compAtk: 10, compRel: 200 },
            2: { hpfOn: true, hpfFreq: 80, hpfSlope: 24, lpfOn: false, lpfFreq: 20000, lpfSlope: 24, p1f: 100, p1g: 0, p2f: 1000, p2g: 0, p3f: 5000, p3g: 0, gain: 0, delay: 0, limitOn: true, limit: 0, routing: 1, isMute: false, phase180: false, compOn: false, compThresh: -20, compRatio: 4, compAtk: 10, compRel: 200 },
            3: { hpfOn: false, hpfFreq: 80, hpfSlope: 24, lpfOn: true, lpfFreq: 100, lpfSlope: 24, p1f: 100, p1g: 0, p2f: 1000, p2g: 0, p3f: 5000, p3g: 0, gain: 0, delay: 0, limitOn: true, limit: 0, routing: 0, isMute: false, phase180: false, compOn: false, compThresh: -20, compRatio: 4, compAtk: 10, compRel: 200 },
            4: { hpfOn: false, hpfFreq: 80, hpfSlope: 24, lpfOn: true, lpfFreq: 85, lpfSlope: 24, p1f: 100, p1g: 0, p2f: 1000, p2g: 0, p3f: 5000, p3g: 0, gain: 0, delay: 0, limitOn: true, limit: 0, routing: 1, isMute: false, phase180: false, compOn: false, compThresh: -20, compRatio: 4, compAtk: 10, compRel: 200 }
        };

        let inputState = {
            L: { gain: 0, peqFreq: 1000, peqGain: 0, isMute: false },
            R: { gain: 0, peqFreq: 1000, peqGain: 0, isMute: false }
        };

        function editDeviceName() {
            let currentName = document.getElementById('dash-title').innerText;
            let newName = prompt("Masukkan Nama Perangkat DLMS Anda:", currentName);
            if (newName !== null && newName.trim() !== "") {
                document.getElementById('dash-title').innerText = newName.trim();
                localStorage.setItem('DLMS_DEVICE_NAME', newName.trim());
            }
        }

        function stepValue(sliderId, step) {
            let slider = document.getElementById(sliderId);
            let val = parseFloat(slider.value) + step;
            if(val < parseFloat(slider.min)) val = parseFloat(slider.min);
            if(val > parseFloat(slider.max)) val = parseFloat(slider.max);
            val = Math.round(val * 100) / 100;
            slider.value = val;
            slider.dispatchEvent(new Event('input')); 
        }

        async function connectBLE() {
            if (isBLEConnected()) {
                alert('Sudah terhubung ke ' + (bleDevice?.name || 'perangkat BLE'));
                return;
            }
            try {
                const nativeBLE = getNativeBLE();
                if (nativeBLE) {
                    bleDevice = await nativeBLE.connect(
                        value => handleBLEMessage({ target: { value } }),
                        onDisconnected
                    );
                    bleNativeConnected = true;
                } else {
                    bleDevice = await navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE_UUID] }] });
                    const server = await bleDevice.gatt.connect();
                    const service = await server.getPrimaryService(SERVICE_UUID);
                    bleCharacteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);
                    await bleCharacteristic.startNotifications();
                    bleCharacteristic.addEventListener('characteristicvaluechanged', handleBLEMessage);
                    bleDevice.addEventListener('gattserverdisconnected', onDisconnected);
                }
                document.body.classList.add('ble-active');
                updateBLEStatus(true);
                // Sinkronkan seluruh state UI hanya sekali saat koneksi baru berhasil.
                await syncFullStateToDevice();
                alert('✅ Terhubung ke ' + (bleDevice.name || 'ESP32-S3'));
            } catch (error) {
                console.error('Error BLE:', error);
                if (error.name !== 'NotFoundError') {
                    alert('❌ Gagal terhubung: ' + error.message);
                }
            }
        }

        function onDisconnected() {
            // Stop generator locally where possible; firmware also stops it on disconnect.
            currentToneMode = 'off';
            bleNativeConnected = false;
            bleCommandQueue.length = 0;
            document.body.classList.remove('ble-active');
            document.body.classList.remove('reactive-glow', 'reactive-bass');
            document.querySelectorAll('.led-block, .mini-led').forEach(led => { led.style.backgroundColor = ''; led.style.boxShadow = ''; });
            document.querySelectorAll('#dash-rta-bars rect').forEach(rect => { rect.setAttribute('height', 0); rect.setAttribute('y', 100); });
            updateBLEStatus(false);
            updateToneUIOff();
        }
        
        function updateBLEStatus(isConnected) {
            const btn = document.getElementById('btnConnectBLE'); const dot = document.getElementById('bleStatusDot'); const text = document.getElementById('bleStatusText');
            if (isConnected) { btn.innerText = 'CONNECTED'; btn.classList.add('connected'); dot.classList.add('on'); text.innerText = 'BLE: CONNECTED (' + (bleDevice?.name || 'ESP32-S3') + ')'; }
            else { btn.innerText = 'CONNECT BLE'; btn.classList.remove('connected'); dot.classList.remove('on'); text.innerText = 'BLE: DISCONNECTED'; bleDevice = null; bleCharacteristic = null; bleNativeConnected = false; }
        }

        async function sendBLEData(text) {
            if (!isBLEConnected()) return;
            const command = String(text);
            const commandKey = command.split(',')[0];
            // Coalesce pending updates for the same control while dragging sliders.
            const queuedIndex = bleCommandQueue.findIndex(item => item.split(',')[0] === commandKey);
            if (queuedIndex >= 0) bleCommandQueue[queuedIndex] = command;
            else bleCommandQueue.push(command);
            if (bleQueueBusy) return;
            bleQueueBusy = true;
            try {
                while (bleCommandQueue.length && isBLEConnected()) {
                    const command = bleCommandQueue.shift();
                    const nativeBLE = getNativeBLE();
                    if (nativeBLE) await nativeBLE.write(command + '\n');
                    else await bleCharacteristic.writeValue(new TextEncoder().encode(command + '\n'));
                    await new Promise(resolve => setTimeout(resolve, 12));
                }
            } catch (error) {
                console.error('Gagal kirim BLE:', error);
                bleCommandQueue.length = 0;
            } finally {
                bleQueueBusy = false;
            }
        }

        function toggleComp(btn) {
            btn.classList.toggle('on');
            const isOn = btn.classList.contains('on');
            btn.innerText = isOn ? 'ON' : 'OFF';
            globalState[currentOut].compOn = isOn;
            sendBLEData('OUT' + currentOut + '_COMP_ON,' + (isOn ? 1 : 0));
            applyGlobalLink(currentOut);
        }

        function updateComp(param, val) {
            const st = globalState[currentOut];
            if (param === 'COMP_THRESH') st.compThresh = Number(val);
            if (param === 'COMP_RATIO') st.compRatio = Number(val);
            if (param === 'COMP_ATK') st.compAtk = Number(val);
            if (param === 'COMP_REL') st.compRel = Number(val);
            sendBLEData('OUT' + currentOut + '_' + param + ',' + val);
            if(param === 'COMP_THRESH') document.getElementById('compThreshVal').innerText = val;
            if(param === 'COMP_RATIO') document.getElementById('compRatioVal').innerText = Number(val).toFixed(1);
            if(param === 'COMP_ATK') document.getElementById('compAtkVal').innerText = val;
            if(param === 'COMP_REL') document.getElementById('compRelVal').innerText = val;
            applyGlobalLink(currentOut);
        }

        function handleBLEMessage(event) {
            const text = new TextDecoder().decode(event.target.value).trim();
            if (text.startsWith('SPECTRUM:')) {
                // SKIP update spectrum bar jika sedang di halaman RTA
                if (rtaPageActive && rtaActive) return;

                const parts = text.split(':')[1].split(',');
                if (parts.length === 7) {
                    const monitorSpectrum = parts.map(p => { const n = parseInt(p, 10); return Number.isFinite(n) ? Math.min(100, Math.max(0, n)) : 0; });
                    if (monitorActive) updateSpectrumMini(monitorSpectrum);
                    for (let i = 0; i < 7; i++) {
                        const val = Math.min(100, Math.max(0, parseInt(parts[i])));
                        const bar = document.getElementById('spec-bar-' + i);
                        if (bar) {
                            bar.style.height = Math.max(2, val) + '%';
                            if (val > 85) bar.style.background = 'var(--neon-red)';
                            else if (val > 65) bar.style.background = 'var(--neon-yellow)';
                            else bar.style.background = 'var(--neon-blue)';
                        }
                        const dashBars = document.querySelectorAll('#dash-rta-bars rect');
                        if(dashBars[i]) {
                            const barHeight = (val / 100) * 100;
                            dashBars[i].setAttribute('height', barHeight);
                            dashBars[i].setAttribute('y', 100 - barHeight);
                            if (val > 85) dashBars[i].setAttribute('fill', 'var(--neon-red)');
                            else if (val > 65) dashBars[i].setAttribute('fill', 'var(--neon-yellow)');
                            else dashBars[i].setAttribute('fill', 'var(--neon-blue)');
                        }
                    }
                    const bassLevel = parseInt(parts[0]) + parseInt(parts[1]); 
                    const trebleLevel = parseInt(parts[5]) + parseInt(parts[6]); 
                    if (bassLevel > 120) {
                        document.body.classList.add('reactive-bass');
                        document.body.classList.remove('reactive-glow');
                    } else if (trebleLevel > 100) {
                        document.body.classList.add('reactive-glow');
                        document.body.classList.remove('reactive-bass');
                    } else {
                        document.body.classList.remove('reactive-bass', 'reactive-glow');
                    }
                }
            }
            else if (text.startsWith('OSC:')) {
                const monitorOscParts = text.split(':')[1].split(',');
                const monitorOscValues = monitorOscParts.map(p => { const n = parseInt(p, 10); return Number.isFinite(n) ? Math.min(100, Math.max(0, n)) : 50; });
                if (monitorActive) updateOscData(monitorOscValues);
                if (graphMode === 'OSC') {
                    const parts = text.split(':')[1].split(',');
                    for(let i=0; i<parts.length; i++) {
                        if(i < oscData.length) oscData[i] = Math.min(100, Math.max(0, parseInt(parts[i])));
                    }
                    drawOscilloscope();
                }
            }
            else if (text.startsWith('LEVEL:')) {
                const parts = text.split(':');
                if(parts.length === 3) {
                    const target = parts[1].replace('_','').toLowerCase();
                    const parsedLevel = parseInt(parts[2], 10);
                    const level = Number.isFinite(parsedLevel) ? Math.min(100, Math.max(0, parsedLevel)) : 0;
                    const monitorTarget = target === 'inl' ? 'inL' : target === 'inr' ? 'inR' : target;
                    if (monitorActive && METER_CHANNELS.includes(monitorTarget)) { updateMeterVisual(monitorTarget, level); updateRmsDisplay(); }
                    document.querySelectorAll('[data-meter="' + target + '"]').forEach(el => { updateBarVisual(el, level); });
                    if (target.startsWith('out')) {
                        const chId = parseInt(target.replace('out',''), 10);
                        if (chId >= 1 && chId <= 4) updateStripMeter(chId, level);
                    }
                }
            }
        }

        function updateBarVisual(container, level) {
            const leds = Array.from(container.querySelectorAll('.mini-led, .led-block')).filter(l => !l.classList.contains('top-led'));
            const topLed = container.querySelector('.top-led');
            leds.forEach((led, i) => {
                const threshold = ((i + 1) / leds.length) * 100;
                if (level >= threshold) {
                    const ratio = (i + 1) / leds.length;
                    let color = 'var(--neon-green)';
                    if (ratio > 0.85) color = 'var(--neon-red)';
                    else if (ratio > 0.65) color = 'var(--neon-yellow)';
                    led.style.backgroundColor = color;
                    led.style.boxShadow = '0 0 3px ' + color;
                } else {
                    led.style.backgroundColor = '#111';
                    led.style.boxShadow = 'none';
                }
            });
            if (topLed) {
                if (level >= 95) { topLed.style.backgroundColor = 'var(--neon-red)'; topLed.style.boxShadow = '0 0 5px var(--neon-red)'; } 
                else { topLed.style.backgroundColor = '#111'; topLed.style.boxShadow = 'none'; }
            }
        }

        function toggleGraphMode() {
            graphMode = (graphMode === 'RTA') ? 'OSC' : 'RTA';
            const toggleBtn = document.getElementById('graph-mode-toggle');
            const rtaGroup = document.getElementById('dash-rta-bars');
            const oscLine = document.getElementById('dash-osc-line');
            if (graphMode === 'RTA') {
                toggleBtn.innerText = 'MODE: RTA';
                rtaGroup.style.opacity = '0.4';
                oscLine.style.opacity = '0';
            } else {
                toggleBtn.innerText = 'MODE: OSC';
                rtaGroup.style.opacity = '0';
                oscLine.style.opacity = '1';
            }
        }

        function drawOscilloscope() {
            const oscLine = document.getElementById('dash-osc-line');
            let path = 'M 0 100 ';
            const step = 200 / (oscData.length - 1);
            for (let i = 0; i < oscData.length; i++) {
                path += `L ${i * step} ${oscData[i]} `;
            }
            oscLine.setAttribute('d', path);
        }

        function toggleLink() {
            isLinked = !isLinked;
            const linkBtn = document.getElementById('linkToggle');
            if (isLinked) {
                linkBtn.classList.add('active');
                linkBtn.innerText = '🔗 LINKED';
            } else {
                linkBtn.classList.remove('active');
                linkBtn.innerText = '🔗 LINK L/R';
            }
        }

        function selectDashChannel(outNum) {
            dashActiveOut = outNum;
            syncDashboardUI(); 
        }

        let isDragging = false;
        let dragNodeIndex = 0;
        let svgRect = null;

        function startDrag(e, nodeIndex) {
            e.preventDefault(); 
            isDragging = true;
            dragNodeIndex = nodeIndex;
            svgRect = document.getElementById('dash-svg').getBoundingClientRect();
            document.addEventListener('mousemove', onDrag);
            document.addEventListener('touchmove', onDrag, {passive: false});
            document.addEventListener('mouseup', endDrag);
            document.addEventListener('touchend', endDrag);
        }

        function onDrag(e) {
            if (!isDragging) return;
            e.preventDefault(); 
            let clientX = e.touches ? e.touches[0].clientX : e.clientX;
            let clientY = e.touches ? e.touches[0].clientY : e.clientY;
            let x = ((clientX - svgRect.left) / svgRect.width) * 200;
            let y = ((clientY - svgRect.top) / svgRect.height) * 100;
            x = Math.max(10, Math.min(190, x));
            y = Math.max(20, Math.min(80, y));
            let gain = (50 - y) / 2;
            gain = Math.round(gain * 10) / 10;
            let freqRatio = (x - 20) / 160;
            let freq = 20 * Math.pow(1000, freqRatio);
            freq = Math.round(freq);
            freq = Math.max(20, freq);
            let st = globalState[dashActiveOut];
            if (dragNodeIndex === 1) { st.p1g = gain; st.p1f = freq; }
            if (dragNodeIndex === 2) { st.p2g = gain; st.p2f = freq; }
            if (dragNodeIndex === 3) { st.p3g = gain; st.p3f = freq; }
            sendBLEData('OUT' + dashActiveOut + '_PEQ' + dragNodeIndex + 'FREQ,' + freq);
            sendBLEData('OUT' + dashActiveOut + '_PEQ' + dragNodeIndex + 'GAIN,' + gain);
            updateDashboardGraph();
        }

        function endDrag() {
            isDragging = false;
            document.removeEventListener('mousemove', onDrag);
            document.removeEventListener('touchmove', onDrag);
            document.removeEventListener('mouseup', endDrag);
            document.removeEventListener('touchend', endDrag);
        }

        function syncDashboardUI() {
            for(let i=1; i<=4; i++) {
                let st = globalState[i];
                let muteBtn = document.getElementById('dash-mute-out' + i);
                if(st.isMute) muteBtn.classList.add('active'); else muteBtn.classList.remove('active');
                let phaseBtn = document.getElementById('dash-phase-' + i);
                if(st.phase180) { phaseBtn.classList.add('active'); phaseBtn.innerText = '180°'; } 
                else { phaseBtn.classList.remove('active'); phaseBtn.innerText = '0°'; }
                let routeL = document.getElementById('route-' + i + '-L');
                let routeR = document.getElementById('route-' + i + '-R');
                if(st.routing === 0) { routeL.classList.add('active'); routeR.classList.remove('active'); }
                else if(st.routing === 1) { routeL.classList.remove('active'); routeR.classList.add('active'); }
                else { routeL.classList.add('active'); routeR.classList.add('active'); }
                let volSlider = document.getElementById('dash-vol-' + i);
                if(volSlider) volSlider.value = st.gain;
                let volLabel = document.getElementById('val-o' + i);
                if(volLabel) volLabel.innerText = st.gain + ' dB';
            }
            updateDashboardGraph();
        }

        function safeLogFreq(freq) {
            freq = Math.max(20, parseInt(freq) || 20);
            return 20 + (Math.log10(freq / 20) / Math.log10(20000 / 20)) * 160;
        }

        function updateDashboardGraph() {
            for(let i = 1; i <= 4; i++) {
                let st = globalState[i];
                let hpfX = st.hpfOn ? ((parseInt(st.hpfFreq) - 20) / 1980) * 150 : 0;
                let lpfX = st.lpfOn ? 50 + ((parseInt(st.lpfFreq) - 500) / 19500) * 150 : 200;
                let x1 = safeLogFreq(st.p1f);
                let x2 = safeLogFreq(st.p2f);
                let x3 = safeLogFreq(st.p3f);
                let py1 = 50 + (st.p1g * -2); let py2 = 50 + (st.p2g * -2); let py3 = 50 + (st.p3g * -2);
                let path = `M 0 100 `;
                if (st.hpfOn) { path += `L ${hpfX*0.8} 100 Q ${hpfX} 100, ${hpfX+10} 50 `; } else { path += `L 0 50 `; }
                path += `L ${x1-10} 50 Q ${x1} ${py1}, ${x1+10} 50 `;
                path += `L ${x2-10} 50 Q ${x2} ${py2}, ${x2+10} 50 `;
                path += `L ${x3-10} 50 Q ${x3} ${py3}, ${x3+10} 50 `;
                if (st.lpfOn) { path += `L ${lpfX-10} 50 Q ${lpfX} 50, ${lpfX*1.1} 100 L 200 100`; } else { path += `L 200 50`; }
                let pathEl = document.getElementById('dash-path-out' + i);
                pathEl.setAttribute('d', path);
                if(i === dashActiveOut) {
                    pathEl.style.opacity = 1;
                    pathEl.style.strokeWidth = "3";
                    pathEl.style.filter = `drop-shadow(0 0 4px var(--color-out${i}))`;
                } else {
                    pathEl.style.opacity = 0.15;
                    pathEl.style.strokeWidth = "1.5";
                    pathEl.style.filter = "none";
                }
            }
            let activeSt = globalState[dashActiveOut];
            let nx1 = safeLogFreq(activeSt.p1f);
            let nx2 = safeLogFreq(activeSt.p2f);
            let nx3 = safeLogFreq(activeSt.p3f);
            let ny1 = 50 + (activeSt.p1g * -2); let ny2 = 50 + (activeSt.p2g * -2); let ny3 = 50 + (activeSt.p3g * -2);
            let n1 = document.getElementById('node-dash-p1');
            let n2 = document.getElementById('node-dash-p2');
            let n3 = document.getElementById('node-dash-p3');
            n1.setAttribute('cx', nx1); n1.setAttribute('cy', ny1); n1.style.opacity = 1;
            n2.setAttribute('cx', nx2); n2.setAttribute('cy', ny2); n2.style.opacity = 1;
            n3.setAttribute('cx', nx3); n3.setAttribute('cy', ny3); n3.style.opacity = 1;
        }

        function applyGlobalLink(sourceOut) {
            if (!isLinked) return;
            let targetOut = (sourceOut % 2 === 0) ? sourceOut - 1 : sourceOut + 1;
            if (targetOut < 1 || targetOut > 4) return;
            let src = globalState[sourceOut];
            let tgt = globalState[targetOut];
            tgt.hpfOn = src.hpfOn; tgt.hpfFreq = src.hpfFreq; tgt.hpfSlope = src.hpfSlope || 24; tgt.hpfSlope = src.hpfSlope || 24;
            tgt.lpfOn = src.lpfOn; tgt.lpfFreq = src.lpfFreq; tgt.lpfSlope = src.lpfSlope || 24; tgt.lpfSlope = src.lpfSlope || 24;
            tgt.p1f = src.p1f; tgt.p1g = src.p1g;
            tgt.p2f = src.p2f; tgt.p2g = src.p2g;
            tgt.p3f = src.p3f; tgt.p3g = src.p3g;
            tgt.gain = src.gain;
            tgt.delay = src.delay;
            tgt.limitOn = src.limitOn; tgt.limit = src.limit;
            tgt.compOn = src.compOn; tgt.compThresh = src.compThresh; tgt.compRatio = src.compRatio;
            tgt.compAtk = src.compAtk; tgt.compRel = src.compRel;
            sendBLEData('OUT' + targetOut + '_HPF_ON,' + (tgt.hpfOn ? 1 : 0));
            sendBLEData('OUT' + targetOut + '_HPFFREQ,' + tgt.hpfFreq);
            sendBLEData('OUT' + targetOut + '_HPFSLOPE,' + tgt.hpfSlope);
            sendBLEData('OUT' + targetOut + '_HPFSLOPE,' + tgt.hpfSlope);
            sendBLEData('OUT' + targetOut + '_LPF_ON,' + (tgt.lpfOn ? 1 : 0));
            sendBLEData('OUT' + targetOut + '_LPFFREQ,' + tgt.lpfFreq);
            sendBLEData('OUT' + targetOut + '_LPFSLOPE,' + tgt.lpfSlope);
            sendBLEData('OUT' + targetOut + '_LPFSLOPE,' + tgt.lpfSlope);
            sendBLEData('OUT' + targetOut + '_PEQ1FREQ,' + tgt.p1f);
            sendBLEData('OUT' + targetOut + '_PEQ1GAIN,' + tgt.p1g);
            sendBLEData('OUT' + targetOut + '_PEQ2FREQ,' + tgt.p2f);
            sendBLEData('OUT' + targetOut + '_PEQ2GAIN,' + tgt.p2g);
            sendBLEData('OUT' + targetOut + '_PEQ3FREQ,' + tgt.p3f);
            sendBLEData('OUT' + targetOut + '_PEQ3GAIN,' + tgt.p3g);
            sendBLEData('OUT' + targetOut + '_GAIN,' + tgt.gain);
            sendBLEData('OUT' + targetOut + '_DELAY,' + tgt.delay);
            sendBLEData('OUT' + targetOut + '_LIMIT_ON,' + (tgt.limitOn ? 1 : 0));
            sendBLEData('OUT' + targetOut + '_LIMIT,' + tgt.limit);
            sendBLEData('OUT' + targetOut + '_COMP_ON,' + (tgt.compOn ? 1 : 0));
            sendBLEData('OUT' + targetOut + '_COMP_THRESH,' + tgt.compThresh);
            sendBLEData('OUT' + targetOut + '_COMP_RATIO,' + tgt.compRatio);
            sendBLEData('OUT' + targetOut + '_COMP_ATK,' + tgt.compAtk);
            sendBLEData('OUT' + targetOut + '_COMP_REL,' + tgt.compRel);
            document.getElementById('dash-vol-' + targetOut).value = tgt.gain;
            updateDashboardGraph();
            if (currentOut === targetOut && document.getElementById('page-setting').classList.contains('active')) {
                loadStateToUI(targetOut);
            }
        }

        function syncVolFromDash(outNum, val) {
            globalState[outNum].gain = Number(val);
            const volLabel = document.getElementById('val-o' + outNum);
            if (volLabel) volLabel.innerText = val + ' dB';
            sendBLEData('OUT' + outNum + '_GAIN,' + val);
            if(currentOut == outNum && document.getElementById('page-setting').classList.contains('active')) {
                document.getElementById('gain').value = val;
                document.getElementById('gainVal').innerText = val + ' dB';
            }
        }

        function openSetting(outNum) { 
            currentOut = outNum; 
            document.querySelectorAll('.out-tab').forEach(t => t.classList.remove('active')); 
            document.getElementById('tab-' + outNum).classList.add('active'); 
            loadStateToUI(outNum); 
            showPage('page-setting'); 
        }
        function switchTab(outNum) { saveCurrentUI(); currentOut = outNum; document.querySelectorAll('.out-tab').forEach(t => t.classList.remove('active')); document.getElementById('tab-' + outNum).classList.add('active'); loadStateToUI(outNum); }
        
        function saveCurrentUI() { 
            let st = globalState[currentOut]; 
            st.hpfOn = document.getElementById('hpfToggle').classList.contains('on'); st.hpfFreq = document.getElementById('hpfFreq').value; 
            st.lpfOn = document.getElementById('lpfToggle').classList.contains('on'); st.lpfFreq = document.getElementById('lpfFreq').value; 
            st.p1f = document.getElementById('peq1_freq').value; st.p1g = document.getElementById('peq1_gain').value; 
            st.p2f = document.getElementById('peq2_freq').value; st.p2g = document.getElementById('peq2_gain').value; 
            st.p3f = document.getElementById('peq3_freq').value; st.p3g = document.getElementById('peq3_gain').value; 
            st.gain = document.getElementById('gain').value; st.delay = document.getElementById('delay').value; 
            st.limitOn = document.getElementById('limToggle').classList.contains('on'); st.limit = Number(document.getElementById('limit').value);
            st.compOn = document.getElementById('compToggle').classList.contains('on');
            st.compThresh = Number(document.getElementById('compThresh').value);
            st.compRatio = Number(document.getElementById('compRatio').value);
            st.compAtk = Number(document.getElementById('compAtk').value);
            st.compRel = Number(document.getElementById('compRel').value);
        }

        function loadStateToUI(outNum) { 
            let st = globalState[outNum]; 
            document.getElementById('hpfFreq').value = st.hpfFreq; document.getElementById('hpfToggle').className = st.hpfOn ? 'toggle-switch on' : 'toggle-switch'; document.getElementById('hpfToggle').innerText = st.hpfOn ? 'ON' : 'OFF';
            const hpfSlope = st.hpfSlope || 24;
            const lpfSlope = st.lpfSlope || 24;
            const hpfCard = document.getElementById('hpfToggle').parentElement;
            const lpfCard = document.getElementById('lpfToggle').parentElement;
            hpfCard.querySelectorAll('.slope-btn').forEach(b => b.classList.toggle('active', Number(b.innerText.trim()) === hpfSlope));
            lpfCard.querySelectorAll('.slope-btn').forEach(b => b.classList.toggle('active', Number(b.innerText.trim()) === lpfSlope));
            document.getElementById('lpfFreq').value = st.lpfFreq; document.getElementById('lpfToggle').className = st.lpfOn ? 'toggle-switch on lpf' : 'toggle-switch lpf'; document.getElementById('lpfToggle').innerText = st.lpfOn ? 'ON' : 'OFF'; 
            document.getElementById('peq1_freq').value = st.p1f; document.getElementById('peq1_gain').value = st.p1g; 
            document.getElementById('peq2_freq').value = st.p2f; document.getElementById('peq2_gain').value = st.p2g; 
            document.getElementById('peq3_freq').value = st.p3f; document.getElementById('peq3_gain').value = st.p3g; 
            document.getElementById('gain').value = st.gain; document.getElementById('delay').value = st.delay; 
            document.getElementById('limit').value = st.limit; document.getElementById('limToggle').className = st.limitOn ? 'toggle-switch on' : 'toggle-switch'; document.getElementById('limToggle').innerText = st.limitOn ? 'ON' : 'OFF';
            document.getElementById('compToggle').className = st.compOn ? 'toggle-switch on' : 'toggle-switch'; document.getElementById('compToggle').innerText = st.compOn ? 'ON' : 'OFF';
            document.getElementById('compThresh').value = st.compThresh; document.getElementById('compThreshVal').innerText = st.compThresh;
            document.getElementById('compRatio').value = st.compRatio; document.getElementById('compRatioVal').innerText = Number(st.compRatio).toFixed(1);
            document.getElementById('compAtk').value = st.compAtk; document.getElementById('compAtkVal').innerText = st.compAtk;
            document.getElementById('compRel').value = st.compRel; document.getElementById('compRelVal').innerText = st.compRel;
            document.getElementById('gainVal').innerText = st.gain + ' dB'; document.getElementById('delayVal').innerText = st.delay + ' ms'; document.getElementById('limitVal').innerText = st.limit + ' dB'; 
            let phaseUtilBtn = document.querySelector('.graph-utils .util-btn.phase');
            if(phaseUtilBtn) { if(st.phase180) { phaseUtilBtn.classList.add('active'); phaseUtilBtn.innerText = '180°'; } else { phaseUtilBtn.classList.remove('active'); phaseUtilBtn.innerText = '0°'; } }
            let muteUtilBtn = document.querySelector('.graph-utils .util-btn.mute');
            if(muteUtilBtn) { if(st.isMute) muteUtilBtn.classList.add('active'); else muteUtilBtn.classList.remove('active'); }
            updateGraph(); 
        }

        function sendOutputStateToDevice(outNum, st) {
            const prefix = 'OUT' + outNum + '_';
            const commands = [
                ['HPF_ON', st.hpfOn ? 1 : 0], ['HPFFREQ', st.hpfFreq], ['HPFSLOPE', st.hpfSlope || 24],
                ['LPF_ON', st.lpfOn ? 1 : 0], ['LPFFREQ', st.lpfFreq], ['LPFSLOPE', st.lpfSlope || 24],
                ['PEQ1FREQ', st.p1f], ['PEQ1GAIN', st.p1g],
                ['PEQ2FREQ', st.p2f], ['PEQ2GAIN', st.p2g],
                ['PEQ3FREQ', st.p3f], ['PEQ3GAIN', st.p3g],
                ['GAIN', st.gain], ['DELAY', st.delay],
                ['LIMIT_ON', st.limitOn ? 1 : 0], ['LIMIT', st.limit],
                ['COMP_ON', st.compOn ? 1 : 0], ['COMP_THRESH', st.compThresh],
                ['COMP_RATIO', st.compRatio], ['COMP_ATK', st.compAtk], ['COMP_REL', st.compRel],
                ['MUTE', st.isMute ? 1 : 0], ['PHASE', st.phase180 ? 1 : 0],
                ['ROUTING', Number.isInteger(st.routing) ? st.routing : 2]
            ];
            commands.forEach(([param, value]) => {
                sendBLEData(prefix + param + ',' + value);
            });
        }

        function sendInputStateToDevice(input, st) {
            sendBLEData('IN_' + input + '_GAIN,' + st.gain);
            sendBLEData('IN_' + input + '_PEQFREQ,' + st.peqFreq);
            sendBLEData('IN_' + input + '_PEQGAIN,' + st.peqGain);
            sendBLEData('IN_' + input + '_MUTE,' + (st.isMute ? 1 : 0));
        }

        // Full UI -> firmware sync. loadStateToUI() remains presentation-only;
        // switching pages/tabs must never write DSP parameters over BLE.
        async function syncFullStateToDevice() {
            if (!isBLEConnected()) return;
            for (let i = 1; i <= 4; i++) sendOutputStateToDevice(i, globalState[i]);
            sendInputStateToDevice('L', inputState.L);
            sendInputStateToDevice('R', inputState.R);
            const master = document.querySelector('#page-front input[type="range"][oninput*="MASTER,"]');
            if (master) sendBLEData('MASTER,' + Number(master.value));
            const bypassBtn = document.getElementById('bypass-all-btn');
            const bypassOn = !!(bypassBtn && bypassBtn.classList.contains('active')) ||
                !!document.querySelector('.preset-btn.bypass.active');
            sendBLEData('ALL_BYPASS,' + (bypassOn ? 1 : 0));
        }

        function resetToDefault() {
            if (!confirm('Reset Out ' + currentOut + ' ke Flat?')) return;
            globalState[currentOut] = {
                hpfOn: false, hpfFreq: 20, lpfOn: false, lpfFreq: 20000,
                p1f: 100, p1g: 0, p2f: 1000, p2g: 0, p3f: 5000, p3g: 0,
                gain: 0, delay: 0, limitOn: false, limit: 0, routing: 2,
                isMute: false, phase180: false, compOn: false,
                compThresh: -20, compRatio: 4, compAtk: 10, compRel: 200
            };
            loadStateToUI(currentOut);
            sendOutputStateToDevice(currentOut, globalState[currentOut]);
            syncDashboardUI();
            applyGlobalLink(currentOut);
        }

        function openInputSetting(ch) { currentInput = ch; document.querySelectorAll('#page-input-setting .out-tab').forEach(t => t.classList.remove('active')); document.getElementById('tab-in-' + ch).classList.add('active'); loadInputUI(ch); showPage('page-input-setting'); }
        function switchInputTab(ch) { saveInputUI(); currentInput = ch; document.querySelectorAll('#page-input-setting .out-tab').forEach(t => t.classList.remove('active')); document.getElementById('tab-in-' + ch).classList.add('active'); loadInputUI(ch); }
        function saveInputUI() { let st = inputState[currentInput]; st.gain = document.getElementById('inputGain').value; st.peqFreq = document.getElementById('in_peq_freq').value; st.peqGain = document.getElementById('in_peq_gain').value; }
        function loadInputUI(ch) { let st = inputState[ch]; document.getElementById('inputGain').value = st.gain; document.getElementById('inputGainVal').innerText = st.gain + ' dB'; document.getElementById('in_peq_freq').value = st.peqFreq; document.getElementById('in_peq_freqVal').innerText = (st.peqFreq >= 1000) ? (st.peqFreq/1000).toFixed(1)+'k' : st.peqFreq; document.getElementById('in_peq_gain').value = st.peqGain; document.getElementById('in_peq_gainVal').innerText = st.peqGain; }
        function updateInputVal(id, unit) { let val = document.getElementById(id).value; document.getElementById(id + 'Val').innerText = val + ' ' + unit; sendBLEData('IN_' + currentInput + '_GAIN,' + val); inputState[currentInput].gain = val; }
        function updateInputGraph() { let f = document.getElementById('in_peq_freq').value; let g = document.getElementById('in_peq_gain').value; document.getElementById('in_peq_freqVal').innerText = (f >= 1000) ? (f/1000).toFixed(1)+'k' : f; document.getElementById('in_peq_gainVal').innerText = g; sendBLEData('IN_' + currentInput + '_PEQFREQ,' + f); sendBLEData('IN_' + currentInput + '_PEQGAIN,' + g); inputState[currentInput].peqFreq = f; inputState[currentInput].peqGain = g; }
        function toggleInputMute(btn) {
            btn.classList.toggle('on');
            const isMute = !btn.classList.contains('on');
            inputState[currentInput].isMute = isMute;
            sendBLEData('IN_' + currentInput + '_MUTE,' + (isMute ? 1 : 0));
        }

        function updateVal(elementId, unit) { 
            let slider = document.getElementById(elementId); 
            let display = document.getElementById(elementId + 'Val'); 
            let val = slider.value; 
            if (unit === 'Hz' && val >= 1000) display.innerText = (val / 1000).toFixed(1) + ' kHz'; 
            else display.innerText = val + ' ' + unit; 
            sendBLEData('OUT' + currentOut + '_' + elementId.toUpperCase() + ',' + val); 
            if(elementId === 'gain') { 
                document.getElementById('dash-vol-' + currentOut).value = val; 
            }
            applyGlobalLink(currentOut);
        }

        function updateGraph() { 
            let hpfVal = parseInt(document.getElementById('hpfFreq').value); let lpfVal = parseInt(document.getElementById('lpfFreq').value); let hpfToggle = document.getElementById('hpfToggle').classList.contains('on'); let lpfToggle = document.getElementById('lpfToggle').classList.contains('on'); 
            document.getElementById('hpfFreqVal').innerText = (hpfVal >= 1000) ? (hpfVal/1000).toFixed(1)+'kHz' : hpfVal+'Hz'; document.getElementById('lpfFreqVal').innerText = (lpfVal >= 1000) ? (lpfVal/1000).toFixed(1)+'kHz' : lpfVal+'Hz'; 
            let hpfX = ((hpfVal - 20) / 1980) * 150; let lpfX = 50 + ((lpfVal - 500) / 19500) * 150; if(!hpfToggle) hpfX = 0; if(!lpfToggle) lpfX = 200; 
            let peq1_f = parseInt(document.getElementById('peq1_freq').value); let peq2_f = parseInt(document.getElementById('peq2_freq').value); let peq3_f = parseInt(document.getElementById('peq3_freq').value); 
            let peq1_g = parseInt(document.getElementById('peq1_gain').value); let peq2_g = parseInt(document.getElementById('peq2_gain').value); let peq3_g = parseInt(document.getElementById('peq3_gain').value); 
            let x1 = safeLogFreq(peq1_f);
            let x2 = safeLogFreq(peq2_f);
            let x3 = safeLogFreq(peq3_f);
            let path = `M 0 100 `; 
            if (hpfToggle) { path += `L ${hpfX*0.8} 100 Q ${hpfX} 100, ${hpfX+10} 50 `; } else { path += `L 0 50 `; } 
            let py1 = 50 + (peq1_g * -2); path += `L ${x1-10} 50 Q ${x1} ${py1}, ${x1+10} 50 `; 
            let py2 = 50 + (peq2_g * -2); path += `L ${x2-10} 50 Q ${x2} ${py2}, ${x2+10} 50 `; 
            let py3 = 50 + (peq3_g * -2); path += `L ${x3-10} 50 Q ${x3} ${py3}, ${x3+10} 50 `; 
            if (lpfToggle) { path += `L ${lpfX-10} 50 Q ${lpfX} 50, ${lpfX*1.1} 100 L 200 100`; } else { path += `L 200 50`; } 
            document.getElementById('curvePath').setAttribute('d', path); document.getElementById('hpfNode').setAttribute('cx', hpfX+5); document.getElementById('hpfNode').setAttribute('cy', 70); document.getElementById('hpfNode').style.opacity = hpfToggle ? 1 : 0; document.getElementById('lpfNode').setAttribute('cx', lpfX-5); document.getElementById('lpfNode').setAttribute('cy', 70); document.getElementById('lpfNode').style.opacity = lpfToggle ? 1 : 0; 
            document.getElementById('peq1_freqVal').innerText = (peq1_f >= 1000) ? (peq1_f/1000).toFixed(1)+'k' : peq1_f+'';
            document.getElementById('peq1_gainVal').innerText = peq1_g;
            document.getElementById('peq2_freqVal').innerText = (peq2_f >= 1000) ? (peq2_f/1000).toFixed(1)+'k' : peq2_f+'';
            document.getElementById('peq2_gainVal').innerText = peq2_g;
            document.getElementById('peq3_freqVal').innerText = (peq3_f >= 1000) ? (peq3_f/1000).toFixed(1)+'k' : peq3_f+'';
            document.getElementById('peq3_gainVal').innerText = peq3_g;
            sendBLEData('OUT' + currentOut + '_HPFFREQ,' + hpfVal); sendBLEData('OUT' + currentOut + '_LPFFREQ,' + lpfVal); sendBLEData('OUT' + currentOut + '_PEQ1FREQ,' + peq1_f); sendBLEData('OUT' + currentOut + '_PEQ1GAIN,' + peq1_g); sendBLEData('OUT' + currentOut + '_PEQ2FREQ,' + peq2_f); sendBLEData('OUT' + currentOut + '_PEQ2GAIN,' + peq2_g); sendBLEData('OUT' + currentOut + '_PEQ3FREQ,' + peq3_f); sendBLEData('OUT' + currentOut + '_PEQ3GAIN,' + peq3_g); 
            applyGlobalLink(currentOut);
        }
        
        function toggleFilter(btn) { btn.classList.toggle('on'); let isOn = btn.classList.contains('on'); btn.innerText = isOn ? 'ON' : 'OFF'; sendBLEData('OUT' + currentOut + '_' + btn.id.replace('Toggle', '').toUpperCase() + '_ON,' + (isOn ? 1 : 0)); if(btn.id === 'hpfToggle' || btn.id === 'lpfToggle') updateGraph(); applyGlobalLink(currentOut); }
        function setSlope(btn, type) {
            const slope = Number(btn.innerText.trim());
            if (slope !== 12 && slope !== 24) return;
            btn.parentElement.querySelectorAll('.slope-btn').forEach(b => b.classList.toggle('active', b === btn));
            const key = type.toLowerCase() + 'Slope';
            if (globalState[currentOut]) globalState[currentOut][key] = slope;
            sendBLEData('OUT' + currentOut + '_' + type + 'SLOPE,' + slope);
            applyGlobalLink(currentOut);
        }
        
        function toggleMute(btn, targetName) { 
            btn.classList.toggle('active'); let isMute = btn.classList.contains('active'); sendBLEData(targetName + '_MUTE,' + (isMute ? 1 : 0)); 
            if(targetName.startsWith('OUT')) { let num = parseInt(targetName.replace('OUT', '')); globalState[num].isMute = isMute; }
        }
        
        function togglePhase(btn, outNum) { 
            btn.classList.toggle('active'); let is180 = btn.classList.contains('active'); btn.innerText = is180 ? '180°' : '0°'; 
            sendBLEData('OUT' + outNum + '_PHASE,' + (is180 ? 1 : 0)); 
            globalState[outNum].phase180 = is180;
            let dashBtn = document.getElementById('dash-phase-' + outNum);
            if(dashBtn) { if(is180) { dashBtn.classList.add('active'); dashBtn.innerText='180°'; } else { dashBtn.classList.remove('active'); dashBtn.innerText='0°'; } }
            let settingBtn = document.querySelector('.graph-utils .util-btn.phase');
            if(settingBtn && currentOut == outNum) { if(is180) { settingBtn.classList.add('active'); settingBtn.innerText='180°'; } else { settingBtn.classList.remove('active'); settingBtn.innerText='0°'; } }
        }

        function toggleRouting(btn, outNum, isRight) { 
            btn.classList.toggle('active'); 
            let isL = document.getElementById('route-' + outNum + '-L').classList.contains('active');
            let isR = document.getElementById('route-' + outNum + '-R').classList.contains('active');
            let st = globalState[outNum];
            if (isL && isR) st.routing = 2; 
            else if (isL) st.routing = 0; 
            else if (isR) st.routing = 1; 
            else st.routing = 3; 
            sendBLEData('OUT' + outNum + '_ROUTING,' + st.routing); 
        }
        function toggleBypass(btn) {
            const isOn = !btn.classList.contains('active');
            document.querySelectorAll('.preset-btn.bypass').forEach(b => {
                b.classList.toggle('active', isOn);
            });
            sendBLEData('ALL_BYPASS,' + (isOn ? 1 : 0));
        }

        function muteAllOutputs() {
            const allMuted = [1, 2, 3, 4].every(i => globalState[i].isMute);
            for (let i = 1; i <= 4; i++) {
                globalState[i].isMute = !allMuted;
                sendBLEData('OUT' + i + '_MUTE,' + (allMuted ? 0 : 1));
            }
            syncDashboardUI();
        }

        function showQuickNotice(feature) {
            alert(feature + ' belum didukung oleh firmware DLMS Mini saat ini.');
        }

        function toggleSaveMode() { saveMode = !saveMode; let btnSave = document.getElementById('btn-save'); if(saveMode) { btnSave.classList.add('saving'); btnSave.innerText = 'PILIH P1-P4'; } else { btnSave.classList.remove('saving'); btnSave.innerText = 'SAVE'; } }
        function handlePresetClick(presetNum) {
            if (document.getElementById('page-setting').classList.contains('active')) saveCurrentUI();
            if (document.getElementById('page-input-setting').classList.contains('active')) saveInputUI();
            if (saveMode) {
                localStorage.setItem('DLMS_PRESET_' + presetNum, JSON.stringify({ outs: globalState, ins: inputState }));
                alert('✅ Settingan disimpan ke P' + presetNum);
                toggleSaveMode();
                updatePresetButtons(presetNum);
                return;
            }
            const savedData = localStorage.getItem('DLMS_PRESET_' + presetNum);
            if (!savedData) { alert('⚠️ Belum ada data di P' + presetNum); return; }
            try {
                const parsed = JSON.parse(savedData);
                if (!parsed.outs || !parsed.ins) throw new Error('Format preset tidak valid');
                for (let i = 1; i <= 4; i++) {
                    const st = parsed.outs[i];
                    if (!st) throw new Error('Data OUT ' + i + ' tidak lengkap');
                    if (st.compOn === undefined) st.compOn = false;
                    if (st.compThresh === undefined) st.compThresh = -20;
                    if (st.compRatio === undefined) st.compRatio = 4;
                    if (st.compAtk === undefined) st.compAtk = 10;
                    if (st.compRel === undefined) st.compRel = 200;
                }
                globalState = parsed.outs;
                inputState = parsed.ins;
                loadStateToUI(currentOut);
                for (let i = 1; i <= 4; i++) sendOutputStateToDevice(i, globalState[i]);
                sendInputStateToDevice('L', inputState.L);
                sendInputStateToDevice('R', inputState.R);
                syncDashboardUI();
                updatePresetButtons(presetNum);
                alert('🔄 Settingan P' + presetNum + ' dimuat ke aplikasi dan perangkat!');
            } catch (error) {
                console.error('Gagal memuat preset:', error);
                alert('❌ Preset tidak valid: ' + error.message);
            }
        }
        function updatePresetButtons(activeNum) { for(let i=1; i<=4; i++) { const btn = document.getElementById('preset-btn-p' + i) || document.getElementById('btn-p' + i); if(btn) btn.classList.toggle('active', i === activeNum); } }
        
        // === TEST TONE / SIGNAL GENERATOR ===
let currentToneMode = 'off';
let currentToneFreq = 1000;
let currentToneVolume = -12;
function formatFreq(freq){return freq>=1000?(freq/1000).toFixed(freq%1000===0?0:1)+' kHz':freq+' Hz'}
function updateToneUIOff(){
    document.querySelectorAll('.tone-mode-btn').forEach(btn=>btn.classList.toggle('active',btn.id==='tone-off'));
    const icon=document.getElementById('tone-status-icon'), label=document.getElementById('tone-status-text'), freq=document.getElementById('tone-freq-display');
    if(icon){icon.innerText='🔇';icon.classList.remove('active');}
    if(label) label.innerText='OFF';
    if(freq) freq.innerText='—';
}
function setToneMode(mode){
    if(!['off','pink','sine'].includes(mode)) return;
    currentToneMode=mode;
    document.querySelectorAll('.tone-mode-btn').forEach(btn=>btn.classList.remove('active'));
    const selected=document.getElementById('tone-'+mode); if(selected) selected.classList.add('active');
    const icon=document.getElementById('tone-status-icon'), label=document.getElementById('tone-status-text'), freq=document.getElementById('tone-freq-display');
    if(!icon||!label||!freq) return;
    if(mode==='off'){icon.innerText='🔇';icon.classList.remove('active');label.innerText='OFF';freq.innerText='—';sendBLEData('SIG_GEN,0')}
    else if(mode==='pink'){icon.innerText='🌸';icon.classList.add('active');label.innerText='PINK NOISE';freq.innerText='Full Range';sendBLEData('SIG_GEN,1')}
    else{icon.innerText='〰️';icon.classList.add('active');label.innerText='SINE WAVE';freq.innerText=formatFreq(currentToneFreq);sendBLEData('SIG_GEN,2');sendBLEData('SIG_FREQ,'+currentToneFreq)}
    const card=document.getElementById('freq-selector-card');if(card){card.style.opacity=mode==='sine'?'1':'0.4';card.style.pointerEvents=mode==='sine'?'auto':'none'}
}
function setToneFreq(freq){
    currentToneFreq=Number(freq);
    document.querySelectorAll('.freq-btn').forEach(btn=>btn.classList.toggle('active',Number(btn.dataset.freq)===currentToneFreq));
    const display=document.getElementById('tone-freq-display');if(display&&currentToneMode==='sine')display.innerText=formatFreq(currentToneFreq);
    sendBLEData('SIG_FREQ,'+currentToneFreq);
}
function updateToneVolume(val){
    const n = Number(val);
    if (!Number.isFinite(n)) return;
    currentToneVolume = Math.max(-60, Math.min(0, n));
    const display=document.getElementById('tone-volume-val');
    if(display) display.innerText=currentToneVolume+' dB';
    // Dedicated generator level; does not modify MASTER.
    sendBLEData('TONE_LEVEL,'+currentToneVolume);
}
function initTestTonePage(){
    const slider=document.getElementById('tone-volume');
    if(slider) slider.value=String(currentToneVolume);
    const val=document.getElementById('tone-volume-val');if(val)val.innerText=currentToneVolume+' dB';
    setToneMode('off');
    setToneFreq(currentToneFreq);
}
function showQuickNotice(name){alert(name+' belum tersedia.')}


function isSystemBLEConnected(){return isBLEConnected();}
function saveToNVS(){if(!isSystemBLEConnected()){showToast('⚠️ BLE belum terhubung!');return;}sendBLEData('SAVE_TO_NVS,1');showToast('Permintaan simpan dikirim; perlu dukungan firmware.');}
function loadFromNVS(){if(!isSystemBLEConnected()){showToast('⚠️ BLE belum terhubung!');return;}sendBLEData('LOAD_FROM_NVS,1');showToast('Permintaan muat dikirim; perlu dukungan firmware.');}
function saveBackupLocal(){const m=document.querySelector('#page-front input[type="range"][oninput*="MASTER,"]');const b={version:'1.0',timestamp:new Date().toISOString(),outs:globalState,ins:inputState,master:m?Number(m.value):0,deviceName:document.getElementById('dash-title').innerText};try{localStorage.setItem('DLMS_BACKUP_FULL',JSON.stringify(b));showToast('📱 Backup tersimpan di HP!');updateSystemInfo();}catch(e){showToast('❌ Gagal menyimpan backup!');}}
function restoreBackupLocal(){const raw=localStorage.getItem('DLMS_BACKUP_FULL');if(!raw){showToast('⚠️ Tidak ada backup!');return;}if(!confirm('Restore backup? Settingan saat ini akan ditimpa!'))return;try{const b=JSON.parse(raw);if(b.outs){for(let i=1;i<=4;i++)if(!b.outs[i])throw Error('Data output tidak lengkap');globalState=b.outs;for(let i=1;i<=4;i++)sendOutputStateToDevice(i,globalState[i]);}if(b.ins){if(!b.ins.L||!b.ins.R)throw Error('Data input tidak lengkap');inputState=b.ins;sendInputStateToDevice('L',inputState.L);sendInputStateToDevice('R',inputState.R);}if(b.master!==undefined){const m=document.querySelector('#page-front input[type="range"][oninput*="MASTER,"]');if(m){m.value=b.master;m.dispatchEvent(new Event('input',{bubbles:true}));}}if(b.deviceName){document.getElementById('dash-title').innerText=b.deviceName;localStorage.setItem('DLMS_DEVICE_NAME',b.deviceName);}syncDashboardUI();showToast('🔄 Backup direstore!');}catch(e){showToast('❌ Backup tidak valid!');}}
function renameDevice(){const el=document.getElementById('dash-title'),old=el.innerText,n=prompt('Nama perangkat baru:',old);if(n===null||!n.trim())return;const name=n.trim().substring(0,20);el.innerText=name;localStorage.setItem('DLMS_DEVICE_NAME',name);showToast('✅ Nama diubah: '+name);}
function resetFactory(){if(!confirm('Reset semua parameter output, input, dan master ke nilai default?'))return;if(!confirm('Yakin? Tindakan ini tidak dapat dibatalkan.'))return;for(let i=1;i<=4;i++){globalState[i]={hpfOn:false,hpfFreq:20,hpfSlope:24,lpfOn:false,lpfFreq:20000,lpfSlope:24,p1f:100,p1g:0,p2f:1000,p2g:0,p3f:5000,p3g:0,gain:0,delay:0,limitOn:false,limit:0,routing:2,isMute:false,phase180:false,compOn:false,compThresh:-20,compRatio:4,compAtk:10,compRel:200};sendOutputStateToDevice(i,globalState[i]);}inputState.L={gain:0,peqFreq:1000,peqGain:0};inputState.R={gain:0,peqFreq:1000,peqGain:0};sendInputStateToDevice('L',inputState.L);sendInputStateToDevice('R',inputState.R);const m=document.querySelector('#page-front input[type="range"][oninput*="MASTER,"]');if(m){m.value=0;m.dispatchEvent(new Event('input',{bubbles:true}));}syncDashboardUI();updateSystemInfo();showToast('Reset parameter selesai; NVS firmware tidak dihapus.');}
function updateSystemInfo(){const c=isSystemBLEConnected(),e=document.getElementById('sys-ble-status');if(e){e.innerText=c?'CONNECTED':'DISCONNECTED';e.className='sys-info-value '+(c?'connected':'disconnected');}let n=0;for(let i=1;i<=4;i++)if(localStorage.getItem('DLMS_PRESET_'+i))n++;const p=document.getElementById('sys-preset-count');if(p)p.innerText=n+'/4 saved';}
function refreshSystemPage(){updateSystemInfo();}


// === MONITOR PAGE ===
let monitorActive = false, scopeFrozen = false, clipCounter = 0, lastClipTime = 0;
let peakHoldValues = {inL:0,inR:0,out1:0,out2:0,out3:0,out4:0};
let currentLevels = {inL:0,inR:0,out1:0,out2:0,out3:0,out4:0};
let currentSpectrum = [0,0,0,0,0,0,0];
let currentOscData = new Array(50).fill(50), scopeCanvas, scopeCtx, scopeAnimFrame;
const METER_CHANNELS = ['inL','inR','out1','out2','out3','out4'];
const SEGMENTS_PER_METER = 14;
function initMonitor(){
    monitorActive=true;
    scopeCanvas=document.getElementById('scope-canvas');
    if(scopeCanvas){scopeCtx=scopeCanvas.getContext('2d');resizeScopeCanvas();}
    METER_CHANNELS.forEach(ch=>{
        const wrap=document.getElementById('meter-'+ch); if(!wrap)return;
        wrap.querySelectorAll('.meter-segment').forEach(el=>el.remove());
        for(let i=0;i<SEGMENTS_PER_METER;i++){const el=document.createElement('div');el.className='meter-segment';wrap.appendChild(el);}
    });
    const ble=document.getElementById('monitor-ble-status');
    if(ble){ble.textContent=isBLEConnected()?'ON':'OFF';ble.style.color=isBLEConnected()?'var(--neon-green)':'var(--neon-red)';}
    if(scopeAnimFrame)cancelAnimationFrame(scopeAnimFrame);
    renderMonitor();
}
function stopMonitor(){monitorActive=false;if(scopeAnimFrame){cancelAnimationFrame(scopeAnimFrame);scopeAnimFrame=null;}}
function resizeScopeCanvas(){if(!scopeCanvas)return;const rect=scopeCanvas.getBoundingClientRect();if(rect.width&&rect.height){scopeCanvas.width=Math.round(rect.width*devicePixelRatio);scopeCanvas.height=Math.round(rect.height*devicePixelRatio);scopeCtx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);}}
function renderMonitor(){if(!monitorActive)return;drawMonitorScope();updatePeakHolds();updateRmsDisplay();scopeAnimFrame=requestAnimationFrame(renderMonitor);}
function drawMonitorScope(){
 if(!scopeCtx||!scopeCanvas)return;
 const w=scopeCanvas.clientWidth,h=scopeCanvas.clientHeight;if(!w||!h)return;
 scopeCtx.fillStyle='#03060a';scopeCtx.fillRect(0,0,w,h);
 scopeCtx.strokeStyle='#1a2a40';scopeCtx.lineWidth=.5;
 for(let i=1;i<4;i++){scopeCtx.beginPath();scopeCtx.moveTo(0,h*i/4);scopeCtx.lineTo(w,h*i/4);scopeCtx.stroke();}
 for(let i=1;i<6;i++){scopeCtx.beginPath();scopeCtx.moveTo(w*i/6,0);scopeCtx.lineTo(w*i/6,h);scopeCtx.stroke();}
 scopeCtx.strokeStyle='#2a3a5a';scopeCtx.setLineDash([4,4]);scopeCtx.beginPath();scopeCtx.moveTo(0,h/2);scopeCtx.lineTo(w,h/2);scopeCtx.stroke();scopeCtx.setLineDash([]);
 const values=currentOscData,step=w/Math.max(1,values.length-1);
 [['#00ff88',0],['#00e5ff',5]].forEach(([color,offset])=>{
  scopeCtx.strokeStyle=color;scopeCtx.lineWidth=1.5;scopeCtx.shadowBlur=5;scopeCtx.shadowColor=color;scopeCtx.beginPath();
  values.forEach((v,i)=>{const y=((Math.max(0,Math.min(100,v+offset)))/100)*h;if(i===0)scopeCtx.moveTo(0,y);else scopeCtx.lineTo(i*step,y);});scopeCtx.stroke();
 });scopeCtx.shadowBlur=0;
}
function updatePeakHolds(){METER_CHANNELS.forEach(ch=>{const el=document.getElementById('peak-'+ch);if(!el)return;peakHoldValues[ch]=peakHoldValues[ch]>currentLevels[ch]?Math.max(currentLevels[ch],peakHoldValues[ch]-.25):currentLevels[ch];el.style.bottom=peakHoldValues[ch]+'%';});}
function updateMeterVisual(channel,level){
 if(!METER_CHANNELS.includes(channel))return;currentLevels[channel]=level;
 const wrap=document.getElementById('meter-'+channel);if(!wrap)return;
 const segs=wrap.querySelectorAll('.meter-segment'),count=Math.round(level/100*segs.length);
 segs.forEach((seg,i)=>{seg.className='meter-segment';if(i<count){const ratio=(i+1)/segs.length;seg.classList.add(ratio>.85?'peak':ratio>.65?'warn':'on');}});
 const dbEl=document.getElementById('db-'+channel);if(dbEl){const db=level>0?Math.round((level-100)/1.6):-99;dbEl.textContent=db>-99?db+' dB':'-∞';}
 if(level>98){const now=Date.now();if(now-lastClipTime>500){clipCounter++;lastClipTime=now;const el=document.getElementById('clip-count');if(el){el.textContent=clipCounter;el.className='value clip-alert';setTimeout(()=>{if(el)el.className='value';},1000);}}}
}
function updateSpectrumMini(values){currentSpectrum=values;let peak=0;values.slice(0,7).forEach((v,i)=>{const bar=document.getElementById('sm-bar-'+i);if(bar){const p=Math.max(2,v);bar.style.height=p+'%';bar.style.background=p>85?'var(--neon-red)':p>65?'var(--neon-yellow)':p>40?'var(--neon-green)':'var(--neon-blue)';}peak=Math.max(peak,v);});const label=document.getElementById('spectrum-peak-label');if(label)label.textContent='PEAK: '+peak+'%';}
function updateOscData(values){if(!scopeFrozen&&values.length)currentOscData=values.slice();}
function updateRmsDisplay(){const avg=(currentLevels.inL+currentLevels.inR)/2;const el=document.getElementById('rms-display');if(el){const db=avg>0?Math.round((avg-100)/1.6):-99;el.textContent=db>-99?db+' dB':'-∞ dB';}}
function toggleScopeFreeze(){scopeFrozen=!scopeFrozen;const btn=document.getElementById('scope-trigger');if(btn){btn.textContent=scopeFrozen?'❄ FROZEN':'▶ AUTO';btn.classList.toggle('frozen',scopeFrozen);}}
window.addEventListener('resize',()=>{if(monitorActive)resizeScopeCanvas();});


// === NOTES PAGE ===
let currentNoteTab = 'midhigh';
let notesData = { midhigh: '', low: '', sub: '', general: '' };
const NOTE_LABELS = { midhigh: 'MID-HIGH', low: 'LOW', sub: 'SUB', general: 'GENERAL' };

function loadNotesFromStorage() {
    try {
        const saved = localStorage.getItem('DLMS_NOTES');
        if (saved) {
            const parsed = JSON.parse(saved);
            if (parsed && typeof parsed === 'object') {
                Object.keys(notesData).forEach(key => {
                    if (typeof parsed[key] === 'string') notesData[key] = parsed[key];
                });
            }
        }
    } catch (err) {
        console.warn('Gagal memuat NOTES dari localStorage:', err);
    }
}
function saveNotesToStorage() {
    const area = document.getElementById('notes-textarea');
    if (area) notesData[currentNoteTab] = area.value;
    try {
        localStorage.setItem('DLMS_NOTES', JSON.stringify(notesData));
        const status = document.getElementById('notes-status');
        if (status) status.textContent = '✓ Tersimpan di perangkat ini';
    } catch (err) {
        const status = document.getElementById('notes-status');
        if (status) status.textContent = 'Gagal menyimpan. Periksa ruang penyimpanan browser.';
    }
}
function initNotesPage() {
    loadNotesFromStorage();
    switchNoteTab(currentNoteTab);
}
function switchNoteTab(tab) {
    if (!Object.prototype.hasOwnProperty.call(notesData, tab)) return;
    const area = document.getElementById('notes-textarea');
    if (area && currentNoteTab) notesData[currentNoteTab] = area.value;
    currentNoteTab = tab;
    document.querySelectorAll('.notes-tab').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.note === tab);
    });
    const title = document.getElementById('notes-editor-title');
    if (title) title.textContent = (NOTE_LABELS[tab] || tab.toUpperCase()) + ' NOTES';
    if (area) area.value = notesData[tab] || '';
    const status = document.getElementById('notes-status');
    if (status) status.textContent = 'Catatan disimpan otomatis di perangkat ini.';
}
function updateCurrentNote(value) {
    notesData[currentNoteTab] = value;
    saveNotesToStorage();
}
function clearCurrentNote() {
    if (!confirm('Hapus semua catatan pada kategori ' + (NOTE_LABELS[currentNoteTab] || currentNoteTab) + '?')) return;
    notesData[currentNoteTab] = '';
    const area = document.getElementById('notes-textarea');
    if (area) area.value = '';
    saveNotesToStorage();
}
function exportNotes() {
    const textContent = Object.keys(notesData).map(key => '=== ' + NOTE_LABELS[key] + ' ===\n' + (notesData[key] || '')).join('\n\n');
    const blob = new Blob([textContent], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'DLMS-notes.txt';
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(url);
}

function showPage(pageId) {
            const page = document.getElementById(pageId);
            if (!page) return;
            const currentPage = document.querySelector('.page.active');
            if (currentPage && currentPage.id === 'page-test-tone' && pageId !== 'page-test-tone' && currentToneMode !== 'off') {
                sendBLEData('SIG_GEN,0');
                currentToneMode = 'off';
                updateToneUIOff();
            }
            document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
            page.classList.add('active');
            if (pageId !== 'page-rta' && rtaActive) toggleRTA();
            rtaPageActive = (pageId === 'page-rta');
            if (pageId === 'page-front') syncDashboardUI();
            if (pageId === 'page-quick') requestAnimationFrame(() => buildMixer());
            if (pageId === 'page-presets') refreshEffectsPage();
            if (pageId === 'page-test-tone') initTestTonePage();
            if (pageId === 'page-system') refreshSystemPage();
            if (pageId !== 'page-monitor' && monitorActive) stopMonitor();
            if (pageId === 'page-monitor') setTimeout(initMonitor, 100);
            if (pageId === 'page-notes') initNotesPage();
            if (pageId === 'page-spectrum') setTimeout(initSpectrum, 100);
            if (pageId !== 'page-spectrum') stopSpectrum();
        }

// === MIXER CHANNEL STRIP BUILDER ===
const CHANNEL_CONFIG = [
    { id:1, name:'OUT 1', color:'var(--color-out1)', glow:'rgba(0,229,255,0.4)', meterKey:'out1' },
    { id:2, name:'OUT 2', color:'var(--color-out2)', glow:'rgba(255,145,0,0.4)', meterKey:'out2' },
    { id:3, name:'OUT 3', color:'var(--color-out3)', glow:'rgba(0,255,136,0.4)', meterKey:'out3' },
    { id:4, name:'OUT 4', color:'var(--color-out4)', glow:'rgba(255,0,51,0.4)', meterKey:'out4' }
];

function buildMixer() {
    const container = document.getElementById('mixer-container');
    if (!container) return;
    container.innerHTML = '';
    CHANNEL_CONFIG.forEach(ch => {
        const st = globalState[ch.id];
        if (!st) return;
        const strip = document.createElement('div');
        strip.className = 'channel-strip' + (st.isMute ? ' muted' : '');
        strip.style.setProperty('--channel-color', ch.color);
        strip.style.setProperty('--channel-glow', ch.glow);
        strip.dataset.channel = ch.id;
        const knobPercent = Math.max(0, Math.min(100, ((Number(st.gain) + 60) / 72) * 100));
        const knobAngle = -135 + (knobPercent / 100) * 270;
        strip.innerHTML = `
            <div class="channel-header">${ch.name}</div>
            <div class="knob-wrapper">
                <div class="knob-label">GAIN</div>
                <div class="knob" data-knob="${ch.id}" style="--knob-fill:${knobPercent}%;--knob-angle:${knobAngle}deg;" role="slider" aria-label="${ch.name} gain" tabindex="0"></div>
                <div class="knob-value" id="knob-val-${ch.id}">${Number(st.gain).toFixed(1)}</div>
            </div>
            <div class="fader-meter-area">
                <div class="led-meter-vertical" id="meter-${ch.id}">${Array.from({length:12},()=>'<div class="led"></div>').join('')}</div>
                <div class="fader-track" data-fader="${ch.id}" role="slider" aria-label="${ch.name} fader" tabindex="0">
                    <div class="fader-thumb" id="thumb-${ch.id}"></div>
                </div>
            </div>
            <div class="db-display" id="db-${ch.id}">${Number(st.gain).toFixed(1)} dB</div>
            <div class="strip-buttons">
                <button class="strip-btn mute-btn ${st.isMute?'active':''}" id="mute-btn-${ch.id}" onclick="toggleStripMute(${ch.id})" aria-pressed="${!!st.isMute}">MUTE</button>
                <button class="strip-btn phase-btn ${st.phase180?'active':''}" id="phase-btn-${ch.id}" onclick="toggleStripPhase(${ch.id})" aria-pressed="${!!st.phase180}">${st.phase180?'180°':'Ø'}</button>
            </div>`;
        container.appendChild(strip);
    });
    container.querySelectorAll('.fader-track').forEach(attachFaderDrag);
    container.querySelectorAll('.knob').forEach(attachKnobDrag);
    CHANNEL_CONFIG.forEach(ch => updateFaderPosition(ch.id));
}

function updateFaderPosition(chId) {
    const track=document.querySelector('.fader-track[data-fader="'+chId+'"]');
    const thumb=document.getElementById('thumb-'+chId);
    if(!track||!thumb||!globalState[chId]) return;
    const gain=Math.max(-60,Math.min(12,Number(globalState[chId].gain)||0));
    const percent=(gain+60)/72;
    const usable=Math.max(0,track.clientHeight-thumb.offsetHeight);
    thumb.style.top=(usable*(1-percent))+'px';
}

function updateKnobVisual(chId) {
    const knob=document.querySelector('.knob[data-knob="'+chId+'"]');
    if(!knob||!globalState[chId]) return;
    const gain=Math.max(-60,Math.min(12,Number(globalState[chId].gain)||0));
    const percent=((gain+60)/72)*100;
    knob.style.setProperty('--knob-fill',percent+'%');
    knob.style.setProperty('--knob-angle',(-135+(percent/100)*270)+'deg');
    const val=document.getElementById('knob-val-'+chId);
    if(val) val.innerText=gain.toFixed(1);
}

function updateDbDisplay(chId) {
    const el=document.getElementById('db-'+chId);
    if(el) el.innerText=Number(globalState[chId].gain).toFixed(1)+' dB';
    const slider=document.getElementById('dash-vol-'+chId);
    if(slider) slider.value=globalState[chId].gain;
    const label=document.getElementById('val-o'+chId);
    if(label) label.innerText=Number(globalState[chId].gain).toFixed(1)+' dB';
}

function setChannelGain(chId,gain) {
    if(!globalState[chId]) return;
    gain=Math.round(Math.max(-60,Math.min(12,Number(gain)||0))*10)/10;
    globalState[chId].gain=gain;
    sendBLEData('OUT'+chId+'_GAIN,'+gain);
    updateFaderPosition(chId);
    updateKnobVisual(chId);
    updateDbDisplay(chId);
    if(isLinked) applyGlobalLink(chId);
}

function attachFaderDrag(track) {
    const chId=Number(track.dataset.fader);
    let pointerId=null;
    const move=e=>{
        const rect=track.getBoundingClientRect();
        const thumbHeight=14;
        const usable=Math.max(1,rect.height-thumbHeight);
        const percent=Math.max(0,Math.min(1,1-((e.clientY-rect.top-thumbHeight/2)/usable)));
        setChannelGain(chId,-60+percent*72);
    };
    track.addEventListener('pointerdown',e=>{
        e.preventDefault();
        pointerId=e.pointerId;
        track.setPointerCapture(pointerId);
        move(e);
    });
    track.addEventListener('pointermove',e=>{if(pointerId===e.pointerId) move(e);});
    const end=e=>{if(pointerId===e.pointerId) pointerId=null;};
    track.addEventListener('pointerup',end);
    track.addEventListener('pointercancel',end);
    track.addEventListener('keydown',e=>{
        if(e.key==='ArrowUp'||e.key==='ArrowRight'){e.preventDefault();setChannelGain(chId,globalState[chId].gain+1);}
        if(e.key==='ArrowDown'||e.key==='ArrowLeft'){e.preventDefault();setChannelGain(chId,globalState[chId].gain-1);}
    });
}

function attachKnobDrag(knob) {
    const chId=Number(knob.dataset.knob);
    let startY=0,startGain=0,pointerId=null;
    knob.addEventListener('pointerdown',e=>{
        e.preventDefault();pointerId=e.pointerId;startY=e.clientY;startGain=Number(globalState[chId].gain);
        knob.setPointerCapture(pointerId);
    });
    knob.addEventListener('pointermove',e=>{
        if(pointerId!==e.pointerId)return;
        setChannelGain(chId,startGain+(startY-e.clientY)*0.3);
    });
    const end=e=>{if(pointerId===e.pointerId)pointerId=null;};
    knob.addEventListener('pointerup',end);
    knob.addEventListener('pointercancel',end);
    knob.addEventListener('keydown',e=>{
        if(e.key==='ArrowUp'||e.key==='ArrowRight'){e.preventDefault();setChannelGain(chId,globalState[chId].gain+1);}
        if(e.key==='ArrowDown'||e.key==='ArrowLeft'){e.preventDefault();setChannelGain(chId,globalState[chId].gain-1);}
    });
}

function toggleStripMute(chId) {
    const btn=document.getElementById('mute-btn-'+chId);
    const strip=btn?.closest('.channel-strip');
    if(!btn||!strip||!globalState[chId])return;
    globalState[chId].isMute=!globalState[chId].isMute;
    btn.classList.toggle('active',globalState[chId].isMute);
    btn.setAttribute('aria-pressed',String(globalState[chId].isMute));
    strip.classList.toggle('muted',globalState[chId].isMute);
    sendBLEData('OUT'+chId+'_MUTE,'+(globalState[chId].isMute?1:0));
    syncDashboardUI();
}

function toggleStripPhase(chId) {
    const btn=document.getElementById('phase-btn-'+chId);
    if(!btn||!globalState[chId])return;
    globalState[chId].phase180=!globalState[chId].phase180;
    btn.classList.toggle('active',globalState[chId].phase180);
    btn.innerText=globalState[chId].phase180?'180°':'Ø';
    btn.setAttribute('aria-pressed',String(globalState[chId].phase180));
    sendBLEData('OUT'+chId+'_PHASE,'+(globalState[chId].phase180?1:0));
    syncDashboardUI();
}

function updateStripMeter(chId,level) {
    const meter=document.getElementById('meter-'+chId);
    if(!meter)return;
    level=Math.max(0,Math.min(100,Number(level)||0));
    const leds=meter.querySelectorAll('.led');
    const activeCount=Math.round((level/100)*leds.length);
    leds.forEach((led,i)=>{
        led.className='led';
        if(i<activeCount){
            if(i>=leds.length-1)led.classList.add('peak');
            else if(i>=leds.length-3)led.classList.add('warn');
            else led.classList.add('on');
        }
    });
}


// === QUICK SCENES DATA ===
const SCENE_PRESETS = {
 flat:{name:'FLAT',outs:{1:{p1g:0,p2g:0,p3g:0,gain:0},2:{p1g:0,p2g:0,p3g:0,gain:0},3:{p1g:0,p2g:0,p3g:0,gain:0},4:{p1g:0,p2g:0,p3g:0,gain:0}}},
 bass:{name:'BASS BOOST',outs:{1:{p1g:3,p2g:0,p3g:0,gain:0},2:{p1g:3,p2g:0,p3g:0,gain:0},3:{p1g:5,p2g:0,p3g:0,gain:2},4:{p1g:6,p2g:0,p3g:0,gain:3}}},
 vocal:{name:'VOCAL',outs:{1:{p1g:-2,p2g:4,p3g:2,gain:1},2:{p1g:-2,p2g:4,p3g:2,gain:1},3:{p1g:0,p2g:2,p3g:0,gain:0},4:{p1g:-3,p2g:0,p3g:0,gain:-2}}},
 rock:{name:'ROCK',outs:{1:{p1g:2,p2g:-1,p3g:3,gain:1},2:{p1g:2,p2g:-1,p3g:3,gain:1},3:{p1g:4,p2g:0,p3g:2,gain:2},4:{p1g:5,p2g:0,p3g:0,gain:2}}},
 electronic:{name:'ELECTRONIC',outs:{1:{p1g:-3,p2g:0,p3g:4,gain:0},2:{p1g:-3,p2g:0,p3g:4,gain:0},3:{p1g:3,p2g:-2,p3g:0,gain:1},4:{p1g:6,p2g:0,p3g:-3,gain:3}}},
 acoustic:{name:'ACOUSTIC',outs:{1:{p1g:-1,p2g:2,p3g:1,gain:0},2:{p1g:-1,p2g:2,p3g:1,gain:0},3:{p1g:0,p2g:1,p3g:0,gain:0},4:{p1g:-2,p2g:0,p3g:0,gain:-1}}}
};
let activeScene = null;
let activeUserPreset = null;
let allEffectsBypassed = false;

function applyScene(sceneName) {
  const scene = SCENE_PRESETS[sceneName];
  if (!scene) return;
  for (let i = 1; i <= 4; i++) {
    const st = scene.outs[i];
    if (!st || !globalState[i]) continue;
    for (const [freqKey, gainKey, freqParam, gainParam, fallback] of [
      ['p1f', 'p1g', 'PEQ1FREQ', 'PEQ1GAIN', 100],
      ['p2f', 'p2g', 'PEQ2FREQ', 'PEQ2GAIN', 1000],
      ['p3f', 'p3g', 'PEQ3FREQ', 'PEQ3GAIN', 5000]
    ]) {
      const freq = Number(st[freqKey] ?? globalState[i][freqKey] ?? fallback);
      if (Number.isFinite(freq) && freq >= 20 && freq <= 20000) {
        globalState[i][freqKey] = freq;
        sendBLEData('OUT' + i + '_' + freqParam + ',' + freq);
      }
      if (st[gainKey] !== undefined) {
        globalState[i][gainKey] = st[gainKey];
        sendBLEData('OUT' + i + '_' + gainParam + ',' + st[gainKey]);
      }
    }
    if (st.gain !== undefined) {
      globalState[i].gain = st.gain;
      sendBLEData('OUT' + i + '_GAIN,' + st.gain);
    }
  }
  activeScene = sceneName;
  activeUserPreset = null;
  document.querySelectorAll('.scene-card').forEach(c => c.classList.toggle('active', c.dataset.scene === sceneName));
  document.querySelectorAll('.preset-large-btn').forEach(b => b.classList.remove('active'));
  syncDashboardUI();
  if (typeof buildMixer === 'function') buildMixer();
  showToast('🎨 Scene "' + scene.name + '" diterapkan');
}

function loadPreset(num) {
 if(!Number.isInteger(num)||num<1||num>4)return;
 // Gunakan loader bawaan agar migrasi field dan pengiriman BLE tetap konsisten.
 handlePresetClick(num);
 const saved=localStorage.getItem('DLMS_PRESET_'+num);
 if(!saved)return;
 try {
  const parsed=JSON.parse(saved);
  if(parsed.outs&&parsed.ins){
   activeUserPreset=num; activeScene=null;
   document.querySelectorAll('.scene-card').forEach(c=>c.classList.remove('active'));
   document.querySelectorAll('.preset-large-btn').forEach(b=>b.classList.toggle('active',b.id==='preset-btn-p'+num || b.id==='btn-p'+num));
   if(typeof buildMixer==='function')buildMixer();
  }
 } catch(e) { /* handlePresetClick menampilkan pesan jika data tidak valid */ }
}

function toggleAllBypass() {
 allEffectsBypassed=!allEffectsBypassed;
 const btn=document.getElementById('bypass-all-btn');
 if(btn){btn.classList.toggle('active',allEffectsBypassed);btn.innerText=allEffectsBypassed?'⚡ BYPASS AKTIF - TAP UNTUK NORMAL':'⚡ BYPASS ALL EFFECTS';}
 // Samakan tombol bypass lama jika masih ada di bagian UI lain.
 document.querySelectorAll('.preset-btn.bypass').forEach(b=>{b.classList.toggle('active',allEffectsBypassed);});
 sendBLEData('ALL_BYPASS,'+(allEffectsBypassed?1:0));
 showToast(allEffectsBypassed?'⚡ Bypass aktif':'🔊 Mode normal');
}

function refreshEffectsPage() {
 document.querySelectorAll('.scene-card').forEach(c=>c.classList.toggle('active',c.dataset.scene===activeScene));
 document.querySelectorAll('.preset-large-btn').forEach(b=>b.classList.toggle('active',b.id==='preset-btn-p'+activeUserPreset || b.id==='btn-p'+activeUserPreset));
 const btn=document.getElementById('bypass-all-btn');
 if(btn){btn.classList.toggle('active',allEffectsBypassed);btn.innerText=allEffectsBypassed?'⚡ BYPASS AKTIF - TAP UNTUK NORMAL':'⚡ BYPASS ALL EFFECTS';}
}

function showToast(msg) {
 let toast=document.getElementById('toast-msg');
 if(!toast){
  toast=document.createElement('div');toast.id='toast-msg';
  toast.style.cssText='position:fixed;bottom:30px;left:50%;transform:translateX(-50%);background:var(--panel-bg);color:var(--neon-green);border:1px solid var(--neon-green);padding:10px 20px;border-radius:20px;font-size:11px;font-weight:bold;z-index:9999;box-shadow:0 4px 12px rgba(0,0,0,.5);opacity:0;transition:opacity .3s;pointer-events:none;max-width:90vw;text-align:center;';
  document.body.appendChild(toast);
 }
 toast.innerText=msg;toast.style.opacity='1';
 clearTimeout(window._toastTimer);
 window._toastTimer=setTimeout(()=>{toast.style.opacity='0';},2000);
}

        window.onload = () => {
            loadStateToUI(1);
            loadInputUI('L');
            syncDashboardUI();
            const savedName = localStorage.getItem('DLMS_DEVICE_NAME');
            if (savedName) document.getElementById('dash-title').innerText = savedName;
            buildMixer();
        };

      const isNativeCapacitor = !!(window.Capacitor && typeof window.Capacitor.isNativePlatform === 'function' && window.Capacitor.isNativePlatform());
      if ('serviceWorker' in navigator && !isNativeCapacitor) {
        window.addEventListener('load', () => {
          navigator.serviceWorker.register('sw.js')
            .then(reg => console.log('✅ Service Worker terdaftar'))
            .catch(err => console.log('❌ Gagal daftar SW:', err));
        });
      }

    // === RTA PAGE VARIABLES ===
    let rtaAudioContext, rtaAnalyser, rtaMicrophone, rtaDataArray, rtaStream;
    let rtaActive = false;
    let rtaSmoothing = 0.8;
    let rtaSensitivity = 5; // 1 = less sensitive, 10 = more sensitive
    let rtaPeakHold = new Array(16).fill(0);
    let rtaPageActive = false;

    // 16 band logaritmik dari frekuensi aktual, bukan indeks FFT yang diasumsikan.
    // Batas bin dihitung ulang dari sample rate dan ukuran FFT perangkat.
    function buildRTABands(sampleRate, fftSize, bandCount = 16) {
        const nyquist = sampleRate / 2;
        const minHz = 31.5;
        const maxHz = Math.min(16000, nyquist * 0.95);
        const binHz = sampleRate / fftSize;
        const edges = [];
        for (let i = 0; i <= bandCount; i++) {
            edges.push(minHz * Math.pow(maxHz / minHz, i / bandCount));
        }
        return Array.from({ length: bandCount }, (_, i) => {
            let start = Math.max(0, Math.floor(edges[i] / binHz));
            let end = Math.min(fftSize / 2, Math.max(start + 1, Math.ceil(edges[i + 1] / binHz)));
            // Untuk band di bawah resolusi FFT, tetap ambil bin terdekat.
            if (start >= fftSize / 2) start = fftSize / 2 - 1;
            if (end <= start) end = start + 1;
            return [start, Math.min(fftSize / 2, end)];
        });
    }

    let rtaBands = [];

    function openRTAPage() {
        rtaPageActive = true;
        document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
        document.getElementById('page-rta').classList.add('active');
    }

    function closeRTAPage() {
        rtaPageActive = false;
        // Auto-stop RTA saat keluar halaman
        if (rtaActive) toggleRTA();
        showPage('page-front');
    }

    function updateRTASmoothing(val) {
        rtaSmoothing = val / 100;
        document.getElementById('rta-smooth-val').innerText = val + '%';
        if (rtaAnalyser) rtaAnalyser.smoothingTimeConstant = rtaSmoothing;
    }

    function updateRTASensitivity(val) {
        rtaSensitivity = Math.max(1, Math.min(10, Number(val) || 5));
        const label = document.getElementById('rta-sensitivity-val');
        if (label) label.innerText = rtaSensitivity + '/10';
    }

    async function toggleRTA() {
        const btn = document.getElementById('rta-toggle-btn');
        const status = document.getElementById('rta-status');
        
        if (!rtaActive) {
            try {
                status.innerText = "⏳ Memeriksa izin mikrofon...";
                if (!navigator.mediaDevices || typeof navigator.mediaDevices.getUserMedia !== 'function') {
                    throw new Error('Akses mikrofon tidak tersedia di WebView ini. Perbarui Android System WebView/Chrome.');
                }

                const cap = window.Capacitor;
                const nativePlatform = !!(cap && typeof cap.isNativePlatform === 'function' && cap.isNativePlatform());
                if (nativePlatform) {
                    const permissionPlugin = window.DLMSMicrophonePermission;
                    if (!permissionPlugin || typeof permissionPlugin.request !== 'function') {
                        throw new Error('Jembatan izin mikrofon native tidak tersedia. Instal ulang APK terbaru.');
                    }
                    const permission = await permissionPlugin.request();
                    if (!permission || permission.granted !== true) {
                        throw new Error('Izin mikrofon ditolak Android. Buka Pengaturan > Aplikasi > DLMS Mini > Izin > Mikrofon, lalu pilih Izinkan.');
                    }
                }

                status.innerText = "⏳ Membuka mikrofon...";
                rtaStream = await navigator.mediaDevices.getUserMedia({
                    audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false }
                });
                const AudioContextClass = window.AudioContext || window.webkitAudioContext;
                if (!AudioContextClass) throw new Error('AudioContext tidak didukung perangkat ini.');
                rtaAudioContext = new AudioContextClass();
                if (rtaAudioContext.state === 'suspended') await rtaAudioContext.resume();
                rtaAnalyser = rtaAudioContext.createAnalyser();
                rtaAnalyser.fftSize = 4096;
                rtaAnalyser.smoothingTimeConstant = rtaSmoothing;
                
                rtaMicrophone = rtaAudioContext.createMediaStreamSource(rtaStream);
                rtaMicrophone.connect(rtaAnalyser);
                
                rtaDataArray = new Uint8Array(rtaAnalyser.frequencyBinCount);
                rtaBands = buildRTABands(rtaAudioContext.sampleRate, rtaAnalyser.fftSize, 16);
                rtaPeakHold = new Array(16).fill(0);
                
                rtaActive = true;
                btn.innerText = "🛑 STOP";
                btn.classList.add('active');
                status.innerText = "🎤 RTA Aktif - Arahkan mic ke speaker";
                
                updateRTA16Band();
            } catch (err) {
                console.error('RTA microphone error:', err);
                const reason = err && err.name ? err.name + ': ' : '';
                const detail = (err && err.message) ? err.message : 'Kesalahan tidak diketahui';
                status.innerText = "❌ " + reason + detail;
                alert("RTA gagal mengakses mikrofon.\n" + reason + detail);
                if (rtaStream) {
                    rtaStream.getTracks().forEach(track => track.stop());
                    rtaStream = null;
                }
                if (rtaAudioContext) {
                    try { await rtaAudioContext.close(); } catch (_) {}
                    rtaAudioContext = null;
                }
                rtaAnalyser = null;
                rtaMicrophone = null;
            }
        } else {
            rtaActive = false;
            if (rtaMicrophone) {
                try { rtaMicrophone.disconnect(); } catch (_) {}
                rtaMicrophone = null;
            }
            if (rtaStream) {
                rtaStream.getTracks().forEach(track => track.stop());
                rtaStream = null;
            }
            if (rtaAudioContext) {
                try { await rtaAudioContext.close(); } catch (_) {}
                rtaAudioContext = null;
            }
            rtaAnalyser = null;
            btn.innerText = "🎤 MULAI";
            btn.classList.remove('active');
            status.innerText = "Tekan MULAI untuk aktifkan mikrofon HP";
            
            // Reset semua bar
            for(let i=0; i<16; i++) {
                let bar = document.getElementById('rta-bar-'+i);
                if(bar) bar.style.height = '0%';
            }
            document.getElementById('rta-peak-db').innerText = '-∞ dB';
            document.getElementById('rta-rms-db').innerText = '-∞ dB';
        }
    }

    function updateRTA16Band() {
    if (!rtaActive || !rtaPageActive) return;
    requestAnimationFrame(updateRTA16Band);
    
    rtaAnalyser.getByteFrequencyData(rtaDataArray);
    
    let totalSum = 0;
    let maxVal = 0;
    
    for (let i = 0; i < 16; i++) {
        let sum = 0, count = 0;
        for (let j = rtaBands[i][0]; j < rtaBands[i][1] && j < rtaDataArray.length; j++) {
            sum += rtaDataArray[j];
            count++;
        }
        let avg = count > 0 ? sum / count : 0;

        // Sensitivitas tampilan + noise gate. Ini mengatur grafik RTA, bukan gain fisik mikrofon.
        const noiseGate = 3 + (10 - rtaSensitivity) * 2.5;
        if (avg < noiseGate) {
            avg = 0;
        } else {
            const displayGain = 0.65 + (rtaSensitivity * 0.07);
            avg = Math.min(255, avg * displayGain);
        }

        // Clamp percent max 100% agar bar tidak overflow.
        let percent = Math.min(100, (avg / 255) * 100);
        
        // Peak hold (turun perlahan)
        if (percent > rtaPeakHold[i]) {
            rtaPeakHold[i] = percent;
        } else {
            rtaPeakHold[i] = Math.max(0, rtaPeakHold[i] - 0.5);
        }
        
        let bar = document.getElementById('rta-bar-'+i);
        if (bar) {
            // FIX: Clamp height max 100%
            bar.style.height = Math.max(1, Math.min(100, percent)) + '%';
            
            // FIX: Warna sesuai legenda (berdasarkan posisi band, bukan level)
            if (i < 4) bar.style.background = 'var(--neon-blue)';        // Sub-Bass
            else if (i < 8) bar.style.background = 'var(--neon-green)';  // Mid
            else if (i < 12) bar.style.background = 'var(--neon-yellow)'; // High-Mid
            else bar.style.background = 'var(--neon-red)';               // Treble
        }
        
        totalSum += avg;
        if (avg > maxVal) maxVal = avg;
    }
    
    // Update dB display
    let avgLevel = totalSum / 16;
    let dbLevel = avgLevel > 0 ? (20 * Math.log10(avgLevel / 255)).toFixed(1) : '-∞';
    let peakDb = maxVal > 0 ? (20 * Math.log10(maxVal / 255)).toFixed(1) : '-∞';
    
    document.getElementById('rta-rms-db').innerText = dbLevel + ' dB';
    document.getElementById('rta-peak-db').innerText = peakDb + ' dB';
}

    // Mode simulasi: analisis AI berjalan tanpa mengirim perintah ke perangkat.
    let aiAudioSimulationMode = false;

    function testModeAI() {
        const previousSpectrum = currentSpectrum;
        const previousLevels = { ...currentLevels };

        try {
            // Data simulasi: bass 150 Hz dominan dan output 1 hampir clipping.
            currentSpectrum = [30, 95, 40, 20, 15, 10, 5];
            currentLevels = {
                ...currentLevels,
                out1: 98,
                out2: 50,
                out3: 50,
                out4: 50
            };
            aiAudioSimulationMode = true;
            aiAudioAssistant();
        } finally {
            // Kembalikan data monitor asli setelah pengujian.
            currentSpectrum = previousSpectrum;
            currentLevels = previousLevels;
            aiAudioSimulationMode = false;
        }
    }

    // === FITUR AI AUDIO ASSISTANT (RULE-BASED OFFLINE) ===
    // Menggunakan data monitor yang benar-benar tersedia: currentSpectrum/currentLevels.
    function aiAudioAssistant() {
        const s = Array.isArray(currentSpectrum) ? currentSpectrum : [];
        const level = currentLevels || {};
        const advice = [];
        const actions = [];
        let hasIssue = false;

        if (s.length < 7 || (s.every(v => Number(v) === 0) &&
            ['inL', 'inR', 'out1', 'out2', 'out3', 'out4'].every(ch => Number(level[ch] || 0) === 0))) {
            alert('Data spektrum/level belum tersedia. Hubungkan BLE, buka MONITOR, lalu tunggu data masuk sebelum menganalisis.');
            return;
        }

        // Aturan 1: Bass boomy (150 Hz dominan)
        if (s[1] > 75 && s[1] > s[2] + 20) {
            advice.push('⚠️ Deteksi: 150 Hz terlalu dominan (boomy/bergumam).\n💡 Saran: atur PEQ Band 1 OUT 3 ke -3 dB.');
            actions.push({
                prompt: 'Terapkan saran bass boomy? OUT 3 PEQ 1 akan disetel ke -3 dB.',
                commands: ['OUT3_PEQ1FREQ,150', 'OUT3_PEQ1GAIN,-3'],
                toast: 'Perintah dikirim: OUT 3 PEQ 1 = -3 dB'
            });
            hasIssue = true;
        }

        // Aturan 2: Vokal tertimbun bass
        if (s[3] < 40 && (s[1] > 50 || s[0] > 50)) {
            advice.push('⚠️ Deteksi: 1 kHz (vokal) rendah sementara bass tinggi.\n💡 Saran: atur PEQ Band 2 OUT 1 dan OUT 2 ke +2 dB.');
            actions.push({
                prompt: 'Terapkan saran vokal? OUT 1 dan OUT 2 PEQ 2 akan disetel ke +2 dB.',
                commands: ['OUT1_PEQ2FREQ,1000', 'OUT1_PEQ2GAIN,2', 'OUT2_PEQ2FREQ,1000', 'OUT2_PEQ2GAIN,2'],
                toast: 'Perintah dikirim: OUT 1 & 2 PEQ 2 = +2 dB'
            });
            hasIssue = true;
        }

        // Aturan 3: Treble terlalu tajam
        if (s[5] > 85 || s[6] > 85) {
            advice.push('⚠️ Deteksi: frekuensi tinggi (6–15 kHz) sangat dominan.\n💡 Saran: atur PEQ Band 3 OUT 1 dan OUT 2 ke -2 dB.');
            actions.push({
                prompt: 'Terapkan saran treble? OUT 1 dan OUT 2 PEQ 3 akan disetel ke -2 dB.',
                commands: ['OUT1_PEQ3FREQ,8000', 'OUT1_PEQ3GAIN,-2', 'OUT2_PEQ3FREQ,8000', 'OUT2_PEQ3GAIN,-2'],
                toast: 'Perintah dikirim: OUT 1 & 2 PEQ 3 = -2 dB'
            });
            hasIssue = true;
        }

        // Aturan 4: Level output mendekati clipping
        if ([level.out1, level.out2, level.out3, level.out4].some(v => Number(v) > 95)) {
            advice.push('🚨 PERINGATAN: level output mendekati clipping! Turunkan MASTER VOL atau gain channel dan periksa level sebelum menaikkan volume lagi.');
            hasIssue = true;
        }

        if (!hasIssue) {
            advice.push('✅ Analisis: tidak ada masalah yang terdeteksi oleh aturan sederhana ini. Ini bukan jaminan sistem audio aman atau sudah seimbang.');
        }

        let result = '🤖 AI AUDIO ASSISTANT (analisis berbasis aturan)\n\n' + advice.join('\n\n');
        if (aiAudioSimulationMode) {
            result += '\n\n🧪 MODE SIMULASI: tidak ada perintah yang dikirim ke perangkat.';
        } else if (actions.length && !isBLEConnected()) {
            result += '\n\nBLE belum terhubung, jadi saran tidak dapat diterapkan dari sini.';
        }
        alert(result);

        // Mode simulasi tidak boleh mengirim perintah ke perangkat.
        if (isBLEConnected() && !aiAudioSimulationMode) {
            actions.forEach(action => {
                if (confirm(action.prompt)) {
                    action.commands.forEach(command => {
                        const freqMatch = command.match(/^OUT([1-4])_PEQ([1-3])FREQ,([0-9.]+)$/);
                        if (freqMatch) {
                            const outNum = Number(freqMatch[1]);
                            const band = Number(freqMatch[2]);
                            globalState[outNum]['p' + band + 'f'] = Number(freqMatch[3]);
                        }
                        sendBLEData(command);
                    });
                    showToast(action.toast);
                }
            });
        }
    }

    
/* === SPECTRUM 4-OUT ENGINE === */
let specCanvas=null,specCtx=null,specAnimFrame=null,specFrozen=false,specSimMode=true,specSmoothing=.7,simTime=0;
const SPEC_COLORS={1:{stroke:'#00e5ff',fill:'rgba(0,229,255,.15)',glow:'rgba(0,229,255,.6)'},2:{stroke:'#ff9100',fill:'rgba(255,145,0,.15)',glow:'rgba(255,145,0,.6)'},3:{stroke:'#00ff88',fill:'rgba(0,255,136,.15)',glow:'rgba(0,255,136,.6)'},4:{stroke:'#ff0033',fill:'rgba(255,0,51,.15)',glow:'rgba(255,0,51,.6)'}};
const SPEC_FREQS=[60,150,400,1000,2400,6000,15000],SPEC_DB_MIN=-60,SPEC_DB_MAX=10;
let specData={1:Array(7).fill(0),2:Array(7).fill(0),3:Array(7).fill(0),4:Array(7).fill(0)},specSmoothData={1:Array(7).fill(0),2:Array(7).fill(0),3:Array(7).fill(0),4:Array(7).fill(0)},specPeak={1:Array(7).fill(0),2:Array(7).fill(0),3:Array(7).fill(0),4:Array(7).fill(0)},specVisible={1:true,2:true,3:true,4:true};
function initSpectrum(){specCanvas=document.getElementById('spectrumCanvas');if(!specCanvas)return;specCtx=specCanvas.getContext('2d');resizeSpecCanvas();if(!specCanvas.dataset.resizeBound){window.addEventListener('resize',resizeSpecCanvas);specCanvas.dataset.resizeBound='1';}if(specAnimFrame)cancelAnimationFrame(specAnimFrame);const b=document.getElementById('spec-sim-btn');if(b)b.classList.toggle('active',specSimMode);renderSpectrumLoop();}
function resizeSpecCanvas(){if(!specCanvas)return;const r=specCanvas.parentElement.getBoundingClientRect();if(!r.width||!r.height)return;const d=window.devicePixelRatio||1;specCanvas.width=Math.round(r.width*d);specCanvas.height=Math.round(r.height*d);specCanvas.style.width=r.width+'px';specCanvas.style.height=r.height+'px';specCtx.setTransform(d,0,0,d,0,0);}
function stopSpectrum(){if(specAnimFrame)cancelAnimationFrame(specAnimFrame);specAnimFrame=null;specCanvas=null;specCtx=null;}
function renderSpectrumLoop(){if(!specCtx||!document.getElementById('page-spectrum')?.classList.contains('active')){specAnimFrame=null;return;}if(specSimMode&&!specFrozen)generateSimData();if(!specFrozen)for(let ch=1;ch<=4;ch++)for(let i=0;i<7;i++)specSmoothData[ch][i]=specSmoothData[ch][i]*specSmoothing+specData[ch][i]*(1-specSmoothing);drawSpectrum();const ble=document.getElementById('spec-ble');if(ble){ble.innerText=typeof isBLEConnected()!=='undefined'&&isBLEConnected()?'ON':'OFF';ble.style.color=typeof isBLEConnected()!=='undefined'&&isBLEConnected()?'var(--neon-green)':'var(--neon-red)';}specAnimFrame=requestAnimationFrame(renderSpectrumLoop);}
function generateSimData(){simTime+=.02;specData[1]=[10+Math.sin(simTime)*5,25+Math.sin(simTime*1.3)*10,50+Math.sin(simTime*.8)*15,70+Math.sin(simTime*1.1)*10,65+Math.sin(simTime*.9)*12,55+Math.sin(simTime*1.5)*15,30+Math.sin(simTime*2)*10];specData[2]=specData[1].map((v,i)=>Math.max(0,v+Math.sin(simTime*.7+i)*8));specData[3]=[40+Math.sin(simTime*1.2)*15,75+Math.sin(simTime*.9)*12,60+Math.sin(simTime*1.4)*10,20+Math.sin(simTime)*8,10+Math.sin(simTime*.8)*5,5,0];specData[4]=[80+Math.sin(simTime*.6)*15,70+Math.sin(simTime*.8)*12,30+Math.sin(simTime*1.1)*10,5,0,0,0];for(let ch=1;ch<=4;ch++)specData[ch]=specData[ch].map(v=>Math.max(0,Math.min(100,v)));}
function freqToX(f,w,p){return p+(Math.log10(f)-Math.log10(20))/(Math.log10(20000)-Math.log10(20))*(w-p-10);}
function drawSpectrum(){if(!specCtx||!specCanvas)return;const w=specCanvas.clientWidth,h=specCanvas.clientHeight;if(!w||!h)return;const l=35,t=10,b=22,r=10;specCtx.clearRect(0,0,w,h);specCtx.fillStyle='#03060a';specCtx.fillRect(0,0,w,h);specCtx.font='9px monospace';specCtx.strokeStyle='#1a2a40';specCtx.lineWidth=.5;specCtx.fillStyle='#4a5a7a';specCtx.textAlign='center';[20,50,100,200,500,1000,2000,5000,10000,20000].forEach(f=>{const x=freqToX(f,w,l);if(x>=l&&x<=w-r){specCtx.beginPath();specCtx.moveTo(x,t);specCtx.lineTo(x,h-b);specCtx.stroke();specCtx.fillText(f>=1000?(f/1000)+'k':String(f),x,h-5);}});specCtx.textAlign='right';[10,0,-10,-20,-30,-40,-50,-60].forEach(db=>{const y=h-b-(db-SPEC_DB_MIN)/(SPEC_DB_MAX-SPEC_DB_MIN)*(h-t-b);specCtx.strokeStyle=db===0?'rgba(255,255,255,.3)':'#1a2a40';specCtx.beginPath();specCtx.moveTo(l,y);specCtx.lineTo(w-r,y);specCtx.stroke();specCtx.fillStyle=db===0?'#fff':'#4a5a7a';specCtx.fillText(String(db),l-4,y+3);});for(let ch=4;ch>=1;ch--)if(specVisible[ch])drawSpecChannel(ch,w,h,l,t,b);updateSpecStats();}
function drawSpecChannel(ch,w,h,l,t,b){const d=specSmoothData[ch],co=SPEC_COLORS[ch],yy=v=>h-b-(v/100)*(h-t-b);if(!specFrozen)for(let i=0;i<7;i++)specPeak[ch][i]=Math.max(d[i],specPeak[ch][i]-.3);specCtx.beginPath();specCtx.moveTo(l,h-b);for(let i=0;i<7;i++){const x=freqToX(SPEC_FREQS[i],w,l),y=yy(d[i]);if(!i)specCtx.lineTo(x,y);else{const px=freqToX(SPEC_FREQS[i-1],w,l),py=yy(d[i-1]),cx=(px+x)/2;specCtx.bezierCurveTo(cx,py,cx,y,x,y);}}specCtx.lineTo(w-10,h-b);specCtx.closePath();specCtx.fillStyle=co.fill;specCtx.fill();specCtx.beginPath();for(let i=0;i<7;i++){const x=freqToX(SPEC_FREQS[i],w,l),y=yy(d[i]);if(!i)specCtx.moveTo(x,y);else{const px=freqToX(SPEC_FREQS[i-1],w,l),py=yy(d[i-1]),cx=(px+x)/2;specCtx.bezierCurveTo(cx,py,cx,y,x,y);}}specCtx.strokeStyle=co.stroke;specCtx.lineWidth=2;specCtx.shadowBlur=8;specCtx.shadowColor=co.glow;specCtx.stroke();specCtx.shadowBlur=0;specCtx.setLineDash([3,3]);specCtx.beginPath();specPeak[ch].forEach((v,i)=>{const x=freqToX(SPEC_FREQS[i],w,l),y=yy(v);if(!i)specCtx.moveTo(x,y);else specCtx.lineTo(x,y);});specCtx.strokeStyle=co.stroke;specCtx.globalAlpha=.5;specCtx.stroke();specCtx.globalAlpha=1;specCtx.setLineDash([]);for(let i=0;i<7;i++){specCtx.beginPath();specCtx.arc(freqToX(SPEC_FREQS[i],w,l),yy(d[i]),3,0,Math.PI*2);specCtx.fillStyle=co.stroke;specCtx.fill();}}
function updateSpecStats(){let p=0,s=0,n=0;for(let ch=1;ch<=4;ch++)if(specVisible[ch])specSmoothData[ch].forEach(v=>{p=Math.max(p,v);s+=v*v;n++;});const db=v=>v>0?(SPEC_DB_MIN+v/100*(SPEC_DB_MAX-SPEC_DB_MIN)).toFixed(1):'-∞';const pe=document.getElementById('spec-peak'),re=document.getElementById('spec-rms');if(pe)pe.innerText=db(p)+' dB';if(re)re.innerText=db(n?Math.sqrt(s/n):0)+' dB';}
function toggleSpectrumFreeze(){specFrozen=!specFrozen;const b=document.getElementById('spec-freeze-btn');if(b){b.innerText=specFrozen?'▶ RESUME':'❄ FREEZE';b.classList.toggle('active',specFrozen);}}
function toggleSpectrumSim(){specSimMode=!specSimMode;const b=document.getElementById('spec-sim-btn');if(b)b.classList.toggle('active',specSimMode);if(typeof showToast==='function')showToast(specSimMode?'🧪 Mode Simulasi AKTIF':'📡 Mode Data Real ESP32');}
function toggleSpecChannel(ch){specVisible[ch]=!specVisible[ch];const e=document.querySelector('.legend-item[data-ch="'+ch+'"]');if(e)e.classList.toggle('hidden',!specVisible[ch]);}
function updateSpecSmooth(v){specSmoothing=Math.max(0,Math.min(95,Number(v)))/100;const e=document.getElementById('spec-smooth-val');if(e)e.innerText=v+'%';}
function aiSpectrumAnalysis(){const panel=document.getElementById('spec-ai-panel'),out=document.getElementById('spec-ai-content');if(!panel||!out)return;panel.classList.add('show');let report='',issues=0;for(let ch=1;ch<=4;ch++){if(!specVisible[ch])continue;const d=specSmoothData[ch],name=['Mid-High R','Mid-High L','Low','Sub'][ch-1],color=SPEC_COLORS[ch].stroke;if(d.some(v=>v>95)){report+='<div style="color:'+color+'">🚨 OUT '+ch+' ('+name+'): CLIPPING — turunkan gain.</div>';issues++;}if((ch===3||ch===4)&&d[1]>85){report+='<div style="color:'+color+'">⚠️ OUT '+ch+': Bass 150Hz dominan (Boomy).</div>';issues++;}if((ch===1||ch===2)&&(d[5]>85||d[6]>85)){report+='<div style="color:'+color+'">⚠️ OUT '+ch+': Treble tajam (Harsh).</div>';issues++;}if((ch===1||ch===2)&&d[3]<25&&(d[0]>60||d[1]>60)){report+='<div style="color:'+color+'">⚠️ OUT '+ch+': Vokal mungkin tertutup bass.</div>';issues++;}}const a=specSmoothData[1].reduce((x,y)=>x+y,0)/7,z=specSmoothData[2].reduce((x,y)=>x+y,0)/7;if(Math.abs(a-z)>20){report+='<div>⚖️ L/R tidak seimbang: '+Math.round(Math.abs(a-z))+'%.</div>';issues++;}out.innerHTML=issues?report+'<div style="margin-top:8px;color:var(--neon-yellow)">Total '+issues+' masalah.</div>':'<div style="color:var(--neon-green)">Tidak ada anomali besar pada data saat ini.</div>';}
function handleSpectrum4Data(text){if(specSimMode||typeof text!=='string'||!text.startsWith('SPECTRUM4:'))return;const rows=text.slice(10).split('|');if(rows.length!==4)return;const parsed=rows.map(x=>x.split(',').map(Number));if(parsed.some(a=>a.length!==7||a.some(v=>!Number.isFinite(v))))return;for(let ch=1;ch<=4;ch++)specData[ch]=parsed[ch-1].map(v=>Math.max(0,Math.min(100,v)));}