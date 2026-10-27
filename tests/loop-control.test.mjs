import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import { compileFirmwareFunctions, extractFunction } from './helpers/firmware-source.mjs';

const source = await readFile(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url), 'utf8');

function controller({ now = 500, blocked = false, concurrent = false } = {}) {
  const events = [];
  const zoneActive = [true, false];
  const pendingStart = [false, true];
  const { tickIrrigationControl } = compileFirmwareFunctions(source, ['tickIrrigationControl'], {
    millis: () => now, lastTimeQuery: now, TIME_QUERY_MS: 1000,
    cachedTm: { tm_hour: 12, tm_min: 0 }, midnightDone: false,
    enforceNoWaterPeriod() {}, checkWindRain() {}, stopAutoZonesForBlock() {},
    zonesCount: 2, zoneActive, pendingStart,
    hasDurationCompleted: z => z === 0,
    turnOffZone(z) { events.push(`off:${z}`); zoneActive[z] = false; },
    turnOnZone(z) { events.push(`on:${z}`); zoneActive[z] = true; },
    lastScheduleTick: 0, SCHEDULE_TICK_MS: 1000,
    isBlockedNow: () => blocked, rainActive: false, windBlocksZone: () => false,
    shouldStartZone: () => false, runZonesConcurrent: concurrent,
  }, { replacements: [[/&nowTime/g, 'nowTime'], [/&cachedTm/g, 'cachedTm']] });
  return { tickIrrigationControl, events, pendingStart };
}

test('Valve expiry is enforced even when the one-second start scheduler is not due', () => {
  const c = controller();
  c.tickIrrigationControl();
  assert.deepEqual(c.events, ['off:0']);
  assert.equal(c.pendingStart[1], true);
});

for (const concurrent of [false, true]) {
  test(`Expired valves stop before queued starts (concurrent=${concurrent})`, () => {
    const c = controller({ now: 1000, concurrent });
    c.tickIrrigationControl();
    assert.deepEqual(c.events, ['off:0', 'on:1']);
    assert.equal(c.pendingStart[1], false);
  });
}

test('Blocked scheduling still expires running valves without starting queued zones', () => {
  const c = controller({ now: 1000, blocked: true });
  c.tickIrrigationControl();
  assert.deepEqual(c.events, ['off:0']);
  assert.equal(c.pendingStart[1], true);
});

test('Valve transitions defer screen painting and the loop checks controls before serving HTTP', () => {
  for (const name of ['turnOnZone', 'turnOffZone']) {
    const body = extractFunction(source, name);
    assert.doesNotMatch(body, /delay\(|HomeScreen\(|fillScreen\(/);
    assert.match(body, /lastScreenRefresh = 0/);
  }
  const loop = extractFunction(source, 'loop');
  assert.ok(loop.indexOf('tickIrrigationControl();') < loop.indexOf('server.handleClient();'));
  assert.match(loop, /server\.handleClient\(\);[^\n]*\n\s*now = millis\(\)/);
});
