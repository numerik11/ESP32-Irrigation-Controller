import assert from 'node:assert/strict';
import test from 'node:test';
import { readFirmware, compileFirmwareFunctions, extractFunction } from './helpers/firmware-source.mjs';
const source = await readFirmware(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url));
const {noWaterPeriodMatches: matches} = compileFirmwareFunctions(source, ['noWaterPeriodMatches'], {});
test('weekend periods include start and exclude end', () => {
  for (const day of [0,6]) {
    assert.equal(matches(65,660,1020,day,659),false);
    assert.equal(matches(65,660,1020,day,660),true);
    assert.equal(matches(65,660,1020,day,1019),true);
    assert.equal(matches(65,660,1020,day,1020),false);
  }
  assert.equal(matches(65,660,1020,1,720),false);
});
test('overnight periods carry selected start days across week boundaries', () => {
  assert.equal(matches(64,1320,360,6,1320),true);
  assert.equal(matches(64,1320,360,0,359),true);
  assert.equal(matches(64,1320,360,0,360),false);
  assert.equal(matches(64,1320,360,6,300),false);
  assert.equal(matches(64,1320,360,0,1320),false);
});
test('all-day, empty and invalid periods', () => {
  assert.equal(matches(2,660,660,1,0),true);
  assert.equal(matches(2,660,660,1,1439),true);
  assert.equal(matches(2,660,660,2,0),false);
  assert.equal(matches(0,0,0,1,720),false);
  assert.equal(matches(127,-1,0,1,720),false);
  assert.equal(matches(127,0,1440,1,720),false);
});
test('blocked periods discard queued runs and stop automatic zones only', () => {
  for (const blocked of [false,true]) {
    const pendingStart=[true,true,false], stopped=[],events=[];
    const {enforceNoWaterPeriod} = compileFirmwareFunctions(source,['enforceNoWaterPeriod'],{
      noWaterPeriodActiveNow:()=>blocked,zonesCount:3,pendingStart,
      zoneActive:[true,true,false],zoneStartedManual:[false,true,false],
      turnOffZone:z=>stopped.push(z),logEvent:(...args)=>events.push(args)
    });
    enforceNoWaterPeriod();
    assert.deepEqual(pendingStart,blocked?[false,false,false]:[true,true,false]);
    assert.deepEqual(stopped,blocked?[0]:[]);
    assert.equal(events.length,blocked?2:0);
  }
});
test('automatic start paths check periods before wind queueing',()=>{
  const start=extractFunction(source,'turnOnZone');
  assert.ok(start.indexOf('noWaterPeriodActiveNow()')<start.indexOf('if (windBlocksZone(z))'));
  const due=extractFunction(source,'shouldStartZone');
  assert.ok(due.indexOf('lastCheckedMinute[zone] = mn')<due.indexOf('noWaterPeriodActiveNow()'));
  assert.ok(extractFunction(source,'loop').includes('enforceNoWaterPeriod();'));
});
