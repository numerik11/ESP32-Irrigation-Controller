import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import vm from 'node:vm';
import test from 'node:test';
import { compileFirmwareFunctions } from './helpers/firmware-source.mjs';

const source = await readFile(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url), 'utf8');
const polling = source.match(/R"SETUPSTATUS\(([\s\S]*?)\)SETUPSTATUS"/)[1];

test('The complete generated Setup JavaScript parses', () => {
  const setup = source.slice(source.indexOf('void handleSetupPage() {'), source.indexOf('// ---------- Schedule POST'));
  const scriptSource = setup.slice(setup.indexOf('html += F("<script>'));
  const fragments = [...scriptSource.matchAll(/html \+= F\(("(?:[^"\\]|\\.)*"|R"(\w+)\(([\s\S]*?)\)\2")\);/g)];
  const html = fragments.map(match => match[2] ? match[3] : JSON.parse(match[1])).join('');
  const script = html.split('<script>')[1].split('</script>')[0];
  assert.ok(script.includes('refreshSetupStatus();'));
  new vm.Script(script);
});

function browser(fetch) {
  const timers = new Map();
  const listeners = {};
  let timerId = 0;
  const context = vm.createContext({
    fetch, AbortController, document: { hidden: false },
    setTimeout(fn, ms) { timers.set(++timerId, { fn, ms }); return timerId; },
    clearTimeout(id) { timers.delete(id); },
    window: { addEventListener(name, fn) { listeners[name] = fn; } },
    g() { return null; },
  });
  vm.runInContext(polling, context);
  return { context, timers, listeners };
}

test('Setup shares an in-flight status request and permits a fresh request after completion', async () => {
  let calls = 0, complete;
  const { context } = browser(() => { calls++; return new Promise(resolve => { complete = resolve; }); });
  const first = context.fetchSetupStatus();
  assert.equal(context.fetchSetupStatus(), first);
  assert.equal(calls, 1);
  complete({ ok: true, json: async () => ({ moisturePct: 42 }) });
  assert.equal((await first).moisturePct, 42);
  const next = context.fetchSetupStatus();
  assert.equal(calls, 2);
  complete({ ok: true, json: async () => ({}) });
  await next;
});

test('Setup cancels a stalled status request and stops polling during navigation', async () => {
  let calls = 0;
  const { context, timers, listeners } = browser((_url, { signal }) => {
    calls++;
    return new Promise((_resolve, reject) => signal.addEventListener('abort', () => reject(Error('aborted'))));
  });
  const first = context.fetchSetupStatus();
  const rejected = assert.rejects(first, /aborted/);
  [...timers.values()][0].fn();
  await rejected;
  assert.equal(timers.size, 0);
  const second = context.fetchSetupStatus();
  const cancelled = assert.rejects(second, /aborted/);
  listeners.pagehide();
  await cancelled;
  assert.equal(await context.fetchSetupStatus(), null);
  assert.equal(calls, 2);
});

test('Setup schedules the next poll only after the current poll finishes', async () => {
  const { context, timers } = browser(async () => ({ ok: true, json: async () => ({}) }));
  let finish;
  context.loadTftStatus = () => new Promise(resolve => { finish = resolve; });
  context.loadMoistureStatus = async () => {};
  const refresh = context.refreshSetupStatus();
  assert.equal(timers.size, 0);
  finish();
  await refresh;
  assert.equal([...timers.values()][0].ms, 5000);
  context.document.hidden = true;
  assert.equal(await context.fetchSetupStatus(), null);
});

test('Changing timezone during an HTTP save configures SNTP without waiting for the network', () => {
  let delays = 0, configurations = 0;
  const { applyTimezoneAndSNTP } = compileFirmwareFunctions(source, ['applyTimezoneAndSNTP'], {
    TZ_IANA: 0, TZ_POSIX: 1, TZ_FIXED: 2, tzMode: 1, tzPosix: 'UTC0',
    time: () => 0, delay: () => delays++, configTzTime: () => configurations++,
  }, { replacements: [
    [/const char\s*\*\s*(\w+)\s*=/g, 'const $1 ='],
    [/char buf\[32\]/g, 'let buf'],
    [/\(long\)/g, ''],
    [/\.c_str\(\)/g, ''],
  ] });
  applyTimezoneAndSNTP(false);
  assert.equal(configurations, 1);
  assert.equal(delays, 0);
  applyTimezoneAndSNTP(true);
  assert.equal(delays, 50, 'startup can still wait for initial time sync');
});
