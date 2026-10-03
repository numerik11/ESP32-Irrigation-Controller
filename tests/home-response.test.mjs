import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import vm from 'node:vm';
import test from 'node:test';
import { compileFirmwareFunctions } from './helpers/firmware-source.mjs';

const source = (await readFile(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url), 'utf8')).replace(/\r\n/g, '\n');
const bufferSource = source.slice(source.indexOf('class HttpHtmlBuffer'), source.indexOf('void handleRoot() {'));

function writer(allocated = true) {
  const chunks = [];
  let connected = true;
  const functions = compileFirmwareFunctions(bufferSource, ['append', 'flush'], {
    buffer_: allocated ? Buffer.alloc(2048) : null, used_: 0, failed_: false, kCapacity: 2048,
    server_: {
      client: () => ({ connected: () => connected }),
      sendContent(data, size) { chunks.push(Buffer.from(data.subarray(0, size))); },
    },
    memcpy(target, input, size) { target.set(input.subarray(0, size)); },
    delay() {},
  }, { replacements: [
    [/\bconst size_t (\w+) =/g, 'const $1 ='],
    [/\bsize_t (\w+) =/g, 'let $1 ='],
    [/buffer_ \+ used_/g, 'buffer_.subarray(used_)'],
    [/data \+ offset/g, 'data.subarray(offset)'],
  ] });
  return { ...functions, chunks, disconnect() { connected = false; } };
}

test('Home streaming preserves bytes across large scripts, boundaries and explicit flushes', () => {
  const w = writer();
  const parts = [Buffer.alloc(2047, 65), Buffer.from('B'), Buffer.alloc(25300, 67), Buffer.from('rain: 2.3 mm </script></body></html>')];
  parts.forEach((part, index) => { w.append(part, part.length); if (index === 2) w.flush(); });
  w.flush();
  assert.deepEqual(Buffer.concat(w.chunks), Buffer.concat(parts));
  assert.ok(w.chunks.every(chunk => chunk.length > 0 && chunk.length <= 2048));
  const count = w.chunks.length;
  w.flush();
  assert.equal(w.chunks.length, count, 'empty flush must not terminate the response');
});

test('Home streaming stops on disconnect and tolerates allocation failure', () => {
  const w = writer();
  w.disconnect();
  const data = Buffer.alloc(10000);
  w.append(data, data.length);
  w.flush();
  assert.equal(w.chunks.length, 0);
  const failed = writer(false);
  failed.append(data, data.length);
  failed.flush();
  assert.equal(failed.chunks.length, 0);
});

for (const [name, endMarker] of [
  ['handleSetupPage', '// ---------- Schedule POST'],
  ['handleLogPage', 'void handleTankCalibration() {'],
  ['handleDiagnosticsPage', '#if ENABLE_OTA'],
  ['handleOtaUpdatePage', 'static void handleOtaUploadData() {'],
  ['handleScheduleHtml', 'static String compactRunDetailText('],
  ['handleTankCalibration', 'static String _safeReadLine('],
]) {
  test(`${name} static HTML and scripts survive 2 KB streaming boundaries`, () => {
    const start = source.indexOf(`void ${name}() {`);
    const end = source.indexOf(endMarker, start);
    assert.ok(start >= 0 && end > start);
    const body = source.slice(start, end);
    const parts = [...body.matchAll(/html \+= F\(("(?:[^"\\]|\\.)*"|R"(\w+)\(([\s\S]*?)\)\2")\);/g)]
      .map(m => Buffer.from(m[2] ? m[3] : JSON.parse(m[1])));
    assert.ok(parts.length > 0);
    // Start near a boundary so even the small schedule page crosses a chunk.
    parts.unshift(Buffer.alloc(2047, 32));
    const expected = Buffer.concat(parts);
    const w = writer();
    for (const part of parts) w.append(part, part.length);
    w.flush();
    assert.deepEqual(Buffer.concat(w.chunks), expected);
    assert.ok(w.chunks.every(chunk => chunk.length <= 2048));
    assert.match(expected.toString(), /<\/html>/);
  });
}

const root = source.slice(source.indexOf('void handleRoot() {'), source.indexOf('void handleSetupPage() {'));
const scriptSource = root.slice(root.indexOf('// --- JS ---'));
const html = [...scriptSource.matchAll(/html \+= F\(("(?:[^"\\]|\\.)*"|R"(\w+)\(([\s\S]*?)\)\2")\);/g)].map(m => m[2] ? m[3] : JSON.parse(m[1])).join('');
const javascript = html.split('<script>')[1].split('</script>')[0].replace('const ZC=;', 'const ZC=16;');

test('The complete generated Home script parses', () => {
  new vm.Script(javascript);
});

test('Home polling speeds up to 1s while any zone is running', async () => {
  const start = javascript.indexOf('let homeStatusBusy=');
  const polling = javascript.slice(start, javascript.indexOf('const ZC=', start));
  const timers = new Map();
  const context = vm.createContext({
    document: { hidden: false }, AbortController,
    setTimeout(fn, ms) { const id = Symbol('timer'); timers.set(id, { fn, ms }); return id; },
    clearTimeout(key) { timers.delete(key); },
    window: { addEventListener() {} },
    fetch() {
      return Promise.resolve({
        ok: true,
        json: async () => ({ zones: [{ active: false }, { active: true }] }),
      });
    },
  });
  vm.runInContext(polling, context);
  await context.refreshStatus();
  assert.ok([...timers.values()].some(t => t.ms === 1000), 'running zones should use a 1s polling cadence');
});

test('Home polling waits for completion and cancels outstanding requests on navigation', async () => {
  const start = javascript.indexOf('let homeStatusBusy=');
  // The polling block ends immediately before the zone-count declaration.
  const polling = javascript.slice(start, javascript.indexOf('const ZC=', start));
  const timers = new Map(), listeners = {};
  let calls = 0, id = 0, signal;
  const context = vm.createContext({
    document: { hidden: false }, AbortController,
    setTimeout(fn, ms) { timers.set(++id, { fn, ms }); return id; },
    clearTimeout(key) { timers.delete(key); },
    window: { addEventListener(name, fn) { listeners[name] = fn; } },
    fetch(_url, options) {
      calls++; signal = options.signal;
      return new Promise((_resolve, reject) => signal.addEventListener('abort', () => reject(Error('aborted'))));
    },
  });
  vm.runInContext(polling, context);
  await context.refreshStatus();
  assert.equal(calls, 1, 'a second refresh must not overlap the first');
  assert.equal([...timers.values()].filter(t => t.ms === 2000).length, 0);
  [...timers.values()].find(t => t.ms === 8000).fn();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal([...timers.values()].filter(t => t.ms === 2000).length, 1);
  const pending = context.refreshStatus();
  assert.equal(calls, 2);
  listeners.pagehide();
  await pending;
  assert.equal(signal.aborted, true);
  assert.equal(timers.size, 0);
});
