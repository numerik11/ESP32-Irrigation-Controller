import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import { compileFirmwareFunctions } from './helpers/firmware-source.mjs';

const source = await readFile(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url), 'utf8');

test('Daily selection uses today instead of the new yesterday entry, including date boundaries', () => {
  const { forecastDayIndex } = compileFirmwareFunctions(source, ['forecastDayIndex'], {
    strlen: value => value.length,
    strncmp: (a, b, length) => a.slice(0, length).localeCompare(b.slice(0, length)),
  }, { replacements: [
    [/size_t i =/g, 'let i ='],
    [/const char\* date =/g, 'const date ='],
    [/dates\.size\(\)/g, 'dates.length'],
    [/dates\[i\]\.as<const char\*>\(\)/g, 'dates[i]'],
  ] });
  assert.equal(forecastDayIndex(['2026-09-14', '2026-09-15', '2026-09-16'], '2026-09-15T10:00'), 1);
  assert.equal(forecastDayIndex(['2026-12-31', '2027-01-01'], '2027-01-01T00:00'), 1);
  assert.equal(forecastDayIndex(['2026-09-15'], '2026-09-15T23:00'), 0);
  assert.equal(forecastDayIndex([null, 'invalid'], '2026-09-15T10:00'), -1);
  assert.equal(forecastDayIndex(['2026-09-15'], ''), -1);
  assert.match(source, /forecast_hours=24&past_hours=0&past_days=1/);
});

test('Daylight cards distinguish longer, shorter, unchanged, polar and missing data', () => {
  const script = source.match(/R"DAYLIGHTJS\(([\s\S]*?)\)DAYLIGHTJS"/)[1];
  const elements = { dayLength: {}, daylightChange: {}, daylightChangeHint: {} };
  const update = new Function('document', script + ';return updateDaylightCards;')({ getElementById: id => elements[id] });
  for (const [length, delta, text, change] of [
    [42660, 129.94, '11h 51m', '+2m 10s'],
    [42660, -129.94, '11h 51m', '-2m 10s'],
    [42660, 0.2, '11h 51m', 'No change'],
    [0, 0, '0h 00m', 'No change'],
    [86400, null, '24h 00m', '--'],
    [null, null, '--', '--'],
  ]) {
    update({ daylightSeconds: length, daylightChangeSeconds: delta });
    assert.equal(elements.dayLength.textContent, text);
    assert.equal(elements.daylightChange.textContent, change);
  }
});
