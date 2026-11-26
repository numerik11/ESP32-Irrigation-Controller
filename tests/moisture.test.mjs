import assert from "node:assert/strict";
import test from "node:test";
import { readFirmware, compileFirmwareFunctions } from "./helpers/firmware-source.mjs";

const source = await readFirmware(
  new URL("../firmware/ESP32-Irrigation/ESP32-Irrigation.ino", import.meta.url),
);

test("Open-Meteo moisture converts volume fractions and rejects invalid values", () => {
  const { meteoSoilPercent } = compileFirmwareFunctions(source, ["meteoSoilPercent"]);
  assert.equal(meteoSoilPercent(0), 0);
  assert.equal(meteoSoilPercent(0.30), 30);
  assert.equal(meteoSoilPercent(0.427), 43);
  assert.equal(meteoSoilPercent(1), 100);
  for (const value of [NaN, Infinity, -0.01, 1.01]) assert.equal(meteoSoilPercent(value), -1);
});

test("Open-Meteo wetness calibration drives the skip threshold without changing probe readings", () => {
  function reading(overrides = {}) {
    return compileFirmwareFunctions(source, ["meteoMoistureVolumePercent", "moisturePercent", "isSoilWetForSmartSkip"], {
      moistureProbeEnabled: true,
      moistureUseMeteo: true,
      meteoMoisturePct: 26,
      meteoMoistureFetchedMs: 1000,
      millis: () => 2000,
      moistureSkipThresholdPct: 60,
      METEO_DRY_VOLUME_PCT: 1,
      METEO_WET_VOLUME_PCT: 36,
      moistureRaw: () => { throw new Error("Meteo must not read ADC"); },
      moistureDryRaw: 3000,
      moistureWetRaw: 1200,
      map: (v, a, b, c, d) => Math.trunc((v - a) * (d - c) / (b - a) + c),
      constrain: (v, a, b) => Math.min(b, Math.max(a, v)),
      ...overrides,
    });
  }
  assert.equal(reading().isSoilWetForSmartSkip(), true);
  assert.equal(reading({ meteoMoisturePct: 25 }).isSoilWetForSmartSkip(), false);
  for (const [volume, wet] of [[1, 0], [8, 5], [18, 25], [23, 50], [36, 100]]) {
    assert.equal(reading({ meteoMoisturePct: volume }).moisturePercent(), wet);
  }
  assert.equal(reading({ meteoMoisturePct: -1, moistureSkipThresholdPct: 0 }).isSoilWetForSmartSkip(), false);
  assert.equal(reading({ moistureProbeEnabled: false }).moisturePercent(), -1);
  assert.equal(reading({ moistureProbeEnabled: false }).isSoilWetForSmartSkip(), false);
  const stale = reading({ millis: () => 1000 + 2 * 60 * 60 * 1000 });
  assert.equal(stale.moisturePercent(), -1);
  assert.equal(stale.isSoilWetForSmartSkip(), false);
  assert.equal(reading({ moistureUseMeteo: false, moistureRaw: () => 2100 }).moisturePercent(), 50);
  assert.equal(reading({ moistureUseMeteo: false, moistureRaw: () => -1 }).moisturePercent(), -1);
});
