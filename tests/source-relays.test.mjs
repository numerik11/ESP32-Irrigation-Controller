import assert from "node:assert/strict";
import test from "node:test";
import { compileFirmwareFunctions, readFirmware } from "./helpers/firmware-source.mjs";

const source = await readFirmware(new URL("../firmware/ESP32-Irrigation/ESP32-Irrigation.ino", import.meta.url));

test("disabled source relays stay off for every request on PCF and GPIO", () => {
  for (const useGpioFallback of [false, true]) {
    for (const mainsPin of [-1, 32]) {
      for (const tankPin of [-1, 33]) {
        for (const mainsOn of [false, true]) {
          for (const tankOn of [false, true]) {
            for (const zoneActive of [false, true]) {
              const levels = new Map();
              const power = [];
              const { setWaterSourceRelays } = compileFirmwareFunctions(source, ["setWaterSourceRelays"], {
                mainsPin, tankPin, useGpioFallback,
                mainsChannel: 4, tankChannel: 5, LOW: 0, HIGH: 1,
                pcfOut: { digitalWrite: (channel, level) => levels.set(channel, level) },
                gpioSourceWrite: (mains, tank) => { levels.set(4, mains ? 0 : 1); levels.set(5, tank ? 0 : 1); },
                gpioPowerSupplyWrite: (on) => power.push(on),
                anyZoneActive: () => zoneActive,
              });
              // First energize all enabled outputs, then ensure OFF is written
              // even when a subsequent request targets a disabled output.
              setWaterSourceRelays(true, true);
              power.length = 0;
              setWaterSourceRelays(mainsOn, tankOn);
              const expectedMains = mainsOn && mainsPin !== -1;
              const expectedTank = tankOn && tankPin !== -1;
              assert.equal(levels.get(4), expectedMains ? 0 : 1);
              assert.equal(levels.get(5), expectedTank ? 0 : 1);
              assert.deepEqual(power, expectedMains || expectedTank ? [true] : zoneActive ? [] : [false]);
            }
          }
        }
      }
    }
  }
});
