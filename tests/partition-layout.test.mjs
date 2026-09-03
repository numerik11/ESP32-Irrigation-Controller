import assert from "node:assert/strict";
import { readFile, stat } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const testDirectory = path.dirname(fileURLToPath(import.meta.url));
const repositoryRoot = path.resolve(testDirectory, "..");

function parsePartitionTable(buffer) {
  const entries = [];
  for (let offset = 0; offset + 32 <= buffer.length; offset += 32) {
    if (buffer.readUInt16LE(offset) !== 0x50aa) break;
    entries.push({
      type: buffer[offset + 2],
      subtype: buffer[offset + 3],
      offset: buffer.readUInt32LE(offset + 4),
      size: buffer.readUInt32LE(offset + 8),
      label: buffer.subarray(offset + 12, offset + 28).toString("ascii").replace(/\0.*$/, ""),
    });
  }
  return entries;
}

for (const target of ["esp32-dev", "esp32-s3-devkitc-1"]) {
  test(`${target} web-flasher image has two OTA slots and enough room`, async () => {
    const targetDirectory = path.join(repositoryRoot, "web-flasher", target);
    const table = parsePartitionTable(await readFile(path.join(targetDirectory, "partitions.bin")));
    const otaData = table.find((entry) => entry.type === 0x01 && entry.subtype === 0x00);
    const ota0 = table.find((entry) => entry.type === 0x00 && entry.subtype === 0x10);
    const ota1 = table.find((entry) => entry.type === 0x00 && entry.subtype === 0x11);

    assert.ok(otaData, "otadata partition is required");
    assert.ok(ota0, "ota_0 application slot is required");
    assert.ok(ota1, "ota_1 application slot is required");
    assert.equal(ota0.size, ota1.size, "OTA application slots must be equally sized");
    assert.ok(ota0.offset + ota0.size <= ota1.offset, "OTA application slots must not overlap");

    const firmware = await stat(path.join(targetDirectory, "firmware.bin"));
    assert.ok(
      firmware.size <= ota0.size,
      `firmware is ${firmware.size} bytes but each OTA slot is only ${ota0.size} bytes`,
    );
  });
}
