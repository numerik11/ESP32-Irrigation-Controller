# Unit tests

Run the complete host-side suite from the repository root:

```sh
npm test
```

The suite has no third-party dependencies and uses Node's built-in test runner
(Node 20 or newer).  It covers pure logic extracted directly from both Arduino
sketches plus the Cloudflare update counter's request validation, CORS, routing,
and idempotent storage behavior. It also checks that every web-flasher image has
two non-overlapping OTA application slots and that its firmware fits each slot.

Hardware drivers, GPIO electrical behavior, Wi-Fi connectivity, filesystem
persistence, display rendering, and live weather requests still require an
ESP32/ESP8266 integration test on the target hardware.

The ESP32 source-contract tests also verify that Browser OTA requires credentials,
uses the application OTA partition, stops active zones before writing, and only
restarts after a complete validated upload. They do not replace a real upload test
on each supported board.
