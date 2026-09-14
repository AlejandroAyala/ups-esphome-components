# CLAUDE.md

Guidance for working in this repository. Read the "Hard-won gotchas" section before
touching component code — several of the traps here cost hours to diagnose and are
invisible in a normal build.

## What this is

ESPHome external components for monitoring USB UPS devices from an ESP32-S3 acting
as a USB host.

| Component | Purpose |
|---|---|
| `components/ups_hid/` | USB communication + protocol implementations + sensor platforms |
| `components/nut_server/` | NUT v1.3 TCP server on port 3493, exposes UPS data to `upsc`/`upsmon` |
| `components/ups_status_led/` | Status LED driven by UPS state |

`configs/` holds reusable YAML packages (sensor bundles, device-type presets,
regional defaults) that user configs `!include`.

## Architecture

Two abstractions, both Strategy pattern, both resolved at runtime:

**Transport** (`transport_interface.h` → `transport_esp32.cpp`, `transport_simulation.cpp`)
wraps USB. Exposes HID report access (control transfers), raw interrupt endpoint
access, and string descriptors. `transport_factory.cpp` picks hardware vs simulation.

**Protocol** (`UpsProtocolBase` in `ups_hid.h`) parses a specific UPS dialect into
`UpsData`. Registered per USB vendor ID in `ProtocolFactory`, with fallbacks.

```
UpsHidComponent (PollingComponent)
  ├── IUsbTransport         ← Esp32UsbTransport | SimulatedTransport
  └── UpsProtocolBase       ← Apc | CyberPower | Megatec | GenericHid
```

`UpsData` (`data_composite.h`) aggregates `BatteryData`, `PowerData`, `DeviceInfo`,
`TestStatus`, `ConfigData`. Unset numeric fields are `NAN`; the sensor publish loop
skips NaN, so a missing value means "no state published" rather than zero.

### Protocols

| Protocol | Vendor | Transport mechanism |
|---|---|---|
| APC HID | `0x051D` | HID Power Device reports |
| CyberPower HID | `0x0764` | Vendor-specific HID reports |
| **Megatec Q1** | `0x0925` | ASCII `Q1`/`F`/`I` over **interrupt endpoints** (not HID reports) |
| Generic HID | fallback | Standard HID-PDC report probing |

Megatec is the odd one out: the UPS exposes a HID-class interface that merely
*tunnels* the classic Megatec serial command set behind Richcomm "armac" framing.
It mirrors NUT's `nutdrv_qx` driver with the `armac` USB subdriver. See the header
comment in `protocol_megatec.h` for wire format and the reference captures.

## Hard-won gotchas

### 1. No work in static constructors — especially no logging

Protocols used to self-register through a `REGISTER_UPS_PROTOCOL_FOR_VENDOR`
macro: a static object whose constructor called `ProtocolFactory`. It failed
both ways:

- While nothing referenced a protocol's translation unit, the linker dropped the
  archive member and the constructor never ran: `Registered Protocols: 0`.
- Once `ups_hid.cpp` referenced the creators, the constructors ran in
  `do_global_ctors`, **before `app_main()`**. `ProtocolFactory` logs, ESPHome's
  logger did not exist yet, and the chip panicked (`LoadProhibited` in
  `Logger::level_for()`). That boot loop made `safe_mode` roll back every build
  from 008e1f1 to rev8 — so it looked like the new firmware was never flashed.

The macros are gone. Built-in protocols are registered explicitly from the
`BUILTIN_PROTOCOLS` table in `ups_hid.cpp`, called from `setup()`. **Add new
protocols to that table.** Keep constructors of any global or static object
trivial: no logging, no ESPHome calls.

Crashes before `app_main()` never reach the network, so over-the-air logs show
only the rolled-back firmware. A USB serial capture (ESPHome Web Serial) of the
first boot after an OTA is what shows them.

### 2. `setup()` runs before WiFi — its logs are unreachable over the network

`get_setup_priority()` returns `setup_priority::DATA` (600); WiFi is 250, and
higher priority runs *first*. Anything logged in `setup()` is emitted before the
API exists and can never reach `esphome logs`. No log level changes this.

Consequences:
- Put diagnostics in `dump_config()`, which ESPHome re-emits to every connecting
  log client.
- Never do risky work (USB bring-up) in `setup()`. A crash there kills the device
  before the API is up, and resetting inside the `safe_mode` window makes the
  bootloader **roll the firmware back**, erasing the evidence.
- Deferring to the first `update()` is **not enough**: that still runs within
  seconds of boot, well inside the `safe_mode` window (`boot_is_good_after`,
  60s). rev7 crashed there and was rolled back (`OTA rollback detected!` in
  `dump_config`, old `compiled on` date). Bring-up now waits for
  `timing::USB_BRINGUP_MIN_UPTIME_MS` of uptime so a fault crashes into the new
  firmware with its logs visible.

### 3. ESPHome log level ranking puts CONFIG *above* INFO

Order: `NONE, ERROR, WARN, INFO, CONFIG, DEBUG, VERBOSE, VERY_VERBOSE`. A tag
pinned to `INFO` hides all `ESP_LOGCONFIG` **and** `ESP_LOGD` from that tag —
including the whole of `dump_config()`. The global `logger.level` is a
compile-time ceiling: a per-tag `DEBUG` does nothing unless the global level is
`DEBUG` too.

### 4. Freeing a timed-out USB transfer is a use-after-free

`usb_host_transfer_free()` on a transfer still queued on an endpoint leaves the
USB stack holding a dangling `callback` and a `context` pointing at a dead stack
frame. It crashes later, when the device next sends data — far from the cause.

On timeout you must abort the endpoint first: `usb_host_endpoint_halt` →
`flush` → `clear`, which completes the transfer and runs its callback, then wait
for that callback before freeing. See `Esp32UsbTransport::abort_queued_transfer()`.
If the callback still does not fire, **leak the transfer** rather than free it.

This matters most on interrupt IN endpoints, where a timeout is the *normal* case
rather than an error.

> Still outstanding: `hid_get_report`, `hid_set_report`, `get_string_descriptor`
> and `submit_control_transfer` have the same free-on-timeout flaw for control
> transfers. Rare in practice, but real.

### 5. `read_data()` runs on the ESPHome main loop

Keep total transaction time bounded. The Megatec timeouts
(`megatec::FIRST_READ_TIMEOUT_MS` etc. in `constants_ups.h`) are deliberately
tight — hardware answers in ~250ms and normally sends the whole reply in one
64-byte packet.

### 6. Bump `component::VERSION` on every change

In `constants_ups.h`, printed by `dump_config()` as `Component Version: revN`.
This is the only reliable way to tell a current firmware from a rolled-back one in
a captured log. **Bump it whenever component sources change.**

### 7. `mark_failed()` takes a `LogString *`

Use `mark_failed(LOG_STR("reason"))`, never a plain `const char *`. The reason
appears in the `[E][component] ups_hid is marked FAILED: <reason>` line on every
connect, so always supply one.

### 8. ESP32-S3 USB PHY is shared

The internal USB PHY serves either USB-Serial/JTAG or USB-OTG. The USB Host driver
takes it over when it installs, so a `USB_SERIAL_JTAG` console goes quiet at that
point. Logs over WiFi are unaffected. Which physical port the UPS is plugged into
matters on dual-port boards.

## Verifying changes without hardware

ESPHome is not necessarily installed locally, and the ESP-IDF toolchain is slow.
The fastest useful loop is compiling component sources natively against **stub
ESPHome/ESP-IDF headers** in a scratch directory:

```bash
g++ -std=gnu++17 -fsyntax-only -Wall -DUSE_ESP32 \
    -include cstdint -include cstring -include string -include algorithm \
    -I<stubs> -Icomponents/ups_hid components/ups_hid/<file>.cpp
```

Stubs needed: `esp_err.h`, `esphome/core/{component,log,helpers,defines,application}.h`,
`usb/{usb_host,usb_types_ch9}.h`, `freertos/{FreeRTOS,semphr,task,portmacro}.h`,
`driver/gpio.h`. Note:

- `-DUSE_ESP32` is required — `protocol_apc.cpp` does not compile without it.
- The preincludes matter: several headers rely on `<cstdint>`/`<string>` arriving
  transitively from real ESPHome headers.
- **Make stubs match the real signatures.** A too-permissive stub
  (`mark_failed(const char *)`) let a real compile error through. Add a negative
  control when a stub models an API you care about.

This approach also supports real behavioural tests: link the protocol against a
fake transport and drive it with captured device bytes. That is how the Megatec
parsers were validated (59 checks against a real `nutdrv_qx -DDDDD` capture), and
how explicit registration was proven to work. A stub compile cannot catch
startup-order faults such as logging from a static constructor.

`tests/` holds pytest config-validation tests that require the `esphome` CLI.

## Adding a protocol

1. Study `protocol_megatec.cpp` (serial-over-HID) or `protocol_cyberpower.cpp`
   (HID reports).
2. Inherit `UpsProtocolBase`; implement `detect()`, `initialize()`, `read_data()`.
3. Add a `create_<name>_protocol()` free function, declared in the header.
4. Add an entry to `BUILTIN_PROTOCOLS` in `ups_hid.cpp` — see gotcha 1.
5. Add the enum value to `DeviceInfo::DetectedProtocol` and its
   `get_protocol_name()` switch.
6. Add the name to the `cv.one_of` protocol list in `__init__.py`, and the vendor
   ID to `KNOWN_VENDOR_IDS`.
7. Constants go in `constants_ups.h` under their own namespace — no magic numbers.
8. Provide simulation support in `transport_simulation.cpp` so the protocol is
   testable with `simulation_mode: true`.
9. Add a `configs/device_types/<device>.yaml` preset and update both READMEs.

If the protocol receives explicit status bits (on-battery, fault, low battery),
set the `status_flags_valid` + `flag_*` fields on `PowerData`/`BatteryData` rather
than relying on values inferred from voltages. Flags only ever *add* to the
inferred state, so they cannot mask a problem the readings reveal.

## Code style

- No C++ exceptions (ESPHome disables them).
- Mutexes for shared data. `update_sensors()` already holds `data_mutex_`, so it
  cannot call the public state getters — they take the same lock. Shared logic
  belongs on the data structs (see `PowerData::is_on_battery()`).
- Named constants in `constants_ups.h` / `constants_hid.h`.
- Reference NUT source files when implementing protocol quirks.
- Comment the non-obvious only — no narration of what the code plainly does.

## Reference material

- NUT source is the authority for protocol behaviour:
  `https://github.com/networkupstools/nut/blob/master/drivers/nutdrv_qx.c`
- To characterise an unsupported UPS on Linux:
  `sudo /usr/lib/nut/nutdrv_qx -DDDDD -a <ups>` (driver path varies; NUT drivers
  live in a libexec dir, not `PATH`). `tools/collect-protocol-debug.sh` hardcodes
  `usbhid-ups`, so it is not useful for `nutdrv_qx` devices as-is.
- Debug logs may not be UTF-8 — `grep` needs `-a`.
