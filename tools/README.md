# Development Tools

This folder contains development tools for ESP32 ESPHome component development and debugging utilities.

## Protocol Debug Collection

### collect-protocol-debug.sh

Automated collection of debug information for new UPS protocol support requests. This tool gathers all necessary data for implementing new UPS protocols.

**Usage:**
```bash
./tools/collect-protocol-debug.sh <ups_name>
```

**Features:**
- Collects USB device information and descriptors
- Captures NUT driver debug output for different states (online, battery, charging)
- Extracts HID paths and report IDs automatically
- Documents UPS variables and supported commands
- Creates organized output directory with all debug files
- Generates markdown-formatted summary ready for GitHub issues

**Requirements:**
- NUT (Network UPS Tools) installed
- UPS configured in `/etc/nut/ups.conf`
- sudo access for USB device access

**Output:**
Creates a timestamped directory containing:
- USB device descriptors and IDs
- HID report captures for each power state
- Complete UPS variable listings
- Summary report for issue creation

### debug-protocol-issue.sh

Automated debug collection for existing protocol implementation issues. Captures real-time data from running ESP32 devices.

**Usage:**
```bash
./tools/debug-protocol-issue.sh <esp32_ip> [ups_name]
```

**Features:**
- Tests ESP32 connectivity and ESPHome API
- Captures real-time sensor values via web API
- Interactive state testing (normal, battery, charging)
- Collects logs during issue reproduction
- Analyzes logs for common problems (USB errors, HID issues, memory)
- Creates structured bug report with all debug data

**Requirements:**
- ESP32 running ESPHome with UPS HID component
- Network access to ESP32 device
- ESPHome CLI tool (optional, for log capture)
- Web API enabled in ESPHome config (recommended)

**Output:**
Creates timestamped directory with:
- JSON sensor states for each test condition
- ESPHome logs (if available)
- Structured bug report ready for GitHub
- Analysis summary with quick diagnostics

## USB Device Management

### scan-usb.sh

Lists ESP32, UPS, and related USB devices connected to the system. Useful for identifying device paths and vendor/product IDs during development.

## NUT Server Testing

### test_nut_server.py

Comprehensive test script for validating the NUT server component functionality. Tests NUT protocol commands, authentication, and concurrent connections.

**Usage:**
```bash
# Basic test with default settings
./test_nut_server.py 192.168.1.200

# Full test with custom parameters
./test_nut_server.py 192.168.1.200 --port 3493 --username nutmon --password nutsecret --ups test_ups

# Run specific test
./test_nut_server.py 192.168.1.200 --test auth

# Test concurrent connections
./test_nut_server.py 192.168.1.200 --test concurrent
```

**Test Categories:**
- `basic` - Test unauthenticated commands (HELP, VERSION, UPSDVER)
- `auth` - Test authenticated commands (LIST, GET VAR)
- `instant` - Test instant commands (beeper control)
- `error` - Test error handling with invalid commands
- `concurrent` - Test multiple simultaneous connections
- `all` - Run all tests (default)

## Protocol Development

Protocols are registered explicitly from the `BUILTIN_PROTOCOLS` table in `ups_hid.cpp`, which `setup()` walks at startup.

### Adding New Vendor-Specific Protocols

1. Create a new protocol class inheriting from `UpsProtocolBase`
2. Implement required methods: `detect()`, `initialize()`, `read_data()`
3. Add a creator function, declared in your protocol header:

   ```cpp
   std::unique_ptr<UpsProtocolBase> create_my_protocol(UpsHidComponent *parent) {
     return std::make_unique<MyProtocol>(parent);
   }
   ```

4. Add an entry to `BUILTIN_PROTOCOLS` in `ups_hid.cpp`:

   ```cpp
   {0x1234, &create_my_protocol, "My Protocol Name", "Description of my protocol", 100},
   ```

Do not register from a static constructor. It runs before `app_main()`, and any logging there panics before ESPHome's logger exists.

### Universal Compatibility

All unknown UPS vendors automatically use the Generic HID protocol as a fallback, providing basic monitoring capabilities without requiring vendor-specific code. This ensures broad compatibility with minimal maintenance.