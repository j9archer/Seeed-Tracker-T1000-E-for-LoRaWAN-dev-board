# T1000-E LoRaWAN Uplink Payload Formats

## Overview

The T1000-E tracker sends various payload types depending on the scan mode and sensors enabled. All uplinks use **FPort 2** and include a **Data ID** byte to identify the payload type.

## Payload Types Summary

| Data ID | Name | Min Size | Max Size | Description |
|---------|------|----------|----------|-------------|
| 0x1E | POWER | 1 byte | 1 byte | Power-on message |
| 0x1F | GPS_SEN_ACC_BAT | 13 bytes | ~70 bytes | GNSS + sensors + accelerometer + battery |
| 0x20 | WIFI_SEN_ACC_BAT | 14+ bytes | ~90 bytes | WiFi + sensors + accelerometer + battery |
| 0x21 | BLE_SEN_ACC_BAT | 14+ bytes | ~90 bytes | BLE + sensors + accelerometer + battery |
| 0x22 | GPS_SEN_BAT | 7 bytes | ~64 bytes | GNSS + sensors + battery |
| 0x23 | WIFI_SEN_BAT | 8+ bytes | ~84 bytes | WiFi + sensors + battery |
| 0x24 | BLE_SEN_BAT | 8+ bytes | ~84 bytes | BLE + sensors + battery |
| 0x25 | SEN_ACC_BAT | 13 bytes | 13 bytes | Sensors + accelerometer + battery only |
| 0x26 | SEN_BAT | 7 bytes | 7 bytes | Sensors + battery only |

---

## Common Header Fields

### Sensor Data (7 bytes)
All payloads with "SEN" include these fields after the Data ID:

| Offset | Size | Field | Type | Description |
|--------|------|-------|------|-------------|
| +1 | 1 | Event State | uint8 | Tracker event flags (see Event State Flags below) |
| +2 | 1 | Battery | uint8 | Battery level (0-100%) |
| +3 | 2 | Temperature | int16 | Temperature in 0.1°C (big-endian) |
| +5 | 2 | Light | uint16 | Light sensor value (big-endian) |

**Total: 7 bytes**

#### Event State Flags

The Event State byte is a bitfield containing various tracker status and trigger flags:

| Bit | Hex | Name | Status | Description |
|-----|-----|------|--------|-------------|
| 0 | 0x01 | Reserved | 🔲 Draft | Reserved for future use |
| 1 | 0x02 | Reserved | 🔲 Draft | Reserved for future use |
| 2 | 0x04 | **On Charge** | 🔲 Draft | Device is charging (likely not attached to person) |
| 3 | 0x08 | **Shock Detected** | 🔲 Draft | Shock/impact event detected since last uplink |
| 4 | 0x10 | **Swim Mode** | 🔲 Draft | Special GNSS mode for water operation |
| 5 | 0x20 | **GNSS Ready** | 🔲 Draft | Quick-fix ready (time sync + fresh almanac + recent position) |
| 6 | 0x40 | **SOS Active** | ✅ Implemented | SOS alarm triggered (button press) |
| 7 | 0x80 | **User Triggered** | ✅ Implemented | Manual scan triggered (button press) |

**GNSS Ready Criteria (Bit 5):**
Set when ALL conditions are met:
- Clock synchronization: GPS time error < 3 seconds (DeviceTimeReq valid)
- Fresh almanac: Last GNSS fix or almanac update within 14 days
- Recent position: Valid position (GNSS or vessel assistance) within 4 hours
- Purpose: Indicates capability for rapid TTFF in MOB (Man Overboard) scenarios

**On Charge Indicator (Bit 2):**
- Detects vessel/shore charging
- Enables different operational modes (e.g., almanac maintenance)
- May indicate tracker not worn/attached to person

**Swim Mode (Bit 4):**
- Specialized GNSS acquisition for water surface operation
- May adjust scan parameters for wet antenna conditions
- Possible integration with MOB detection

**Shock Detection (Bit 3):**
- Accelerometer-based impact detection
- Could indicate fall, collision, or MOB event
- Threshold and sensitivity TBD

**TODO - Additional Candidate Flags:**
- Motion state (stationary/moving) - alternative to separate motion events
- Low battery warning (< 20%)
- Temperature alarm (out of safe range)
- Light level change (day/night transition)
- BLE proximity (beacon in range)
- WiFi coverage available

**Current Implementation (v1.0.0-b3):**
- Only bits 6 and 7 are actively used
- Bits 0-5 available for new features
- Event state resets to 0x00 after uplink (except persistent flags like GNSS Ready)

### Accelerometer Data (6 bytes)
Payloads with "ACC" add these fields after sensors:

| Offset | Size | Field | Type | Description |
|--------|------|-------|------|-------------|
| +7 | 2 | Accel X | int16 | X-axis acceleration (big-endian) |
| +9 | 2 | Accel Y | int16 | Y-axis acceleration (big-endian) |
| +11 | 2 | Accel Z | int16 | Z-axis acceleration (big-endian) |

**Total: 6 bytes**

---

## Detailed Payload Structures

### 0x1E: POWER (Power-On Message)

Sent once after device powers on or reboots.

| Byte | Field | Description |
|------|-------|-------------|
| 0 | Data ID | 0x1E |

**Total Size: 1 byte**

---

### 0x22: GPS_SEN_BAT (GNSS + Sensors + Battery)

GNSS scan with sensor data, without accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x22 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | variable | GNSS NAV | LR11xx GNSS NAV message (binary) |

**GNSS NAV Message:**
- Format: LR11xx proprietary binary format
- Destination: GNSS solver (LoRa Cloud, etc.)
- Contains: Satellite pseudoranges, Doppler, timestamps
- Typical size: ~50-60 bytes
- Maximum size: 242 bytes (LoRaWAN max)

**Typical Size: 57-67 bytes**

---

### 0x1F: GPS_SEN_ACC_BAT (GNSS + Sensors + Accelerometer + Battery)

GNSS scan with full sensor suite including accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x1F |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 6 | Accelerometer | X, Y, Z axes |
| 14 | variable | GNSS NAV | LR11xx GNSS NAV message (binary) |

**Typical Size: 63-73 bytes**

---

### 0x23: WIFI_SEN_BAT (WiFi + Sensors + Battery)

WiFi scan results with sensor data, without accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x23 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 1 | AP Count | Number of access points (N) |
| 9 | N × 7 | AP Records | N access point records |

**Access Point Record (7 bytes each):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 6 | MAC Address | WiFi AP MAC address (big-endian) |
| 6 | 1 | RSSI | Signal strength (signed int8, dBm) |

**Example with 3 APs:**
- Header: 9 bytes
- AP data: 3 × 7 = 21 bytes
- **Total: 30 bytes**

**Size Range: 8 bytes (no APs) to ~84 bytes (max ~11 APs)**

---

### 0x20: WIFI_SEN_ACC_BAT (WiFi + Sensors + Accelerometer + Battery)

WiFi scan with full sensor suite including accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x20 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 6 | Accelerometer | X, Y, Z axes |
| 14 | 1 | AP Count | Number of access points (N) |
| 15 | N × 7 | AP Records | N access point records |

**Example with 3 APs:**
- Header: 15 bytes
- AP data: 3 × 7 = 21 bytes
- **Total: 36 bytes**

**Size Range: 14 bytes (no APs) to ~90 bytes (max ~11 APs)**

---

### 0x24: BLE_SEN_BAT (BLE + Sensors + Battery)

BLE beacon scan results with sensor data, without accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x24 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 1 | Beacon Count | Number of BLE beacons (N) |
| 9 | N × 7 | Beacon Records | N beacon records |

**BLE Beacon Record (7 bytes each):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 6 | MAC Address | BLE beacon MAC address (big-endian, reversed) |
| 6 | 1 | RSSI | Signal strength (signed int8, dBm) |

**Note:** MAC addresses are stored in reverse byte order (memcpyr)

**TODO - Optimization:** If using beacons with unique minor IDs, could reduce to:
- 2 bytes: Minor ID (beacon identifier)
- 1 byte: RSSI
- **Savings: 4 bytes per beacon** (3-byte MAC prefix removed)
- Alternative: Send last 3 bytes of MAC + RSSI = 4 bytes per beacon (3 bytes saved)

**Example with 3 beacons:**
- Header: 9 bytes
- Beacon data: 3 × 7 = 21 bytes
- **Total: 30 bytes**

**Size Range: 8 bytes (no beacons) to ~84 bytes (max ~11 beacons)**

---

### 0x21: BLE_SEN_ACC_BAT (BLE + Sensors + Accelerometer + Battery)

BLE beacon scan with full sensor suite including accelerometer.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x21 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 6 | Accelerometer | X, Y, Z axes |
| 14 | 1 | Beacon Count | Number of BLE beacons (N) |
| 15 | N × 7 | Beacon Records | N beacon records |

**Example with 3 beacons:**
- Header: 15 bytes
- Beacon data: 3 × 7 = 21 bytes
- **Total: 36 bytes**

**Size Range: 14 bytes (no beacons) to ~90 bytes (max ~11 beacons)**

---

### 0x25: SEN_ACC_BAT (Sensors + Accelerometer + Battery Only)

Sensor-only uplink when no positioning scan is performed.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x25 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |
| 8 | 6 | Accelerometer | X, Y, Z axes |

**Total Size: 13 bytes**

---

### 0x26: SEN_BAT (Sensors + Battery Only)

Minimal sensor uplink without accelerometer or positioning.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Data ID | 0x26 |
| 1 | 7 | Sensor Data | Event state, battery, temp, light |

**Total Size: 7 bytes**

---

## Scan Configuration Limits

### WiFi Scan
- **Max Results per Scan:** Configured by `wifi_scan_max` (default: 3)
- **Bytes per AP:** 7 bytes (6-byte MAC + 1-byte RSSI)
- **Scan Duration:** Configured by `wifi_scan_duration` (default: 3 seconds)
- **Channels:** 2.4 GHz WiFi channels 1-14 (configurable)
- **Types:** WiFi B/G/N supported

### BLE Scan
- **Max Results per Scan:** Configured by `ble_scan_max` (default: 3)
- **Bytes per Beacon:** 7 bytes (6-byte MAC + 1-byte RSSI) - **TODO: Reduce to 3-4 bytes**
- **Scan Duration:** Configured by `ble_scan_duration` (default: 3 seconds)
- **Filter:** iBeacon format detection
- **Sorting:** Results sorted by RSSI (strongest first)
- **Current Limitation:** Full MAC address sent (many beacons share OUI prefix)
- **TODO:** Make `ble_scan_max` configurable via downlink parameter
  - **Use Case 1:** Single strongest beacon (proximity detection, power saving)
  - **Use Case 2:** 3-5 beacons (basic indoor positioning)
  - **Use Case 3:** 5+ beacons (high-accuracy indoor geolocation)

### GNSS Scan
- **NAV Message Size:** Variable (typically 50-60 bytes)
- **Scan Duration:** Configured by `gnss_scan_duration` (default: 30 seconds)
- **Adaptive Duration:** Up to 750 seconds for almanac maintenance
- **Constellations:** GPS and/or BeiDou
- **Format:** LR11xx binary NAV message for cloud solving

---

## Airtime Estimates (SF7, 125 kHz, EU868)

Based on typical payload sizes:

| Payload Type | Typical Size | Airtime (SF7) | Airtime (SF12) |
|--------------|--------------|---------------|----------------|
| POWER | 1 byte | ~51 ms | ~1.5 s |
| SEN_BAT | 7 bytes | ~62 ms | ~1.8 s |
| SEN_ACC_BAT | 13 bytes | ~72 ms | ~2.1 s |
| WIFI_SEN_BAT (3 APs) | 30 bytes | ~98 ms | ~2.9 s |
| WIFI_SEN_ACC_BAT (3 APs) | 36 bytes | ~108 ms | ~3.2 s |
| BLE_SEN_BAT (3 beacons) | 30 bytes | ~98 ms | ~2.9 s |
| BLE_SEN_ACC_BAT (3 beacons) | 36 bytes | ~108 ms | ~3.2 s |
| GPS_SEN_BAT (typical) | 60 bytes | ~139 ms | ~4.1 s |
| GPS_SEN_ACC_BAT (typical) | 66 bytes | ~149 ms | ~4.4 s |

**Note:** Actual airtime depends on spreading factor (SF), bandwidth, and coding rate. SF7 shown for best case, SF12 for worst case.

---

## Decoder Implementation

A JavaScript decoder (`T1000E_Decoder.js`) is available in the repository root. Key decoder functions:

- `deserialize()`: Main decoding function (switch on Data ID)
- `getMacAndRssiObj()`: Parse WiFi/BLE MAC+RSSI pairs
- `getT1000EUplinkHeaderWithSensor()`: Extract sensor data
- `getT1000EUplinkHeaderWithSensorAnd3Axis()`: Extract sensor + accelerometer

The decoder is compatible with:
- ChirpStack v3/v4
- The Things Network v3
- LoRa Cloud Device & Application Services

---

## Future Optimization Opportunities

To minimize airtime in future versions, consider:

1. **Remove redundant data:**
   - Accelerometer data when not in motion (use Event State motion bit instead)
   - Light sensor if not used
   - Temperature/light if unchanged from previous uplink

2. **Expand Event State utilization:**
   - Use remaining 6 bits (0-5) for status flags
   - Replaces need for separate status bytes
   - Examples: charging, GNSS ready, shock, swim mode, motion, low battery

3. **Compress sensor data:**
   - 8-bit temperature instead of 16-bit (±1°C resolution)
   - Battery in 5% increments (5 bits)
   - Light sensor logarithmic encoding

3. **Optimize positioning payloads:**
   - **BLE beacons:** Use minor ID (2 bytes) instead of full MAC (6 bytes) - saves 4 bytes/beacon
   - **BLE scan count:** Configurable via downlink (1 for proximity, 3-5 for positioning, 5+ for indoor)
   - **WiFi:** Send only last 3 bytes of MAC if same OUI - saves 3 bytes/AP
   - **WiFi scan count:** Similar configurable parameter (1-3 for outdoor, 5+ for indoor)
   - Send only MAC addresses for WiFi/BLE (no RSSI if solver doesn't need it)
   - Delta encoding for nearby APs
   - Compress GNSS NAV using LR11xx solver modes

4. **Context-aware payloads:**
   - GNSS Ready flag allows backend to predict TTFF performance
   - On Charge flag enables shore-side vs. at-sea operational distinctions
   - Swim Mode enables specialized handling for water-based positioning
   - Shock detection for immediate MOB/incident response

5. **Variable payload formats:**
   - Flag-based encoding (only send what changed)
   - Separate frequent/infrequent data streams

**Current payload sizes are reasonable for most use cases.** Typical uplinks (30-70 bytes) fit well within LoRaWAN duty cycle limits and provide good positioning accuracy.

---

## Version History

- **v1.0.0-b3**: Current implementation
  - 9 payload types (0x1E, 0x1F-0x26)
  - WiFi/BLE/GNSS support
  - Vessel assistance downlinks
  - DeviceTimeReq for time sync

---

## Related Documentation

- `VESSEL_ASSISTANCE.md` - Downlink formats for GNSS assistance
- `CHARGING_ALMANAC_MAINTENANCE.md` - Almanac update behavior
- `USB_SERIAL_DEBUG.md` - Logging and diagnostics
- `T1000E_Decoder.js` - JavaScript payload decoder
