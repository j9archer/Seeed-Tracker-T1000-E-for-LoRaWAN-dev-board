function decodeUplink (input) {
    const bytes = input.bytes || []
    const fPort = parseInt(input.fPort)
    const payload = bytes2HexString(bytes)

    if (fPort === 5 || fPort === 7) {
        return { data: decodeCrewPresenceCompact(bytes, payload, fPort) }
    }
    if (fPort === 6) {
        return { data: decodeCrewAlert(bytes, payload, fPort) }
    }
    if (fPort === 8) {
        return { data: decodeCrewHealthEvent(bytes, payload, fPort) }
    }
    if (fPort === 11) {
        return { data: decodeCrewRfFingerprint(bytes, payload, fPort) }
    }

    return { data: invalid(payload, fPort, `Unsupported RemEX fPort ${fPort}`) }
}

function baseDecoded (payload, fPort) {
    return {
        payload,
        fPort,
        messages: []
    }
}

function invalid (payload, fPort, errMessage) {
    return {
        valid: false,
        err: 1,
        payload,
        fPort,
        errMessage,
        messages: []
    }
}

function decodeCrewPresenceCompact (bytes, payload, fPort) {
    if (bytes.length < 5) {
        return invalid(payload, fPort, 'Compact presence payload too short')
    }

    const schemaPhase = u8(bytes, 0)
    const schemaVersion = schemaVersionOf(schemaPhase)
    if (schemaVersion !== 1) {
        return invalid(payload, fPort, `Unsupported compact presence schema ${schemaVersion}`)
    }

    const sourceFlags = u8(bytes, 4)
    const sourceType = sourceFlags & 0x07
    let offset = 5
    const localSource = {
        type: sourceTypeName(sourceType),
        typeRaw: sourceType,
        accepted: (sourceFlags & 0x08) !== 0,
        rssiPresent: (sourceFlags & 0x10) !== 0,
        identityPresent: (sourceFlags & 0x20) !== 0
    }

    if (sourceType === 1) {
        if (bytes.length !== 10) return invalid(payload, fPort, 'BLE compact presence must be 10 bytes')
        localSource.bleMajor = beaconMajorString(u16le(bytes, offset))
        localSource.bleMinor = u16le(bytes, offset + 2)
        localSource.rssi = s8(bytes, offset + 4)
        offset += 5
    } else if (sourceType === 2 || sourceType === 3) {
        if (bytes.length !== 13) return invalid(payload, fPort, 'WiFi compact presence must be 13 bytes')
        localSource.bssid = macString(bytes, offset)
        localSource.rssi = s8(bytes, offset + 6)
        localSource.wifiFlags = u8(bytes, offset + 7)
        localSource.provisional = sourceType === 3
        offset += 8
    } else if (bytes.length !== 5) {
        return invalid(payload, fPort, 'Compact presence has unexpected extension bytes')
    }

    const decoded = baseDecoded(payload, fPort)
    const presence = {
        schemaVersion,
        phase: phaseOf(schemaPhase),
        phaseName: presencePhaseName(phaseOf(schemaPhase)),
        eventFlags: eventFlagsFromRaw(u8(bytes, 1)),
        battery: s8(bytes, 2),
        locationAgeMin: u8(bytes, 3),
        sourceFlags,
        localSource,
        length: offset
    }
    decoded.messages.push(presenceMeasurements(presence))
    return decoded
}

function decodeCrewHealthEvent (bytes, payload, fPort) {
    if (bytes.length < 1) {
        return invalid(payload, fPort, 'Health/event payload too short')
    }

    const schemaFamily = u8(bytes, 0)
    const schemaVersion = schemaVersionOf(schemaFamily)
    const family = phaseOf(schemaFamily)
    if (schemaVersion !== 1) {
        return invalid(payload, fPort, `Unsupported health/event schema ${schemaVersion}`)
    }

    const decoded = baseDecoded(payload, fPort)
    let healthEvent

    if (family === 6) {
        if (bytes.length !== 7) {
            return invalid(payload, fPort, 'FCntDown sync payload must be 7 bytes')
        }

        const flagsRaw = u8(bytes, 1)
        healthEvent = {
            schemaVersion,
            family,
            familyName: healthEventFamilyName(family),
            flagsRaw,
            flags: {
                syncPending: (flagsRaw & 0x01) !== 0,
                staleDownlinkSeen: (flagsRaw & 0x02) !== 0
            },
            battery: s8(bytes, 2),
            fcntDown: u32le(bytes, 3)
        }
    } else {
        if (bytes.length !== 5) {
            return invalid(payload, fPort, 'Health/event payload must be 5 bytes')
        }

        healthEvent = {
            schemaVersion,
            family,
            familyName: healthEventFamilyName(family),
            eventFlags: eventFlagsFromRaw(u8(bytes, 1)),
            battery: s8(bytes, 2),
            value1: u8(bytes, 3),
            value2: u8(bytes, 4)
        }
    }

    decoded.messages.push(healthMeasurements(healthEvent))
    return decoded
}

function decodeCrewRfFingerprint (bytes, payload, fPort) {
    if (bytes.length < 5) {
        return invalid(payload, fPort, 'RF fingerprint payload too short')
    }

    const schemaPhase = u8(bytes, 0)
    const schemaVersion = schemaVersionOf(schemaPhase)
    if (schemaVersion !== 1) {
        return invalid(payload, fPort, `Unsupported RF fingerprint schema ${schemaVersion}`)
    }

    const recordCount = u8(bytes, 4)
    const records = []
    let offset = 5
    for (let i = 0; i < recordCount; i++) {
        const recordType = u8(bytes, offset)
        if (recordType === 1) {
            if (bytes.length < offset + 6) return invalid(payload, fPort, 'Truncated BLE RF fingerprint record')
            const majorRaw = u16le(bytes, offset + 1)
            records.push({
                type: 'BLE',
                recordType,
                major: beaconMajorString(majorRaw),
                minor: u16le(bytes, offset + 3),
                rssi: s8(bytes, offset + 5)
            })
            offset += 6
        } else if (recordType === 2) {
            if (bytes.length < offset + 9) return invalid(payload, fPort, 'Truncated WiFi RF fingerprint record')
            records.push({
                type: 'WiFi',
                recordType,
                mac: macString(bytes, offset + 1),
                rssi: s8(bytes, offset + 7),
                flags: u8(bytes, offset + 8)
            })
            offset += 9
        } else {
            return invalid(payload, fPort, `Unsupported RF fingerprint record type ${recordType}`)
        }
    }

    if (offset !== bytes.length) {
        return invalid(payload, fPort, 'RF fingerprint has trailing bytes')
    }

    const decoded = baseDecoded(payload, fPort)
    const rfFingerprint = {
        schemaVersion,
        phase: phaseOf(schemaPhase),
        phaseName: presencePhaseName(phaseOf(schemaPhase)),
        eventFlags: eventFlagsFromRaw(u8(bytes, 1)),
        battery: s8(bytes, 2),
        locationAgeMin: u8(bytes, 3),
        recordCount,
        records
    }
    decoded.messages.push(rfFingerprintMeasurements(rfFingerprint))
    return decoded
}

function decodeCrewAlert (bytes, payload, fPort) {
    if (bytes.length < 1) {
        return invalid(payload, fPort, 'Empty alert payload')
    }

    const subtype = u8(bytes, 0)
    if (subtype === 0x20) return decodeMobPosition(bytes, payload, fPort)
    if (subtype === 0x21) return decodeMobCancelled(bytes, payload, fPort)
    if (subtype === 0x22) return decodeMobNoFix(bytes, payload, fPort)
    if (subtype === 0x23) return decodeSosContext(bytes, payload, fPort)
    if (subtype === 0x24) return decodeGnssProof(bytes, payload, fPort)

    return invalid(payload, fPort, `Unsupported alert subtype 0x${subtype.toString(16).padStart(2, '0')}`)
}

function decodeMobPosition (bytes, payload, fPort) {
    if (bytes.length !== 13 && bytes.length !== 16) {
        return invalid(payload, fPort, 'MOB position payload must be 13 or 16 bytes')
    }

    const modeRaw = u8(bytes, 1)
    const qualityFlags = u8(bytes, 11)
    const vectorPresent = bytes.length === 16
    const cogRaw = vectorPresent ? u16le(bytes, 13) : 0xFFFF
    const sogRaw = vectorPresent ? u8(bytes, 15) : 0xFF
    const vectorValid = vectorPresent && (qualityFlags & 0x08) !== 0 && cogRaw !== 0xFFFF && sogRaw !== 0xFF

    const alert = {
        msgType: 0x20,
        msgTypeName: 'MOB position',
        modeRaw,
        mode: modeRaw & 0x7F,
        latitude: i32le(bytes, 2) / 1000000,
        longitude: i32le(bytes, 6) / 1000000,
        hdop: u8(bytes, 10) / 10,
        qualityFlags,
        fixValid: (qualityFlags & 0x01) !== 0,
        qualityOk: (qualityFlags & 0x02) !== 0,
        vectorValid,
        cog: vectorValid ? cogRaw / 2 : null,
        sog: vectorValid ? Math.min(sogRaw, 0xFE) / 10 : null,
        onCharge: ((modeRaw & 0x80) !== 0) || ((qualityFlags & 0x04) !== 0),
        battery: s8(bytes, 12)
    }
    return alertDecoded(payload, fPort, alert)
}

function decodeMobCancelled (bytes, payload, fPort) {
    if (bytes.length !== 4 && bytes.length !== 5) {
        return invalid(payload, fPort, 'MOB cancelled payload must be 4 or 5 bytes')
    }

    const flags = bytes.length === 5 ? u8(bytes, 4) : 0
    return alertDecoded(payload, fPort, {
        msgType: 0x21,
        msgTypeName: 'MOB cancelled',
        elapsedS: u16be(bytes, 1),
        flags,
        onCharge: (flags & 0x01) !== 0,
        battery: s8(bytes, 3),
        approvedBeaconSeen: true
    })
}

function decodeMobNoFix (bytes, payload, fPort) {
    if (bytes.length !== 5) {
        return invalid(payload, fPort, 'MOB no-fix payload must be 5 bytes')
    }

    const modeRaw = u8(bytes, 1)
    return alertDecoded(payload, fPort, {
        msgType: 0x22,
        msgTypeName: 'MOB no fix',
        modeRaw,
        mode: modeRaw & 0x7F,
        elapsedS: u16be(bytes, 2),
        battery: s8(bytes, 4),
        onCharge: (modeRaw & 0x80) !== 0,
        gpsValid: false
    })
}

function decodeSosContext (bytes, payload, fPort) {
    if (bytes.length < 7) {
        return invalid(payload, fPort, 'SOS context payload too short')
    }

    const schemaPhase = u8(bytes, 1)
    const schemaVersion = schemaVersionOf(schemaPhase)
    if (schemaVersion !== 1) {
        return invalid(payload, fPort, `Unsupported SOS context schema ${schemaVersion}`)
    }

    const eventRaw = u8(bytes, 2) | 0x40
    const sourceFlags = u8(bytes, 4)
    const evidenceFlags = u8(bytes, 5)
    const sourceType = sourceFlags & 0x07
    const localSource = {
        type: sourceTypeName(sourceType),
        typeRaw: sourceType,
        accepted: ((sourceFlags | evidenceFlags) & 0x08) !== 0,
        rssiPresent: (sourceFlags & 0x10) !== 0,
        identityPresent: (sourceFlags & 0x20) !== 0
    }

    let offset = 7
    if (sourceType === 1 && localSource.identityPresent) {
        if (bytes.length < offset + 5) return invalid(payload, fPort, 'Truncated SOS BLE evidence')
        localSource.bleMajor = beaconMajorString(u16le(bytes, offset))
        localSource.bleMinor = u16le(bytes, offset + 2)
        localSource.rssi = s8(bytes, offset + 4)
        offset += 5
    } else if ((sourceType === 2 || sourceType === 3) && localSource.identityPresent) {
        if (bytes.length < offset + 8) return invalid(payload, fPort, 'Truncated SOS WiFi evidence')
        localSource.bssid = macString(bytes, offset)
        localSource.wifiFlags = u8(bytes, offset + 6)
        localSource.rssi = s8(bytes, offset + 7)
        localSource.provisional = sourceType === 3
        offset += 8
    }

    const gnssAttempted = (evidenceFlags & 0x04) !== 0
    const gnssFixValid = (evidenceFlags & 0x10) !== 0
    const noFixYet = ((evidenceFlags & 0x40) !== 0) || (gnssAttempted && !gnssFixValid)
    const alert = {
        msgType: 0x23,
        msgTypeName: noFixYet ? 'SOS no fix yet' : 'SOS context',
        schemaVersion,
        phase: phaseOf(schemaPhase),
        phaseName: presencePhaseName(phaseOf(schemaPhase)),
        eventFlags: eventFlagsFromRaw(eventRaw),
        battery: s8(bytes, 3),
        sourceFlags,
        localSource,
        evidenceFlags,
        bleAttempted: (evidenceFlags & 0x01) !== 0,
        wifiAttempted: (evidenceFlags & 0x02) !== 0,
        gnssAttempted,
        localSourceAccepted: localSource.accepted,
        gnssFixValid,
        gnssQualityOk: (evidenceFlags & 0x20) !== 0,
        noFixYet,
        uncertain: noFixYet && !localSource.accepted,
        mobConfirmed: false,
        gpsValid: gnssFixValid,
        onCharge: (eventRaw & 0x04) !== 0,
        reason: sosContextReason(u8(bytes, 6))
    }

    if (gnssFixValid) {
        if (bytes.length < offset + 10) return invalid(payload, fPort, 'Truncated SOS GNSS evidence')
        alert.latitude = i32le(bytes, offset) / 1000000
        alert.longitude = i32le(bytes, offset + 4) / 1000000
        alert.hdop = u8(bytes, offset + 8) / 10
        alert.sats = u8(bytes, offset + 9)
        offset += 10
    }

    if (offset !== bytes.length) {
        return invalid(payload, fPort, 'SOS context has trailing bytes')
    }

    return alertDecoded(payload, fPort, alert)
}

function decodeGnssProof (bytes, payload, fPort) {
    if (bytes.length !== 16) {
        return invalid(payload, fPort, 'GNSS proof payload must be 16 bytes')
    }

    const schemaPhase = u8(bytes, 1)
    const schemaVersion = schemaVersionOf(schemaPhase)
    if (schemaVersion !== 1) {
        return invalid(payload, fPort, `Unsupported GNSS proof schema ${schemaVersion}`)
    }

    const qualityFlags = u8(bytes, 5)
    const alert = {
        msgType: 0x24,
        msgTypeName: 'GNSS vessel proof',
        schemaVersion,
        phase: phaseOf(schemaPhase),
        phaseName: presencePhaseName(phaseOf(schemaPhase)),
        eventFlags: eventFlagsFromRaw(u8(bytes, 2)),
        battery: s8(bytes, 3),
        locationAgeMin: u8(bytes, 4),
        qualityFlags,
        fixValid: (qualityFlags & 0x01) !== 0,
        qualityOk: (qualityFlags & 0x02) !== 0,
        onCharge: (qualityFlags & 0x04) !== 0,
        hdop: u8(bytes, 6) / 10,
        sats: u8(bytes, 7),
        latitude: i32le(bytes, 8) / 1000000,
        longitude: i32le(bytes, 12) / 1000000,
        source: 'GNSS'
    }
    return alertDecoded(payload, fPort, alert)
}

function alertDecoded (payload, fPort, alert) {
    const decoded = baseDecoded(payload, fPort)
    decoded.messages.push(alertMeasurements(alert))
    return decoded
}

function commonMeasurements (body) {
    const measurements = [
        measurement('4200', 'Event Status', body.eventFlags),
        measurement('3000', 'Battery', body.battery)
    ]

    if (body.locationAgeMin !== undefined) {
        measurements.push(measurement('6001', 'Location Age', body.locationAgeMin))
    }
    return measurements
}

function presenceMeasurements (presence) {
    const measurements = commonMeasurements(presence)

    if (presence.localSource.typeRaw === 1 && presence.localSource.bleMajor !== undefined) {
        measurements.push(measurement('5002', 'BLE Scan', [{
            major: presence.localSource.bleMajor,
            minor: presence.localSource.bleMinor,
            rssi: presence.localSource.rssi
        }]))
    }

    if ((presence.localSource.typeRaw === 2 || presence.localSource.typeRaw === 3) && presence.localSource.bssid !== undefined) {
        measurements.push(measurement('5001', 'Wi-Fi Scan', [{
            mac: presence.localSource.bssid,
            rssi: presence.localSource.rssi,
            flags: presence.localSource.wifiFlags,
            provisional: presence.localSource.provisional
        }]))
    }
    return measurements
}

function healthMeasurements (healthEvent) {
    const measurements = [
        measurement('3000', 'Battery', healthEvent.battery),
        measurement('6003', 'Crew Health Event', healthEvent)
    ]

    if (healthEvent.eventFlags !== undefined) {
        measurements.unshift(measurement('4200', 'Event Status', healthEvent.eventFlags))
    }

    return measurements
}

function rfFingerprintMeasurements (fingerprint) {
    const measurements = commonMeasurements(fingerprint)
    measurements.push(measurement('6004', 'RF Fingerprint', fingerprint))

    const bleRecords = fingerprint.records.filter((record) => record.type === 'BLE')
    if (bleRecords.length > 0) {
        measurements.push(measurement('5002', 'BLE Scan', bleRecords))
    }

    const wifiRecords = fingerprint.records.filter((record) => record.type === 'WiFi')
    if (wifiRecords.length > 0) {
        measurements.push(measurement('5001', 'Wi-Fi Scan', wifiRecords))
    }
    return measurements
}

function alertMeasurements (alert) {
    const measurements = []
    if (alert.eventFlags) {
        measurements.push(measurement('4200', 'Event Status', alert.eventFlags))
    }

    measurements.push(measurement('6000', 'Crew Alert', alert))
    measurements.push(measurement('3000', 'Battery', alert.battery))

    if (alert.latitude !== undefined && alert.longitude !== undefined) {
        measurements.push(measurement('4198', 'GNSS Latitude', alert.latitude))
        measurements.push(measurement('4197', 'GNSS Longitude', alert.longitude))
    }
    return measurements
}

function measurement (measurementId, type, measurementValue) {
    return {
        measurementId,
        type,
        measurementValue
    }
}

function eventFlagsFromRaw (value) {
    return {
        raw: value,
        onCharge: (value & 0x04) !== 0,
        shockDetected: (value & 0x08) !== 0,
        swimMode: (value & 0x10) !== 0,
        gnssReady: (value & 0x20) !== 0,
        sosActive: (value & 0x40) !== 0,
        userTriggered: (value & 0x80) !== 0
    }
}

function sourceTypeName (value) {
    switch (value) {
        case 1: return 'BLE'
        case 2: return 'WiFi fixed'
        case 3: return 'WiFi provisional'
        case 4: return 'GNSS'
        default: return 'none'
    }
}

function presencePhaseName (phase) {
    switch (phase) {
        case 0: return 'routine'
        case 1: return 'onboard'
        case 2: return 'uncertain'
        case 3: return 'MOB'
        case 4: return 'PIW'
        case 5: return 'SOS'
        default: return `reserved ${phase}`
    }
}

function healthEventFamilyName (family) {
    switch (family) {
        case 1: return 'battery health'
        case 2: return 'low battery'
        case 3: return 'shock'
        case 4: return 'light'
        case 5: return 'temperature'
        case 6: return 'FCntDown sync'
        default: return `reserved ${family}`
    }
}

function sosContextReason (value) {
    switch (value) {
        case 1: return 'no approved BLE'
        case 2: return 'WiFi provisional rejected'
        case 3: return 'GNSS unavailable'
        case 4: return 'GNSS timeout'
        case 5: return 'GNSS no fix yet'
        default: return value === 0 ? 'none' : `reserved ${value}`
    }
}

function schemaVersionOf (schemaPhase) {
    return (schemaPhase >> 4) & 0x0F
}

function phaseOf (schemaPhase) {
    return schemaPhase & 0x0F
}

function macString (bytes, offset) {
    return bytes.slice(offset, offset + 6)
        .map((b) => (b & 0xFF).toString(16).padStart(2, '0').toUpperCase())
        .join(':')
}

function beaconMajorString (value) {
    return String(value).padStart(5, '0')
}

function bytes2HexString (bytes) {
    return bytes.map((b) => (b & 0xFF).toString(16).padStart(2, '0')).join('').toUpperCase()
}

function u8 (bytes, offset) {
    return bytes[offset] & 0xFF
}

function s8 (bytes, offset) {
    const value = u8(bytes, offset)
    return value > 127 ? value - 256 : value
}

function u16le (bytes, offset) {
    return u8(bytes, offset) | (u8(bytes, offset + 1) << 8)
}

function u16be (bytes, offset) {
    return (u8(bytes, offset) << 8) | u8(bytes, offset + 1)
}

function i32le (bytes, offset) {
    const value = (u8(bytes, offset) |
        (u8(bytes, offset + 1) << 8) |
        (u8(bytes, offset + 2) << 16) |
        (u8(bytes, offset + 3) << 24))
    return value | 0
}

function u32le (bytes, offset) {
    return (u8(bytes, offset) |
        (u8(bytes, offset + 1) << 8) |
        (u8(bytes, offset + 2) << 16) |
        (u8(bytes, offset + 3) << 24)) >>> 0
}

if (typeof module !== 'undefined') {
    module.exports = { decodeUplink }
}
