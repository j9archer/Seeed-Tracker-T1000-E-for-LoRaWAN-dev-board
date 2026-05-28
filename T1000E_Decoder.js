function decodeUplink (input) {
    const bytes = input['bytes']
    const fport = parseInt(input['fPort'])
    const bytesString = bytes2HexString(bytes)
    const originMessage = bytesString.toLocaleUpperCase()
    const decoded = {
        valid: true,
        err: 0,
        fPort: fport,
        payload: bytesString,
        messages: []
    }
    if (fport === 199 || fport === 192) {
        decoded.messages.push({fport: fport, payload: bytesString})
        return { data: decoded }
    }
    if (fport === 6) {
        const alertDecoded = decodeCrewAlertUplink(bytes, bytesString, fport)
        if (alertDecoded.valid) {
            return { data: alertDecoded }
        }
    }
    let measurement = messageAnalyzed(originMessage)
    if (measurement.length === 0) {
        decoded.valid = false
        return { data: decoded }
    }

    for (let message of measurement) {
        if (message.length === 0) {
            continue
        }
        let elements = []
        for (let element of message) {
            if (element.errorCode) {
                decoded.err = element.errorCode
                decoded.errMessage = element.error
            } else {
                elements.push(element)
            }
        }
        if (elements.length > 0) {
            decoded.messages.push(elements)
        }
    }
    if (fport === 7) {
        annotateChargerPortMeasurements(decoded.messages)
    }
    // decoded.messages = measurement
    return { data: decoded }
}

function annotateChargerPortMeasurements (messages) {
    for (const message of messages) {
        if (!Array.isArray(message)) {
            continue
        }
        for (const element of message) {
            if (element.type !== 'Event Status') {
                continue
            }
            const eventFlags = element.value || element.measurementValue || { raw: 0 }
            if (typeof eventFlags.raw !== 'number') {
                eventFlags.raw = 0
            }
            eventFlags.onCharge = true
            eventFlags.raw |= 0x04
            element.value = eventFlags
            element.measurementValue = eventFlags
            if (element.motionId !== undefined) {
                element.motionId = eventFlags.raw
            }
        }
    }
}

function decodeCrewAlertUplink (bytes, bytesString, fport) {
    const nowMs = Date.now()
    const timestampSec = Math.floor(nowMs / 1000)
    const dataId = u8(bytes, 0)
    const decoded = {
        valid: true,
        err: 0,
        payload: bytesString,
        firmwareType: 'custom',
        fPort: fport,
        alertPort: true,
        messages: []
    }

    if (dataId === 0x20 && bytes.length === 13) {
        const modeRaw = u8(bytes, 1)
        const qualityFlags = u8(bytes, 11)
        const latitude = i32le(bytes, 2) / 1000000
        const longitude = i32le(bytes, 6) / 1000000
        decoded.mob = {
            msgType: 0x04,
            msgTypeName: 'MOB position',
            modeRaw,
            mode: modeRaw & 0x7F,
            latitude,
            longitude,
            hdop: u8(bytes, 10) / 10,
            qualityFlags,
            fixValid: (qualityFlags & 0x01) !== 0,
            qualityOk: (qualityFlags & 0x02) !== 0,
            onCharge: ((modeRaw & 0x80) !== 0) || ((qualityFlags & 0x04) !== 0),
            battery: s8(bytes, 12)
        }
        decoded.messages.push(crewAlertMeasurements(decoded.mob, timestampSec, nowMs))
        return decoded
    }

    if (dataId === 0x21 && (bytes.length === 4 || bytes.length === 5)) {
        const flags = bytes.length === 5 ? u8(bytes, 4) : 0
        decoded.mob = {
            msgType: 0x01,
            msgTypeName: 'MOB cancelled',
            elapsedS: u16be(bytes, 1),
            flags,
            onCharge: (flags & 0x01) !== 0,
            battery: s8(bytes, 3),
            approvedBeaconSeen: true
        }
        decoded.messages.push(crewAlertMeasurements(decoded.mob, timestampSec, nowMs))
        return decoded
    }

    if (dataId === 0x22 && bytes.length === 5) {
        const modeRaw = u8(bytes, 1)
        decoded.mob = {
            msgType: 0x02,
            msgTypeName: 'MOB no fix',
            modeRaw,
            mode: modeRaw & 0x7F,
            elapsedS: u16be(bytes, 2),
            battery: s8(bytes, 4),
            onCharge: (modeRaw & 0x80) !== 0,
            gpsValid: false
        }
        decoded.messages.push(crewAlertMeasurements(decoded.mob, timestampSec, nowMs))
        return decoded
    }

    decoded.valid = false
    decoded.err = 1
    decoded.errMessage = `Unsupported crew alert payload on fPort ${fport}`
    return decoded
}

function crewAlertMeasurements (mob, collectTime, timestamp) {
    const measurements = [
        {
            measurementId: '6000',
            type: 'Crew Alert',
            measureTime: collectTime,
            timestamp: timestamp,
            value: mob,
            measurementValue: mob
        },
        {
            measurementId: '3000',
            type: 'Battery',
            measureTime: collectTime,
            timestamp: timestamp,
            value: mob.battery,
            measurementValue: mob.battery
        }
    ]

    if (mob.latitude !== undefined && mob.longitude !== undefined) {
        measurements.push({
            measurementId: '4198',
            type: 'GNSS Latitude',
            measureTime: collectTime,
            timestamp: timestamp,
            value: mob.latitude,
            measurementValue: mob.latitude
        })
        measurements.push({
            measurementId: '4197',
            type: 'GNSS Longitude',
            measureTime: collectTime,
            timestamp: timestamp,
            value: mob.longitude,
            measurementValue: mob.longitude
        })
    }

    return measurements
}

function messageAnalyzed (messageValue) {
    try {
        let frames = unpack(messageValue)
        let measurementResultArray = []
        for (let i = 0; i < frames.length; i++) {
            let item = frames[i]
            let dataId = item.dataId
            let dataValue = item.dataValue
            let measurementArray = deserialize(dataId, dataValue)
            measurementResultArray.push(measurementArray)
        }
        return measurementResultArray
    } catch (e) {
        return e.toString()
    }
}

function unpack (messageValue) {
    let frameArray = []

    for (let i = 0; i < messageValue.length; i++) {
        let remainMessage = messageValue
        let dataId = remainMessage.substring(0, 2).toUpperCase()
        let dataValue
        let dataObj = {}
        let packageLen
        switch (dataId) {
            case '1E':
                packageLen = 26
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '1F':
                packageLen = 42
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '20':
                scanCount = parseInt(remainMessage.substring(26, 28), 16)
                packageLen = (21 + (scanCount - 1) * 7) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '21':
                scanCount = parseInt(remainMessage.substring(26, 28), 16)
                packageLen = (21 + (scanCount - 1) * 7) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '22':
                packageLen = 30
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    // Some firmwares can send a short 0x22 frame with only
                    // event/battery/temp (5 bytes total including dataId).
                    // Decode what is available instead of dropping the frame.
                    if (remainMessage.length >= 10) {
                        packageLen = remainMessage.length
                    } else {
                        return frameArray
                    }
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '23':
                scanCount = parseInt(remainMessage.substring(14, 16), 16)
                packageLen = (15 + (scanCount - 1) * 7) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '24':
                scanCount = parseInt(remainMessage.substring(14, 16), 16)
                packageLen = (15 + (scanCount - 1) * 7) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '25':
                packageLen = 26
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '26':
                packageLen = 14
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '27':
                scanCount = parseInt(remainMessage.substring(26, 28), 16)
                // Custom BLE+accel frame: ID/header/count is 14 bytes, each iBeacon record is 5 bytes.
                packageLen = (14 + (scanCount * 5)) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            case '28':
                scanCount = parseInt(remainMessage.substring(14, 16), 16)
                // Custom BLE frame: ID/header/count is 8 bytes, each iBeacon record is 5 bytes.
                packageLen = (8 + (scanCount * 5)) * 2
                if (remainMessage.length === packageLen + 2) {
                    packageLen += 2
                }
                if (remainMessage.length < packageLen) {
                    return frameArray
                }
                dataValue = remainMessage.substring(2, packageLen)
                messageValue = remainMessage.substring(packageLen)
                dataObj = {
                    'dataId': dataId, 'dataValue': dataValue
                }
                break
            default:
                return frameArray
        }
        if (dataValue.length < 2) {
            break
        }
        frameArray.push(dataObj)
    }
    return frameArray
}

function deserialize (dataId, dataValue) {
    let measurementArray = []
    let collectTime = 0
    // Use both seconds (legacy measureTime) and milliseconds (new timestamp)
    const nowMs = Date.now()
    const timestamp = nowMs // ms
    const timestampSec = Math.floor(nowMs / 1000)
    switch (dataId) {
        case '1E':
            measurementArray = [
                {
                    measurementId: '3000', type: 'Battery', measurementValue: this.getBattery(dataValue.substring(0, 2))
                }, {
                    measurementId: '3502', type: 'Firmware Version', measurementValue: this.getSoftVersion(dataValue.substring(2, 6))
                }, {
                    measurementId: '3001', type: 'Hardware Version', measurementValue: this.getHardVersion(dataValue.substring(6, 10))
                }, {
                    measurementId: '3965', type: 'Positioning Strategy', measurementValue: this.getPositioningStrategy(dataValue.substring(10, 12))
                }, {
                    measurementId: '3900', type: 'Uplink Interval', measurementValue: this.getMinsByMin(dataValue.substring(12, 16))},
                {
                    measurementId: '3974', type: 'Accelerometer Enable', measurementValue: this.getInt(dataValue.substring(16, 18))
                }, {
                    measurementId: '3941', type: 'SOS Mode', measurementValue: this.getSOSMode(dataValue.substring(18, 20))
                }, {
                    measurementId: '3972', type: 'WI-FI Scan Limitation', measurementValue: this.getInt(dataValue.substring(20, 22))
                }, {
                    measurementId: '3973', type: 'Beacon Scan Limitation', measurementValue: this.getInt(dataValue.substring(22, 24))
                }
            ]
            break
        case '1F':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensorAnd3Axis(dataValue, collectTime, 31, timestamp)
            measurementArray.push({
                measurementId: '4197',
                measureTime: collectTime,
                timestamp: timestamp,
                type: 'GNSS Latitude',
                value: this.getSensorValue(dataValue.substring(24, 32), 1000000),
                measurementValue: this.getSensorValue(dataValue.substring(24, 32), 1000000),
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            measurementArray.push({
                measurementId: '4198',
                measureTime: collectTime,
                timestamp: timestamp,
                type: 'GNSS Longitude',
                value: this.getSensorValue(dataValue.substring(32, 40), 1000000),
                measurementValue: this.getSensorValue(dataValue.substring(32, 40), 1000000),
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, 40, collectTime, timestamp)
            break
        case '20':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensorAnd3Axis(dataValue, collectTime, 32, timestamp)
            scanMax = this.getInt(dataValue.substring(24, 26))
            const wifiAccExpectedLen = 26 + (scanMax * 14)
            const wifiAccScanEnd = locationAgeFromTail(dataValue, wifiAccExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            measurementArray.push({
                measurementId: '5001',
                type: 'Wi-Fi Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                value: this.getMacAndRssiObj(dataValue.substring(26, wifiAccScanEnd)),
                measurementValue: this.getMacAndRssiObj(dataValue.substring(26, wifiAccScanEnd)),
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, wifiAccExpectedLen, collectTime, timestamp)
            break
        case '21':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensorAnd3Axis(dataValue, collectTime, 33, timestamp)
            scanMax = this.getInt(dataValue.substring(24, 26))
            const bleAccExpectedLen = 26 + (scanMax * 14)
            const bleAccScanEnd = locationAgeFromTail(dataValue, bleAccExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            const stockBleAccScan = this.getMacAndRssiObj(dataValue.substring(26, bleAccScanEnd))
            measurementArray.push({
                measurementId: '5002',
                type: 'BLE Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                value: stockBleAccScan,
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, bleAccExpectedLen, collectTime, timestamp)
            break
        case '22':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, 34, timestamp)
            if (dataValue.length >= 20) {
                const latValue = this.getSensorValue(dataValue.substring(12, 20), 1000000)
                measurementArray.push({
                    measurementId: '4197',
                    measureTime: collectTime,
                    timestamp: timestamp,
                    type: 'GNSS Latitude',
                    value: latValue,
                    measurementValue: latValue,
                    motionId: this.getMotionId(dataValue.substring(0, 2))
                })
            }
            if (dataValue.length >= 28) {
                const lonValue = this.getSensorValue(dataValue.substring(20, 28), 1000000)
                measurementArray.push({
                    measurementId: '4198',
                    measureTime: collectTime,
                    timestamp: timestamp,
                    type: 'GNSS Longitude',
                    value: lonValue,
                    measurementValue: lonValue,
                    motionId: this.getMotionId(dataValue.substring(0, 2))
                })
            }
            addLocationAgeMeasurement(measurementArray, dataValue, 28, collectTime, timestamp)
            break
        case '23':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, 35, timestamp)
            scanMax = this.getInt(dataValue.substring(12, 14))
            const wifiExpectedLen = 14 + (scanMax * 14)
            const wifiScanEnd = locationAgeFromTail(dataValue, wifiExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            measurementArray.push({
                measurementId: '5001',
                type: 'Wi-Fi Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                value: this.getMacAndRssiObj(dataValue.substring(14, wifiScanEnd)),
                measurementValue: this.getMacAndRssiObj(dataValue.substring(14, wifiScanEnd)),
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, wifiExpectedLen, collectTime, timestamp)
            break
        case '24':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, 36, timestamp)
            scanMax = this.getInt(dataValue.substring(12, 14))
            const bleExpectedLen = 14 + (scanMax * 14)
            const bleScanEnd = locationAgeFromTail(dataValue, bleExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            const stockBleScan = this.getMacAndRssiObj(dataValue.substring(14, bleScanEnd))
            measurementArray.push({
                measurementId: '5002',
                type: 'BLE Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                value: stockBleScan,
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, bleExpectedLen, collectTime, timestamp)
            break
        case '25':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensorAnd3Axis(dataValue, collectTime, 37, timestamp)
            addLocationAgeMeasurement(measurementArray, dataValue, 24, collectTime, timestamp)
            break
        case '26':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, 38, timestamp)
            addLocationAgeMeasurement(measurementArray, dataValue, 12, collectTime, timestamp)
            break
        case '27':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensorAnd3Axis(dataValue, collectTime, 39, timestamp)
            scanMax = this.getInt(dataValue.substring(24, 26))
            const customBleAccExpectedLen = 26 + (scanMax * 10)
            const customBleAccScanEnd = locationAgeFromTail(dataValue, customBleAccExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            const customBleAccScan = this.getIBeaconAndRssiObj(dataValue.substring(26, customBleAccScanEnd))
            measurementArray.push({
                measurementId: '5002',
                type: 'BLE Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                measurementValue: customBleAccScan,
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, customBleAccExpectedLen, collectTime, timestamp)
            break
        case '28':
            collectTime = timestampSec
            measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, 40, timestamp)
            scanMax = this.getInt(dataValue.substring(12, 14))
            const customBleExpectedLen = 14 + (scanMax * 10)
            const customBleScanEnd = locationAgeFromTail(dataValue, customBleExpectedLen) === null ? dataValue.length : dataValue.length - 2
            if (!scanMax || scanMax === 0) {
                break
            }
            const customBleScan = this.getIBeaconAndRssiObj(dataValue.substring(14, customBleScanEnd))
            measurementArray.push({
                measurementId: '5002',
                type: 'BLE Scan',
                measureTime: collectTime,
                timestamp: timestamp,
                measurementValue: customBleScan,
                motionId: this.getMotionId(dataValue.substring(0, 2))
            })
            addLocationAgeMeasurement(measurementArray, dataValue, customBleExpectedLen, collectTime, timestamp)
            break
    }
    return measurementArray
}

function getMotionId (str) {
    return getInt(str)
}

function getPositingStatus (str) {
    let status = getInt(str)
    switch (status) {
        case 0:
            return {id:status, statusName:"Positioning successful."}
        case 1:
            return {id:status, statusName:"The GNSS scan timed out and failed to obtain the location."}
        case 2:
            return {id:status, statusName:"The Wi-Fi scan timed out and failed to obtain the location."}
        case 3:
            return {id:status, statusName:"The Wi-Fi + GNSS scan timed out and failed to obtain the location."}
        case 4:
            return {id:status, statusName:"The GNSS + Wi-Fi scan timed out and failed to obtain the location."}
        case 5:
            return {id:status, statusName:"The Bluetooth scan timed out and failed to obtain the location."}
        case 6:
            return {id:status, statusName:"The Bluetooth + Wi-Fi scan timed out and failed to obtain the location."}
        case 7:
            return {id:status, statusName:"The Bluetooth + GNSS scan timed out and failed to obtain the location."}
        case 8:
            return {id:status, statusName:"The Bluetooth + Wi-Fi + GNSS scan timed out and failed to obtain the location."}
        case 9:
            return {id:status, statusName:"Location Server failed to parse the GNSS location."}
        case 10:
            return {id:status, statusName:"Location Server failed to parse the Wi-Fi location."}
        case 11:
            return {id:status, statusName:"Location Server failed to parse the Bluetooth location."}
        case 12:
            return {id:status, statusName:"Failed to parse the GNSS location due to the poor accuracy."}
        case 13:
            return {id:status, statusName:"Time synchronization failed."}
        case 14:
            return {id:status, statusName:"Failed to obtain location due to the old Almanac."}
    }
    return getInt(str)
}

function getUpShortInfo (messageValue) {
    return [
        {
            measurementId: '3000', type: 'Battery', measurementValue: getBattery(messageValue.substring(0, 2))
        }, {
            measurementId: '3502', type: 'Firmware Version', measurementValue: getSoftVersion(messageValue.substring(2, 6))
        }, {
            measurementId: '3001', type: 'Hardware Version', measurementValue: getHardVersion(messageValue.substring(6, 10))
        }, {
            measurementId: '3940', type: 'Work Mode', measurementValue: getWorkingMode(messageValue.substring(10, 12))
        }, {
            measurementId: '3965', type: 'Positioning Strategy', measurementValue: getPositioningStrategy(messageValue.substring(12, 14))
        }, {
            measurementId: '3942', type: 'Heartbeat Interval', measurementValue: getMinsByMin(messageValue.substring(14, 18))
        }, {
            measurementId: '3943', type: 'Periodic Interval', measurementValue: getMinsByMin(messageValue.substring(18, 22))
        }, {
            measurementId: '3944', type: 'Event Interval', measurementValue: getMinsByMin(messageValue.substring(22, 26))
        }, {
            measurementId: '3945', type: 'Sensor Enable', measurementValue: getInt(messageValue.substring(26, 28))
        }, {
            measurementId: '3941', type: 'SOS Mode', measurementValue: getSOSMode(messageValue.substring(28, 30))
        }
    ]
}

function getMotionSetting (str) {
    return [
        {measurementId: '3946', type: 'Motion Enable', measurementValue: getInt(str.substring(0, 2))},
        {measurementId: '3947', type: 'Any Motion Threshold', measurementValue: getSensorValue(str.substring(2, 6), 1)},
        {measurementId: '3948', type: 'Motion Start Interval', measurementValue: getMinsByMin(str.substring(6, 10))},
    ]
}

function getStaticSetting (str) {
    return [
        {measurementId: '3949', type: 'Static Enable', measurementValue: getInt(str.substring(0, 2))},
        {measurementId: '3950', type: 'Device Static Timeout', measurementValue: getMinsByMin(str.substring(2, 6))}
    ]
}

function getShockSetting (str) {
    return [
        {measurementId: '3951', type: 'Shock Enable', measurementValue: getInt(str.substring(0, 2))},
        {measurementId: '3952', type: 'Shock Threshold', measurementValue: getInt(str.substring(2, 6))}
    ]
}

function getTempSetting (str) {
    return [
        {measurementId: '3953', type: 'Temp Enable', measurementValue: getInt(str.substring(0, 2))},
        {measurementId: '3954', type: 'Event Temp Interval', measurementValue: getMinsByMin(str.substring(2, 6))},
        {measurementId: '3955', type: 'Event Temp Sample Interval', measurementValue: getSecondsByInt(str.substring(6, 10))},
        {measurementId: '3956', type: 'Temp ThMax', measurementValue: getSensorValue(str.substring(10, 14), 10)},
        {measurementId: '3957', type: 'Temp ThMin', measurementValue: getSensorValue(str.substring(14, 18), 10)},
        {measurementId: '3958', type: 'Temp Warning Type', measurementValue: getInt(str.substring(18, 20))}
    ]
}

function getLightSetting (str) {
    return [
        {measurementId: '3959', type: 'Light Enable', measurementValue: getInt(str.substring(0, 2))},
        {measurementId: '3960', type: 'Event Light Interval', measurementValue: getMinsByMin(str.substring(2, 6))},
        {measurementId: '3961', type: 'Event Light Sample Interval', measurementValue: getSecondsByInt(str.substring(6, 10))},
        {measurementId: '3962', type: 'Light ThMax', measurementValue: getSensorValue(str.substring(10, 14), 10)},
        {measurementId: '3963', type: 'Light ThMin', measurementValue: getSensorValue(str.substring(14, 18), 10)},
        {measurementId: '3964', type: 'Light Warning Type', measurementValue: getInt(str.substring(18, 20))}
    ]
}

function getShardFlag (str) {
    let bitStr = getByteArray(str)
    return {
        count: parseInt(bitStr.substring(0, 4), 2),
        index: parseInt(bitStr.substring(4), 2)
    }
}

function getBattery (batteryStr) {
    return loraWANV2DataFormat(batteryStr)
}

function locationAgeFromTail (dataValue, expectedNoAgeHexLen) {
    if (dataValue.length === expectedNoAgeHexLen + 2) {
        return getInt(dataValue.substring(expectedNoAgeHexLen, expectedNoAgeHexLen + 2))
    }
    return null
}

function addLocationAgeMeasurement (measurementArray, dataValue, expectedNoAgeHexLen, collectTime, timestamp) {
    const locationAgeMin = locationAgeFromTail(dataValue, expectedNoAgeHexLen)
    if (locationAgeMin === null) {
        return
    }
    measurementArray.push({
        measurementId: '6001',
        type: 'Location Age',
        measureTime: collectTime,
        timestamp: timestamp,
        value: locationAgeMin,
        measurementValue: locationAgeMin
    })
}

function u8 (bytes, offset) {
    return bytes[offset] & 0xFF
}

function s8 (bytes, offset) {
    const value = u8(bytes, offset)
    return value > 127 ? value - 256 : value
}

function u16be (bytes, offset) {
    return (u8(bytes, offset) << 8) | u8(bytes, offset + 1)
}

function i32le (bytes, offset) {
    return (u8(bytes, offset) |
        (u8(bytes, offset + 1) << 8) |
        (u8(bytes, offset + 2) << 16) |
        (u8(bytes, offset + 3) << 24))
}

function getSoftVersion (softVersion) {
    return `${loraWANV2DataFormat(softVersion.substring(0, 2))}.${loraWANV2DataFormat(softVersion.substring(2, 4))}`
}
function getHardVersion (hardVersion) {
    return `${loraWANV2DataFormat(hardVersion.substring(0, 2))}.${loraWANV2DataFormat(hardVersion.substring(2, 4))}`
}

function getSecondsByInt (str) {
    return getInt(str)
}

function getMinsByMin (str) {
    return getInt(str)
}

function getSensorValue (str, dig) {
    if (str === '8000') {
        return null
    } else {
        return loraWANV2DataFormat(str, dig)
    }
}

function bytes2HexString (arrBytes) {
    var str = ''
    for (var i = 0; i < arrBytes.length; i++) {
        var tmp
        var num = arrBytes[i]
        if (num < 0) {
            tmp = (255 + num + 1).toString(16)
        } else {
            tmp = num.toString(16)
        }
        if (tmp.length === 1) {
            tmp = '0' + tmp
        }
        str += tmp
    }
    return str
}
function loraWANV2DataFormat (str, divisor = 1) {
    let strReverse = bigEndianTransform(str)
    let str2 = toBinary(strReverse)
    if (str2.substring(0, 1) === '1') {
        let arr = str2.split('')
        let reverseArr = arr.map((item) => {
            if (parseInt(item) === 1) {
                return 0
            } else {
                return 1
            }
        })
        str2 = parseInt(reverseArr.join(''), 2) + 1
        return parseFloat('-' + str2 / divisor)
    }
    return parseInt(str2, 2) / divisor
}

function bigEndianTransform (data) {
    let dataArray = []
    for (let i = 0; i < data.length; i += 2) {
        dataArray.push(data.substring(i, i + 2))
    }
    return dataArray
}

function toBinary (arr) {
    let binaryData = arr.map((item) => {
        let data = parseInt(item, 16)
            .toString(2)
        let dataLength = data.length
        if (data.length !== 8) {
            for (let i = 0; i < 8 - dataLength; i++) {
                data = `0` + data
            }
        }
        return data
    })
    return binaryData.toString().replace(/,/g, '')
}

function getSOSMode (str) {
    return loraWANV2DataFormat(str)
}

function getMacAndRssiObj (pair) {
    let pairs = []
    if (pair.length % 14 === 0) {
        for (let i = 0; i < pair.length; i += 14) {
            let mac = getMacAddress(pair.substring(i, i + 12))
            if (mac) {
                let rssi = getInt8RSSI(pair.substring(i + 12, i + 14))
                pairs.push({mac: mac, rssi: rssi})
            } else {
                continue
            }
        }
    }
    return pairs
}

function getIBeaconAndRssiObj (pair) {
    let pairs = []
    // Custom RemEX firmware uses 5-byte BLE records:
    // major(2) identifies the vessel beacon, minor(2) is the proposed DR, rssi(1) is signed dBm.
    if (pair.length % 10 === 0) {
        for (let i = 0; i < pair.length; i += 10) {
            // Keep the vessel BLE tag ID as a fixed-width decimal string.
            // RemEX Node-RED vessel matching uses this ID directly, so 1234 must
            // decode as "01234" rather than numeric 1234.
            let major = String(getInt(pair.substring(i, i + 4))).padStart(5, '0')
            let minor = getInt(pair.substring(i + 4, i + 8))
            let rssi = getInt8RSSI(pair.substring(i + 8, i + 10))
            pairs.push({major: major, minor: minor, rssi: rssi})
        }
    }
    return pairs
}

function getMacAddress (str) {
    if (str.toLowerCase() === 'ffffffffffff') {
        return null
    }
    let macArr = []
    for (let i = 1; i < str.length; i++) {
        if (i % 2 === 1) {
            macArr.push(str.substring(i - 1, i + 1))
        }
    }
    let mac = ''
    for (let i = 0; i < macArr.length; i++) {
        mac = mac + macArr[i]
        if (i < macArr.length - 1) {
            mac = mac + ':'
        }
    }
    return mac
}

function getInt8RSSI (str) {
    return loraWANV2DataFormat(str)
}

function getInt (str) {
    return parseInt(str, 16)
}

function getEventStatus (str) {
    let bitStr = getByteArray(str)
    let bitArr = []
    for (let i = 0; i < bitStr.length; i++) {
        bitArr[i] = bitStr.substring(i, i + 1)
    }
    bitArr = bitArr.reverse()
    let event = []
    for (let i = 0; i < bitArr.length; i++) {
        if (bitArr[i] !== '1') {
            continue
        }
        switch (i){
            case 0:
                event.push({id:1, eventName:"Reserved bit 0."})
                break
            case 1:
                event.push({id:2, eventName:"Reserved bit 1."})
                break
            case 2:
                event.push({id:3, eventName:"On charge."})
                break
            case 3:
                event.push({id:4, eventName:"Shock event."})
                break
            case 4:
                event.push({id:5, eventName:"Swim mode."})
                break
            case 5:
                event.push({id:6, eventName:"GNSS ready."})
                break
            case 6:
                event.push({id:7, eventName:"SOS event."})
                break
            case 7:
                event.push({id:8, eventName:"Press once event."})
                break
        }
    }
    return event
}

function getEventFlags (str) {
    const value = getInt(str)
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

function getByteArray (str) {
    let bytes = []
    for (let i = 0; i < str.length; i += 2) {
        bytes.push(str.substring(i, i + 2))
    }
    return toBinary(bytes)
}

function getWorkingMode (workingMode) {
    return getInt(workingMode)
}

function getPositioningStrategy (strategy) {
    return getInt(strategy)
}

function getUTCTimestamp(str){
    return parseInt(loraWANV2PositiveDataFormat(str)) * 1000
}

function loraWANV2PositiveDataFormat (str, divisor = 1) {
    let strReverse = bigEndianTransform(str)
    let str2 = toBinary(strReverse)
    return parseInt(str2, 2) / divisor
}

function getT1000EUplinkHeaderWithSensorAnd3Axis (dataValue, collectTime, dataId, timestamp) {
    let measurementArray = this.getT1000EUplinkHeaderWithSensor(dataValue, collectTime, dataId, timestamp)
    let value = this.getSignSensorValue(dataValue.substring(12, 16), 1)
    if (value !== null) {
        measurementArray.push({
            measurementId: '4210',
            type: 'Accel X',
            measureTime: collectTime,
            timestamp: timestamp,
            value: value,
            measurementValue: value,
            motionId: this.getMotionId(dataValue.substring(0, 2))
        })
    }
    value = this.getSignSensorValue(dataValue.substring(16, 20), 1)
    if (value !== null) {
        measurementArray.push({
            measurementId: '4211',
            type: 'Accel Y',
            measureTime: collectTime,
            timestamp: timestamp,
            value: value,
            measurementValue: value,
            motionId: this.getMotionId(dataValue.substring(0, 2))
        })
    }
    value = this.getSignSensorValue(dataValue.substring(20, 24), 1)
    if (value !== null) {
        measurementArray.push({
            measurementId: '4212',
            type: 'Accel Z',
            measureTime: collectTime,
            timestamp: timestamp,
            value: value,
            measurementValue: value,
            motionId: this.getMotionId(dataValue.substring(0, 2))
        })
    }
    return measurementArray
}

function getT1000EUplinkHeaderWithSensor (dataValue, collectTime, dataId, timestamp) {
    let measurementArray = this.getT1000EUplinkHeader(dataValue, collectTime, dataId, timestamp)
    const motionId = this.getMotionId(dataValue.substring(0, 2))
    if (dataValue.length >= 8) {
        const tempVal = this.getSensorValue(dataValue.substring(4, 8), 10)
        measurementArray.push({
            measurementId: '4097',
            type: 'Air Temperature',
            measureTime: collectTime,
            timestamp: timestamp,
            value: tempVal,
            measurementValue: tempVal,
            motionId
        })
    }
    if (dataValue.length >= 12) {
        const lightVal = this.getSensorValue(dataValue.substring(8, 12))
        measurementArray.push({
            measurementId: '4199',
            type: 'Light',
            measureTime: collectTime,
            timestamp: timestamp,
            value: lightVal,
            measurementValue: lightVal,
            motionId
        })
    }
    return measurementArray
}

function getT1000EUplinkHeader (dataValue, collectTime, dataId, timestamp) {
    const eventStateHex = dataValue.substring(0, 2)
    let eventFlags = this.getEventFlags(eventStateHex)
    let measurementArray = []
    const motionId = this.getMotionId(eventStateHex)
    // Event Status (use 4200 as measurementId per target schema)
    measurementArray.push({
        measurementId: '4200',
        type: 'Event Status',
        measureTime: collectTime,
        timestamp: timestamp,
        value: eventFlags,
        measurementValue: eventFlags,
        motionId
    })
    // Battery
    if (dataValue.length >= 4) {
        const batt = this.getBattery(dataValue.substring(2, 4))
        measurementArray.push({
            measurementId: '3000',
            type: 'Battery',
            measureTime: collectTime,
            timestamp: timestamp,
            value: batt,
            measurementValue: batt,
            motionId
        })
    }
    return measurementArray
}

function getSignSensorValue (str, dig = 1) {
    if (this.isNull(str)) {
        return null
    }
    return this.loraWANV2DataFormat(str, dig)
}
function isNull (str) {
    if (str.substring(0, 1) !== '8') {
        return false
    }
    for (let i = 1; i < str.length; i++) {
        if (str.substring(i, i + 1) !== '0') {
            return false
        }
    }
    return true
}
