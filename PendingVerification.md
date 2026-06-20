# Pending Verification Tracker

If changes are merged without being fully tested, note that in this file.
Include the change and what tests would be sufficient to verify.
ie: `Added support for CRC checking to the PT library, test that crc_ok is true
when PT board connected and that CRC is false when PT board disconnected.`

## Not Yet Verified:
 - All the PT changes - verify all functionality in pressure_sensors/ (Robert)
 - serial_comms import - pulled from ASTRA, we should still test
 - Fallback serial - should see `[Fallback Serial]` prints during begin()
 - All the TC changes - verify all functionality in temperature_sensors/