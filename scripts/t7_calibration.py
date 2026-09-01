"""LabJack T7 pressure transducer calibration data."""

# T7 Channel Configuration: Channel -> (Name, Serial Number, Slope, Offset)
# Slope and Offset are used for raw voltage to pressure conversion: pressure = raw_voltage * slope + offset
CHANNELS = {
    0: ("Valve Upstream", 11448836, 149.9736607, -4.64377129),
    2: ("Venturi Throat", 5589424, 99.28998837, 2.079860712),
    4: ("Venturi Upstream", 11409838, 147.9777035, -3.692041074)
}


def read_pressures(handle, channels=None):
    """Reads raw voltages from LabJack AIN channels and calculates pressures.
    
    Args:
        handle: LabJack device handle
        channels: Optional dict of channels to read. Defaults to CHANNELS.
    
    Returns:
        Dictionary mapping channel number to calculated pressure value
    """
    from labjack import ljm
    
    if channels is None:
        channels = CHANNELS
    
    ain_names = [f"AIN{c}" for c in channels.keys()]
    raw_voltages = ljm.eReadNames(handle, len(channels), ain_names)
    
    pressures = {}
    for channel_id, raw_v, (_, _, slope, offset) in zip(channels.keys(), raw_voltages, channels.values()):
        pressures[channel_id] = raw_v * slope + offset
    return pressures


def read_pressures_and_voltages(handle, channels=None):
    """Reads raw voltages from LabJack AIN channels and calculates pressures.
    
    Args:
        handle: LabJack device handle
        channels: Optional dict of channels to read. Defaults to CHANNELS.
    
    Returns:
        Tuple of (voltages_dict, pressures_dict) where each maps channel number to value
    """
    from labjack import ljm
    
    if channels is None:
        channels = CHANNELS
    
    ain_names = [f"AIN{c}" for c in channels.keys()]
    raw_voltages = ljm.eReadNames(handle, len(channels), ain_names)
    
    voltages = {}
    pressures = {}
    for channel_id, raw_v, (_, _, slope, offset) in zip(channels.keys(), raw_voltages, channels.values()):
        voltages[channel_id] = raw_v
        pressures[channel_id] = raw_v * slope + offset
    return voltages, pressures
