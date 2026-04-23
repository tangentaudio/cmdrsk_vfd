# Commander SK trip code lookup table.
# Source: Commander SK Advanced User Guide, Issue 10, Table 10-17.
# Mirrors cmdrsk_vfd.h sk_trip_table for use in Python GUI code.

TRIP_CODES = {
    1:   ("UU",    "DC bus under-voltage"),
    2:   ("OU",    "DC bus over-voltage"),
    3:   ("OI.AC", "AC instantaneous over-current"),
    4:   ("OI.br", "Braking resistor instantaneous over-current"),
    6:   ("Et",    "External trip"),
    7:   ("O.SPd", "Overspeed"),
    18:  ("tunE",  "Auto-tune stopped before completion"),
    19:  ("It.br", "I²t on braking resistor"),
    20:  ("It.AC", "I²t on drive output current"),
    21:  ("O.ht1", "Drive over-heat (IGBT thermal model)"),
    22:  ("O.ht2", "Drive over-heat (heatsink temperature)"),
    24:  ("th",    "Motor thermistor trip"),
    26:  ("O.Ld1", "+24V or digital output overload"),
    27:  ("O.ht3", "Drive over-heat (DC bus thermal model)"),
    28:  ("cL1",   "Analog input 1 current loss"),
    30:  ("SCL",   "Serial comms timeout (external keypad)"),
    31:  ("EEF",   "Internal EEPROM failure (load defaults to clear)"),
    32:  ("PH",    "Input phase loss or high imbalance"),
    33:  ("rS",    "Stator resistance measurement failure"),
    35:  ("CL.bt", "Trip initiated from control word bit 12"),
    100: ("reset", "Drive reset command acknowledged"),
    102: ("O.ht4", "Power module rectifier over temperature"),
    182: ("C.Err", "SmartStick data error"),
    183: ("C.dAt", "SmartStick data does not exist"),
    185: ("C.Acc", "SmartStick read/write fail"),
    186: ("C.rtg", "SmartStick rating mismatch"),
    189: ("O.cL",  "Overload on current loop analog input"),
    199: ("dESt",  "Destination parameter clash"),
    200: ("SL.HF", "Solutions Module hardware fault"),
    201: ("SL.tO", "Solutions Module watchdog timeout"),
    202: ("SL.Er", "Solutions Module error (see Pr 15.50)"),
    203: ("SL.nF", "Solutions Module not installed"),
    204: ("SL.dF", "Solutions Module different type installed"),
}


def trip_name(code):
    """Return short display name for a trip code."""
    if 40 <= code <= 89:
        return "t{:03d}".format(code)
    if 220 <= code <= 232:
        return "HF{:d}".format(code - 200)
    entry = TRIP_CODES.get(code)
    if entry:
        return entry[0]
    return "?(code {:d})".format(code)


def trip_description(code):
    """Return human-readable description for a trip code."""
    if 40 <= code <= 89:
        return "User-defined PLC ladder trip"
    if 220 <= code <= 232:
        return "Fatal hardware fault — power cycle required"
    entry = TRIP_CODES.get(code)
    if entry:
        return entry[1]
    return "Unknown trip code"
