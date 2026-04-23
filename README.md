# cmdrsk_vfd

LinuxCNC userspace HAL component to control an **Emerson / Control Techniques
Commander SK** VFD via RS-485 Modbus RTU.

Written by Steve Richardson (steve@tangentaudio.com) — December 2013  
Adapted from Michael Haberler's `vfs11_vfd.c`, itself adapted from Steve Padnos'
`gs2_vfd.c`.

---

## Prerequisites

### Software

```
sudo apt install libmodbus-dev
```

For a RIP (Run In Place) LinuxCNC build, the headers and libraries are taken
from `~/devel/linuxcnc` automatically.  For a system-installed LinuxCNC:

```
sudo apt install linuxcnc-uspace-dev    # or linuxcnc-dev depending on distro
```

### Commander SK drive configuration

The following parameters must be set on the drive before connecting:

| Parameter | Setting | Description |
|-----------|---------|-------------|
| Pr 11.24  | `Modbus RTU` | Serial protocol selection |
| Pr 11.25  | `19200` (or your baud) | Baud rate |
| Pr 11.23  | `1` (or desired node address) | Modbus slave address |
| Pr 6.43   | `On(1)` | Control word enable |

The driver uses the **precision reference** (Pr 1.14 = 5, Pr 1.18) for speed
control and the **control word** (Pr 6.42) for run/stop/direction.

---

## Build

### RIP build (default — uses `~/devel/linuxcnc`)

```bash
make
```

Override the RIP path if needed:

```bash
make LCNC_HOME=/path/to/linuxcnc
```

### System-installed LinuxCNC build

```bash
make INSTALLED=1
```

### Install

```bash
sudo make install          # installs to /usr/bin/cmdrsk_vfd
make install PREFIX=~/bin  # or a custom location
```

> **Note:** `Submakefile` is the legacy in-tree build file from when this
> driver lived inside the LinuxCNC source tree.  It is retained for reference
> but is superseded by `Makefile`.

---

## Configuration

Copy `cmdrsk.ini` to your machine configuration directory and add a `[CMDRSK]`
section (or include it from your main machine INI):

```ini
[CMDRSK]
DEVICE=/dev/ttyUSB0    # RS-485 adapter device
BAUD=19200
BITS=8
PARITY=none
STOPBITS=2
SERIAL_MODE=rs485
RTS_MODE=up            # or: none (auto-direction adapter), down
TARGET=1               # Modbus slave address (must match Pr 11.23)
POLLCYCLES=10          # slow-register read interval
RECONNECT_DELAY=1
```

See `cmdrsk.ini` for full documentation of every parameter.

---

## HAL Integration

Add to your HAL file (see `cmdrsk_vfd.hal` for a complete example):

```hal
loadusr -Wn cmdrsk-vfd cmdrsk_vfd -n cmdrsk-vfd

# Machine-on awareness — suppresses Modbus when VFD is unpowered
net machine-is-on  halui.machine.is-on  =>  cmdrsk-vfd.enabled

# Spindle motion
net spindle-fwd    motion.spindle-forward    =>  cmdrsk-vfd.spindle-fwd
net spindle-rev    motion.spindle-reverse    =>  cmdrsk-vfd.spindle-rev
net spindle-on     motion.spindle-on         =>  cmdrsk-vfd.spindle-on
net spindle-rpm    motion.spindle-speed-out  =>  cmdrsk-vfd.speed-command

# Feedback
net spindle-at-speed  cmdrsk-vfd.at-speed  =>  motion.spindle-at-speed
```

### HAL Pin Reference

#### Input pins (LinuxCNC → driver → VFD)

| Pin | Type | Description |
|-----|------|-------------|
| `enabled` | bit | **Machine-on gate.** When FALSE, all Modbus activity is suspended and output pins are zeroed. Wire to `halui.machine.is-on`. |
| `spindle-on` | bit | Spindle run command. |
| `spindle-fwd` | bit | Commanded forward direction. |
| `spindle-rev` | bit | Commanded reverse direction. |
| `speed-command` | float | Commanded spindle speed (RPM). |
| `fault-reset` | bit | **Rising edge** sends a drive reset (writes 100 to Pr 10.38 — the correct serial reset method per the Commander SK manual, section 4.1). |
| `max-speed` | bit | When TRUE, skips slow-register reads for maximum Modbus throughput (spindle-orient / jog mode). |

#### Output pins (VFD → driver → LinuxCNC)

| Pin | Type | Source | Description |
|-----|------|--------|-------------|
| `at-speed` | bit | Pr 10.06 / status word | Drive is at commanded speed. |
| `is-stopped` | bit | Pr 10.03 / status word | Output frequency is at zero. |
| `drive-ok` | bit | Pr 10.01 / status word | Drive is healthy (not tripped). |
| `fault` | bit | derived | Inverse of `drive-ok`. Asserts on any trip. |
| `drive-active` | bit | Pr 10.02 / status word | Inverter output bridge is live. |
| `spindle-running-rev` | bit | Pr 10.14 / status word | Drive is **actually** running in reverse (as opposed to commanded direction). |
| `hardware-enable` | bit | Pr 6.29 | State of the hardware enable terminal. See note below. |
| `drive-enable` | bit | Pr 6.15 | Software drive enable state. |
| `modbus-ok` | bit | derived | Asserts after 10 consecutive successful Modbus transactions. |
| `frequency-command` | float | computed | Speed reference sent to drive (Hz). |
| `frequency-out` | float | Pr 5.01 | Actual drive output frequency (Hz). |
| `motor-RPM` | float | Pr 5.04 | Actual motor RPM. |
| `current-load-percentage` | float | Pr 4.20 | Drive load as % of rated current. |
| `max-rpm` | float | Pr 1.06 | Maximum RPM calculated from VFD EEPROM at startup. |
| `min-rpm` | float | Pr 1.07 | Minimum RPM calculated from VFD EEPROM at startup. |
| `frequency-limit-high` | float | Pr 1.06 | Maximum output frequency (Hz). |
| `frequency-limit-low` | float | Pr 1.07 | Minimum output frequency (Hz). |
| `status` | s32 | Pr 10.40 | Raw status word (bitmask, see header). |
| `trip-code` | s32 | Pr 10.20 | Most recent trip code number. |
| `alarm-code` | s32 | Pr 10.20 | Alias for trip-code. |
| `error-count` | s32 | driver | Cumulative count of failed Modbus transactions. |
| `inverter-load-percentage` | float | — | Reserved (currently zero). |

#### HAL parameters (settable at runtime via `halcmd setp`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `loop-time` | 0.010 | Poll loop sleep time (seconds). |
| `tolerance` | 0.01 | At-speed frequency tolerance (fraction, e.g. 0.01 = 1%). |
| `rated-hz` | from VFD | Motor nameplate frequency (Hz) — read from Pr 5.06 at startup. |
| `rated-rpm` | from VFD | Motor nameplate RPM — read from Pr 5.08 at startup. |
| `rpm-limit` | 20000 | Maximum commanded RPM cap. |

---

## Power Drawbar / Hardware Interlock Note

The `hardware-enable` pin reflects the state of the Commander SK **hardware
enable terminal** (Pr 6.29).  On machines that use a pneumatic power drawbar,
this terminal is wired to the drawbar interlock: the enable signal opens
whenever the drawbar is actuated to eject or clamp a tool.

**This is a normal, expected, momentary event — do not wire it to e-stop.**

When the interlock opens:
- The drive inhibits itself automatically (displays "inh")
- `hardware-enable` goes FALSE
- `drive-active` goes FALSE
- `spindle-running-rev` and `at-speed` clear

LinuxCNC does not need to intervene.  Useful things to do with this pin:
- Drive a "spindle inhibited" indicator in your GUI or panel
- Prevent an inadvertent spindle-start while the drawbar is active (HAL ladder
  logic: AND `spindle-on` with `hardware-enable` before wiring to any output)

---

## Fault Handling

When the drive trips:
- `fault` asserts, `drive-ok` clears
- The most recent trip code is logged to `stderr` with a human-readable name
  (e.g. `FAULT trip=OI.AC (AC instantaneous over current)`)
- The last 10 trip codes are stored; `trip-code` reflects the most recent

To reset after a trip:
1. Correct the cause of the trip
2. Pulse `fault-reset` high (rising edge only)

The driver writes 100 to Pr 10.38, which is the correct serial-interface reset
method per the Commander SK Advanced User Guide, section 4.1, method 4.

Note: `OI.AC` and `OI.br` trips cannot be reset for 10 seconds after the fault.
`UU` (under-voltage) trips reset automatically when supply voltage recovers.
`EEF` (EEPROM failure) and `HF20–HF32` hardware faults cannot be reset at all.

---

## Troubleshooting

### Modbus errors at startup

Set `DEBUG=1` in `[CMDRSK]` and/or `MODBUS_DEBUG=1` to see raw Modbus frames.
Run the driver from a terminal:

```bash
cmdrsk_vfd -n cmdrsk-vfd -I /path/to/machine.ini -d -m
```

Common causes:
- Wrong `DEVICE` path
- `RTS_MODE` mismatch — try `none` if your adapter handles direction automatically
- Wrong `PARITY` or `STOPBITS` — verify against Pr 11.25 on the drive
- Drive not set to Modbus RTU mode (Pr 11.24)

### `modbus-ok` never asserts

The pin asserts after 10 consecutive successful transactions.  If it stays
low, check for wiring issues or baud-rate mismatch.  `error-count` will
increment on every failed transaction.

### Drive not responding to speed commands

Verify that `Pr 6.43 = On(1)` (control word enable) is set on the drive.
Without this, the driver can read status but the control word writes are ignored.

### `enabled` pin stays FALSE

The `enabled` pin must be wired to a machine-on signal.  If it is left
unconnected it defaults to FALSE and the driver will not communicate.

---

## Credits

Adapted from Michael Haberler's `vfs11_vfd.c`, which was adapted from Steve
Padnos' `gs2_vfd.c`, with modifications from John Thornton.

Trip code table sourced from: *Commander SK Advanced User Guide*, Issue 10,
Table 10-17, Nidec Control Techniques Ltd, December 2017.

Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.  
Copyright (C) 2009–2012 Michael Haberler  
Copyright (C) 2013–2026 Steve Richardson

Licensed under the GNU Lesser General Public License, version 2.

---

## TODO

- **Probe Basic / QtPyVCP user tab** — a comprehensive VFD status and control
  panel as a Probe Basic user tab.  Because HAL has no string pin type, the
  `trip-code` s32 value must be mapped to the human-readable fault name
  (e.g. `OI.AC`) in the Python/UI layer.  The tab would show:
  - Named fault string (Python dict lookup of `trip-code`)
  - Drive status indicators: `drive-ok`, `drive-active`, `at-speed`, `is-stopped`
  - Hardware-enable / power drawbar interlock indicator
  - Fault reset button (pulses `fault-reset`)
  - Live frequency, RPM, and load readouts
