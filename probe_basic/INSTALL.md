# Probe Basic VFD Monitor — Installation Guide

Integrates Commander SK VFD status, drawbar interlock, and fault handling
into a Probe Basic (QtPyVCP) installation.

## Features

- **Drawbar interlock**: Blocks spindle and program start when the power
  drawbar is active and the machine is idle. Running programs (e.g. during
  tool changes) are not interrupted.
- **VFD fault handling**: Aborts running programs and inhibits spindle on
  VFD trip. Displays trip code with human-readable description. Provides
  a soft-reset button to clear faults without power cycling.
- **Popup notifications**: Non-modal overlays for drawbar activation
  (auto-dismiss) and VFD faults (persistent until reset). Designed for
  fullscreen touchscreen operation — no window manager interference, no
  keyboard focus changes.
- **Sidebar status**: Compact VFD status in the sidebar user tab showing
  drive status, drawbar state, and fault information.

## Prerequisites

- LinuxCNC with `cmdrsk_vfd` driver installed and configured
- Probe Basic 0.6.x with QtPyVCP 5.x
- The following `cmdrsk_vfd` HAL pins must be available:
  - `spindle-vfd.fault` (bit, out)
  - `spindle-vfd.trip-code` (s32, out)
  - `spindle-vfd.hardware-enable` (bit, out)
  - `spindle-vfd.drive-ok` (bit, out)
  - `spindle-vfd.fault-reset` (bit, in)

## Installation

### 1. Copy the VFD monitor widget

Copy the `vfd_monitor/` directory into your Probe Basic config's
`user_tabs/` directory:

```bash
cp -r vfd_monitor/ ~/linuxcnc/configs/<your-config>/user_tabs/vfd_monitor/
```

Probe Basic auto-discovers user tabs from the `USER_TABS_PATH` directory
set in your INI file (typically `user_tabs/`).

### 2. Append HAL interlock logic

The file `vfd_interlock.hal` contains the interlock logic and widget
wiring. Its contents must be appended to your `probe_basic_postgui.hal`
(or equivalent POSTGUI_HALFILE).

**Important:** The interlock requires a `not` component. If your postgui
file already has `loadrt not` (Probe Basic's default timer logic uses one),
you must consolidate into a single `loadrt` call with named instances.

See `probe_basic_postgui.hal` in this directory for a complete working
example. The key changes are:

1. Replace `loadrt not` with `loadrt not names=not.0,not.drawbar`
2. Add `loadrt and2` and `loadrt or2` with named instances
3. Add the interlock wiring (drawbar + fault logic)
4. Add the monitor widget signal connections

### 3. Verify your INI file

Ensure your INI file has the user tabs path configured:

```ini
[DISPLAY]
USER_TABS_PATH = user_tabs/
```

## File Reference

| File | Purpose |
|------|---------|
| `vfd_monitor/vfd_monitor.py` | Sidebar widget + popup controller |
| `vfd_monitor/vfd_monitor.ui` | Qt Designer sidebar layout |
| `vfd_monitor/trip_codes.py` | Trip code lookup table (mirrors `cmdrsk_vfd.h`) |
| `vfd_monitor/__init__.py` | Python package marker |
| `vfd_interlock.hal` | HAL interlock logic (reference/template) |
| `probe_basic_postgui.hal` | Complete working postgui example |

## HAL Signal Reference

| Signal | Source | Purpose |
|--------|--------|---------|
| `vfd-fault` | `spindle-vfd.fault` | VFD fault state |
| `vfd-trip-code` | `spindle-vfd.trip-code` | Numeric trip code |
| `vfd-hw-enable` | `spindle-vfd.hardware-enable` | Hardware enable (drawbar) |
| `vfd-drive-ok` | `spindle-vfd.drive-ok` | Drive communication OK |
| `drawbar-active` | `not.drawbar.out` | Inverted hardware-enable |
| `spindle-inhibit` | `or2.spindle-inhibit.out` | Combined inhibit → `spindle.0.inhibit` |
| `vfd-fault-reset` | `vfd-monitor.fault-reset` | GUI reset → `spindle-vfd.fault-reset` |

## Behavior Summary

| Condition | Spindle | Start Program | Running Program |
|-----------|---------|---------------|-----------------|
| Normal | ✅ | ✅ | continues |
| Drawbar + idle | ❌ blocked | ❌ blocked | N/A |
| Drawbar + running | ❌ blocked | N/A | ✅ continues |
| VFD fault | ❌ blocked | ❌ blocked | ❌ aborted |

## Customization

- **Startup delay**: Set `STARTUP_DELAY` in `cmdrsk.ini` (seconds to wait
  after machine enable before polling VFD, default 3).
- **Popup appearance**: Edit the `DRAWBAR_STYLE` and `FAULT_STYLE`
  stylesheet constants in `vfd_monitor.py`.
- **Trip codes**: Update `trip_codes.py` if using a different Control
  Techniques drive with different fault codes.
