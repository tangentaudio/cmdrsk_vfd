#!/usr/bin/env python3
"""
sk_simulator.py — Modbus RTU slave simulating a Commander SK VFD.

Runs a pymodbus serial server on a PTY and accepts register-level commands
via a control FIFO so the test harness can dynamically change VFD state.

Usage:
    python3 sk_simulator.py <serial_port> <control_fifo>

Control commands (write one per line to the FIFO):
    SET <register> <value>          Set a single holding register
    HEALTHY                         Reset to healthy idle state
    RUNNING_FWD <hz_x10> <rpm>      Simulate forward run
    RUNNING_REV <hz_x10> <rpm>      Simulate reverse run
    TRIP <code>                     Simulate a drive trip
    DRAWBAR_ON                      Power drawbar active (hw enable low)
    DRAWBAR_OFF                     Power drawbar released
    QUIT                            Shut down the simulator
"""

import sys
import os
import threading
import time
import logging

from pymodbus.datastore import (
    ModbusSlaveContext,
    ModbusServerContext,
    ModbusSequentialDataBlock,
)
from pymodbus.server import StartSerialServer

# ---------------------------------------------------------------------------
# Commander SK register addresses (same as cmdrsk_vfd.h, wire = param - 1)
# ---------------------------------------------------------------------------
# We store everything by *parameter number* in our state dict, but the
# Modbus wire address is param-1.  pymodbus uses 0-based addressing so
# we allocate a block starting at address 0 with enough room.

REG = {
    "MAX_SET_SPEED":        106,
    "MIN_SET_SPEED":        107,
    "REFERENCE_SELECTOR":   114,
    "PRECISION_REF_COARSE": 118,
    "PERCENT_LOAD":         420,
    "MOTOR_FREQUENCY":      501,
    "MOTOR_SPEED":          504,
    "RATED_FREQUENCY":      506,
    "RATED_RPM":            508,
    "DRIVE_ENABLE":         615,
    "HARDWARE_ENABLE":      629,
    "CONTROL_WORD":         642,
    "CONTROL_WORD_ENABLE":  643,
    "LAST_TRIP":            1020,  # 1020-1029 = trip log
    "USER_TRIP":            1038,
    "STATUS_WORD":          1040,
    "SW_VERSION":           1129,
    "SW_SUBVERSION":        1134,
    "DSP_VERSION":          1135,
}

# Status word bits
ST_DRIVE_OK         = 1 << 0
ST_DRIVE_ACTIVE     = 1 << 1
ST_ZERO_SPEED       = 1 << 2
ST_AT_SET_SPEED     = 1 << 5
ST_DIRECTION_RUNNING = 1 << 13

# Maximum register address we need (wire address)
MAX_ADDR = 1200

log = logging.getLogger("sk_sim")


def wire(param):
    """Convert parameter number to Modbus wire address."""
    return param - 1


class SKSimulator:
    """Commander SK register state manager."""

    def __init__(self, context):
        self.context = context
        self.slave_ctx = context[1]  # slave ID 1

    def set_reg(self, param, value):
        """Set a holding register by parameter number."""
        # pymodbus ModbusSequentialDataBlock: setValues(address, [values])
        # For holding registers (function code 3), use fc=3
        self.slave_ctx.setValues(3, wire(param), [value & 0xFFFF])

    def get_reg(self, param):
        """Read a holding register by parameter number."""
        vals = self.slave_ctx.getValues(3, wire(param), 1)
        return vals[0]

    def set_healthy_idle(self):
        """Configure registers for a healthy, idle drive."""
        # Motor nameplate
        self.set_reg(REG["RATED_FREQUENCY"], 500)   # 50.0 Hz
        self.set_reg(REG["RATED_RPM"],       1440)
        self.set_reg(REG["MAX_SET_SPEED"],   500)    # 50.0 Hz
        self.set_reg(REG["MIN_SET_SPEED"],   50)     # 5.0 Hz

        # Version info
        self.set_reg(REG["SW_VERSION"],    2)
        self.set_reg(REG["SW_SUBVERSION"], 13)
        self.set_reg(REG["DSP_VERSION"],   1)

        # Healthy status: drive OK, zero speed
        self.set_reg(REG["STATUS_WORD"], ST_DRIVE_OK | ST_ZERO_SPEED)

        # Output readings
        self.set_reg(REG["MOTOR_FREQUENCY"], 0)
        self.set_reg(REG["MOTOR_SPEED"],     0)
        self.set_reg(REG["PERCENT_LOAD"],    0)

        # Enable states
        self.set_reg(REG["HARDWARE_ENABLE"], 1)
        self.set_reg(REG["DRIVE_ENABLE"],    1)

        # Clear trip log
        for i in range(10):
            self.set_reg(REG["LAST_TRIP"] + i, 0)

        log.info("State: HEALTHY IDLE")

    def set_running_fwd(self, hz_x10=250, rpm=720):
        """Simulate forward run at given frequency and RPM."""
        status = ST_DRIVE_OK | ST_DRIVE_ACTIVE | ST_AT_SET_SPEED
        self.set_reg(REG["STATUS_WORD"], status)
        self.set_reg(REG["MOTOR_FREQUENCY"], hz_x10)
        self.set_reg(REG["MOTOR_SPEED"], rpm)
        self.set_reg(REG["PERCENT_LOAD"], 150)  # 15.0%
        log.info("State: RUNNING FWD %.1f Hz, %d RPM", hz_x10/10, rpm)

    def set_running_rev(self, hz_x10=250, rpm=720):
        """Simulate reverse run."""
        status = (ST_DRIVE_OK | ST_DRIVE_ACTIVE |
                  ST_AT_SET_SPEED | ST_DIRECTION_RUNNING)
        self.set_reg(REG["STATUS_WORD"], status)
        self.set_reg(REG["MOTOR_FREQUENCY"], hz_x10)
        self.set_reg(REG["MOTOR_SPEED"], rpm)
        self.set_reg(REG["PERCENT_LOAD"], 150)
        log.info("State: RUNNING REV %.1f Hz, %d RPM", hz_x10/10, rpm)

    def set_trip(self, code):
        """Simulate a drive trip."""
        self.set_reg(REG["STATUS_WORD"], ST_ZERO_SPEED)  # no DRIVE_OK
        self.set_reg(REG["MOTOR_FREQUENCY"], 0)
        self.set_reg(REG["MOTOR_SPEED"], 0)
        self.set_reg(REG["PERCENT_LOAD"], 0)
        self.set_reg(REG["LAST_TRIP"], code)
        log.info("State: TRIPPED code=%d", code)

    def set_drawbar(self, active):
        """Simulate power drawbar interlock."""
        self.set_reg(REG["HARDWARE_ENABLE"], 0 if active else 1)
        log.info("State: DRAWBAR %s", "ACTIVE" if active else "RELEASED")


def control_loop(sim, fifo_path):
    """Read commands from the control FIFO and update simulator state."""
    while True:
        try:
            with open(fifo_path, "r") as fifo:
                for line in fifo:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue

                    parts = line.split()
                    cmd = parts[0].upper()
                    log.info("CMD: %s", line)

                    if cmd == "QUIT":
                        log.info("Shutting down")
                        os._exit(0)
                    elif cmd == "HEALTHY":
                        sim.set_healthy_idle()
                    elif cmd == "SET" and len(parts) >= 3:
                        reg = int(parts[1], 0)
                        val = int(parts[2], 0)
                        sim.set_reg(reg, val)
                    elif cmd == "RUNNING_FWD":
                        hz = int(parts[1]) if len(parts) > 1 else 250
                        rpm = int(parts[2]) if len(parts) > 2 else 720
                        sim.set_running_fwd(hz, rpm)
                    elif cmd == "RUNNING_REV":
                        hz = int(parts[1]) if len(parts) > 1 else 250
                        rpm = int(parts[2]) if len(parts) > 2 else 720
                        sim.set_running_rev(hz, rpm)
                    elif cmd == "TRIP" and len(parts) >= 2:
                        sim.set_trip(int(parts[1]))
                    elif cmd == "DRAWBAR_ON":
                        sim.set_drawbar(True)
                    elif cmd == "DRAWBAR_OFF":
                        sim.set_drawbar(False)
                    else:
                        log.warning("Unknown command: %s", line)
        except Exception as e:
            log.error("Control loop error: %s", e)
            time.sleep(0.1)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <serial_port> <control_fifo>")
        sys.exit(1)

    serial_port = sys.argv[1]
    control_fifo = sys.argv[2]

    logging.basicConfig(
        level=logging.INFO,
        format="[sk_sim] %(message)s",
    )

    # Create datastore: a block of holding registers from address 0
    store = ModbusSlaveContext(
        hr=ModbusSequentialDataBlock(0, [0] * MAX_ADDR),
        zero_mode=True,  # addresses are 0-based
    )
    context = ModbusServerContext(slaves={1: store}, single=False)

    sim = SKSimulator(context)
    sim.set_healthy_idle()

    # Start control FIFO listener in a background thread
    ctrl_thread = threading.Thread(
        target=control_loop, args=(sim, control_fifo), daemon=True
    )
    ctrl_thread.start()

    log.info("Starting Modbus RTU server on %s", serial_port)

    # This blocks — runs the Modbus server
    StartSerialServer(
        context=context,
        port=serial_port,
        baudrate=19200,
        parity="N",
        stopbits=2,
        bytesize=8,
        timeout=1,
    )


if __name__ == "__main__":
    main()
