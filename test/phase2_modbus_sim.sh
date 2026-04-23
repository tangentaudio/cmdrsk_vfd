#!/bin/bash
#
# Phase 2: Modbus simulation tests for cmdrsk_vfd
#
# Uses sk_simulator.py (pymodbus RTU slave) on a socat PTY pair to verify
# the driver correctly reads VFD status, handles faults, processes spindle
# commands, and responds to the power drawbar interlock.
#
# Prerequisites:
#   - LinuxCNC RIP built at ~/devel/linuxcnc
#   - socat installed
#   - cmdrsk_vfd built (run 'make' first)
#   - test venv set up (run ./test/setup_env.sh first)
#
# Usage:
#   ./test/phase2_modbus_sim.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BINARY="$PROJECT_DIR/cmdrsk_vfd"
TEST_INI="$SCRIPT_DIR/test.ini"
VENV_DIR="$SCRIPT_DIR/.venv"
COMP_NAME="cmdrsk-vfd"
LCNC_HOME="${LCNC_HOME:-$HOME/devel/linuxcnc}"
RIP_ENV="$LCNC_HOME/scripts/rip-environment"

# Temp files
PTY_DRV="/tmp/cmdrsk_pty_drv"       # driver side
PTY_SIM="/tmp/cmdrsk_pty_sim"       # simulator side
CTRL_FIFO="/tmp/cmdrsk_sim_ctrl"    # control pipe for simulator

PASS=0
FAIL=0
SOCAT_PID=""
SIM_PID=""
HALRUN_PID=""
HAL_FIFO=""

# --- helpers ---------------------------------------------------------------

cleanup() {
    echo ""
    echo "--- cleanup ---"
    # Unload HAL components
    "$LCNC_HOME/bin/halcmd" unloadusr all 2>/dev/null || true
    sleep 1
    exec 3>&- 2>/dev/null || true
    [ -n "$HALRUN_PID" ] && { kill "$HALRUN_PID" 2>/dev/null; wait "$HALRUN_PID" 2>/dev/null; } || true
    [ -n "$SIM_PID" ]    && { kill "$SIM_PID" 2>/dev/null; wait "$SIM_PID" 2>/dev/null; } || true
    [ -n "$SOCAT_PID" ]  && { kill "$SOCAT_PID" 2>/dev/null; } || true
    rm -f "$PTY_DRV" "$PTY_SIM" "$CTRL_FIFO" "$HAL_FIFO"
    echo "--- done ---"
}
trap cleanup EXIT

pass() { echo "  PASS: $1"; PASS=$((PASS + 1)); }
fail() { echo "  FAIL: $1"; FAIL=$((FAIL + 1)); }

hal() { "$LCNC_HOME/bin/halcmd" "$@"; }

pin_value() { hal getp "$1" 2>/dev/null; }

# Send a command to the simulator
sim_cmd() {
    echo "$*" > "$CTRL_FIFO"
    sleep 0.2  # give simulator time to process
}

# Wait for Modbus communication to stabilise
wait_modbus() {
    local timeout=${1:-10}
    local i=0
    while [ "$i" -lt "$timeout" ]; do
        if [ "$(pin_value "${COMP_NAME}.modbus-ok")" = "TRUE" ]; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

check_pin_bool() {
    local pin="$1" expected="$2" desc="$3"
    local actual
    actual=$(pin_value "${COMP_NAME}.${pin}")
    if [ "$actual" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc (expected $expected, got '$actual')"
    fi
}

check_pin_s32() {
    local pin="$1" expected="$2" desc="$3"
    local actual
    actual=$(pin_value "${COMP_NAME}.${pin}")
    if [ "$actual" = "$expected" ]; then
        pass "$desc"
    else
        fail "$desc (expected $expected, got '$actual')"
    fi
}

check_pin_float_nonzero() {
    local pin="$1" desc="$2"
    local actual
    actual=$(pin_value "${COMP_NAME}.${pin}")
    if [ "$actual" != "0" ] && [ -n "$actual" ]; then
        pass "$desc (value=$actual)"
    else
        fail "$desc (got '$actual')"
    fi
}

check_pin_float_zero() {
    local pin="$1" desc="$2"
    local actual
    actual=$(pin_value "${COMP_NAME}.${pin}")
    if [ "$actual" = "0" ]; then
        pass "$desc"
    else
        fail "$desc (expected 0, got '$actual')"
    fi
}

# --- preflight -------------------------------------------------------------

echo "=== cmdrsk_vfd Phase 2: Modbus Simulation Tests ==="
echo ""

[ -f "$BINARY" ]  || { echo "ERROR: binary not found — run 'make'"; exit 1; }
[ -f "$RIP_ENV" ] || { echo "ERROR: RIP env not found at $RIP_ENV"; exit 1; }
command -v socat >/dev/null || { echo "ERROR: socat not installed"; exit 1; }

# Check venv
if [ ! -f "$VENV_DIR/bin/python3" ]; then
    echo "ERROR: test venv not found — run ./test/setup_env.sh first"
    exit 1
fi

source "$RIP_ENV"

# Kill stale HAL sessions
halrun -U 2>/dev/null || true
sleep 1

# --- setup -----------------------------------------------------------------

echo "--- creating virtual serial ports ---"
socat PTY,raw,echo=0,link="$PTY_DRV" PTY,raw,echo=0,link="$PTY_SIM" &
SOCAT_PID=$!
sleep 1
[ -e "$PTY_DRV" ] || { echo "ERROR: socat failed"; exit 1; }
echo "  driver=$PTY_DRV  simulator=$PTY_SIM"

# Create control FIFO
rm -f "$CTRL_FIFO"
mkfifo "$CTRL_FIFO"

echo "--- starting Commander SK simulator ---"
"$VENV_DIR/bin/python3" "$SCRIPT_DIR/sk_simulator.py" "$PTY_SIM" "$CTRL_FIFO" \
    > /tmp/cmdrsk_sim_stdout.log 2>&1 &
SIM_PID=$!
sleep 2

if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "ERROR: simulator failed to start"
    cat /tmp/cmdrsk_sim_stdout.log
    exit 1
fi
echo "  simulator PID=$SIM_PID"

echo "--- starting HAL and loading cmdrsk_vfd ---"
HAL_FIFO=$(mktemp -u /tmp/cmdrsk_hal_XXXXXX)
mkfifo "$HAL_FIFO"
halrun < "$HAL_FIFO" &
HALRUN_PID=$!
exec 3>"$HAL_FIFO"
sleep 1

# Override the DEVICE in INI to use our driver-side PTY
hal loadusr -Wn "$COMP_NAME" "$BINARY" -n "$COMP_NAME" -I "$TEST_INI" -d \
    2>/tmp/cmdrsk_test_stderr.log &

# The INI points to /tmp/cmdrsk_test_pty but we created PTY_DRV.
# Fix: update test.ini to use PTY_DRV path, or symlink.
ln -sf "$(readlink -f "$PTY_DRV")" /tmp/cmdrsk_test_pty 2>/dev/null || true

sleep 3

if ! hal show comp 2>/dev/null | grep -q "$COMP_NAME"; then
    echo "ERROR: component failed to load"
    cat /tmp/cmdrsk_test_stderr.log
    exit 1
fi
echo "  component loaded"

# Enable the driver
hal setp "${COMP_NAME}.enabled" TRUE 2>/dev/null

echo "--- waiting for Modbus communication ---"
if wait_modbus 15; then
    pass "modbus-ok asserted"
else
    fail "modbus-ok did not assert within 15s"
    echo ""
    echo "  stderr:"
    tail -20 /tmp/cmdrsk_test_stderr.log
    echo ""
    echo "  simulator log:"
    tail -20 /tmp/cmdrsk_sim_stdout.log
    # Continue anyway — some tests may still pass
fi

# ===================================================================
# Test 1: read_initial — motor parameters read from VFD EEPROM
# ===================================================================

echo ""
echo "--- Test 1: Initial parameter read ---"

# Simulator has: max=50.0Hz, min=5.0Hz, rated=50.0Hz/1440RPM
check_pin_float_nonzero "max-rpm"            "max-rpm populated from VFD"
check_pin_float_nonzero "frequency-limit-high" "upper freq limit populated"
check_pin_float_nonzero "frequency-limit-low"  "lower freq limit populated"

# ===================================================================
# Test 2: Healthy idle state
# ===================================================================

echo ""
echo "--- Test 2: Healthy idle state ---"

# Give a couple more poll cycles for slow registers
sleep 2

check_pin_bool "drive-ok"        "TRUE"  "drive-ok in healthy state"
check_pin_bool "fault"           "FALSE" "no fault in healthy state"
check_pin_bool "drive-active"    "FALSE" "drive not active at idle"
check_pin_bool "is-stopped"      "TRUE"  "is-stopped at idle"
check_pin_bool "hardware-enable" "TRUE"  "hardware enable asserted"
check_pin_bool "drive-enable"    "TRUE"  "software drive enable asserted"
check_pin_float_zero "frequency-out"  "zero output frequency at idle"
check_pin_s32  "trip-code"       "0"     "no trip code at idle"

# ===================================================================
# Test 3: Spindle forward
# ===================================================================

echo ""
echo "--- Test 3: Spindle forward ---"

sim_cmd "RUNNING_FWD 250 720"
sleep 2

check_pin_bool  "drive-ok"            "TRUE"  "drive-ok during fwd run"
check_pin_bool  "drive-active"        "TRUE"  "drive-active during fwd run"
check_pin_bool  "at-speed"            "TRUE"  "at-speed during fwd run"
check_pin_bool  "is-stopped"          "FALSE" "not stopped during fwd run"
check_pin_bool  "spindle-running-rev" "FALSE" "not reverse during fwd run"
check_pin_float_nonzero "frequency-out"     "frequency-out > 0 during fwd run"
check_pin_float_nonzero "motor-RPM"         "motor-RPM > 0 during fwd run"

# ===================================================================
# Test 4: Spindle reverse
# ===================================================================

echo ""
echo "--- Test 4: Spindle reverse ---"

sim_cmd "RUNNING_REV 300 860"
sleep 2

check_pin_bool  "drive-active"        "TRUE"  "drive-active during rev run"
check_pin_bool  "spindle-running-rev" "TRUE"  "spindle-running-rev during rev run"

# ===================================================================
# Test 5: Drive trip (fault detection)
# ===================================================================

echo ""
echo "--- Test 5: Drive trip ---"

sim_cmd "TRIP 3"
sleep 3  # extra time for trip code reads

check_pin_bool "drive-ok"  "FALSE" "drive-ok false after trip"
check_pin_bool "fault"     "TRUE"  "fault asserted after trip"
check_pin_s32  "trip-code" "3"     "trip-code = 3 (OI.AC overcurrent)"

# Check that the trip was logged to stderr
if grep -q "FAULT" /tmp/cmdrsk_test_stderr.log; then
    pass "trip logged to stderr"
else
    fail "trip logged to stderr"
fi

# ===================================================================
# Test 6: Fault reset
# ===================================================================

echo ""
echo "--- Test 6: Fault reset ---"

# Restore healthy state in simulator (real VFD would do this on reset)
sim_cmd "HEALTHY"
sleep 1

# Trigger reset via rising edge on fault-reset pin
hal setp "${COMP_NAME}.fault-reset" TRUE 2>/dev/null
sleep 1
hal setp "${COMP_NAME}.fault-reset" FALSE 2>/dev/null
sleep 2

check_pin_bool "drive-ok" "TRUE"  "drive-ok restored after reset"
check_pin_bool "fault"    "FALSE" "fault cleared after reset"

# ===================================================================
# Test 7: Power drawbar interlock
# ===================================================================

echo ""
echo "--- Test 7: Power drawbar interlock ---"

sim_cmd "DRAWBAR_ON"
sleep 3  # wait for slow register poll

check_pin_bool "hardware-enable" "FALSE" "hardware-enable low during drawbar"

sim_cmd "DRAWBAR_OFF"
sleep 3

check_pin_bool "hardware-enable" "TRUE"  "hardware-enable restored after drawbar"

# ===================================================================
# Test 8: Disable/re-enable cycle
# ===================================================================

echo ""
echo "--- Test 8: Disable/re-enable cycle ---"

hal setp "${COMP_NAME}.enabled" FALSE 2>/dev/null
sleep 2

check_pin_float_zero "frequency-out" "outputs zeroed after disable"
check_pin_bool "modbus-ok" "FALSE"   "modbus-ok false after disable"

hal setp "${COMP_NAME}.enabled" TRUE 2>/dev/null

if wait_modbus 15; then
    pass "modbus-ok re-asserted after re-enable"
else
    fail "modbus-ok did not re-assert after re-enable"
fi

check_pin_bool "drive-ok" "TRUE" "drive-ok restored after re-enable"

# --- Summary ---------------------------------------------------------------

echo ""
echo "================================================================"
echo "  Phase 2 Results: $PASS passed, $FAIL failed"
echo "================================================================"

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "  driver stderr (last 20 lines):"
    tail -20 /tmp/cmdrsk_test_stderr.log 2>/dev/null || true
    echo ""
    echo "  simulator log (last 20 lines):"
    tail -20 /tmp/cmdrsk_sim_stdout.log 2>/dev/null || true
    exit 1
fi

exit 0
