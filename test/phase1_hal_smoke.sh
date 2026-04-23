#!/bin/bash
#
# Phase 1 smoke test for cmdrsk_vfd
#
# Tests HAL pin registration, default values, and the enabled=false idle
# path using a virtual serial port (socat PTY pair).  No VFD hardware needed.
#
# Prerequisites:
#   - LinuxCNC RIP built at ~/devel/linuxcnc
#   - socat installed
#   - cmdrsk_vfd built (run 'make' first)
#
# Usage:
#   ./test/phase1_hal_smoke.sh
#
# Returns 0 on success, 1 on any failure.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BINARY="$PROJECT_DIR/cmdrsk_vfd"
TEST_INI="$SCRIPT_DIR/test.ini"
PTY_LINK="/tmp/cmdrsk_test_pty"
COMP_NAME="cmdrsk-vfd"
LCNC_HOME="${LCNC_HOME:-$HOME/devel/linuxcnc}"
RIP_ENV="$LCNC_HOME/scripts/rip-environment"

PASS=0
FAIL=0
SOCAT_PID=""
HALRUN_PID=""
HAL_FIFO=""

# --- helpers ---------------------------------------------------------------

cleanup() {
    echo ""
    echo "--- cleanup ---"
    # Stop HAL which will kill our loaded component
    "$LCNC_HOME/bin/halcmd" unloadusr all 2>/dev/null || true
    sleep 1
    # Close the FIFO fd — this causes halrun to see EOF and exit
    exec 3>&- 2>/dev/null || true
    if [ -n "$HALRUN_PID" ]; then
        # Give halrun a moment, then force-kill if needed
        sleep 1
        kill "$HALRUN_PID" 2>/dev/null || true
        wait "$HALRUN_PID" 2>/dev/null || true
    fi
    # Kill socat
    if [ -n "$SOCAT_PID" ] && kill -0 "$SOCAT_PID" 2>/dev/null; then
        kill "$SOCAT_PID" 2>/dev/null || true
    fi
    rm -f "$PTY_LINK" "$HAL_FIFO"
    echo "--- done ---"
}
trap cleanup EXIT

pass() {
    echo "  PASS: $1"
    PASS=$((PASS + 1))
}

fail() {
    echo "  FAIL: $1"
    FAIL=$((FAIL + 1))
}

check() {
    local description="$1"
    shift
    if "$@" >/dev/null 2>&1; then
        pass "$description"
    else
        fail "$description"
    fi
}

# Run a halcmd command in the RIP environment
hal() {
    "$LCNC_HOME/bin/halcmd" "$@"
}

# Get a HAL pin value
pin_value() {
    hal getp "$1" 2>/dev/null
}

# Check a pin exists with expected type and direction
check_pin() {
    local pin="$1"
    local expected_type="$2"   # float, bit, s32, u32
    local expected_dir="$3"    # IN, OUT

    local info
    info=$(hal show pin "$pin" 2>/dev/null | grep "$pin" | head -1)
    if [ -z "$info" ]; then
        fail "pin $pin exists"
        return
    fi
    pass "pin $pin exists"

    if echo "$info" | grep -qi "$expected_type"; then
        pass "pin $pin type=$expected_type"
    else
        fail "pin $pin type=$expected_type (got: $info)"
    fi

    if echo "$info" | grep -q "$expected_dir"; then
        pass "pin $pin dir=$expected_dir"
    else
        fail "pin $pin dir=$expected_dir (got: $info)"
    fi
}

# --- preflight -------------------------------------------------------------

echo "=== cmdrsk_vfd Phase 1: HAL Smoke Test ==="
echo ""

# Check prerequisites
if [ ! -f "$BINARY" ]; then
    echo "ERROR: binary not found at $BINARY — run 'make' first"
    exit 1
fi

if [ ! -f "$RIP_ENV" ]; then
    echo "ERROR: RIP environment not found at $RIP_ENV"
    exit 1
fi

if ! command -v socat >/dev/null 2>&1; then
    echo "ERROR: socat not installed"
    exit 1
fi

# Source RIP environment
source "$RIP_ENV"

# Kill any stale HAL session from a previous crashed run
halrun -U 2>/dev/null || true
sleep 1

# --- setup -----------------------------------------------------------------

echo "--- creating virtual serial port ---"
socat -d -d PTY,raw,echo=0,link="$PTY_LINK" PTY,raw,echo=0 &
SOCAT_PID=$!
sleep 1

if [ ! -e "$PTY_LINK" ]; then
    echo "ERROR: socat failed to create $PTY_LINK"
    exit 1
fi
echo "  socat PID=$SOCAT_PID, PTY=$PTY_LINK"

# --- Test 1: Component loads and registers pins ----------------------------

echo ""
echo "--- Test 1: Component loading and pin registration ---"

# Start HAL runtime in the background using a named pipe for control
HAL_FIFO=$(mktemp -u /tmp/cmdrsk_hal_XXXXXX)
mkfifo "$HAL_FIFO"
halrun < "$HAL_FIFO" &
HALRUN_PID=$!
exec 3>"$HAL_FIFO"  # keep the fifo open so halrun doesn't exit
sleep 1

# Load our component
echo "  loading cmdrsk_vfd..."
hal loadusr -Wn "$COMP_NAME" "$BINARY" -n "$COMP_NAME" -I "$TEST_INI" -d 2>/tmp/cmdrsk_test_stderr.log &
sleep 3

# Check component appeared
if hal show comp 2>/dev/null | grep -q "$COMP_NAME"; then
    pass "component '$COMP_NAME' registered"
else
    fail "component '$COMP_NAME' registered"
    echo ""
    echo "stderr output:"
    cat /tmp/cmdrsk_test_stderr.log
    echo ""
    echo "=== ABORTED: component failed to load ==="
    exit 1
fi

# --- Test 2: All expected pins exist with correct types --------------------

echo ""
echo "--- Test 2: Pin registration (type and direction) ---"

# Output pins
check_pin "${COMP_NAME}.alarm-code"                s32   OUT
check_pin "${COMP_NAME}.at-speed"                   bit   OUT
check_pin "${COMP_NAME}.current-load-percentage"    float OUT
check_pin "${COMP_NAME}.drive-active"               bit   OUT
check_pin "${COMP_NAME}.drive-enable"               bit   OUT
check_pin "${COMP_NAME}.drive-ok"                   bit   OUT
check_pin "${COMP_NAME}.error-count"                s32   OUT
check_pin "${COMP_NAME}.fault"                      bit   OUT
check_pin "${COMP_NAME}.frequency-command"          float OUT
check_pin "${COMP_NAME}.frequency-limit-high"       float OUT
check_pin "${COMP_NAME}.frequency-limit-low"        float OUT
check_pin "${COMP_NAME}.frequency-out"              float OUT
check_pin "${COMP_NAME}.hardware-enable"            bit   OUT
check_pin "${COMP_NAME}.inverter-load-percentage"   float OUT
check_pin "${COMP_NAME}.is-stopped"                 bit   OUT
check_pin "${COMP_NAME}.max-rpm"                    float OUT
check_pin "${COMP_NAME}.min-rpm"                    float OUT
check_pin "${COMP_NAME}.modbus-ok"                  bit   OUT
check_pin "${COMP_NAME}.motor-RPM"                  float OUT
check_pin "${COMP_NAME}.spindle-running-rev"        bit   OUT
check_pin "${COMP_NAME}.status"                     s32   OUT
check_pin "${COMP_NAME}.trip-code"                  s32   OUT

# Input pins
check_pin "${COMP_NAME}.enabled"                    bit   IN
check_pin "${COMP_NAME}.fault-reset"                bit   IN
check_pin "${COMP_NAME}.max-speed"                  bit   IN
check_pin "${COMP_NAME}.speed-command"              float IN
check_pin "${COMP_NAME}.spindle-fwd"                bit   IN
check_pin "${COMP_NAME}.spindle-on"                 bit   IN
check_pin "${COMP_NAME}.spindle-rev"                bit   IN

# --- Test 3: Default values -----------------------------------------------

echo ""
echo "--- Test 3: Default pin values ---"

check_default() {
    local pin="$1"
    local expected="$2"
    local actual
    actual=$(pin_value "${COMP_NAME}.${pin}")
    if [ "$actual" = "$expected" ]; then
        pass "$pin = $expected"
    else
        fail "$pin = $expected (got: '$actual')"
    fi
}

check_default "enabled"         "FALSE"
check_default "fault"           "FALSE"
check_default "fault-reset"     "FALSE"
check_default "drive-ok"        "FALSE"
check_default "drive-active"    "FALSE"
check_default "spindle-on"      "FALSE"
check_default "spindle-fwd"     "TRUE"
check_default "spindle-rev"     "FALSE"
check_default "modbus-ok"       "FALSE"
check_default "hardware-enable" "FALSE"
check_default "drive-enable"    "FALSE"
check_default "at-speed"        "FALSE"
check_default "is-stopped"      "FALSE"
check_default "max-speed"       "FALSE"
check_default "alarm-code"      "0"
check_default "trip-code"       "0"
check_default "error-count"     "0"
check_default "status"          "0"

# --- Test 4: Enabled=false idle path (no Modbus errors) --------------------

echo ""
echo "--- Test 4: Enabled=false idle path ---"

# Component starts with enabled=FALSE by default.
# Wait a couple seconds and verify no Modbus errors accumulated.
sleep 2

errors_before=$(pin_value "${COMP_NAME}.error-count")
sleep 2
errors_after=$(pin_value "${COMP_NAME}.error-count")

if [ "$errors_before" = "$errors_after" ]; then
    pass "no Modbus errors while disabled (error-count stable at $errors_after)"
else
    fail "no Modbus errors while disabled (went from $errors_before to $errors_after)"
fi

if [ "$(pin_value "${COMP_NAME}.modbus-ok")" = "FALSE" ]; then
    pass "modbus-ok is FALSE while disabled"
else
    fail "modbus-ok is FALSE while disabled"
fi

# Verify outputs stay zeroed
if [ "$(pin_value "${COMP_NAME}.frequency-out")" = "0" ]; then
    pass "frequency-out stays 0 while disabled"
else
    fail "frequency-out stays 0 while disabled"
fi

# --- Test 5: Enable transition (will get Modbus errors, but shouldn't crash)

echo ""
echo "--- Test 5: Enable transition (expect Modbus timeouts) ---"

hal setp "${COMP_NAME}.enabled" TRUE 2>/dev/null
sleep 3

# Component should still be alive
if hal show comp 2>/dev/null | grep -q "$COMP_NAME"; then
    pass "component survived enable transition"
else
    fail "component survived enable transition (crashed!)"
fi

# Error count should have increased (no slave responding)
errors_after_enable=$(pin_value "${COMP_NAME}.error-count")
if [ "$errors_after_enable" -gt "$errors_after" ]; then
    pass "Modbus errors accumulated with no slave (error-count=$errors_after_enable)"
else
    fail "expected Modbus errors with no slave (error-count=$errors_after_enable)"
fi

# modbus-ok should still be FALSE (no successful transactions)
if [ "$(pin_value "${COMP_NAME}.modbus-ok")" = "FALSE" ]; then
    pass "modbus-ok remains FALSE with no slave"
else
    fail "modbus-ok remains FALSE with no slave"
fi

# --- Test 6: Disable transition (outputs should zero) ----------------------

echo ""
echo "--- Test 6: Disable transition ---"

hal setp "${COMP_NAME}.enabled" FALSE 2>/dev/null
sleep 2

if [ "$(pin_value "${COMP_NAME}.frequency-out")" = "0" ]; then
    pass "outputs zeroed after disable"
else
    fail "outputs zeroed after disable"
fi

# Error count should stop growing
errors_t1=$(pin_value "${COMP_NAME}.error-count")
sleep 2
errors_t2=$(pin_value "${COMP_NAME}.error-count")

if [ "$errors_t1" = "$errors_t2" ]; then
    pass "errors stopped after disable (stable at $errors_t2)"
else
    fail "errors stopped after disable ($errors_t1 -> $errors_t2)"
fi

# --- Summary ---------------------------------------------------------------

echo ""
echo "================================================================"
echo "  Phase 1 Results: $PASS passed, $FAIL failed"
echo "================================================================"

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "  stderr output from cmdrsk_vfd:"
    cat /tmp/cmdrsk_test_stderr.log 2>/dev/null || true
    exit 1
fi

exit 0
