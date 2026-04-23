#!/bin/bash
#
# Phase 3: Hardware validation — interactive test at the physical machine.
#
# Run this WHILE LinuxCNC is already running with the machine enabled.
# It will prompt you to perform physical actions and capture HAL pin state.
#
# Usage:
#   ./test/phase3_hardware.sh
#
# Results are saved to /tmp/cmdrsk_phase3_results.txt

set -euo pipefail

LOG="/tmp/cmdrsk_phase3_results.txt"
COMP="spindle-vfd"
PASS=0
FAIL=0
WARN=0

log() { echo "$*" | tee -a "$LOG"; }
blank() { echo "" | tee -a "$LOG"; }

snapshot() {
    log "  --- pin snapshot ($1) ---"
    halcmd show pin "$COMP" 2>&1 | tee -a "$LOG"
    blank
}

pin() { halcmd getp "${COMP}.$1" 2>/dev/null; }

check() {
    local pin="$1" expected="$2" desc="$3"
    local actual
    actual=$(pin "$pin")
    if [ "$actual" = "$expected" ]; then
        log "  PASS: $desc ($pin = $actual)"
        PASS=$((PASS + 1))
    else
        log "  FAIL: $desc ($pin expected=$expected got=$actual)"
        FAIL=$((FAIL + 1))
    fi
}

check_nonzero() {
    local pin="$1" desc="$2"
    local actual
    actual=$(pin "$pin")
    if [ "$actual" != "0" ] && [ -n "$actual" ]; then
        log "  PASS: $desc ($pin = $actual)"
        PASS=$((PASS + 1))
    else
        log "  FAIL: $desc ($pin = $actual, expected nonzero)"
        FAIL=$((FAIL + 1))
    fi
}

warn_check() {
    local pin="$1" expected="$2" desc="$3"
    local actual
    actual=$(pin "$pin")
    if [ "$actual" = "$expected" ]; then
        log "  OK:   $desc ($pin = $actual)"
    else
        log "  WARN: $desc ($pin expected=$expected got=$actual)"
        WARN=$((WARN + 1))
    fi
}

pause() {
    echo ""
    echo ">>> $1"
    echo ">>> Press ENTER when ready..."
    read -r
    echo ""
}

# ===================================================================

echo "" > "$LOG"
log "=== cmdrsk_vfd Phase 3: Hardware Validation ==="
log "    $(date)"
log "    Host: $(hostname)"
blank

# --- Preflight ---------------------------------------------------------

log "--- Preflight ---"

if ! halcmd show comp 2>/dev/null | grep -q "$COMP"; then
    log "ERROR: $COMP not loaded. Is LinuxCNC running?"
    exit 1
fi
log "  component loaded OK"

if [ "$(pin enabled)" != "TRUE" ]; then
    log "ERROR: enabled=FALSE. Machine must be powered on."
    exit 1
fi
log "  enabled=TRUE (machine on)"
check "modbus-ok" "TRUE" "modbus communication active"
blank

# ===================================================================
# Test A: Idle state
# ===================================================================

log "--- Test A: Idle state (spindle stopped) ---"
snapshot "idle"

check "drive-ok"        "TRUE"  "drive OK"
check "fault"           "FALSE" "no fault"
check "drive-active"    "FALSE" "drive not active"
check "is-stopped"      "TRUE"  "zero speed"
check "hardware-enable" "TRUE"  "hardware enable asserted"
check "drive-enable"    "TRUE"  "software drive enable"
check_nonzero "frequency-limit-high"  "frequency limit populated"
check_nonzero "max-rpm"               "max RPM populated"
blank

# ===================================================================
# Test B: Spindle forward
# ===================================================================

pause "Command spindle FORWARD at moderate speed (e.g. M3 S300 in MDI)"

log "--- Test B: Spindle forward ---"
sleep 3  # let it stabilise
snapshot "forward"

check "drive-ok"            "TRUE"  "drive OK during fwd"
check "drive-active"        "TRUE"  "drive active during fwd"
check "is-stopped"          "FALSE" "not at zero speed"
check "at-speed"            "TRUE"  "at commanded speed"
check_nonzero "frequency-out"       "output frequency > 0"
check_nonzero "motor-RPM"           "motor RPM > 0"

log "  INFO: frequency-command = $(pin frequency-command)"
log "  INFO: frequency-out     = $(pin frequency-out)"
log "  INFO: motor-RPM         = $(pin motor-RPM)"
log "  INFO: current-load-pct  = $(pin current-load-percentage)"
blank

# ===================================================================
# Test C: Spindle reverse
# ===================================================================

pause "Command spindle REVERSE at moderate speed (e.g. M4 S300 in MDI)"

log "--- Test C: Spindle reverse ---"
sleep 3
snapshot "reverse"

check "drive-active"        "TRUE"  "drive active during rev"
check_nonzero "frequency-out"       "output frequency > 0 during rev"

# Note: spindle-running-rev depends on the reversed HAL wiring
rev_val=$(pin spindle-running-rev)
log "  INFO: spindle-running-rev = $rev_val (depends on wiring direction)"
blank

# ===================================================================
# Test D: Spindle stop
# ===================================================================

pause "Stop the spindle (M5 in MDI)"

log "--- Test D: Spindle stop ---"
sleep 3
snapshot "stopped"

check "drive-active" "FALSE" "drive not active after stop"
check "is-stopped"   "TRUE"  "zero speed after stop"
blank

# ===================================================================
# Test E: Power drawbar / hardware inhibit
# ===================================================================

pause "Cycle the POWER DRAWBAR (or whatever triggers hardware enable interlock). Press ENTER WHILE it is active (tool releasing)"

log "--- Test E: Hardware inhibit (drawbar active) ---"
snapshot "drawbar-active"

check "hardware-enable" "FALSE" "hardware enable dropped during drawbar"
blank

pause "Release the drawbar (let it re-engage). Press ENTER after"

log "--- Test E2: Hardware inhibit released ---"
sleep 2
snapshot "drawbar-released"

check "hardware-enable" "TRUE" "hardware enable restored after drawbar"
blank

# ===================================================================
# Test F: Fault trip
# ===================================================================

pause "TRIGGER A FAULT (E-stop, or pull the hardware enable while spinning — whatever is safe). Press ENTER after the drive trips"

log "--- Test F: Fault trip ---"
sleep 2
snapshot "faulted"

check "drive-ok" "FALSE" "drive-ok dropped on fault"
check "fault"    "TRUE"  "fault asserted"
check_nonzero "trip-code"   "trip code populated"

log "  INFO: trip-code  = $(pin trip-code)"
log "  INFO: alarm-code = $(pin alarm-code)"
log "  INFO: status     = $(pin status)"
blank

# ===================================================================
# Test G: Fault reset
# ===================================================================

pause "Clear the fault condition (re-close E-stop, etc) but do NOT reset yet. Press ENTER when the fault condition is cleared but drive is still tripped"

log "--- Test G: Fault reset via HAL pin ---"
log "  sending fault-reset rising edge..."
halcmd setp "${COMP}.fault-reset" TRUE 2>/dev/null
sleep 2
halcmd setp "${COMP}.fault-reset" FALSE 2>/dev/null
sleep 3
snapshot "after-reset"

check "drive-ok" "TRUE"  "drive-ok restored after reset"
check "fault"    "FALSE" "fault cleared after reset"
blank

# --- Summary -----------------------------------------------------------

blank
log "================================================================"
log "  Phase 3 Results: $PASS passed, $FAIL failed, $WARN warnings"
log "  Results saved to: $LOG"
log "================================================================"
blank

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
