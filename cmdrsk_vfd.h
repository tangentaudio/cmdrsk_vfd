#ifndef _cmdrsk_vfd_h
#define _cmdrsk_vfd_h

#include <stdio.h>  /* for snprintf in sk_trip_name()/sk_trip_desc() */

/*
 * Commander SK parameter register addresses.
 * CT Modbus addressing: Pr X.Y -> register = X*100 + Y
 * The GETPARAM/SETPARAM macros subtract 1 for the wire-level address.
 */

#define PR_MAX_SET_SPEED	106
#define PR_MIN_SET_SPEED	107

#define PR_REFERENCE_SELECTOR	114
#define PR_PRECISION_REF_COARSE 118
#define PR_PRECISION_REF_FINE	119
#define PR_PRECISION_REF_SELECT	144

#define PR_PERCENT_LOAD		420

#define PR_MOTOR_FREQUENCY	501
#define PR_MOTOR_VOLTAGE	502
#define PR_MOTOR_POWER		503
#define PR_MOTOR_SPEED		504
#define PR_RATED_FREQUENCY	506
#define PR_RATED_CURRENT	507
#define PR_RATED_RPM		508
#define PR_RATED_VOLTAGE	509
#define PR_SWITCHING_FREQUENCY	537

#define PR_RUN_YEARS_DAYS	622
#define PR_RUN_HOURS_MINS	623

#define PR_SEQ_RUN_FORWARD	630
#define PR_SEQ_JOG_FORWARD	631
#define PR_SEQ_RUN_REVERSE	632
#define PR_SEQ_FWD_REV		633
#define PR_SEQ_RUN		634
#define PR_SEQ_JOG_REVERSE	637
#define PR_SEQ_N_STOP		639
#define PR_SEQ_LATCHING		640
#define PR_CONTROL_WORD		642

/* Pr 6.15: software drive enable (RW, reflects internal enable state) */
#define PR_DRIVE_ENABLE		615
/* Pr 6.29: hardware enable terminal state (reflects physical enable input) */
#define PR_HARDWARE_ENABLE	629
#define PR_CONTROL_WORD_ENABLE	643


#define PR_DRIVE_OK		1001
#define PR_DRIVE_ACTIVE		1002
#define PR_ZERO_SPEED		1003
#define PR_AT_BELOW_MINIMUM	1004
#define PR_BELOW_SET_SPEED	1005
#define PR_AT_SET_SPEED		1006
#define PR_ABOVE_SET_SPEED	1007
#define PR_ALARM_OVERLOAD	1017
#define PR_ALARM_TEMPERATURE	1018
#define PR_ALARM_GENERAL	1019
#define PR_LAST_TRIP		1020
#define PR_TRIP_1		1021
#define PR_TRIP_2		1022
#define PR_TRIP_3		1023
#define PR_TRIP_4		1024
#define PR_TRIP_5		1025
#define PR_TRIP_6		1026
#define PR_TRIP_7		1027
#define PR_TRIP_8		1028
#define PR_TRIP_9		1029

#define PR_STATUS_WORD		1040

#define PR_SW_VERSION		1129
#define PR_SW_SUBVERSION	1134
#define PR_DSP_VERSION		1135

/*
 * Pr 10.38: User trip / drive reset register.
 * Write 100 to perform a drive reset (per Commander SK Advanced User Guide
 * section 4.1, method 4). Valid user trip codes are 1-255 except 100 and 255.
 */
#define PR_USER_TRIP		1038


// PR_STATUS_WORD bit values
// 15	Unused
// 14	Pr10.15 - Mains loss detected
// 13	Pr10.14 - Direction running
// 12	Pr10.13 - Direction commanded
// 11	Pr10.12 - Braking resistor alarm
// 10	Pr10.11 - Dynamic brake active
// 9	Pr10.10 - Regenerating
// 8	Pr10.09 - Drive output at current limit
// 7	Pr10.08 - Load reached
// 6	Pr10.07 - Above set speed
// 5	Pr10.06 - At set speed
// 4	Pr10.05 - Below set speed
// 3	Pr10.04 - Running at or below minimum speed
// 2	Pr10.03 - Zero speed
// 1	Pr10.02 - Drive active
// 0	Pr10.01 - Drive OK
#define ST_MAINS_LOSS		(1<<14)
#define ST_DIRECTION_RUNNING 	(1<<13)
#define ST_DIRECTION_COMMANDED	(1<<12)
#define ST_RESISTOR_ALARM	(1<<11)
#define ST_DYNAMIC_BRAKING	(1<<10)
#define ST_REGENERATING		(1<<9)
#define ST_AT_CURRENT_LIMIT	(1<<8)
#define ST_LOAD_REACHED		(1<<7)
#define ST_ABOVE_SET_SPEED	(1<<6)
#define ST_AT_SET_SPEED		(1<<5)
#define ST_BELOW_SET_SPEED	(1<<4)
#define ST_AT_BELOW_MINIMUM	(1<<3)
#define ST_ZERO_SPEED		(1<<2)
#define ST_DRIVE_ACTIVE		(1<<1)
#define ST_DRIVE_OK		(1<<0)


// PR_CONTROL_WORD bit values
// 15	Reserved
// 14	Keypad watchdog
// 13	Pr10.33 - Reset drive  (0->1 edge resets; prefer Pr10.38=100 via serial)
// 12	Trip drive
// 11	Reserved
// 10	Reserved
// 9	Pr6.37 - Jog reverse
// 8	Pr1.42 - Analog/preset reference
// 7	Auto/manual
// 6	Pr6.39 - /STOP
// 5	Pr6.34 - Run
// 4	Pr6.33 - Fwd/Reverse
// 3	Pr6.32 - Run Reverse
// 2	Pr6.31 - Jog forward
// 1	Pr6.30 - Run forward
// 0	Pr6.15 - Drive enable
#define CTL_RESET_DRIVE		(1<<13)
#define CTL_TRIP_DRIVE		(1<<12)
#define CTL_JOG_REVERSE		(1<<9)
#define CTL_AUTO_MANUAL		(1<<7)
#define CTL_N_STOP		(1<<6)
#define CTL_RUN			(1<<5)
#define CTL_FWD_REV		(1<<4)
#define CTL_RUN_REVERSE		(1<<3)
#define CTL_JOG_FORWARD		(1<<2)
#define CTL_RUN_FORWARD		(1<<1)
#define CTL_DRIVE_ENABLE	(1<<0)


// PR_REFERENCE_SELECTOR values
// 5: Precision reference (selected for Modbus speed control)
#define SEL_REFERENCE_PRECISION	0x05


/*
 * Commander SK trip code lookup table.
 * Source: Commander SK Advanced User Guide, Issue 10, Table 10-17.
 *
 * Usage: iterate until code==0 (sentinel).
 * User trips t040-t089 (codes 40-89) are handled as a range.
 * Hardware faults HF01-HF19 (fatal, display only) appear as HFxx on panel
 * but the numeric codes stored in Pr 10.20 are not publicly documented;
 * they will fall through to the unknown-code path.
 */
typedef struct {
    int code;
    const char *name;
    const char *description;
} sk_trip_t;

/* Note: Hardware enable (Pr 6.29) going low during power drawbar operation
 * is a NORMAL, expected, ephemeral condition for tool changes.  It must NOT
 * be wired to e-stop.  Expose it as a HAL output pin for operator display
 * and spindle-inhibit interlock logic only. */

static const sk_trip_t sk_trip_table[] = {
    {1,   "UU",    "DC bus under voltage"},
    {2,   "OU",    "DC bus over voltage"},
    {3,   "OI.AC", "AC instantaneous over current"},
    {4,   "OI.br", "Braking resistor instantaneous over current"},
    {6,   "Et",    "External trip"},
    {7,   "O.SPd", "Overspeed"},
    {18,  "tunE",  "Auto-tune stopped before completion"},
    {19,  "It.br", "I2t on braking resistor"},
    {20,  "It.AC", "I2t on drive output current"},
    {21,  "O.ht1", "Drive over-heat (IGBT thermal model)"},
    {22,  "O.ht2", "Drive over-heat (heatsink temperature)"},
    {24,  "th",    "Motor thermistor trip"},
    {26,  "O.Ld1", "+24V or digital output overload"},
    {27,  "O.ht3", "Drive over-heat (DC bus thermal model)"},
    {28,  "cL1",   "Analog input 1 current loss"},
    {30,  "SCL",   "Serial comms timeout (external keypad)"},
    {31,  "EEF",   "Internal EEPROM failure (load defaults to clear)"},
    {32,  "PH",    "Input phase loss or high imbalance"},
    {33,  "rS",    "Stator resistance measurement failure"},
    {35,  "CL.bt", "Trip initiated from control word bit 12"},
    {100, "(reset)","Drive reset command acknowledged"},
    {102, "O.ht4", "Power module rectifier over temperature"},
    {182, "C.Err", "SmartStick data error"},
    {183, "C.dAt", "SmartStick data does not exist"},
    {185, "C.Acc", "SmartStick read/write fail"},
    {186, "C.rtg", "SmartStick rating mismatch"},
    {189, "O.cL",  "Overload on current loop analog input"},
    {199, "dESt",  "Destination parameter clash"},
    {200, "SL.HF", "Solutions Module hardware fault"},
    {201, "SL.tO", "Solutions Module watchdog timeout"},
    {202, "SL.Er", "Solutions Module error (see Pr 15.50)"},
    {203, "SL.nF", "Solutions Module not installed"},
    {204, "SL.dF", "Solutions Module different type installed"},
    {0,   NULL,    NULL}  /* sentinel */
};

/* Return display name for a trip code. */
static inline const char *sk_trip_name(int code)
{
    const sk_trip_t *t;
    static char buf[16];
    if (code >= 40 && code <= 89) {
        snprintf(buf, sizeof(buf), "t%03d", code);
        return buf;
    }
    if (code >= 220 && code <= 232) {
        snprintf(buf, sizeof(buf), "HF%d", code - 200);
        return buf;
    }
    for (t = sk_trip_table; t->name != NULL; t++) {
        if (t->code == code)
            return t->name;
    }
    snprintf(buf, sizeof(buf), "?(code %d)", code);
    return buf;
}

/* Return description for a trip code. */
static inline const char *sk_trip_desc(int code)
{
    const sk_trip_t *t;
    if (code >= 40 && code <= 89)
        return "User-defined PLC ladder trip";
    if (code >= 220 && code <= 232)
        return "Fatal hardware fault - power cycle required";
    for (t = sk_trip_table; t->name != NULL; t++) {
        if (t->code == code)
            return t->description;
    }
    return "Unknown trip code";
}

#endif /* _cmdrsk_vfd_h */
