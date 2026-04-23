/*
 * cmdrsk_vfd.c — LinuxCNC userspace HAL component for the
 * Emerson / Control Techniques Commander SK VFD via RS-485 Modbus RTU.
 *
 * Steve Richardson <steve@tangentaudio.com>, December 2013
 * Adapted from Michael Haberler's vfs11_vfd.c, itself adapted from
 * Steve Padnos' gs2_vfd.c (with modifications from John Thornton).
 *
 * Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.
 * Copyright (C) 2009-2012 Michael Haberler
 * Copyright (C) 2013-2026 Steve Richardson
 *
 * Licensed under the GNU Lesser General Public License, version 2.
 */

#ifndef ULAPI
#error This is intended as a userspace component only.
#endif

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdarg.h>

#include "rtapi.h"
#include "hal.h"
#include <modbus.h>
#include <inifile.h>
#include "cmdrsk_vfd.h"

#ifdef DEBUG
#define DBG(fmt, ...)                                   \
    do {                                                \
        if (param.debug) printf(fmt, ## __VA_ARGS__);   \
    } while (0)
#else
#define DBG(fmt, ...)
#endif

/*
 * Commander SK registers are not contiguous and must be read individually.
 * We prioritise status/frequency reads on every cycle and defer slower
 * registers (load%, RPM, hardware-enable) to every Nth cycle.
 */
#define POLLCYCLES      10      /* slow-register read interval (cycles) */
#define MODBUS_MIN_OK   10      /* consecutive OK transactions before modbus-ok asserts */
#define MAX_RPM         20000   /* cap commanded RPM */

/* HAL shared memory data */
typedef struct {
    /* Output pins — drive status */
    hal_s32_t   *status;                /* raw status word (Pr 10.40) */
    hal_float_t *freq_cmd;              /* frequency command sent to VFD (Hz) */
    hal_float_t *freq_out;              /* actual output frequency (Hz) */
    hal_float_t *RPM;                   /* actual motor RPM */
    hal_float_t *inv_load_pct;          /* inverter load percentage (reserved) */
    hal_float_t *load_current_pct;      /* drive load as % of rated current */
    hal_float_t *max_rpm;               /* max RPM from VFD EEPROM (Pr 1.06) */
    hal_float_t *min_rpm;               /* min RPM from VFD EEPROM (Pr 1.07) */
    hal_s32_t   *trip_code;             /* most recent trip code (Pr 10.20) */
    hal_s32_t   *alarm_code;            /* alias for trip_code */
    hal_bit_t   *at_speed;              /* ST_AT_SET_SPEED from status word */
    hal_bit_t   *is_stopped;            /* ST_ZERO_SPEED from status word */
    hal_bit_t   *modbus_ok;             /* asserts after MODBUS_MIN_OK good txns */
    hal_bit_t   *drive_ok;              /* Pr 10.01 — not tripped */
    hal_bit_t   *drive_active;          /* Pr 10.02 — inverter output is live */
    hal_bit_t   *fault;                 /* inverse of drive_ok */
    hal_bit_t   *spindle_running_rev;   /* Pr 10.14 — actual run direction */
    hal_bit_t   *hardware_enable;       /* Pr 6.29 — hardware enable terminal.
                                         * Goes low during power drawbar use;
                                         * this is NORMAL.  Do NOT wire to e-stop. */
    hal_bit_t   *drive_enable;          /* Pr 6.15 — software drive enable */
    hal_float_t *upper_limit_hz;        /* VFD max output frequency (Hz) */
    hal_float_t *lower_limit_hz;        /* VFD min output frequency (Hz) */
    hal_s32_t   *errorcount;            /* cumulative failed Modbus transaction count */

    /* Input pins — commands from LinuxCNC */
    hal_float_t *speed_command;         /* commanded spindle speed (RPM) */
    hal_bit_t   *spindle_on;            /* spindle run command */
    hal_bit_t   *spindle_fwd;           /* commanded forward */
    hal_bit_t   *spindle_rev;           /* commanded reverse */
    hal_bit_t   *enabled;               /* machine-on gate; skip Modbus when false */
    hal_bit_t   *fault_reset;           /* rising edge → reset via Pr 10.38=100 */
    hal_bit_t   *max_speed;             /* skip slow registers for max throughput */

    /* HAL parameters (settable at runtime via halcmd setp) */
    hal_float_t looptime;               /* poll loop sleep (seconds) */
    hal_float_t speed_tolerance;        /* at-speed tolerance (fraction) */
    hal_float_t motor_rated_hz;         /* motor nameplate frequency */
    hal_float_t motor_rated_rpm;        /* motor nameplate RPM */
    hal_float_t rpm_limit;              /* do-not-exceed RPM */
} haldata_t;

/* Configuration and runtime state */
typedef struct params {
    char        *modname;
    int         modbus_debug;
    int         debug;
    int         slave;
    int         pollcycles;
    char        *device;
    int         baud;
    int         bits;
    char        parity;
    int         stopbits;
    int         rts_mode;
    int         rts_delay_us;           /* µs delay after RTS toggle */
    int         serial_mode;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    char        *progname;
    char        *section;
    FILE        *fp;
    char        *inifile;
    int         reconnect_delay;
    int         startup_delay;          /* seconds to wait after enable */
    modbus_t    *ctx;
    haldata_t   *haldata;
    int         hal_comp_id;
    int         read_initial_done;
    int         modbus_ok;
    uint16_t    failed_reg;             /* register of last failed transaction */
    int         last_errno;
    int         report_device;
    int         old_fault_reset;        /* previous fault-reset for edge detect */
    int         was_enabled;            /* previous enabled for transition detect */
} params_type, *param_pointer;

/* Default options — overridden by INI file or command line */
static params_type param = {
    .modname            = NULL,
    .modbus_debug       = 0,
    .debug              = 0,
    .slave              = 1,
    .pollcycles         = POLLCYCLES,
    .device             = "/dev/ttyS1",
    .baud               = 19200,
    .bits               = 8,
    .parity             = 'N',
    .stopbits           = 2,
    .serial_mode        = -1,
    .rts_mode           = -1,
    .rts_delay_us       = -1,
    .response_timeout   = { .tv_sec = 0, .tv_usec = 500000 },
    .byte_timeout       = { .tv_sec = 0, .tv_usec = 500000 },
    .progname           = "cmdrsk_vfd",
    .section            = "CMDRSK",
    .fp                 = NULL,
    .inifile            = NULL,
    .reconnect_delay    = 1,
    .startup_delay      = 3,
    .ctx                = NULL,
    .haldata            = NULL,
    .hal_comp_id        = -1,
    .read_initial_done  = 0,
    .modbus_ok          = 0,
    .failed_reg         = 0,
    .last_errno         = 0,
    .report_device      = 0,
    .old_fault_reset    = 0,
    .was_enabled        = 0,
};

static int connection_state;
enum connstate { NOT_CONNECTED, OPENING, CONNECTING, CONNECTED, RECOVER, DONE };

static char *option_string = "dhrmn:S:I:";
static struct option long_options[] = {
    {"debug",         no_argument,       0, 'd'},
    {"help",          no_argument,       0, 'h'},
    {"modbus-debug",  no_argument,       0, 'm'},
    {"report-device", no_argument,       0, 'r'},
    {"ini",           required_argument, 0, 'I'},
    {"section",       required_argument, 0, 'S'},
    {"name",          required_argument, 0, 'n'},
    {0, 0, 0, 0}
};

static void windup(param_pointer p)
{
    if (p->haldata && *(p->haldata->errorcount)) {
        fprintf(stderr, "%s: %d modbus errors\n",
                p->progname, *(p->haldata->errorcount));
        fprintf(stderr, "%s: last command register: 0x%.4x\n",
                p->progname, p->failed_reg);
        fprintf(stderr, "%s: last error: %s\n",
                p->progname, modbus_strerror(p->last_errno));
    }
    if (p->hal_comp_id >= 0)
        hal_exit(p->hal_comp_id);
    if (p->ctx)
        modbus_close(p->ctx);
}

static void toggle_modbus_debug(int sig)
{
    (void)sig;
    param.modbus_debug = !param.modbus_debug;
    modbus_set_debug(param.ctx, param.modbus_debug);
}

static void toggle_debug(int sig)
{
    (void)sig;
    param.debug = !param.debug;
}

static void quit(int sig)
{
    (void)sig;
    if (param.debug)
        fprintf(stderr, "quit(connection_state=%d)\n", connection_state);

    switch (connection_state) {
    case CONNECTING:
        /* modbus_connect() was interrupted — won't return to main loop */
        windup(&param);
        exit(0);
        break;
    default:
        connection_state = DONE;
        break;
    }
}

/* ------------------------------------------------------------------ */
/* INI file parsing                                                    */
/* ------------------------------------------------------------------ */

enum kwdresult { NAME_NOT_FOUND, KEYWORD_INVALID, KEYWORD_FOUND };
#define MAX_KWD 10

static int findkwd(param_pointer p, const char *name, int *result,
                   const char *keyword, int value, ...)
{
    const char *word;
    va_list ap;
    const char *kwds[MAX_KWD], **s;
    int nargs = 0;

    if ((word = iniFind(p->fp, name, p->section)) == NULL)
        return NAME_NOT_FOUND;

    kwds[nargs++] = keyword;
    va_start(ap, value);

    while (keyword != NULL) {
        if (!strcasecmp(word, keyword)) {
            *result = value;
            va_end(ap);
            return KEYWORD_FOUND;
        }
        keyword = va_arg(ap, const char *);
        kwds[nargs++] = keyword;
        if (keyword)
            value = va_arg(ap, int);
    }

    fprintf(stderr, "%s: %s:[%s]%s: found '%s' - not one of: ",
            p->progname, p->inifile, p->section, name, word);
    for (s = kwds; *s; s++)
        fprintf(stderr, "%s ", *s);
    fprintf(stderr, "\n");
    va_end(ap);
    return KEYWORD_INVALID;
}

static int read_ini(param_pointer p)
{
    const char *s;
    double f;
    int value;

    if ((p->fp = fopen(p->inifile, "r")) == NULL) {
        fprintf(stderr, "%s: can't open INI file '%s'\n",
                p->progname, p->inifile);
        return -1;
    }

    if (!p->debug)
        iniFindInt(p->fp, "DEBUG", p->section, &p->debug);
    if (!p->modbus_debug)
        iniFindInt(p->fp, "MODBUS_DEBUG", p->section, &p->modbus_debug);

    iniFindInt(p->fp, "BITS", p->section, &p->bits);
    iniFindInt(p->fp, "BAUD", p->section, &p->baud);
    iniFindInt(p->fp, "STOPBITS", p->section, &p->stopbits);
    iniFindInt(p->fp, "TARGET", p->section, &p->slave);
    iniFindInt(p->fp, "POLLCYCLES", p->section, &p->pollcycles);
    iniFindInt(p->fp, "RECONNECT_DELAY", p->section, &p->reconnect_delay);
    iniFindInt(p->fp, "STARTUP_DELAY", p->section, &p->startup_delay);

    if ((s = iniFind(p->fp, "DEVICE", p->section)))
        p->device = strdup(s);

    if (iniFindDouble(p->fp, "RESPONSE_TIMEOUT", p->section, &f)) {
        p->response_timeout.tv_sec = (int)f;
        p->response_timeout.tv_usec = (f - p->response_timeout.tv_sec) * 1000000;
    }
    if (iniFindDouble(p->fp, "BYTE_TIMEOUT", p->section, &f)) {
        p->byte_timeout.tv_sec = (int)f;
        p->byte_timeout.tv_usec = (f - p->byte_timeout.tv_sec) * 1000000;
    }

    value = p->parity;
    if (findkwd(p, "PARITY", &value,
                "even", 'E',
                "odd",  'O',
                "none", 'N',
                NULL) == KEYWORD_INVALID)
        return -1;
    p->parity = value;

#ifdef MODBUS_RTU_RTS_UP
    if (findkwd(p, "RTS_MODE", &p->rts_mode,
                "up",   MODBUS_RTU_RTS_UP,
                "down", MODBUS_RTU_RTS_DOWN,
                "none", MODBUS_RTU_RTS_NONE,
                NULL) == KEYWORD_INVALID)
        return -1;
#else
    if (iniFind(p->fp, "RTS_MODE", p->section) != NULL) {
        fprintf(stderr, "%s: warning - RTS_MODE requires libmodbus >= 3.1 "
                "(installed: %s)\n",
                p->progname, LIBMODBUS_VERSION_STRING);
    }
#endif

    iniFindInt(p->fp, "RTS_DELAY_US", p->section, &p->rts_delay_us);

    return 0;
}

static void usage(int argc, char **argv)
{
    (void)argc;
    printf("Usage:  %s [options]\n"
           "  Userspace HAL component for Commander SK VFD (RS-485 Modbus RTU)\n"
           "  Typically loaded via: loadusr cmdrsk_vfd [options]\n"
           "\n"
           "Options:\n"
           "  -I, --ini <inifile>      INI file (default: $INI_FILE_NAME)\n"
           "  -S, --section <name>     INI section (default: CMDRSK)\n"
           "  -n, --name <name>        HAL component name (default: cmdrsk_vfd)\n"
           "  -d, --debug              Enable debug output (toggle: USR1)\n"
           "  -m, --modbus-debug       Enable Modbus frame dumps (toggle: USR2)\n"
           "  -r, --report-device      Report VFD version info at startup\n"
           "  -h, --help               Show this help\n",
           argv[0]);
}

/* ------------------------------------------------------------------ */
/* Modbus register access macros                                       */
/* ------------------------------------------------------------------ */

/* CT Modbus addressing: wire-level register = param - 1 */
#define SETPARAM(param, from)                                       \
    do {                                                            \
        curr_param = param - 1;                                     \
        if (modbus_write_register(ctx, param - 1, from) < 0)        \
            goto failed;                                            \
    } while (0)

#define GETPARAM(param, into)                                       \
    do {                                                            \
        curr_param = param - 1;                                     \
        if (modbus_read_registers(ctx, param - 1, 1, into) != 1)    \
            goto failed;                                            \
    } while (0)

/* ------------------------------------------------------------------ */
/* Modbus write — speed reference, control word, fault reset           */
/* ------------------------------------------------------------------ */

static int write_data(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    static hal_float_t last_freq_cmd = -1.0;
    static uint16_t last_control_reg = 0xFFFF;

    int freq_reg;
    uint16_t control_reg, curr_param;

    /*
     * Fault reset — rising edge of fault-reset pin.
     * Writes 100 to Pr 10.38 per Commander SK manual section 4.1, method 4.
     */
    if (*(haldata->fault_reset) && !p->old_fault_reset) {
        DBG("write_data: fault-reset rising edge — sending drive reset\n");
        SETPARAM(PR_USER_TRIP, 100);
        last_freq_cmd = -1.0;
        last_control_reg = 0xFFFF;
    }
    p->old_fault_reset = *(haldata->fault_reset);

    /* Speed reference */
    *(haldata->freq_cmd) = (*(haldata->speed_command) *
                            *(haldata->upper_limit_hz)) / *(haldata->max_rpm);

    if (*(haldata->freq_cmd) != last_freq_cmd) {
        freq_reg = (int)fabs(*(haldata->freq_cmd) * 10.0);
        SETPARAM(PR_PRECISION_REF_COARSE, freq_reg);
        usleep(10000);
        SETPARAM(PR_REFERENCE_SELECTOR, SEL_REFERENCE_PRECISION);
        usleep(10000);
        DBG("write_data: freq_cmd=%f, freq_reg=%d\n",
            *(haldata->freq_cmd), freq_reg);
    }
    last_freq_cmd = *(haldata->freq_cmd);

    /* Control word */
    control_reg = CTL_DRIVE_ENABLE | CTL_AUTO_MANUAL;
    if (*(haldata->spindle_on) && *(haldata->spindle_fwd) &&
        !*(haldata->spindle_rev)) {
        control_reg |= CTL_N_STOP | CTL_RUN_FORWARD;
    } else if (*(haldata->spindle_on) && *(haldata->spindle_rev) &&
               !*(haldata->spindle_fwd)) {
        control_reg |= CTL_N_STOP | CTL_RUN_REVERSE;
    } else {
        control_reg &= ~CTL_N_STOP;
    }

    if (control_reg != last_control_reg) {
        SETPARAM(PR_CONTROL_WORD, control_reg);
        usleep(10000);
        SETPARAM(PR_CONTROL_WORD_ENABLE, 1);
        usleep(10000);
        DBG("write_data: control_reg=%4.4X\n", control_reg);
    }
    last_control_reg = control_reg;

    return 0;

failed:
    p->failed_reg = curr_param;
    (*haldata->errorcount)++;
    p->last_errno = errno;
    return errno;
}

/* ------------------------------------------------------------------ */
/* Modbus read — status word, frequency, slow registers                */
/* ------------------------------------------------------------------ */

static int read_initial(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    uint16_t curr_param;
    uint16_t sw_version, sw_subversion, dsp_version;
    uint16_t rated_freq, rated_rpm;
    uint16_t max_freq, min_freq;

    GETPARAM(PR_MAX_SET_SPEED, &max_freq);
    GETPARAM(PR_MIN_SET_SPEED, &min_freq);
    GETPARAM(PR_RATED_FREQUENCY, &rated_freq);
    GETPARAM(PR_RATED_RPM, &rated_rpm);

    haldata->motor_rated_hz  = rated_freq / 10.0;
    haldata->motor_rated_rpm = rated_rpm / 1.0;

    *(haldata->upper_limit_hz) = max_freq / 10.0;
    *(haldata->max_rpm) = *(haldata->upper_limit_hz) *
                          haldata->motor_rated_rpm / haldata->motor_rated_hz;

    *(haldata->lower_limit_hz) = min_freq / 10.0;
    *(haldata->min_rpm) = *(haldata->lower_limit_hz) *
                          haldata->motor_rated_rpm / haldata->motor_rated_hz;

    if (p->report_device) {
        GETPARAM(PR_SW_VERSION, &sw_version);
        GETPARAM(PR_SW_SUBVERSION, &sw_subversion);
        GETPARAM(PR_DSP_VERSION, &dsp_version);
        fprintf(stderr, "%s: sw=%d.%d  dsp=%d\n",
                p->progname, sw_version, sw_subversion, dsp_version);
    }
    return 0;

failed:
    p->failed_reg = curr_param;
    p->last_errno = errno;
    (*haldata->errorcount)++;
    if (p->debug)
        fprintf(stderr, "%s: read_initial: modbus_read_registers(0x%4.4x): %s\n",
                p->progname, curr_param, modbus_strerror(errno));
    return p->last_errno;
}

static int read_data(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    int retval;
    uint16_t curr_param, val, status_word, freq;
    static int pollcount = 0;

    if (!p->read_initial_done) {
        if ((retval = read_initial(ctx, haldata, p)))
            return retval;
        p->read_initial_done = 1;
    }

    GETPARAM(PR_STATUS_WORD, &status_word);
    *(haldata->status) = status_word;

    GETPARAM(PR_MOTOR_FREQUENCY, &freq);
    /* Commander SK returns Pr 5.01 as signed int16 (negative = reverse) */
    *(haldata->freq_out) = abs((int16_t)freq) / 10.0;

    /* Decode status word bits */
    *(haldata->drive_ok)           = (status_word & ST_DRIVE_OK)         ? 1 : 0;
    *(haldata->drive_active)       = (status_word & ST_DRIVE_ACTIVE)     ? 1 : 0;
    *(haldata->is_stopped)         = (status_word & ST_ZERO_SPEED)       ? 1 : 0;
    *(haldata->at_speed)           = (status_word & ST_AT_SET_SPEED)     ? 1 : 0;
    *(haldata->spindle_running_rev)= (status_word & ST_DIRECTION_RUNNING)? 1 : 0;
    *(haldata->fault) = *(haldata->drive_ok) ? 0 : 1;

    /* Trip detection and logging */
    if (!(status_word & ST_DRIVE_OK)) {
        uint16_t trip_codes[10];
        int i;
        for (i = 0; i < 10; i++)
            GETPARAM(PR_LAST_TRIP + i, &trip_codes[i]);

        *(haldata->alarm_code) = trip_codes[0];
        if (p->debug || trip_codes[0] != *(haldata->trip_code)) {
            fprintf(stderr, "%s: FAULT trip=%s (%s)\n",
                    p->progname,
                    sk_trip_name(trip_codes[0]),
                    sk_trip_desc(trip_codes[0]));
        }
        *(haldata->trip_code) = trip_codes[0];
    }

    /* Slow registers — every Nth cycle, not in max-speed mode */
    if ((pollcount == 0) && !(*haldata->max_speed)) {
        GETPARAM(PR_PERCENT_LOAD, &val);
        *(haldata->load_current_pct) = val / 10.0;

        GETPARAM(PR_MOTOR_SPEED, &val);
        /* Commander SK returns Pr 5.04 as signed int16 (negative = reverse) */
        *(haldata->RPM) = abs((int16_t)val) * 1.0;

        GETPARAM(PR_HARDWARE_ENABLE, &val);
        *(haldata->hardware_enable) = val ? 1 : 0;

        GETPARAM(PR_DRIVE_ENABLE, &val);
        *(haldata->drive_enable) = val ? 1 : 0;
    }

    pollcount++;
    if (pollcount >= p->pollcycles)
        pollcount = 0;

    p->last_errno = retval = 0;
    return 0;

failed:
    p->failed_reg = curr_param;
    p->last_errno = errno;
    (*haldata->errorcount)++;
    if (p->debug)
        fprintf(stderr, "%s: read_data: modbus_read_registers(0x%4.4x): %s\n",
                p->progname, curr_param, modbus_strerror(errno));
    return p->last_errno;
}

/* ------------------------------------------------------------------ */
/* HAL pin registration                                                */
/* ------------------------------------------------------------------ */

#define PIN(x)                          \
    do {                                \
        status = (x);                   \
        if (status != 0) return status; \
    } while (0)

static int hal_setup(int id, haldata_t *h, const char *name)
{
    int status;

    /* Output pins */
    PIN(hal_pin_s32_newf(HAL_OUT,  &(h->alarm_code),        id, "%s.alarm-code", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->at_speed),          id, "%s.at-speed", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->load_current_pct),  id, "%s.current-load-percentage", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->freq_cmd),          id, "%s.frequency-command", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->freq_out),          id, "%s.frequency-out", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->inv_load_pct),      id, "%s.inverter-load-percentage", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->is_stopped),        id, "%s.is-stopped", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->max_rpm),           id, "%s.max-rpm", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->min_rpm),           id, "%s.min-rpm", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->modbus_ok),         id, "%s.modbus-ok", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->RPM),               id, "%s.motor-RPM", name));
    PIN(hal_pin_s32_newf(HAL_OUT,  &(h->status),            id, "%s.status", name));
    PIN(hal_pin_s32_newf(HAL_OUT,  &(h->trip_code),         id, "%s.trip-code", name));
    PIN(hal_pin_s32_newf(HAL_OUT,  &(h->errorcount),        id, "%s.error-count", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->upper_limit_hz),    id, "%s.frequency-limit-high", name));
    PIN(hal_pin_float_newf(HAL_OUT,&(h->lower_limit_hz),    id, "%s.frequency-limit-low", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->fault),             id, "%s.fault", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->drive_ok),          id, "%s.drive-ok", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->drive_active),      id, "%s.drive-active", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->spindle_running_rev),id,"%s.spindle-running-rev", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->hardware_enable),   id, "%s.hardware-enable", name));
    PIN(hal_pin_bit_newf(HAL_OUT,  &(h->drive_enable),      id, "%s.drive-enable", name));

    /* Input pins */
    PIN(hal_pin_float_newf(HAL_IN, &(h->speed_command),     id, "%s.speed-command", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->spindle_fwd),       id, "%s.spindle-fwd", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->spindle_on),        id, "%s.spindle-on", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->spindle_rev),       id, "%s.spindle-rev", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->enabled),           id, "%s.enabled", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->fault_reset),       id, "%s.fault-reset", name));
    PIN(hal_pin_bit_newf(HAL_IN,   &(h->max_speed),         id, "%s.max-speed", name));

    /* HAL parameters */
    PIN(hal_param_float_newf(HAL_RW, &(h->looptime),        id, "%s.loop-time", name));
    PIN(hal_param_float_newf(HAL_RW, &(h->motor_rated_hz),  id, "%s.rated-hz", name));
    PIN(hal_param_float_newf(HAL_RW, &(h->motor_rated_rpm), id, "%s.rated-rpm", name));
    PIN(hal_param_float_newf(HAL_RW, &(h->rpm_limit),       id, "%s.rpm-limit", name));
    PIN(hal_param_float_newf(HAL_RW, &(h->speed_tolerance), id, "%s.tolerance", name));

    return 0;
}
#undef PIN

/* ------------------------------------------------------------------ */
/* Pin initialisation                                                  */
/* ------------------------------------------------------------------ */

static int set_defaults(param_pointer p)
{
    haldata_t *h = p->haldata;

    /* Output pins */
    *(h->status)            = 0;
    *(h->freq_cmd)          = 0;
    *(h->freq_out)          = 0;
    *(h->RPM)               = 0;
    *(h->inv_load_pct)      = 0;
    *(h->load_current_pct)  = 0;
    *(h->upper_limit_hz)    = 0;
    *(h->lower_limit_hz)    = 0;
    *(h->trip_code)         = 0;
    *(h->alarm_code)        = 0;
    *(h->at_speed)          = 0;
    *(h->is_stopped)        = 0;
    *(h->modbus_ok)         = 0;
    *(h->errorcount)        = 0;
    *(h->fault)             = 0;
    *(h->drive_ok)          = 0;
    *(h->drive_active)      = 0;
    *(h->spindle_running_rev)= 0;
    *(h->hardware_enable)   = 0;
    *(h->drive_enable)      = 0;

    /* Input pins */
    *(h->speed_command)     = 0;
    *(h->spindle_on)        = 0;
    *(h->spindle_fwd)       = 1;
    *(h->spindle_rev)       = 0;
    *(h->enabled)           = 0;
    *(h->fault_reset)       = 0;
    *(h->max_speed)         = 0;

    /* HAL parameters */
    h->looptime         = 0.010;
    h->speed_tolerance  = 0.01;
    h->rpm_limit        = MAX_RPM;

    p->failed_reg = 0;
    return 0;
}

/* Zero all output pins — called when enabled goes false (VFD unpowered). */
static void zero_outputs(haldata_t *h)
{
    *(h->status)            = 0;
    *(h->freq_cmd)          = 0;
    *(h->freq_out)          = 0;
    *(h->RPM)               = 0;
    *(h->inv_load_pct)      = 0;
    *(h->load_current_pct)  = 0;
    *(h->upper_limit_hz)    = 0;
    *(h->lower_limit_hz)    = 0;
    *(h->trip_code)         = 0;
    *(h->alarm_code)        = 0;
    *(h->at_speed)          = 0;
    *(h->is_stopped)        = 0;
    *(h->modbus_ok)         = 0;
    *(h->fault)             = 0;
    *(h->drive_ok)          = 0;
    *(h->drive_active)      = 0;
    *(h->spindle_running_rev)= 0;
    *(h->hardware_enable)   = 0;
    *(h->drive_enable)      = 0;
}
/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    struct timespec loop_timespec, remaining;
    int opt;
    param_pointer p = &param;
    int retval = -1;

    p->progname = argv[0];
    connection_state = NOT_CONNECTED;
    p->inifile = getenv("INI_FILE_NAME");

    while ((opt = getopt_long(argc, argv, option_string,
                              long_options, NULL)) != -1) {
        switch (opt) {
        case 'n': p->modname       = strdup(optarg); break;
        case 'm': p->modbus_debug  = 1;              break;
        case 'd': p->debug         = 1;              break;
        case 'S': p->section       = optarg;          break;
        case 'I': p->inifile       = optarg;          break;
        case 'r': p->report_device = 1;              break;
        case 'h':
        default:
            usage(argc, argv);
            exit(0);
        }
    }

    if (p->inifile) {
        if (read_ini(p))
            goto finish;
        if (!p->modname)
            p->modname = "cmdrsk_vfd";
    } else {
        fprintf(stderr, "%s: ERROR: no INI file — use '--ini file' or "
                "set INI_FILE_NAME\n", p->progname);
        goto finish;
    }

    signal(SIGINT,  quit);
    signal(SIGTERM, quit);
    signal(SIGUSR1, toggle_debug);
    signal(SIGUSR2, toggle_modbus_debug);

    /* Create HAL component */
    p->hal_comp_id = hal_init(p->modname);
    if (p->hal_comp_id < 0 || connection_state == DONE) {
        fprintf(stderr, "%s: ERROR: hal_init(%s) failed: %d\n",
                p->progname, p->modname, p->hal_comp_id);
        retval = p->hal_comp_id;
        goto finish;
    }

    /* Allocate HAL shared memory */
    p->haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if (p->haldata == NULL || connection_state == DONE) {
        fprintf(stderr, "%s: ERROR: unable to allocate shared memory\n",
                p->modname);
        retval = -1;
        goto finish;
    }

    if (hal_setup(p->hal_comp_id, p->haldata, p->modname))
        goto finish;

    set_defaults(p);
    hal_ready(p->hal_comp_id);
    DBG("%s: HAL ready, using libmodbus %s\n",
        p->progname, LIBMODBUS_VERSION_STRING);

    /* Open serial port */
    connection_state = OPENING;
    p->ctx = modbus_new_rtu(p->device, p->baud, p->parity,
                            p->bits, p->stopbits);
    if (p->ctx == NULL) {
        fprintf(stderr, "%s: ERROR: modbus_new_rtu(%s): %s\n",
                p->progname, p->device, modbus_strerror(errno));
        goto finish;
    }

    if (modbus_set_slave(p->ctx, p->slave) < 0) {
        fprintf(stderr, "%s: ERROR: invalid slave number: %d\n",
                p->modname, p->slave);
        goto finish;
    }

    if (modbus_connect(p->ctx) != 0) {
        fprintf(stderr, "%s: ERROR: couldn't open serial device: %s\n",
                p->modname, modbus_strerror(errno));
        goto finish;
    }

    if (p->serial_mode != -1 &&
        modbus_rtu_set_serial_mode(p->ctx, p->serial_mode) < 0) {
        fprintf(stderr, "%s: ERROR: modbus_rtu_set_serial_mode(%d): %s\n",
                p->modname, p->serial_mode, modbus_strerror(errno));
        goto finish;
    }

#ifdef MODBUS_RTU_RTS_UP
    if (p->rts_mode != -1 &&
        modbus_rtu_set_rts(p->ctx, p->rts_mode) < 0) {
        fprintf(stderr, "%s: ERROR: modbus_rtu_set_rts(%d): %s\n",
                p->modname, p->rts_mode, modbus_strerror(errno));
        goto finish;
    }
#endif

    if (p->rts_delay_us >= 0) {
        modbus_rtu_set_rts_delay(p->ctx, p->rts_delay_us);
        DBG("%s: RTS delay set to %d µs\n", p->progname, p->rts_delay_us);
    }

    modbus_set_debug(p->ctx, p->modbus_debug);

    /* Apply configured timeouts to the modbus context.
     * Without this, libmodbus uses its compiled-in defaults which may
     * not be appropriate for USB-to-RS485 adapters. */
#if LIBMODBUS_VERSION_CHECK(3,1,2)
    modbus_set_response_timeout(p->ctx,
                                p->response_timeout.tv_sec,
                                p->response_timeout.tv_usec);
    modbus_set_byte_timeout(p->ctx,
                            p->byte_timeout.tv_sec,
                            p->byte_timeout.tv_usec);
#else
    modbus_set_response_timeout(p->ctx, &p->response_timeout);
    modbus_set_byte_timeout(p->ctx, &p->byte_timeout);
#endif
    DBG("%s: serial port %s connected\n", p->progname, p->device);
    DBG("%s: response timeout: %ld.%06lds, byte timeout: %ld.%06lds\n",
        p->progname,
        (long)p->response_timeout.tv_sec, (long)p->response_timeout.tv_usec,
        (long)p->byte_timeout.tv_sec, (long)p->byte_timeout.tv_usec);

    /* ---------------------------------------------------------- */
    /* Main poll loop                                              */
    /* ---------------------------------------------------------- */

    connection_state = CONNECTED;
    while (connection_state != DONE) {

        while (connection_state == CONNECTED) {

            /* Enabled-pin awareness: when the VFD is unpowered, skip
             * all Modbus traffic and zero output pins. */
            if (!*(p->haldata->enabled)) {
                if (p->was_enabled) {
                    DBG("%s: enabled->false: suspending Modbus\n",
                        p->progname);
                    zero_outputs(p->haldata);
                    p->read_initial_done = 0;
                    p->modbus_ok = 0;
                }
                p->was_enabled = 0;

                if (p->haldata->looptime < 0.001) p->haldata->looptime = 0.001;
                if (p->haldata->looptime > 2.0)   p->haldata->looptime = 2.0;
                loop_timespec.tv_sec  = (time_t)(p->haldata->looptime);
                loop_timespec.tv_nsec = (long)((p->haldata->looptime -
                                         loop_timespec.tv_sec) * 1000000000l);
                nanosleep(&loop_timespec, &remaining);
                continue;
            }

            /* Transition: disabled → enabled */
            if (!p->was_enabled) {
                DBG("%s: enabled->true: resuming Modbus\n", p->progname);
                if (p->startup_delay > 0) {
                    DBG("%s: waiting %ds for VFD boot\n",
                        p->progname, p->startup_delay);
                    sleep(p->startup_delay);
                }
                p->was_enabled = 1;
                p->read_initial_done = 0;
            }

            /* Read VFD status */
            if ((retval = read_data(p->ctx, p->haldata, p))) {
                p->modbus_ok = 0;
                /* Flush stale bytes to prevent corrupting the next
                 * transaction after a timeout or CRC error. */
                modbus_flush(p->ctx);
            } else {
                p->modbus_ok++;
            }

            *(p->haldata->modbus_ok) = (p->modbus_ok > MODBUS_MIN_OK) ? 1 : 0;

            /* Write speed/control to VFD */
            if ((retval = write_data(p->ctx, p->haldata, p))) {
                p->modbus_ok = 0;
                if (retval == EBADF || retval == ECONNRESET ||
                    retval == EPIPE) {
                    connection_state = RECOVER;
                }
            } else {
                p->modbus_ok++;
            }

            *(p->haldata->modbus_ok) = (p->modbus_ok > MODBUS_MIN_OK) ? 1 : 0;

            /* Rate-limit the poll loop */
            if (p->haldata->looptime < 0.001) p->haldata->looptime = 0.001;
            if (p->haldata->looptime > 2.0)   p->haldata->looptime = 2.0;
            loop_timespec.tv_sec  = (time_t)(p->haldata->looptime);
            loop_timespec.tv_nsec = (long)((p->haldata->looptime -
                                     loop_timespec.tv_sec) * 1000000000l);
            if (!p->haldata->max_speed)
                nanosleep(&loop_timespec, &remaining);
        }

        /* Connection state machine */
        switch (connection_state) {
        case DONE:
            modbus_flush(p->ctx);
            break;

        case RECOVER:
            DBG("%s: recovering connection\n", p->progname);
            set_defaults(p);
            p->read_initial_done = 0;
            modbus_flush(p->ctx);
            modbus_close(p->ctx);
            while (connection_state != CONNECTED &&
                   connection_state != DONE) {
                sleep(p->reconnect_delay);
                if (!modbus_connect(p->ctx)) {
                    connection_state = CONNECTED;
                    DBG("%s: reconnected\n", p->progname);
                } else {
                    fprintf(stderr, "%s: recovery: modbus_connect(): %s\n",
                            p->progname, modbus_strerror(errno));
                }
            }
            break;

        default:
            break;
        }
    }
    retval = 0;

finish:
    windup(p);
    return retval;
}
