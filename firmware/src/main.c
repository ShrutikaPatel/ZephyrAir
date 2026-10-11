#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "bmp390_bridge.h"
#include "dsp_filter.h"

#define BMP390_NODE DT_NODELABEL(bmp390)
static const struct i2c_dt_spec bmp390_i2c = I2C_DT_SPEC_GET(BMP390_NODE);
static struct bmp3_dev bmp3_sensor;

/* Runtime state controlled via Zephyr Shell */
static volatile bool     g_streaming_enabled = true;
static volatile uint32_t g_odr_hz            = 10;

struct raw_sensor_data {
    uint32_t timestamp_ms;
    float    raw_pressure_pa;
    float    raw_temp_c;
};

struct telemetry_data {
    uint32_t timestamp_ms;
    float    pressure_pa;
    float    temp_c;
    float    altitude_m;
};

K_MSGQ_DEFINE(raw_data_msgq, sizeof(struct raw_sensor_data), 10, 4);
K_MSGQ_DEFINE(telemetry_msgq, sizeof(struct telemetry_data), 10, 4);

/* Sensor acquisition thread */
#define SENSOR_THREAD_STACK_SIZE 2048
#define SENSOR_THREAD_PRIORITY   5

void sensor_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    int ret = bmp390_bridge_init(&bmp3_sensor, &bmp390_i2c);
    if (ret != 0) {
        printk("[Sensor Thread] Failed to initialize BMP390 (%d)\n", ret);
        return;
    }

    while (1) {
        if (g_streaming_enabled) {
            struct raw_sensor_data raw;
            raw.timestamp_ms = k_uptime_get_32();

            ret = bmp390_bridge_sample_fetch(&bmp3_sensor,
                                             &raw.raw_pressure_pa,
                                             &raw.raw_temp_c);
            if (ret == 0) {
                k_msgq_put(&raw_data_msgq, &raw, K_NO_WAIT);
            } else {
                printk("[Sensor Thread] Sample fetch failed after %d retries\n",
                       BMP390_I2C_MAX_RETRIES);
            }
        }

        uint32_t hz = g_odr_hz;
        if (hz == 0) {
            hz = 1;
        }
        k_msleep(1000U / hz);
    }
}

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK_SIZE,
                sensor_thread_entry, NULL, NULL, NULL,
                SENSOR_THREAD_PRIORITY, 0, 0);

/* Signal processing thread */
#define PROCESS_THREAD_STACK_SIZE 1024
#define PROCESS_THREAD_PRIORITY   6

void process_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct raw_sensor_data raw;
    struct dsp_iir_filter pressure_filter;

    dsp_iir_init(&pressure_filter, DSP_DEFAULT_IIR_ALPHA);

    while (1) {
        k_msgq_get(&raw_data_msgq, &raw, K_FOREVER);

        float filtered_pressure = dsp_iir_update(&pressure_filter, raw.raw_pressure_pa);

        struct telemetry_data telem = {
            .timestamp_ms = raw.timestamp_ms,
            .pressure_pa  = filtered_pressure,
            .temp_c       = raw.raw_temp_c,
            .altitude_m   = dsp_calculate_altitude(filtered_pressure)
        };

        k_msgq_put(&telemetry_msgq, &telem, K_NO_WAIT);
    }
}

K_THREAD_DEFINE(process_tid, PROCESS_THREAD_STACK_SIZE,
                process_thread_entry, NULL, NULL, NULL,
                PROCESS_THREAD_PRIORITY, 0, 0);

/* UART telemetry output thread */
#define UART_THREAD_STACK_SIZE 1024
#define UART_THREAD_PRIORITY   7

void uart_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct telemetry_data telem;

    while (1) {
        k_msgq_get(&telemetry_msgq, &telem, K_FOREVER);

        printk("P:%.2f,T:%.2f,A:%.2f\n",
               (double)telem.pressure_pa,
               (double)telem.temp_c,
               (double)telem.altitude_m);
    }
}

K_THREAD_DEFINE(uart_tid, UART_THREAD_STACK_SIZE,
                uart_thread_entry, NULL, NULL, NULL,
                UART_THREAD_PRIORITY, 0, 0);

/* Shell command handlers */
static int cmd_sensor_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    g_streaming_enabled = true;
    shell_print(sh, "Sensor streaming started (%u Hz).", g_odr_hz);
    return 0;
}

static int cmd_sensor_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    g_streaming_enabled = false;
    shell_print(sh, "Sensor streaming stopped.");
    return 0;
}

static int cmd_sensor_odr(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: sensor odr <hz> (1..50)");
        return -EINVAL;
    }

    int hz = atoi(argv[1]);
    if (hz < 1 || hz > 50) {
        shell_error(sh, "Invalid ODR '%s'. Expected 1..50 Hz.", argv[1]);
        return -EINVAL;
    }

    g_odr_hz = (uint32_t)hz;
    shell_print(sh, "Sensor ODR set to %u Hz (%u ms).",
                g_odr_hz, 1000U / g_odr_hz);
    return 0;
}

static int cmd_sensor_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct bmp390_diag_stats stats;
    bmp390_bridge_get_stats(&stats);

    shell_print(sh, "BMP390 Status:");
    shell_print(sh, "  Streaming     : %s", g_streaming_enabled ? "ACTIVE" : "STOPPED");
    shell_print(sh, "  Sample Rate   : %u Hz (%u ms)", g_odr_hz, 1000U / g_odr_hz);
    shell_print(sh, "  Chip ID       : 0x%02X", bmp3_sensor.chip_id);
    shell_print(sh, "  Total Samples : %u", stats.total_samples);
    shell_print(sh, "  I2C Retries   : %u", stats.i2c_retries);
    shell_print(sh, "  I2C Failures  : %u", stats.i2c_failures);
    shell_print(sh, "  Bus Health    : %s", stats.in_error_state ? "FAULT" : "OK");
    return 0;
}

static int cmd_sensor_fault(const struct shell *sh, size_t argc, char **argv)
{
#ifdef CONFIG_EMUL
    if (argc < 2) {
        shell_error(sh, "Usage: sensor fault <count>");
        return -EINVAL;
    }

    int count = atoi(argv[1]);
    if (count < 0 || count > 100) {
        shell_error(sh, "Invalid fault count (0..100).");
        return -EINVAL;
    }

    bmp390_emul_inject_faults((uint32_t)count);
    shell_print(sh, "Injected %d I2C fault(s).", count);
    return 0;
#else
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    shell_print(sh, "Fault injection is only supported on native_sim (CONFIG_EMUL).");
    return -ENOTSUP;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor,
    SHELL_CMD(start,  NULL, "Start sensor streaming",                cmd_sensor_start),
    SHELL_CMD(stop,   NULL, "Stop sensor streaming",                 cmd_sensor_stop),
    SHELL_CMD(odr,    NULL, "Set sampling rate in Hz (1..50)",       cmd_sensor_odr),
    SHELL_CMD(status, NULL, "Show sensor and I2C diagnostic stats",  cmd_sensor_status),
    SHELL_CMD(fault,  NULL, "Inject simulated I2C NACK faults",      cmd_sensor_fault),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensor, &sub_sensor, "BMP390 sensor control commands", NULL);

int main(void)
{
    printk("ZephyrAir initialized (BMP390 I2C telemetry node)\n");
    return 0;
}