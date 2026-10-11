#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include "bmp390_bridge.h"
#include "dsp_filter.h"

/* ====================================================================
 * Devicetree & Bosch BMP3 Device Handle
 * ==================================================================== */
#define BMP390_NODE DT_NODELABEL(bmp390)
static const struct i2c_dt_spec bmp390_i2c = I2C_DT_SPEC_GET(BMP390_NODE);
static struct bmp3_dev bmp3_sensor;

/* ====================================================================
 * Data Structures & Message Queues
 * ==================================================================== */

/* Raw data produced by the sensor thread */
struct raw_sensor_data {
    uint32_t timestamp_ms;
    float    raw_pressure_pa;
    float    raw_temp_c;
};

/* Processed data ready for UART transmission */
struct telemetry_data {
    uint32_t timestamp_ms;
    float    pressure_pa;
    float    temp_c;
    float    altitude_m;
};

/* Queue 1: sensor_thread -> process_thread (holds up to 10 items) */
K_MSGQ_DEFINE(raw_data_msgq, sizeof(struct raw_sensor_data), 10, 4);

/* Queue 2: process_thread -> uart_thread (holds up to 10 items) */
K_MSGQ_DEFINE(telemetry_msgq, sizeof(struct telemetry_data), 10, 4);

/* ====================================================================
 * Thread 1: Sensor Acquisition Thread (Priority: 5)
 * ==================================================================== */
#define SENSOR_THREAD_STACK_SIZE 2048
#define SENSOR_THREAD_PRIORITY   5

void sensor_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printk("[Sensor Thread] Started\n");

    /*
     * Milestone M6: Initialize Bosch BMP390 via Official Bosch API Bridge
     */
    int ret = bmp390_bridge_init(&bmp3_sensor, &bmp390_i2c);
    if (ret != 0) {
        printk("[Sensor Thread] FATAL: Could not initialize BMP390 (%d)\n", ret);
        return;
    }

    printk("[Sensor Thread] Entering 10 Hz Bosch API acquisition loop...\n");

    while (1) {
        struct raw_sensor_data raw;
        raw.timestamp_ms = k_uptime_get_32();

        /* Fetch compensated pressure (Pa) and temperature (°C) via Bosch API */
        ret = bmp390_bridge_sample_fetch(&bmp3_sensor,
                                         &raw.raw_pressure_pa,
                                         &raw.raw_temp_c);
        if (ret == 0) {
            k_msgq_put(&raw_data_msgq, &raw, K_NO_WAIT);
        } else {
            printk("[Sensor Thread] Warning: sample fetch failed (%d)\n", ret);
        }

        /* 10 Hz sampling rate (100 ms sleep) */
        k_msleep(100);
    }
}

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK_SIZE,
                sensor_thread_entry, NULL, NULL, NULL,
                SENSOR_THREAD_PRIORITY, 0, 0);

/* ====================================================================
 * Thread 2: Data Processing Thread (Priority: 6)
 * ==================================================================== */
#define PROCESS_THREAD_STACK_SIZE 1024
#define PROCESS_THREAD_PRIORITY   6

void process_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printk("[Process Thread] Started\n");

    struct raw_sensor_data raw;
    struct dsp_iir_filter pressure_filter;

    /* Initialize 1st-order IIR low-pass filter (alpha = 0.15) */
    dsp_iir_init(&pressure_filter, DSP_DEFAULT_IIR_ALPHA);

    while (1) {
        /* Block until a new raw sample arrives */
        k_msgq_get(&raw_data_msgq, &raw, K_FOREVER);

        float filtered_pressure = dsp_iir_update(&pressure_filter, raw.raw_pressure_pa);

        struct telemetry_data telem = {
            .timestamp_ms = raw.timestamp_ms,
            .pressure_pa  = filtered_pressure,
            .temp_c       = raw.raw_temp_c,
            .altitude_m   = dsp_calculate_altitude(filtered_pressure)
        };

        /* Send to Queue 2 for UART transmission */
        k_msgq_put(&telemetry_msgq, &telem, K_NO_WAIT);
    }
}

K_THREAD_DEFINE(process_tid, PROCESS_THREAD_STACK_SIZE,
                process_thread_entry, NULL, NULL, NULL,
                PROCESS_THREAD_PRIORITY, 0, 0);

/* ====================================================================
 * Thread 3: UART Telemetry Thread (Priority: 7)
 * ==================================================================== */
#define UART_THREAD_STACK_SIZE 1024
#define UART_THREAD_PRIORITY   7

void uart_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printk("[UART Thread] Started\n");

    struct telemetry_data telem;

    while (1) {
        /* Block until processed telemetry data is available */
        k_msgq_get(&telemetry_msgq, &telem, K_FOREVER);

        /* Output matching our roadmap's exact UART protocol:
         * P:<pressure>,T:<temp>,A:<altitude> */
        printk("P:%.2f,T:%.2f,A:%.2f\n",
               (double)telem.pressure_pa,
               (double)telem.temp_c,
               (double)telem.altitude_m);
    }
}

K_THREAD_DEFINE(uart_tid, UART_THREAD_STACK_SIZE,
                uart_thread_entry, NULL, NULL, NULL,
                UART_THREAD_PRIORITY, 0, 0);

/* ====================================================================
 * Main Thread (Priority: 0, initializes system)
 * ==================================================================== */
int main(void)
{
    printk("\n============================================\n");
    printk("  ZephyrAir: Sensor Subsystem Initialized   \n");
    printk("  Target: native_sim / nRF5340             \n");
    printk("============================================\n");

    return 0;
}