#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <zephyr/drivers/i2c.h>

/* Get Devicetree specification for BMP390 */
#define BMP390_NODE DT_NODELABEL(bmp390)
static const struct i2c_dt_spec bmp390_dev = I2C_DT_SPEC_GET(BMP390_NODE);

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
 * Thread 1: Sensor Acquisition Thread (Priority: 5) ::: Priorities 1 to 4 left as headroom for future threads
 * ==================================================================== */
#define SENSOR_THREAD_STACK_SIZE 1024
#define SENSOR_THREAD_PRIORITY   5

void sensor_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printk("[Sensor Thread] Started\n");

     /* =================================================================
     * I2C Hardware Handshake with Bosch BMP390
     * ================================================================= */
    printk("[Sensor Thread] Checking I2C bus for BMP390...\n");
    if (!i2c_is_ready_dt(&bmp390_dev)) {
        printk("[Sensor Thread] ERROR: I2C bus controller not ready!\n");
        return;
    }
    uint8_t chip_id_reg = 0x00; // BMP390 CHIP_ID register address
    uint8_t chip_id_val = 0;    // Buffer to hold response
    /* Perform the 2-step I2C read dance */
    int ret = i2c_write_read_dt(&bmp390_dev, &chip_id_reg, 1, &chip_id_val, 1);
    if (ret != 0) {
        printk("[Sensor Thread] ERROR: I2C read failed with code %d\n", ret);
        return;
    }
    if (chip_id_val == 0x60) {
        printk("\n**************************************************************\n");
        printk(">>> [BMP390] HANDSHAKE SUCCESS!                               <<<\n");
        printk(">>> Found Bosch BMP390 at I2C 0x%02X | Chip ID verified: 0x%02X <<<\n",
               bmp390_dev.addr, chip_id_val);
        printk("**************************************************************\n\n");
    } else {
        printk("[Sensor Thread] ERROR: Unexpected Chip ID 0x%02X (Expected 0x60)\n", chip_id_val);
        return;
    }

    uint32_t sample_idx = 0;

    while (1) {
        struct raw_sensor_data raw;
        raw.timestamp_ms = k_uptime_get_32();

        /* For now, simulating a slow barometric drift + noise. */
        raw.raw_pressure_pa = 101325.0f + 100.0f * sinf(sample_idx * 0.05f);
        raw.raw_temp_c      = 23.5f + 1.2f * cosf(sample_idx * 0.02f);

        /* Send to Queue 1 */
        if (k_msgq_put(&raw_data_msgq, &raw, K_NO_WAIT) != 0) {
            printk("[Sensor Thread] Warning: raw_data_msgq full!\n");
        }

        sample_idx++;

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

/* Barometric formula: calculates altitude above sea-level */
static inline float calculate_altitude(float pressure_pa)
{
    const float sea_level_pa = 101325.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

void process_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printk("[Process Thread] Started\n");

    struct raw_sensor_data raw;
    float filtered_pressure = 101325.0f;

    while (1) {
        /* Block until a new raw sample arrives */
        k_msgq_get(&raw_data_msgq, &raw, K_FOREVER);

        /* Simple 1st-order IIR low-pass filter: y[n] = 0.85*y[n-1] + 0.15*x[n] */
        filtered_pressure = (0.85f * filtered_pressure) + (0.15f * raw.raw_pressure_pa);

        struct telemetry_data telem = {
            .timestamp_ms = raw.timestamp_ms,
            .pressure_pa  = filtered_pressure,
            .temp_c       = raw.raw_temp_c,
            .altitude_m   = calculate_altitude(filtered_pressure)
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

        /* P:<pressure>,T:<temp>,A:<altitude> */
        printk("P:%.2f,T:%.2f,A:%.2f\n",
               telem.pressure_pa,
               telem.temp_c,
               telem.altitude_m);
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