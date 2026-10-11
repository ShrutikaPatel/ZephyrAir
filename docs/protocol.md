# Communication & Telemetry Protocol

## 1. UART Telemetry Frame Format

The `uart_thread` outputs newline-terminated ASCII frames at the active sampling rate (default 10 Hz):

```text
P:<pressure_pa>,T:<temp_c>,A:<altitude_m>\n
```

### Example Output
```text
P:101423.88,T:24.66,A:-8.23
P:101423.54,T:24.69,A:-8.20
P:101423.01,T:24.71,A:-8.15
```

| Field | Unit | Description |
| :--- | :---: | :--- |
| `P` | Pascals (`Pa`) | Compensated and IIR-filtered atmospheric pressure |
| `T` | Celsius (`°C`) | Compensated ambient temperature |
| `A` | Meters (`m`) | Barometric altitude relative to standard sea-level pressure (`101325 Pa`) |

---

## 2. Interactive Zephyr Shell Commands

The firmware exposes a `sensor` command group over the Zephyr Shell (`CONFIG_SHELL=y`):

| Command | Arguments | Description |
| :--- | :--- | :--- |
| `sensor start` | None | Starts or resumes periodic sensor acquisition |
| `sensor stop` | None | Pauses periodic sensor acquisition |
| `sensor odr <hz>` | `1` .. `50` | Updates the acquisition Output Data Rate (Hz) at runtime |
| `sensor status` | None | Prints operational state, ODR, sample count, I2C retry count, and bus health |
| `sensor fault <n>` | `0` .. `100` | Injects `n` consecutive I2C `NACK` faults into the BMP390 emulator |

---

## 3. I2C Bus Transactions (BMP390 at `0x76`)

1. **Initialization (`bmp3_init`)**
   - Reads `CHIP_ID` (`0x00`) and verifies `0x60`.
   - Writes `0xB6` to `CMD` (`0x7E`) to trigger a soft reset.
   - Reads 21 calibration bytes from `0x31`–`0x45` and computes floating-point compensation coefficients.
2. **Configuration (`bmp3_set_sensor_settings` & `bmp3_set_op_mode`)**
   - Writes interleaved register/value pairs to enable pressure and temperature (`PWR_CTRL` `0x1B`), 4x pressure oversampling (`OSR` `0x1C`), 50 Hz ODR (`ODR` `0x1D`), and switches to Normal Mode.
3. **Periodic Data Read (`bmp3_get_sensor_data`)**
   - Burst-reads 6 bytes from `DATA_0`..`DATA_5` (`0x04`–`0x09`) and applies Bosch polynomial compensation.
