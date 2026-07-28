# DAPU firmware — acquisition, processing and logging

Implementation of the DAPU section of `Embarcado (2).tex`, limited to what the
Verification & Validation section needs: **sensor acquisition, on-board
processing and SD logging**. The links to the other subsystems are *not*
implemented (see [Out of scope](#out-of-scope)).

Target: **STM32H723VGT6, WeAct Studio core board**, 200 MHz, HSE 25 MHz.

---

## 1. Pin map

### Analog — all on one ADC1 scan

| Pin | Channel | Rank | Signal |
|---|---|---|---|
| PA0 | ADC1_INP16 | 1 | Aileron deflection (flex sensor) |
| PA1 | ADC1_INP17 | 2 | Angle of attack vane (potentiometer) |
| PC0 | ADC1_INP10 | 3 | Elevator deflection (flex sensor) |
| PC1 | ADC1_INP11 | 4 | Rudder deflection (flex sensor) |
| PC4 | ADC1_INP4 | 5 | Battery voltage divider |
| PC5 | ADC1_INP8 | 6 | Pitot — MPS20N0040D → INA333 → buffer |
| PB1 | ADC1_INP5 | 7 | Electret microphone |

### Digital

| Function | Peripheral | Pins |
|---|---|---|
| Barometer bus | SPI1, mode 0, 6.25 MHz | PA5 SCK, PA6 MISO, PA7 MOSI |
| BMP280 CS | GPIO, idle **high**, very high speed | PE7 |
| MS5611 CS | GPIO, idle **high**, very high speed | PE9 |
| **IMU bus** | **I2C1, ~385 kHz, address 0x68/0x69** | **PB8 SCL, PB9 SDA** |
| ICM20948 CS | unused (SPI fallback only) | PE8 |
| GPS NEO-M8N | USART2 + DMA1_Stream1 | PA2 TX, PD6 RX |
| Console | USART1, 115200 | PA9 TX, PA10 RX |
| SD card | SDMMC1, 4-bit, 25 MHz | PC8–PC12, PD2 |
| ADC sample clock | TIM2 TRGO (no pin) | — |
| ADC transfer | DMA1_Stream0 | — |

### Indicators

| Pin | Meaning |
|---|---|
| PE3 `LED_BOARD` | On-board LED. Blink code, see below |
| PB3 `LED_RUN` | 1 Hz heartbeat once the 200 Hz loop is running |
| PB4 `LED_SENSORS` | Solid = every expected sensor answered at boot |
| PB5 `LED_SD` | Toggles on every block written to the card |
| PB6 `LED_GNCU` | Reserved (link not implemented) |
| PB7 `LED_CGU` | Reserved (link not implemented) |
| PE10 `L_workingStatus` | On after a successful boot |
| PA8 `Buzzer` | Reserved |

Unused / parked: PA3, PB0 (analog), PE11 (input, was the ADC2 trigger),
PE12/PE14 (SPI4 — not connected on this board).

### PE3 blink code

The on-board LED is the only diagnostic that works with nothing wired to the
board, so every way the firmware can die is given a distinct pattern:

Every power-up starts with **three quick flashes** from the top of `main()`,
before the MPU and the clock tree are touched. They run off the HSI, so they
appear no matter what fails afterwards.

| PE3 | Meaning |
|---|---|
| Nothing at all, ever | The firmware is not executing: power, BOOT0/reset, or nothing was flashed |
| 3 flashes, then dark | Reached `main()` and stopped in a HAL polling loop, or is in the ~3 s sensor probe + calibration |
| 3 flashes, then 1 blink per group | `Error_Handler()` — a peripheral refused to initialise, or a task failed to be created |
| 3 flashes, then 3 blinks per group | Hard fault |
| 3 flashes, then 4 blinks per group | FreeRTOS stack overflow — read `pcTaskName` in the debugger |
| 3 flashes, then 5 blinks per group | FreeRTOS heap exhausted — raise `configTOTAL_HEAP_SIZE` |
| 3 flashes, then steady 1 Hz | Normal: scheduler running, 200 Hz loop on time |

`DAPU_BlinkForever()` enables the GPIOE clock itself, so the blink codes work
even when the failure happens before `MX_GPIO_Init()` — which is exactly where
a clock configuration failure lands.

Set `LED_BOARD_ACTIVE_LOW` in `main.h` to `0` if the LED on your board lights
the other way round (only affects the solid-on states; blinking works either
way).

---

## 2. Building

The project **compiles and runs as delivered** — a verification build with the
STM32CubeIDE 13.3 toolchain links clean (175 kB flash, 126 kB RAM).

`DAPU.ioc` has been updated to match the code, so regenerating should be a
no-op. **Before pressing "Generate Code" in CubeMX, check these seven things**,
because a few of them were written into the `.ioc` by hand:

1. **TIM2 is deliberately NOT in CubeMX.** It only emits TRGO — no pin, no
   channel, no interrupt — and CubeMX refuses to treat that as a configured
   peripheral: every regeneration drops it from the IP list, deletes the
   `MX_TIM2_Init()` call and comments `HAL_TIM_MODULE_ENABLED` back out,
   leaving `tim.c` orphaned and the build full of timer errors. Instead:
   * `tim.c` / `tim.h` are maintained by hand
   * `MX_TIM2_Init()` is called from `/* USER CODE BEGIN 2 */` in `main.c`
   * `HAL_TIM_MODULE_ENABLED` is a **project preprocessor define** in
     `.cproject`, not a setting in `stm32h7xx_hal_conf.h`

   All three survive regeneration. Do **not** activate TIM2 in the GUI unless
   you also delete the `MX_TIM2_Init()` call, or it will be initialised twice.
   After any regeneration, confirm `ADC1.ExternalTrigConv` is still
   `ADC_EXTERNALTRIG_T2_TRGO`; the Boot task prints `ADC scan : NOT RUNNING`
   if the trigger has gone missing.
2. **ADC1** has 7 ranks in the order of the table above, Scan Conversion Mode
   *Enabled*, Resolution 16 bit, Clock Prescaler *Asynchronous div 2*,
   External Trigger = *Timer 2 Trigger Out event*, rising edge,
   DMA Continuous Requests / Conversion Data Management = *DMA Circular*,
   Overrun = *Overwritten*, sampling time `387.5 Cycles` on every rank.
3. **DMA**: `ADC1` on DMA1 Stream0, half-word/half-word, circular;
   `USART2_RX` on DMA1 Stream1, byte/byte, circular.
4. **NVIC**: DMA1 Stream0, DMA1 Stream1, SDMMC1, USART1 and USART2 interrupts
   all enabled.
5. **FreeRTOS** → Config parameters → `TOTAL_HEAP_SIZE` = **81920**. The 12
   tasks need ~49 kB of stack alone and will not be created with the CubeMX
   default of 15360; `osThreadNew()` returning NULL now trips `Error_Handler()`
   rather than booting a half-populated system.
6. **SDMMC1** ClockDiv = `2`.
7. **SPI1**: 8 bits, CPOL Low, CPHA 1 Edge, Prescaler 16, and *Master Keep IO
   State* **enabled**.

Also verify in *Project → Properties → C/C++ Build → Settings*:

* **MCU GCC Compiler → Include paths** contains `../Drivers/CMSIS/DSP/Include`
* **MCU GCC Compiler → Preprocessor** defines `ARM_MATH_CM7`
* **MCU GCC Linker → Libraries**: `arm_cortexM7lfdp_math`, search path
  `../Drivers/CMSIS/DSP/Lib/GCC`. Keep it **relative**: a
  `${workspace_loc:...}` path expands to an absolute Windows path that CubeIDE
  emits unquoted (`-LC:\GitHub\...`), and the build shell eats the backslashes,
  producing `cannot find -larm_cortexM7lfdp_math` even though the file is there.
* **MCU GCC Compiler → Preprocessor** also defines `HAL_TIM_MODULE_ENABLED`
  (see item 1 above)
* **MCU/MPU Settings**: *Use float with printf from newlib-nano* is ticked —
  otherwise every `%f` prints blank. Note this is on the *MCU/MPU Settings*
  page, not under the linker.

None of the four settings above live in the `.ioc`, so CubeMX cannot undo
them — but a *fresh clone* of the project will not have them either.

CMSIS-DSP (headers + the prebuilt Cortex-M7 hard-float archive) has already
been copied into `Drivers/CMSIS/DSP/`. If linking it ever becomes a problem,
set `DAPU_USE_CMSIS_DSP` to `0` in `dapu_config.h`: `rpm_fft.c` falls back to a
bundled radix-2 FFT with no external dependency.

---

## 3. Tasks

Priorities follow Tab. *Prioridades DAPU* of the document, minus the tasks that
are out of scope. FreeRTOS numeric priorities are in parentheses.

| Task | Doc prio | CMSIS priority | Rate | Notes |
|---|---|---|---|---|
| `Boot` | — | Realtime (48) | once | start-up sequence, then exits |
| `SD_Writer` | 14 | High7 (47) | on demand | drains the ring buffer to the card |
| `LogSample` | — | High6 (46) | 200 Hz | builds one record per base cycle |
| `GPS` | 12 | AboveNormal7 (39) | 100 Hz poll | NAV-PVT arrives at 10 Hz |
| `IMU` | 11 | AboveNormal6 (38) | 200 Hz | magnetometer sub-sampled to 100 Hz |
| `Pitot` | 9 | AboveNormal4 (36) | 200 Hz | |
| `BMP280` | 8 | AboveNormal3 (35) | 100 Hz | |
| `MS5611` | 7 | AboveNormal2 (34) | 10 ms step → 50 Hz | |
| `AOA` | 3 | Normal2 (26) | 200 Hz | |
| `RPM` | 2 | Normal1 (25) | 1 Hz | blocks on the 1 s microphone block |
| `Flex` | 1 | Normal (24) | 200 Hz | 3 surfaces + battery |
| `Console` | — | Low (8) | 20 Hz | live plot, deliberately lowest |

All periodic tasks use `osDelayUntil()`, so a late cycle does not shift the
following ones. Each one calls `dapu_boot_wait()` first, so no sensor is
touched — and no `osDelayUntil` baseline is taken — until `Boot` has finished.

> **Never put blocking code in `MX_FREERTOS_Init()`.** The ARM_CM4F port starts
> with `uxCriticalNesting = 0xaaaaaaaa` and only zeroes it in
> `xPortStartScheduler()`, so the `taskEXIT_CRITICAL()` inside the very first
> `osMutexNew()` fails to restore `BASEPRI`. SysTick stays masked until
> `osKernelStart()` runs, `HAL_GetTick()` stops advancing, and any `HAL_Delay()`
> after that point hangs forever. Creating RTOS objects there is fine; anything
> that waits belongs in `BootTask`.

---

## 4. Calibrate before recording data

`Core/Inc/dapu_config.h` holds every tunable. These are **placeholders** and
must be measured on the bench first:

```c
#define VBAT_DIVIDER_RATIO           11.0f     /* Vbat / Vadc of your divider */
#define PITOT_V_PER_PA               1.25e-3f  /* from the INA333 design point */
#define AOA_GAIN_DEG_PER_V           1.0f      /* aoa_deg = a*V + b */
#define AOA_OFFSET_DEG               1.0f
#define FLEX_AIL_GAIN_DEG_PER_V      1.0f      /* one a/b pair per surface  */
#define FLEX_AIL_OFFSET_DEG          1.0f
/* ... FLEX_ELE_*, FLEX_RUD_* ... */
#define RPM_THRUST_K                 0.0f      /* T[N] = k * rpm^2, static test */
```

Every one of them is a simple `a*V + b`, exactly as requested, so a bench
calibration is two numbers per channel. Since the log stores **both** the raw
counts and the converted values, a wrong constant can also be corrected
afterwards in Python from the raw column.

The firmware calibrates three things by itself at every boot (~2 s, aircraft
must be **still and level**):

* residual gyro bias, subtracted by the IMU task
* reference pressure of each barometer → altitude starts at `h = 0`
* pitot zero offset voltage

Results are printed on USART1 at boot.

---

## 5. Console (USART1, 115200)

Single keystrokes:

| Key | Action |
|---|---|
| `p` | Toggle the live plot. **Turn it off for the runs that matter** — it is the only thing that competes with the SD writer. |
| `s` | Status summary: sensor flags, measured rate of every task, GPS frame/checksum counters, records written and dropped. |
| `q` | Flush and close the log file (safe to remove the card). |
| `r` | Open a new log file. |

The plot is Teleplot/Serial Studio format (`>name:value`), same as the F4 bench
code. Recording is continuous from boot regardless; `q`/`r` only exist so a run
can be ended without losing the last second of data.

---

## 6. Log format and decoding

Logging starts automatically at boot into the first free `LOGnnnn.BIN`.

* 1 kB header: magic, version, record size, base rate, real microphone sample
  rate, and the **schema** (a Python `struct` format plus the field names)
* then fixed 224-byte little-endian records at 200 Hz (~45 kB/s)
* every record carries a magic word, a monotonic `sample_id`, a microsecond
  timestamp and a CRC-16

Because the schema travels inside the file, the decoder never has to be kept in
sync by hand — adding a field to `log_record_t` is enough.

```bash
python Tools/decode_log.py LOG0001.BIN
```

prints the record count, duration, status flags and — directly usable for the
V&V chapter — a **configured vs. measured rate** table built from the
per-sensor sequence counters:

```
acquisition task         configured   measured    error
log record (base loop)       200.0     200.50     0.3%
ICM20948 accel/gyro          200.0     200.00     0.0%
BMP280                       100.0      99.75    -0.3%
MS5611                        50.0      49.62    -0.8%
NEO-M8N NAV-PVT               10.0       9.52    -4.8%
```

For the plots:

```bash
python Tools/decode_log.py LOG0001.BIN --csv run1.csv --drop-reserved
```

The decoder resynchronises on the record magic after a truncated write and
rejects records whose CRC fails, so a log cut short by a power loss still
decodes up to the last good record.

---

## 7. Deviations from `Embarcado (2).tex`

Worth recording in the report, with the reason:

1. **BMP280 instead of BMP390** — that is the part on the bench. Only
   `bmp280.c/h` would change. It also cannot run 100 Hz at ×16 oversampling
   (≈23 Hz max), so it is configured ×2 pressure / ×1 temperature with IIR
   coefficient 4, giving ≈123 Hz and meeting the 100 Hz of the table.
2. **One ADC instead of two** — all seven analog channels are in a single
   ADC1 sequence triggered by TIM2 at 1024 Hz, rather than a 200 Hz scan plus a
   separate 1024 Hz microphone converter. The microphone still gets an exactly
   periodic, jitter-free clock (which is what the FFT requires), the 200 Hz
   tasks decimate the same buffer by averaging 5 consecutive scans — which also
   reduces their noise — and one ADC and one timer are freed.
3. **Microphone rate is 1024.0026 Hz, not 1024 Hz** — 1024 Hz cannot be divided
   exactly out of the 100 MHz timer clock. The residual 2.6 ppm is carried into
   `ADC_SCAN_RATE_HZ` and into the file header, so the FFT bin scaling stays
   exact. Bin width is 1.0000 Hz.
4. **GPS at 10 Hz needs 115200 baud** — a 100-byte NAV-PVT at 10 Hz is 10 kbit/s
   and does not fit in the module's factory 9600. `gps_init()` raises the link
   to 115200 (trying both bauds so it works whatever state the module was left
   in) and saves it. The F4 code had this as a pending TODO.
5. **GPS dynamic model 6 (airborne < 1 g)** instead of the F4's 3 (pedestrian).
   Set `GPS_DYN_MODEL` back to `3` for static bench tests.
6. **MS5611 at 50 Hz via a 3-state machine** — the part has no continuous mode
   and one OSR-4096 conversion takes 9.04 ms, so pressure and temperature are
   converted alternately on a 10 ms tick, producing a fully compensated pair
   every 20 ms at maximum resolution.
7. **Magnetometer read through the ICM20948's own I2C master** in continuous
   mode rather than one-shot. The original driver spent 2–3 ms of busy-wait
   `HAL_Delay()` per magnetometer read, which is not compatible with a 200 Hz
   task.
8. **SD writing is decoupled by a 32 kB ring buffer.** The document puts
   "Salvar dado" at the highest priority; that is preserved, but the 200 Hz
   sampler only ever pushes into the ring, so a card housekeeping pause can
   never stretch the acquisition period — at worst it overflows the ring, which
   is counted and flagged in the record status.
9. **The ICM20948 is on I2C1, not SPI.** The SPI link to the module on the
    bench could never be made to answer, so the IMU was moved to its own I2C
    bus on PB8/PB9 while the barometers keep SPI1. `ICM20948_USE_I2C` in
    `dapu_config.h` switches back. The cost is bus time: one 12-byte burst is
    ~375 us at 385 kHz versus ~30 us on SPI, i.e. 7.5 % of the 5 ms period.
    Acceptable on the bench; the final PCB should use SPI. Accelerometer and
    gyroscope are read as a single 12-byte burst (`ACCEL_XOUT_H`..`GYRO_ZOUT_L`
    are contiguous) and the user-bank register is cached, which together
    halve the traffic and guarantee both come from the same sample instant.
10. **All SPI register access is a single `HAL_SPI_TransmitReceive()`**, never
    `HAL_SPI_Transmit()` followed by `HAL_SPI_Receive()`. The H7 SPI is a
    different peripheral from the F4's: it is disabled at the end of every
    `HAL_SPI_*` call, so a split read stops and restarts SCK in the middle of a
    transaction with CS still asserted, and the slave sees spurious clock
    edges. The same driver code works unchanged on an F4, which makes this a
    trap when porting. `SPI_MASTER_KEEP_IO_STATE_ENABLE` (AFCNTR) is also set
    on SPI1 so the pins stay driven while the peripheral is off.
11. **`icm20948_init()` no longer spins forever** on a missing chip, and the
   accelerometer low-pass filter write, which the upstream library sent to the
   gyroscope register, is corrected. Full scales are ±8 g / ±1000 dps
   (`dapu_config.h`) rather than ±16 g / ±2000 dps, for resolution within the
   ±3 g of RA.04.
12. **HAL timebase is still SysTick.** CubeMX will warn about this with
    FreeRTOS enabled; it works, and changing it was not worth the extra moving
    part in this build.

---

## 8. Out of scope

Not implemented in this build, by design:

* **GNCU link** (SPI2 + PD0 chip select) — the peripheral is still initialised
  by CubeMX but nothing drives it.
* **CGU link** (USART3) — same.
* **Federated EKFs** (attitude, altitude, velocity/position). All the inputs
  they need are acquired, converted and logged, so they can be added as three
  tasks slotting into the priority gaps left at 10, 6 and 5.
* **State machine** (Boot / Self-Test / Warmup / Armed / Flight / Failsafe) —
  the boot sequence performs the Self-Test parameter initialisation the
  document assigns to that state, but there is no supervisor task.
* **Health management** and the **IWDG watchdog**. Sensor presence, task
  sequence counters, ring overflow, SD errors and battery voltage are all
  already measured and logged, which is most of the raw material a health
  manager needs.
