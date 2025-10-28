# DS3231 I2C Driver

This library provides an interface for the DS3231 Real-Time Clock (RTC) using the ESP-IDF I2C master driver.

## Default Configuration

Upon initialization within the main application, this driver configures the DS3231 with a specific set of defaults optimized for low-power operation and resilience in battery-powered applications.

The `ds3231_set_default_config()` function is called at startup, which writes the value `0x34` to the Control Register (0x0E) and clears key status flags.

### Control Register (0x0E) Defaults

The following settings are applied to ensure robust timekeeping with minimal power draw:

| Bit | Flag Name                               | Recommended Value | Rationale for Lowest Power & Resilience                                                              |
|-----|-----------------------------------------|-------------------|------------------------------------------------------------------------------------------------------|
| 7   | EOSC (Enable Oscillator)                | 0 (Run on V<sub>BAT</sub>)  | **Crucial for Resilience:** Ensures the oscillator runs continuously when main power is lost, maintaining accurate timekeeping. |
| 6   | BBSQW (Battery-Backed Square-Wave)      | 0 (Disabled on V<sub>BAT</sub>) | **Crucial for Low Power:** Disables the Square-Wave output when on battery, minimizing current draw. |
| 5   | CONV (Convert Temperature)              | 0                 | Keep temperature conversion set to auto-mode.                                                        |
| 4   | RS2 (Rate Select 2)                     | 1                 | When INTCN=0, sets SQW output to 1 Hz (when V<sub>CC</sub> is present).                                |
| 3   | RS1 (Rate Select 1)                     | 1                 | When INTCN=0, sets SQW output to 1 Hz (when V<sub>CC</sub> is present).                                |
| 2   | INTCN (Interrupt Control)               | 0 (Square-Wave Mode) | Sets the INT/SQW pin to SQW mode since interrupts are not used by default.                             |
| 1   | A2IE (Alarm 2 Interrupt Enable)         | 0 (Disabled)      | Prevents the INT/SQW pin from being asserted by an unused alarm setting.                               |
| 0   | A1IE (Alarm 1 Interrupt Enable)         | 0 (Disabled)      | Prevents the INT/SQW pin from being asserted by an unused alarm setting.                               |

main.c applies these setting on each boot, chepaer DS3231 breakout boards don't tolerate power fluctuations/cut offs too well, often winding up with corrupted configuration registers, resulting in erratic behavior. this fix is just as crude as those cheap *ss breakouts 
from AliExpress, but keeps (literally) the clock ticking :.-D

### Status Register (0x0F)

The startup routine also clears the **Oscillator Stop Flag (OSF)** and any pending **Alarm Flags (A1F, A2F)** to ensure the device starts in a clean, known state. This is critical for validating that the time is reliable after a power cycle.