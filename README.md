# stm-can

STM32 firmware for York Formula Student.

## Project structure

* `src/bms.cc` - Battery management system firmware
* `src/bus_logger.cc` - Logs all messages on the CAN bus to SWD
* `src/etc.cc` - Electronic throttle control firmware
* `system/` - CMSIS and startup code for Cortex-M3
* `test/` - Host-runnable unit tests for platform independent code

## Building the firmware

Ensure a suitable ARM toolchain is in the `PATH` and run

    cmake --preset release -GNinja
    cmake --build build-release
    
### Flashing with stlink

If [stlink](https://github.com/texane/stlink) is installed, flash targets will be available for each executable. For
example, `flash-etc`.

## Building the unit tests

    cmake --preset host -GNinja
    cmake --build build-host
    ./build-host/tests
