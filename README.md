# Spotykach Template

This project can be used as a starting point for a project.

Prepared by Nick Donaldson, Infrasonic Audio LLC and licensed under
MIT license.

## Setup

Clone the repo recursively or run `git submodule update --init --recursive`
to update submodules (libDaisy + DaisySP)

Note that the ws2812 driver requires a slight modification to libDaisy,
as such the libDaisy submodule commit ref points at a specific branch within
the Infrasonic Audio fork, containing a few other modifications as well, namely
within the MIDI classes. See code for example usage.

## Compiling and Installing

### Installing the Bootloader

_NOTE: you must have physical access to the Daisy Seed itself in order to flash the bootloader onto the Seed_

The firmware depends on the official v6.2 version of the [Daisy Bootloader](https://electro-smith.github.io/libDaisy/md_doc_2md_2__a7___getting-_started-_daisy-_bootloader.html),
You must install the bootloader onto the seed FIRST before doing anything else.

**This only needs to be done once per seed**

To install the Daisy bootloader you must first put the Seed into ST System Bootloader mode:

1. Connect the Seed to your computer with a USB cable using its built-in USB Micro-B connector
2. Hold down the BOOT button and press the RESET button once
3. Open a terminal and navigate (`cd`) to the root directory of this repository (the one containing the `Makefile`)
4. Program the bootloader into internal flash by invoking `make program-boot` from a terminal

Alternatively you can use the [Daisy Web Programmer](https://electro-smith.github.io/Programmer/) to install the bootloader.

1. First, download the [bootloader binary](https://raw.githubusercontent.com/electro-smith/libDaisy/master/core/dsy_bootloader_v6_2-extdfu-2000ms.bin)
2. Connect the Seed to your computer via its built-in USB Micro-B connector
3. Hold down the BOOT button and press the RESET button once
4. Open the [Daisy Web Programmer](https://electro-smith.github.io/Programmer/) web app in Chrome
5. Pair with the Seed as per the instructions on the Web Programmer (should show up as "DFU in FS Mode")
6. Use the "Browse..." button to select the bootloader binary you downloaded
7. Program it onto the Seed using the "Program" button.

**IMPORTANT**: DO NOT use the "Flash Bootloader Image" button in the Daisy Web Programmer to program the bootloader.
That feature of the web programmer uses an older out-of-date version of the Daisy Bootloader that will not work correctly.

## Compiling the Firmware

First you must build the libraries. For convenience there is a target in the `Makefile` for this, so you
simply need to run the following command from a terminal (in the root directory of this repository):

`make -j8 libs`

_Note: The `-j8` flag runs make with 8 parallel jobs, feel free to change the number. It works best
if you use the number of cores (including hyperthread cores) on your machine._

Then, build the actual firmware code:

`make -j8`

If successful the compiled binaries will end up in the `build/` directory along with many other
intermediate build files:

```
spotykach-hwtest.bin
spotykach-hwtest.elf
```

The `.elf` file is mainly used for debugging. The `.bin` file is the one that the DFU
utilities will flash onto the Seed.

Invoke the `make` command with `DEBUG=1` before it in order to enable logging via the internal
USB connector:

`make clean && DEBUG=1 make -j8`

Note that if Debug Logging is enabled, execution will halt until serial monitor is attached
to display the USB serial output. This is so you don't miss the logging from the SD card test.

## Flashing the Firmware

The bootloader version used in this project enables USB DFU firmware updating from the _external_
USB port - i.e. the USB-C port on the rear of the main PCB, NOT the one on the Seed. Application
firmware can only be flashed using the USB-C port.

Before flashing the firmware, ensure the Seed is properly seated on the female headers on the PCB
and that connecting the USB-C connector to power successfully powers the Seed and PCB.

### Option 1: Using `make`

1. Compile the firmware using the steps above
2. Connect the USB-C connector on the main PCB to the computer (ensure the cable is not power-only)
3. Press the RESET button on the Seed once and release it. The red LED on the seed will pulse slowly.
4. Within 2 seconds of releasing RESET, press the BOOT button once and release it.
5. The red LED should flash several times quickly and then resume pulsing slowly.
6. Run the command `make program-dfu` from a terminal

Once finished, the device will automatically boot the new firmware.

### Option 2: From a precompiled .bin file

1. Download the .bin file for the firmware (someone will need to provide a compiled .bin file)
2. Connect the USB-C connector on the main PCB to the computer (ensure the cable is not power-only)
3. Press the RESET button on the Seed once and release it. The red LED on the seed will pulse slowly.
4. Within 2 seconds of releasing RESET, press the BOOT button once and release it.
5. The red LED should flash several times quickly and then resume pulsing slowly. This indicates that the Seed is in bootloader / firmware update mode.
6. Go to the [Daisy Web Programmer](https://electro-smith.github.io/Programmer/) in Chrome and pair with the device named "Daisy Bootloader"
7. Select the .bin file you downloaded using "Choose file..."
8. Click "Program" at the bottom to program the firmware

Wait for the process to finish. Once finished, the seed should automatically reboot and
run the firmware after a two-second delay.

### ⚠️ A Word of Caution

For any of these flashing workflows using the Daisy Web Programmer, make
*extra sure* that you are in the correct bootloader mode (ST or Daisy)
and that you choose the correct file.

The official Daisy Web Programmer will gladly attempt to flash any file, even
a completely random file, into any flash address on the chip without proper validation.
This can "brick" (temporarily) the device and require reinstallation of either the
bootloader, the firmware binary, or both.
