# STM32 CCD acquisition experiments

Experimental firmware for testing TCD CCD timing and signal capture with STM32. The repository contains separate STM32CubeIDE projects exploring timer-generated CCD control signals, ADC acquisition with DMA and USB CDC communication.

## Start here

| Project | Purpose |
| --- | --- |
| [TCDFunctionTest](TCDFunctionTest) | CCD capture experiment with timer/PWM control, ADC DMA and USB CDC |
| [onepulsemode](onepulsemode) | Separate timer / one-pulse experiment |

The most useful entry points are each project's `Core/Src/main.c` and `.ioc` configuration. Generated IDE metadata, build output and history files are also present; they are not the main implementation.

## Acquisition path

In [TCDFunctionTest/Core/Src/main.c](TCDFunctionTest/Core/Src/main.c):

- TIM2, TIM3, TIM4 and TIM5 coordinate PWM/timing signals.
- ADC1 uses 12-bit resolution and channel 3.
- TIM4 channel 4 triggers ADC conversions.
- DMA transfers ADC readings into a CCD pixel buffer.
- `SingleCapture()` starts and stops capture; timer callbacks track the capture sequence.
- `CDCReceiveCallback()` recognizes a lowercase `start` command.
- USB CDC is used to transmit captured data.

These are experimental implementations, not a finalized acquisition protocol or calibrated spectrometer.

## Build and inspect

1. Install STM32CubeIDE and import the individual project using its existing project files.
2. Inspect its `.ioc`, linker script and clock configuration to confirm the MCU and board match your hardware. The repository includes STM32F401CCUx configuration.
3. Connect the matching board and debug probe, build the selected project, then flash/debug it.
4. Inspect CCD clock, SH and ICG signals with an oscilloscope or logic analyzer before evaluating the ADC capture.
5. Check the USB CDC receive and transmit code for the exact command and payload format of the selected revision.

A compatible CCD and analog front end are needed for meaningful sensor measurements. No reproducible performance benchmark or hardware validation report is included here.

## Related host-side work

[My spectrometer UI fork](https://github.com/FaizanTabassum/spectrometer_UI) contains Python acquisition and plotting experiments. Its current parser expects a different frame layout from this firmware experiment; verify and align commands, lengths and sample encoding before connecting the two.

## Status

Preserved as a development and hardware-debugging project. Useful areas for further work include a documented binary protocol, capture-length validation, error handling and reproducible timing measurements.
