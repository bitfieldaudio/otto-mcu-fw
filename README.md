# toot-mcu-fw
This is the MCU firmware for TOOT synthesizer

The folder toot-mcu-fw contains the STM32CubeIDE Eclipse project for the TOOT MCU firmware. While there are many toolchains available for developing on STM32, STM32Cube was chosen for the following reasons:

- Free
- Supported by ST Micro
- Mac, Windows, Linux support
- All-in-one package install containing toolchain and IDE
- Will allow seamless migration between STM32 processor families

Initial development will be done with the help of STM32's HAL library of drivers. Later development will gradually remove the HAL library in favor of more efficient custom implementations. The STM32Cube Code Generator is quite nice for visualizing the configuration of hardware peripherals and identifying potential conflicts.
