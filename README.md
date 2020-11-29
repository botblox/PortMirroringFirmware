# IP175G Port Mirroring Configuration code

## Aims
* This code is designed as a reference to help program BotBlox SwitchBlox managed switch
* You can use this code as a reference to copy into your STM32CubeIDE project that you can use to configure the SwitchBlox

## Port mirroring 
* This configuration will allow you to configure port mirroring where communication between a specific network socket can be duplicated onto another device connected to another port.

## How to use
* Firstly, load `STM32CubeIDE` application and create a new project with `STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)` and select `IP175G_PORTMIRRORING_CONFIG.ioc` in this repo.
* Navigate to `Core/Src/main.c` in the new project and replace with `Core/Src/main.c` in this repo.
* Configure the Port Mirroring as per the instructions in `Core/Src/main.c`

## Upkeep
* We encourage and want to hear from clients how they are using our code!
* If you find a bug in this code or have a suggestion for an improvement, we would love to know so feel free to make an issue on the page and one of our maintainers will get back to you!
