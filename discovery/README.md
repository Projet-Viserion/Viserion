# Dicovery

## How to compile with TrueSTUDIO

The code on the Discovery exceeding 32KB, the TrueSTUDIO IDE for STM32 was used. This IDE can be downloaded from the [Attolic official website] (https://atollic.com/resources/download/).

The following describes how to configure a project on TrueSTUDIO to compile the project code.

### Creating a project

1. Create a new project `File -> Project -> C Project`. 
2. Choose `Embbeded C Project` as `Project type`.
3. Choose its location and a name and click on `Next`.
4. Choose the development board `STM32F3 -> Boards -> STM32F3_Discovery` then `Next`.
5. Do not modify anything for `software settings` then `Next`.
6. Finally for `hardware debug setting` choose `STLink` then click `Next`.
7. Do not change anything on the last window (the `debug` and `release` targets must be selected) and click on `Finish`.


### Importing files

1. Delete (right click and then `delete`) the repertories `Drivers` and `src` (which has been automatically created with the project).
2. Import the files from the `geicar` repertory with a right click on the project then `import` and `File system`. Select the `discovery` directory of the `geicar` project.
3. Expands the `discovery` directory and check the `AHRS`, `Drivers` and `src` repertories and click on `Finish`.

### Project configuration

1. Open the project properties window `Project -> Properties`.
2. Go to `C/C++ General -> Path and symbols` and in the `Includes` tab click on `Add` then check `is to workspace path` and click on `workspace` to select the directory `AHRS` and validate (twice `OK`).
3. In the `Symbols` tab click on `Add` and add in the name field `USE_FULL_LL_DRIVER`. Redo the same thing with `USE_HAL_DRIVER`.


### Deleting templates

1. Go to the `Drivers -> STM32F3xx_HAL_Driver -> Src` directory and right-click `Resource configuration -> Exclude from Build...` on : `stm32f3xx_hal_msp_template.c`, `stm32f3xx_hal_timebase_rtc_alarm_template.c`, `stm32f3xx_hal_timebase_rtc_wakeup_template.c` and `stm32f3xx_hal_timebase_tim_template.c`.


### Compilation, loading and execution

1. To compile the project launch `Project -> Build Project`.
2. Plug the USB link to the board. 
3. To flash the code and switch to debug mode, right-click on the project and select `Debug as -> Embedded C/C++ Application`.
4. If necessary, update the ST-Link probe driver.
5. Run as a debug.

Remark that the executable is flashed.
