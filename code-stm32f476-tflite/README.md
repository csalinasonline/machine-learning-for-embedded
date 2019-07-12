STM32F103 cmake template
----

This is a template cmake project for the stm32f7xx. So what's
so special about it? Well, it supports the following things:

* CMAKE !!
* STM32 HAL Driver Library
* FreeRTOS
* My custom header lib

## Usage

To select the which libraries you want to use you need to provide
cmake with the proper options. By default all the options are set
to `OFF`. The supported options are:

* `USE_HAL_DRIVER`: If set to `ON` enables the HAL Driver library
* `USE_FREERTOS`: If set to `ON` enables FreeRTOS

You also need to provide cmake with the source folder by pointing
the folder to the `SRC` parameter.

Finally, you also need to provide the path of the toolchain to
use in the `CMAKE_TOOLCHAIN`.

## Using CubeMX
Usually is more convenient to create your project with CubeMX,
then after you setup all the hardware and peripherals you can create
the code (I prefer `SW4STM32`, but it doens't really matter in this case).
Then after the code is exported then you just need to copy the files
that CubeMX customizes for your setup.

The files that usually you need to get and place them in your
`source/src` folder are:

* main.h
* main.c
* stm32f7xx_hal_conf.h
* stm32f7xx_hal_msp.c
* stm32f7xx_it.h
* stm32f7xx_it.c
* system_stm32f7xx.c (in case you have custom clocks)

In your case there might be more files. Usually are the files
that are in the exported `Inc` and `Src` folder.


## Cloning the code
Because this repo has dependencies on other submodules, in order to
fetch the repo use the following command:

```sh
git clone --recursive -j8 git@bitbucket.org:dimtass/stm32f7xx-cmake-template.git

# or for http
git clone --recursive -j8 https://dimtass@bitbucket.org/dimtass/stm32f7xx-cmake-template.git
```

## Examples
To use the `HAL Driver` library example run this command:
```sh
CLEANBUILD=true USE_HAL_DRIVER=ON SRC=src_c_hal ./build.sh
```

To use the `freertos` example run this command:
```sh
CLEANBUILD=true USE_HAL_DRIVER=ON USE_FREERTOS=ON SRC=src_c_freertos ./build.sh
```

To flash the HEX file in windows use st-link utility like this:
```"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p build-stm32\src_\stm32-cmake-template.hex -Rst```

To flash the bin in Linux:
```st-flash --reset write build-stm32/src_/stm32-cmake-template.bin 0x8000000```

Just replace `src_` with the proper folder in your case

## Testing
I've also added a script to test the current supported default projects.
To use it just run:

```sh
./test.sh
```

If everything goes right you should see something like this:

```sh
Building test case: CLEANBUILD=true USE_HAL_DRIVER=ON SRC=src_c_hal
---RESULT: SUCCESS

Building test case: CLEANBUILD=true USE_HAL_DRIVER=ON USE_FREERTOS=ON SRC=src_c_freertos
---RESULT: SUCCESS

Building test case: CLEANBUILD=true USE_HAL_DRIVER=ON SRC=src_cpp_hal
---RESULT: SUCCESS

Building test case: CLEANBUILD=true USE_HAL_DRIVER=ON USE_FREERTOS=ON SRC=src_cpp_freertos
---RESULT: SUCCESS
```

## FW details
* `CMSIS version`: 5.0.4
* `HAL Driver Library version`: 1.2.6

