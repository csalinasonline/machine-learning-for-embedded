################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/app_x-cube-ai.c \
../Src/main.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f7xx.c 

OBJS += \
./Src/app_x-cube-ai.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/app_x-cube-ai.d \
./Src/main.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DARM_MATH_CM7 '-D__FPU_PRESENT=1' -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Inc" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/AI/AI/data" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/AI/AI/include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/DSP/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/Application/SystemPerformance/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


