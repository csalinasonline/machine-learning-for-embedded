################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/AI/AI/src/network.c 

OBJS += \
./Middlewares/ST/AI/AI/src/network.o 

C_DEPS += \
./Middlewares/ST/AI/AI/src/network.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/AI/AI/src/%.o: ../Middlewares/ST/AI/AI/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DARM_MATH_CM7 '-D__FPU_PRESENT=1' -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Inc" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/AI/AI/data" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/AI/AI/include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/DSP/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Drivers/CMSIS/Include" -I"/home/dimtass/Downloads/stm32f746-code-keras/stm32f746-code-keras/Middlewares/ST/Application/SystemPerformance/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


