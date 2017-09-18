################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/I2C.c \
../src/main.c \
../src/sine_wave.c \
../src/stm32f1xx_hal_msp.c \
../src/stm32f1xx_it.c 

OBJS += \
./src/I2C.o \
./src/main.o \
./src/sine_wave.o \
./src/stm32f1xx_hal_msp.o \
./src/stm32f1xx_it.o 

C_DEPS += \
./src/I2C.d \
./src/main.d \
./src/sine_wave.d \
./src/stm32f1xx_hal_msp.d \
./src/stm32f1xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F105xC -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -I"../system/include/stm32f1xx" -I"../system/include/cmsis/device" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


