################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_Module/Src/beep.c \
../User_Module/Src/delay.c \
../User_Module/Src/exti.c \
../User_Module/Src/key.c \
../User_Module/Src/led.c \
../User_Module/Src/retarget.c \
../User_Module/Src/sys.c \
../User_Module/Src/usart.c 

OBJS += \
./User_Module/Src/beep.o \
./User_Module/Src/delay.o \
./User_Module/Src/exti.o \
./User_Module/Src/key.o \
./User_Module/Src/led.o \
./User_Module/Src/retarget.o \
./User_Module/Src/sys.o \
./User_Module/Src/usart.o 

C_DEPS += \
./User_Module/Src/beep.d \
./User_Module/Src/delay.d \
./User_Module/Src/exti.d \
./User_Module/Src/key.d \
./User_Module/Src/led.d \
./User_Module/Src/retarget.d \
./User_Module/Src/sys.d \
./User_Module/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
User_Module/Src/%.o User_Module/Src/%.su User_Module/Src/%.cyclo: ../User_Module/Src/%.c User_Module/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/STWorkSpace/project2/User_Module/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User_Module-2f-Src

clean-User_Module-2f-Src:
	-$(RM) ./User_Module/Src/beep.cyclo ./User_Module/Src/beep.d ./User_Module/Src/beep.o ./User_Module/Src/beep.su ./User_Module/Src/delay.cyclo ./User_Module/Src/delay.d ./User_Module/Src/delay.o ./User_Module/Src/delay.su ./User_Module/Src/exti.cyclo ./User_Module/Src/exti.d ./User_Module/Src/exti.o ./User_Module/Src/exti.su ./User_Module/Src/key.cyclo ./User_Module/Src/key.d ./User_Module/Src/key.o ./User_Module/Src/key.su ./User_Module/Src/led.cyclo ./User_Module/Src/led.d ./User_Module/Src/led.o ./User_Module/Src/led.su ./User_Module/Src/retarget.cyclo ./User_Module/Src/retarget.d ./User_Module/Src/retarget.o ./User_Module/Src/retarget.su ./User_Module/Src/sys.cyclo ./User_Module/Src/sys.d ./User_Module/Src/sys.o ./User_Module/Src/sys.su ./User_Module/Src/usart.cyclo ./User_Module/Src/usart.d ./User_Module/Src/usart.o ./User_Module/Src/usart.su

.PHONY: clean-User_Module-2f-Src

