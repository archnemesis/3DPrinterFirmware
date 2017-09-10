################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F207xx -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/system" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/STM32F2xx_HAL_Driver/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/STM32F2xx_HAL_Driver/Inc/Legacy" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/CMSIS/Device/ST/STM32F2xx/Include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif/ppp/polarssl" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/posix" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/system/arch" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/CMSIS/Include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


