################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/dma.c \
../Src/ethernetif.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/heater.c \
../Src/lwip.c \
../Src/main.c \
../Src/stepper.c \
../Src/stm32f2xx_hal_msp.c \
../Src/stm32f2xx_hal_timebase_TIM.c \
../Src/stm32f2xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f2xx.c \
../Src/tim.c \
../Src/usart.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/adc.o \
./Src/dma.o \
./Src/ethernetif.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/heater.o \
./Src/lwip.o \
./Src/main.o \
./Src/stepper.o \
./Src/stm32f2xx_hal_msp.o \
./Src/stm32f2xx_hal_timebase_TIM.o \
./Src/stm32f2xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f2xx.o \
./Src/tim.o \
./Src/usart.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/adc.d \
./Src/dma.d \
./Src/ethernetif.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/heater.d \
./Src/lwip.d \
./Src/main.d \
./Src/stepper.d \
./Src/stm32f2xx_hal_msp.d \
./Src/stm32f2xx_hal_timebase_TIM.d \
./Src/stm32f2xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f2xx.d \
./Src/tim.d \
./Src/usart.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -D_TIMEVAL_DEFINED -D_SYS_TIME_H_ '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F207xx -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/system" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/STM32F2xx_HAL_Driver/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/STM32F2xx_HAL_Driver/Inc/Legacy" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/CMSIS/Device/ST/STM32F2xx/Include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/netif/ppp/polarssl" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/posix" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Middlewares/Third_Party/LwIP/system/arch" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Drivers/CMSIS/Include" -I"E:/Projects/3D Printer Project/3DPrinterFirmware/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


