################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DigitalIoPin.cpp \
../src/MAXIM1249.cpp \
../src/SPI.cpp \
../src/cr_cpp_config.cpp \
../src/cr_startup_lpc15xx.cpp \
../src/spi_test_3.cpp 

C_SRCS += \
../src/crp.c \
../src/sysinit.c 

OBJS += \
./src/DigitalIoPin.o \
./src/MAXIM1249.o \
./src/SPI.o \
./src/cr_cpp_config.o \
./src/cr_startup_lpc15xx.o \
./src/crp.o \
./src/spi_test_3.o \
./src/sysinit.o 

CPP_DEPS += \
./src/DigitalIoPin.d \
./src/MAXIM1249.d \
./src/SPI.d \
./src/cr_cpp_config.d \
./src/cr_startup_lpc15xx.d \
./src/spi_test_3.d 

C_DEPS += \
./src/crp.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -std=c++11 -D__NEWLIB__ -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\lpc_board_nxp_lpcxpresso_1549\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\lpc_chip_15xx\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\src\include" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\src\portable\GCC\ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -mcpu=cortex-m3 -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=c11 -D__NEWLIB__ -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\lpc_board_nxp_lpcxpresso_1549\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\lpc_chip_15xx\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\inc" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\src\include" -I"C:\Users\OMISTAJA\Documents\MCUXpressoIDE_10.2.1_795\workspace\Solar_Project\FreeRTOS\src\portable\GCC\ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


