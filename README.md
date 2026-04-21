# 实验：BMI088 陀螺仪读取

## 任务

debug 读取 BMI088 陀螺仪数据。

## 配置

配置 SPI

## 操作

此项目由 ARM-MDK 迁移到此。

> **对于 DSP Library**
>
> 此处 DSP Library 由 CMakeLists.txt 控制
>
> ```CMakeLists.txt
> # Add project symbols (macros)
> target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
>     # Add user defined symbols
>     ARM_MATH_CM4
>     ARM_MATH_MATRIX_CHECK
>     ARM_MATH_ROUNDING
> )
> ```
>
> 如上，需要添加 ARM_MATH_CM4 宏来启用 DSP Library 的 Cortex-M4 优化实现，否则会使用通用的 C 实现。
>
> 验证是否启用：
>
> ```Text
> ninja: Entering directory `build/Debug'
> [1/57] C:\Users\ASUS\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin\arm-none-eabi-gcc.exe -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -IC:/Users/ASUS/Documents/micropico/can-motor-test/Mybsp/Inc -IC:/Users/ASUS/Documents/micropico/can-motor-test/Mymiddlewares -IC:/Users/ASUS/Documents/micropico/can-motor-test/Algorithm -IC:/Users/ASUS/Documents/micropico/can-motor-test/Mydrive/Inc -IC:/Users/ASUS/Documents/micropico/can-motor-test/Application/Inc -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Core/Inc -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Drivers/CMSIS/Include -IC:/Users/ASUS/Documents/micropico/can-motor-test/cmake/stm32cubemx/../../Middlewares/ST/ARM/DSP/Inc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard  -x assembler-with-cpp -MMD -MP -g -MD -MT CMakeFiles/can-motor-test.dir/startup_stm32f407xx.s.obj -MF CMakeFiles\can-motor-test.dir\startup_stm32f407xx.s.obj.d -o CMakeFiles/can-motor-test.dir/startup_stm32f407xx.s.obj -c C:/Users/ASUS/Documents/micropico/can-motor-test/startup_stm32f407xx.s
> ```
> `-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING` 表示已启用 ARM_MATH_CM4、ARM_MATH_MATRIX_CHECK 和 ARM_MATH_ROUNDING 宏
>

## 结果

![image](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/motorTest/bmi.png)
