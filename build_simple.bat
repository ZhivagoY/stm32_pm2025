@echo off
echo === STM32 Project Builder ===
echo.

echo Step 1: Cleaning previous build...
del *.o *.elf *.bin *.map 2>nul

echo Step 2: Compiling main.c...
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=c99 -Os -DSTM32F103C8 -DSTM32F10X_MD -Icmsis -Istm32 -c main.c -o main.o
if errorlevel 1 (
    echo ERROR compiling main.c!
    pause
    exit /b 1
)

echo Step 3: Compiling startup file...
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=c99 -Os -DSTM32F103C8 -DSTM32F10X_MD -Icmsis -Istm32 -c stm32/startup/startup_stm32f10x_md.c -o startup.o
if errorlevel 1 (
    echo ERROR compiling startup!
    pause
    exit /b 1
)

echo Step 4: Compiling system file...
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=c99 -Os -DSTM32F103C8 -DSTM32F10X_MD -Icmsis -Istm32 -c stm32/system_stm32f10x.c -o system.o
if errorlevel 1 (
    echo ERROR compiling system!
    pause
    exit /b 1
)

echo Step 5: Linking all files...
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -nostdlib -nostartfiles -specs=nosys.specs -Wl,--gc-sections -Wl,-Map=project.map -T stm32f103c8.ld main.o startup.o system.o -o project.elf
if errorlevel 1 (
    echo ERROR linking!
    echo Trying alternative link method...
    arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -nostartfiles -specs=nosys.specs -Wl,--gc-sections -Wl,-Map=project.map -T stm32f103c8.ld main.o startup.o system.o -o project.elf -lc -lm
    if errorlevel 1 (
        echo Still errors in linking
        pause
        exit /b 1
    )
)

echo Step 6: Creating BIN file...
arm-none-eabi-objcopy -O binary project.elf project.bin

echo Step 7: Creating HEX file...
arm-none-eabi-objcopy -O ihex project.elf project.hex

echo.
echo === BUILD SUCCESSFUL! ===
echo Created files:
dir *.elf *.bin *.hex

echo.
echo Firmware size:
arm-none-eabi-size project.elf

echo.
pause
