</div>
<div align="middle">
  <h1>STM32 Blue Pill with FreeRTOS </h1>
</div>

</div>
<div align="left">
	<a href="https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/LICENSE">
		<img src="https://img.shields.io/static/v1.svg?label=License&message=GNU GPL v3.0&color=blue&style=flat-square" height="20">
  	</a>
</div>

## Third Party Code
This project includes code from [stm32f429](https://github.com/MaJerle/stm32f429/tree/main/00-STM32F429_LIBRARIES), and [MPU6050](https://github.com/leech001/MPU6050) licensed under GPL v3.0:
- [Core/Inc/fonts.h](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Inc/fonts.h): from [here](https://github.com/MaJerle/stm32f429/blob/main/00-STM32F429_LIBRARIES/tm_stm32f4_fonts.h)
- [Core/Src/fonts.c](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Src/fonts.c): from [here](https://github.com/MaJerle/stm32f429/blob/main/00-STM32F429_LIBRARIES/tm_stm32f4_fonts.c)
- [Core/Inc/ssd1306.h](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Inc/ssd1306.h): from [here](https://github.com/MaJerle/stm32f429/blob/main/00-STM32F429_LIBRARIES/tm_stm32f4_ssd1306.h)
- [Core/Src/ssd1306.c](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Src/ssd1306.c): from [here](https://github.com/MaJerle/stm32f429/blob/main/00-STM32F429_LIBRARIES/tm_stm32f4_ssd1306.c)
- [Core/Inc/mpu6050.h](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Inc/mpu6050.h): from [here](https://github.com/leech001/MPU6050/blob/master/Src/mpu6050.h).
- [Core/Src/mpu6050.c](https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/Core/Src/mpu6050.c): from [here](https://github.com/leech001/MPU6050/blob/master/Src/mpu6050.c).


## STM32 VS Code Extension setup
1. This repository uses STM32 VS Code Extension, please ensure the following items are installed:<br>	
    *  [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubeprog.html)
    *  [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)
    *  [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html)<br><br>	
2.  Ensure STM32 VS Code Extension is installed.<br><br>
3.  Once the extension is installed, in the extensions tab, select STM32 VS Code Extension -> Manage (gear icon) -> Settings, insert the path for **<ins> STM32CubeMX</ins>** executable and **<ins>STM32CubeCLT</ins>** folder. <br><br>	
4.  Launch the STM32CubeMX,open the .ioc file and select generate code. <br><br>
5.  Go the .vscode/launch.json, insert the following configuration for **<ins>OpenOCD</ins>** debug:

        {
              "name": "Debug (OpenOCD)", //no opencod for py32
        "cwd": "${workspaceRoot}",
        "executable": "${workspaceFolder}/build/Debug/100_integrated_system_prototype.elf",
        "armToolchainPath": "${config:STM32VSCodeExtension.cubeCLT.path}/GNU-tools-for-STM32/bin",
        "toolchainPrefix": "arm-none-eabi",
        "gdbPath": "gdb-multiarch",
        "request": "launch",
        "type": "cortex-debug",
        "servertype": "openocd",
        "interface": "swd",
        "preLaunchTask": "${defaultBuildTask}",
        "device": "STM32F103C8Tx",
        // "runToEntryPoint": "main",
        "svdFile": "${config:STM32VSCodeExtension.cubeCLT.path}/STMicroelectronics_CMSIS_SVD/STM32F103.svd",
        "configFiles": [
          "interface/stlink.cfg",
          "target/stm32f1x.cfg"
        ],
        "preLaunchCommands": [
          "monitor halt",
          "monitor sleep 200"
        ],
        "postLaunchCommands": [
          "monitor halt",
          "monitor sleep 200"
        ]
          } 

## Flashing the code
1.  Once the code is ready, right click CmakeLists.txt and select Configure All Projects.<br><br>
2.  Right click CmakeLists.txt again, and select Build All Projects.<br><br>
3.  To flash to the STM32 blue pill, opens up the command pallate using ctrl + shift + p, select `Tasks: Run Task` -> `CubeProg: Flash project (SWD)`.<br><br>
4.  To start debug mode, Run and Debug (ctrl + shift + D ), select `Debug (OpenOCD)` and start debugging (F5).<br><br>
5.  Alternatively, you can add `"Build + Flash"` in the OpenOCD `"preLaunchTask":`, or execute `Tasks: Run Task` -> `Build + Flash` using command pallate.<br><br>

## Pin Configuraiotn:
<div align="middle">
	<img src="https://github.com/YJ0528/stm32_freertos_blue_pill/blob/main/images/Screenshot%20from%202024-11-16%2004-10-54.png" height="500">
	<br><br>		
</div>



